#!/usr/bin/env python3
"""Host client for the remote test harness (see ../README.md).

Usage:  rc.py [--url http://<nano-ip>] <command> [args]      (or set ROBOT_URL)

  ping | status | state | stop | gyrozero
  tel <ms>                telemetry period on the Uno (0 = off)
  drive <left> <right> <ms>
  watch <seconds>         print telemetry for a while
  raw "<line>"            send a raw command line to the Uno

Everything sent and received is logged to runs/<timestamp>-<name>/log.jsonl.
Moves always end with a STOP, even on errors or Ctrl-C.
"""
import argparse
import json
import os
import random
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

HERE = os.path.dirname(os.path.abspath(__file__))

# Local copies of the firmware limits (the Uno enforces them too).
MAX_SPEED = 200
MAX_DRIVE_MS = 2000

SENSORS = ("F", "R", "B", "L")
T_FIELDS = (
    "ms", "F", "R", "B", "L",
    "mx", "my", "mz", "ax", "ay", "az", "gx", "gy", "gz",
    "yaw_decideg", "ml", "mr",
)

# Used to ask the Nano "where is the end of the log?" without receiving any history.
SYNC_SINCE = 1_000_000_000


class RobotError(Exception):
    pass


@dataclass
class Reply:
    ok: bool
    text: str


def parse_telemetry(line: str) -> Optional[dict]:
    """Parse a 'T ...' line from the Uno into a dict, or None if it is not a valid telemetry line."""
    parts = line.split()
    if not parts or parts[0] != "T" or len(parts) != 1 + len(T_FIELDS):
        return None
    try:
        values = [int(p) for p in parts[1:]]
    except ValueError:
        return None
    frame = dict(zip(T_FIELDS, values))
    frame["yaw_deg"] = frame["yaw_decideg"] / 10.0
    return frame


class Robot:
    def __init__(self, url: str, name: str = "session", timeout: float = 3.0):
        self.url = url.rstrip("/")
        self.timeout = timeout
        self.since: Optional[int] = None  # None until we have synced to the end of the Nano's log
        self._id = random.randint(100, 8000)  # random start so ids from earlier sessions never match
        self.dropped = 0
        self.inbox = []  # (seq, line) received but not yet consumed
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.run_dir = os.path.join(HERE, "runs", f"{stamp}-{name}")
        os.makedirs(self.run_dir, exist_ok=True)
        self._log = open(os.path.join(self.run_dir, "log.jsonl"), "a", buffering=1)

    # ---- logging ----
    def log(self, kind: str, **fields):
        fields.update({"t": time.time(), "kind": kind})
        self._log.write(json.dumps(fields) + "\n")

    def close(self):
        self._log.close()

    # ---- HTTP ----
    def _get(self, path: str) -> str:
        try:
            with urllib.request.urlopen(self.url + path, timeout=self.timeout) as r:
                return r.read().decode(errors="replace")
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise RobotError(f"GET {path} failed: {e}") from e

    def _post(self, path: str, form: dict) -> str:
        data = urllib.parse.urlencode(form).encode()
        req = urllib.request.Request(self.url + path, data=data, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return r.read().decode(errors="replace")
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise RobotError(f"POST {path} failed: {e}") from e

    def status(self) -> str:
        return self._get("/status")

    # ---- receiving ----
    def fetch(self):
        """Pull new lines from the Nano's buffer into the inbox. The first call skips history from earlier sessions."""
        syncing = self.since is None
        body = self._get(f"/log?since={SYNC_SINCE if syncing else self.since}")
        lines = body.split("\n")
        header = lines[0].split()  # ['next', N, 'dropped', M]
        if len(header) < 4 or header[0] != "next":
            raise RobotError(f"unexpected /log response: {lines[0]!r}")
        if syncing:
            self.since = int(header[1])
            return
        if int(header[3]) > 0:
            self.dropped += int(header[3])
            self.log("dropped", count=int(header[3]))
        for text in lines[1:]:
            if not text:
                continue
            seq_text, _, line = text.partition(" ")
            seq = int(seq_text)
            self.since = max(self.since, seq)
            self.inbox.append((seq, line))
            self.log("rx", seq=seq, line=line, frame=parse_telemetry(line))

    def frames(self) -> list:
        """Remove and return all telemetry frames currently in the inbox."""
        out, keep = [], []
        for item in self.inbox:
            frame = parse_telemetry(item[1])
            if frame:
                out.append(frame)
            else:
                keep.append(item)
        self.inbox = keep
        return out

    def _take(self, predicate) -> Optional[str]:
        for i, (_, line) in enumerate(self.inbox):
            if predicate(line):
                del self.inbox[i]
                return line
        return None

    # ---- commands ----
    def send(self, line: str) -> int:
        if self.since is None:
            self.fetch()  # sync before the first command so we cannot confuse old replies for new ones
        self._id += 1
        cid = self._id
        self.log("tx", id=cid, line=line)
        self._post("/cmd", {"cmd": f"#{cid} {line}"})
        return cid

    def cmd(self, line: str, timeout: float = 2.0) -> Reply:
        """Send a command and wait for its '#id OK/ERR ...' reply."""
        cid = self.send(line)
        tag = f"#{cid} "
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.fetch()
            text = self._take(lambda l: l.startswith(tag))
            if text is not None:
                rest = text[len(tag):]
                return Reply(rest.startswith("OK"), rest)
            time.sleep(0.1)
        raise RobotError(f"no reply to '{line}' within {timeout}s")

    def wait_event(self, timeout: float) -> str:
        """Wait for an 'E ...' event line from the Uno."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.fetch()
            text = self._take(lambda l: l.startswith("E "))
            if text is not None:
                return text
            time.sleep(0.05)
        raise RobotError("no event line within timeout")

    def stop(self):
        try:
            self.cmd("STOP", timeout=2.0)
        except RobotError:
            # try once more without waiting for a reply
            try:
                self.send("STOP")
            except RobotError:
                pass

    def drive(self, left: int, right: int, ms: int) -> str:
        """Run a timed move; returns the final event line ('E done' or 'E guard ...')."""
        if abs(left) > MAX_SPEED or abs(right) > MAX_SPEED or not 1 <= ms <= MAX_DRIVE_MS:
            raise RobotError("drive arguments outside the safe limits")
        reply = self.cmd(f"DRIVE {left} {right} {ms}")
        if not reply.ok:
            raise RobotError(f"DRIVE refused: {reply.text}")
        return self.wait_event(ms / 1000.0 + 1.5)

    def telemetry(self, seconds: float, period_ms: int = 200) -> list:
        """Collect telemetry frames for a while. Returns a list of frame dicts."""
        reply = self.cmd(f"TEL {period_ms}")
        if not reply.ok:
            raise RobotError(f"TEL refused: {reply.text}")
        frames = []
        try:
            end = time.time() + seconds
            while time.time() < end:
                self.fetch()
                frames.extend(self.frames())
                time.sleep(0.1)
        finally:
            try:
                self.cmd("TEL 0")
            except RobotError:
                pass
        return frames


def show_frame(f: dict):
    d = " ".join(f"{s}={f[s]:>4}" for s in SENSORS)
    print(f"{f['ms']:>8} ms  {d}  mag=({f['mx']},{f['my']},{f['mz']})"
          f"  gyro=({f['gx']},{f['gy']},{f['gz']})  yaw={f['yaw_deg']:.1f}  motors=({f['ml']},{f['mr']})")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=os.environ.get("ROBOT_URL"), help="Nano base URL, e.g. http://192.168.1.50")
    ap.add_argument("--name", default="cli", help="run folder suffix")
    ap.add_argument("command")
    ap.add_argument("args", nargs="*")
    a = ap.parse_args()
    if not a.url:
        print("Set --url or ROBOT_URL", file=sys.stderr)
        return 2

    robot = Robot(a.url, a.name)
    moved = False
    try:
        c, args = a.command, a.args
        if c == "status":
            print(robot.status(), end="")
        elif c in ("ping", "stop", "gyrozero", "state"):
            reply = robot.cmd(c.upper(), timeout=4.0 if c == "gyrozero" else 2.0)
            print(reply.text)
            if c == "state":
                time.sleep(0.4)
                robot.fetch()
                for frame in robot.frames():
                    show_frame(frame)
        elif c == "tel":
            print(robot.cmd(f"TEL {int(args[0])}").text)
        elif c == "drive":
            moved = True
            print(robot.drive(int(args[0]), int(args[1]), int(args[2])))
        elif c == "watch":
            for frame in robot.telemetry(float(args[0])):
                show_frame(frame)
        elif c == "raw":
            print(robot.cmd(" ".join(args)).text)
        else:
            print(f"unknown command {c}", file=sys.stderr)
            return 2
        return 0
    except (RobotError, IndexError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    finally:
        if moved:
            robot.stop()
        robot.close()
        print(f"(log: {robot.run_dir})", file=sys.stderr)


if __name__ == "__main__":
    sys.exit(main())
