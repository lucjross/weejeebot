#!/usr/bin/env python3
"""A fake Nano + Uno for developing and testing rc.py without hardware.

Run:  python3 mock_robot.py [port]      (default 8765), then use --url http://127.0.0.1:<port>
It speaks the same HTTP API as nano_relay and answers the same commands as uno_remote, with made-up sensor values.
"""
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer

LOG_LINES = 40
lock = threading.Lock()
lines = []          # (seq, text)
next_seq = 1
tel_period = 0.0


def push(text: str):
    global next_seq
    with lock:
        lines.append((next_seq, text))
        next_seq += 1
        del lines[:-LOG_LINES]


def telemetry_line() -> str:
    ms = int(time.time() * 1000) % 100000000
    return f"T {ms} 300 250 260 90 -20 -6150 -12930 390 -778 16905 -580 -701 -1230 0 0 0"


def handle_command(text: str):
    parts = text.split()
    cid = ""
    if parts and parts[0].startswith("#"):
        cid = parts.pop(0) + " "
    if not parts:
        push(f"{cid}ERR empty")
        return
    c = parts[0]
    if c == "PING":
        push(f"{cid}OK pong")
    elif c == "STOP":
        push(f"{cid}OK")
    elif c == "STATE":
        push(f"{cid}OK")
        push(telemetry_line())
    elif c == "TEL":
        global tel_period
        ms = int(parts[1])
        tel_period = ms / 1000.0
        push(f"{cid}OK")
    elif c == "DRIVE":
        left, right, ms = int(parts[1]), int(parts[2]), int(parts[3])
        if abs(left) > 200 or abs(right) > 200 or not 1 <= ms <= 2000:
            push(f"{cid}ERR range")
        else:
            push(f"{cid}OK")
            threading.Timer(ms / 1000.0, lambda: push("E done")).start()
    elif c == "GYROZERO":
        push(f"{cid}OK bias=-60")
    else:
        push(f"{cid}ERR unknown")


def telemetry_loop():
    while True:
        time.sleep(0.05)
        if tel_period > 0:
            push(telemetry_line())
            time.sleep(tel_period)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _send(self, body: str, code: int = 200):
        data = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        url = urllib.parse.urlparse(self.path)
        q = urllib.parse.parse_qs(url.query)
        if url.path == "/log":
            since = int(q.get("since", ["0"])[0])
            with lock:
                newest = next_seq - 1
                oldest = lines[0][0] if lines else 1
                missed = max(0, oldest - (since + 1))
                out = f"next {newest} dropped {missed}\n"
                out += "".join(f"{s} {t}\n" for s, t in lines if s > since)
            self._send(out)
        elif url.path == "/status":
            self._send(f"uptime_ms {int(time.time() * 1000) % 100000000}\nrssi -50\nnext_seq {next_seq}\n")
        else:
            self._send("mock robot\n")

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        form = urllib.parse.parse_qs(self.rfile.read(length).decode())
        if self.path == "/cmd" and "cmd" in form:
            handle_command(form["cmd"][0])
            self._send("queued\n")
        else:
            self._send("missing cmd\n", 400)


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8765
    threading.Thread(target=telemetry_loop, daemon=True).start()
    print(f"mock robot on http://127.0.0.1:{port}")
    HTTPServer(("127.0.0.1", port), Handler).serve_forever()
