#!/usr/bin/env python3
"""Test 3: which physical sensor is which. No motion commands are sent.

Run it in the terminal where you can watch the prompts (in Claude Code: type  ! python3 <this file> --url http://<nano-ip> ).
For each sensor in turn it prints a prompt; hold a hand about 5 cm in front of that sensor until the next prompt.
It reports which sensor index actually dropped, and flags any mismatch with the expected position.

Usage: t03_identify.py --url http://<nano-ip> [--lead 3] [--hold 4]
"""
import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from rc import Robot, RobotError, SENSORS  # noqa: E402

NAMES = {"F": "FRONT", "R": "RIGHT", "B": "BACK", "L": "LEFT"}
NEAR_MM = 90   # a hand ~5 cm away should read well under this


def collect(robot, seconds):
    frames = []
    end = time.time() + seconds
    while time.time() < end:
        robot.fetch()
        frames.extend(robot.frames())
        time.sleep(0.1)
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=os.environ.get("ROBOT_URL"))
    ap.add_argument("--lead", type=float, default=3.0, help="seconds to get your hand in place")
    ap.add_argument("--hold", type=float, default=4.0, help="seconds recorded per sensor")
    a = ap.parse_args()
    if not a.url:
        print("Set --url or ROBOT_URL", file=sys.stderr)
        return 2

    robot = Robot(a.url, "t03_identify")
    results = []
    try:
        if not robot.cmd("TEL 200").ok:
            raise RobotError("TEL refused")
        print("Starting in 3 s. Keep the bot still. Hold your hand ~5 cm in front of each sensor when told.\n", flush=True)
        collect(robot, 3.0)
        for expect in SENSORS:
            print(f">>> COVER the {NAMES[expect]} sensor with your hand NOW (about 5 cm away) <<<", flush=True)
            collect(robot, a.lead)
            frames = collect(robot, a.hold)
            means = {s: statistics.fmean(f[s] for f in frames if f[s] >= 0) for s in SENSORS if any(f[s] >= 0 for f in frames)}
            nearest = min(means, key=means.get)
            ok = nearest == expect and means[nearest] < NEAR_MM
            results.append((expect, nearest, means, ok))
            print(f"    lowest: {nearest} ({means[nearest]:.0f} mm) -> {'OK' if ok else 'MISMATCH'}\n", flush=True)
        print("Done. You can take your hand away.", flush=True)
    except RobotError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    finally:
        try:
            robot.cmd("TEL 0")
        except RobotError:
            pass
        robot.close()

    print("\n| expected | responded | " + " | ".join(SENSORS) + " | result |")
    print("|---|---|" + "---|" * (len(SENSORS) + 1))
    for expect, nearest, means, ok in results:
        cells = " | ".join(f"{means.get(s, float('nan')):.0f}" for s in SENSORS)
        print(f"| {expect} | {nearest} | {cells} | {'OK' if ok else 'MISMATCH'} |")
    print(f"\n(run folder: {robot.run_dir})")
    return 0 if all(r[3] for r in results) else 3


if __name__ == "__main__":
    sys.exit(main())
