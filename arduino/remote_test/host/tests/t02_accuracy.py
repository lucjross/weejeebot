#!/usr/bin/env python3
"""Test 2: ultrasonic sensor accuracy against a ruler. No motion commands are sent.

Two subcommands share one data file, data/accuracy.csv (kept across sessions):

  capture --sensor F,B --true-mm 50 [--seconds 5] [--note "..."]
      Record telemetry for a few seconds while a flat target is a MEASURED distance in front of the named
      sensor(s) (window to target, in mm), and append one row per sensor.
  fit
      Per sensor, fit  reading = gain * true + offset  over all captured points and print the corrections.

Use a flat, hard target (a book or board) square to the sensor, with the bot still. Measure from the sensor face.

Usage: t02_accuracy.py --url http://<nano-ip> capture ... | t02_accuracy.py fit
"""
import argparse
import csv
import os
import statistics
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from rc import Robot, SENSORS  # noqa: E402

DATA = os.path.join(HERE, "..", "data", "accuracy.csv")
COLUMNS = ["time", "sensor", "true_mm", "mean_mm", "std_mm", "n", "note"]


def capture(a):
    names = [s.strip().upper() for s in a.sensor.split(",")]
    for s in names:
        if s not in SENSORS:
            sys.exit(f"unknown sensor {s}; use {','.join(SENSORS)}")
    robot = Robot(a.url, "t02_accuracy")
    try:
        frames = robot.telemetry(a.seconds)
    finally:
        robot.close()
    if len(frames) < 3:
        sys.exit("not enough data received")

    os.makedirs(os.path.dirname(DATA), exist_ok=True)
    new = not os.path.exists(DATA)
    with open(DATA, "a", newline="") as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(COLUMNS)
        for s in names:
            d = [f[s] for f in frames if f[s] >= 0]
            if not d:
                print(f"{s}: no valid readings")
                continue
            row = [datetime.now().isoformat(timespec="seconds"), s, a.true_mm, f"{statistics.fmean(d):.1f}",
                   f"{statistics.pstdev(d):.1f}", len(d), a.note]
            w.writerow(row)
            print(f"{s}: true {a.true_mm} mm -> read {row[3]} mm (std {row[4]}))")
    others = [s for s in SENSORS if s not in names]
    ctl = ", ".join(f"{s}={statistics.fmean(f[s] for f in frames if f[s] >= 0):.0f}" for s in others
                    if any(f[s] >= 0 for f in frames))
    print(f"controls (not logged; compare between runs to check the bot did not move): {ctl} mm")


def fit(_a):
    if not os.path.exists(DATA):
        sys.exit("no data yet; run capture first")
    by = {}
    with open(DATA, newline="") as fh:
        for r in csv.DictReader(fh):
            by.setdefault(r["sensor"], []).append((float(r["true_mm"]), float(r["mean_mm"])))
    print("sensor  points   gain    offset(mm)   residuals (reading - fit, mm)")
    for s in SENSORS:
        pts = by.get(s, [])
        if not pts:
            print(f"{s:>5}  0")
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        if len(pts) >= 2 and len(set(xs)) >= 2:
            mx, my = statistics.fmean(xs), statistics.fmean(ys)
            gain = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
            off = my - gain * mx
            res = ", ".join(f"{y - (gain * x + off):+.1f}" for x, y in zip(xs, ys))
            print(f"{s:>5}  {len(pts):>6}  {gain:6.3f}  {off:+9.1f}    {res}")
        else:
            errs = ", ".join(f"{y - x:+.0f}" for x, y in pts)
            print(f"{s:>5}  {len(pts):>6}  (need 2+ distances to fit; error vs ruler: {errs} mm)")
    print("\nraw points (true -> read):")
    for s in SENSORS:
        for x, y in sorted(by.get(s, [])):
            print(f"  {s:>2}: {x:6.0f} -> {y:6.1f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=os.environ.get("ROBOT_URL"))
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("--sensor", required=True, help="comma-separated, e.g. F,B")
    c.add_argument("--true-mm", type=float, required=True)
    c.add_argument("--seconds", type=float, default=5.0)
    c.add_argument("--note", default="")
    sub.add_parser("fit")
    a = ap.parse_args()
    if a.cmd == "capture":
        if not a.url:
            sys.exit("Set --url or ROBOT_URL")
        capture(a)
    else:
        fit(a)


if __name__ == "__main__":
    main()
