#!/usr/bin/env python3
"""Test 5: drive the bot along the board's x axis in short steps and record the sonars at every stop.

The bot is ASSUMED to start at heading 0: front sensor facing the board's right wall, +x toward it. The compass is not
read; the gyro is zeroed at the start and only used to report yaw drift. If forward moves the bot the other way on the
probe step, the script aborts.

The bot never leaves the margins of the original zumo_avr code: its centre stays at least 16 cm from the end walls
and 9 cm from the side walls (MOVE_X_LIMIT / MOVE_Y_LIMIT). The centre position is computed from the sonar readings
(sensor face to centre = 7 cm, as USS_DIST_ADJ_cm), and every step is checked BEFORE it is sent. The Uno's own wall
guards stay on as the backstop. A person must watch, with a hand near the battery switch.

Because F+B and R+L are fixed by the bot's size, the sweep shows, with no tape:
  - the ratio of the front and back gains (their changes are equal and opposite),
  - sideways drift (R+L stays constant, R changes) and yaw drift,
  - whether the pair sums hold near the walls, and the noise by range.
It cannot give the absolute gain of one sensor.

Usage: t05_sweep.py --url http://<nano-ip> [--legs 2] [--speed 100] [--step-mm 25] [--ramp 300] [--trim 0.086] [--dry-run]

--ramp raises the speed linearly over that many ms at the start of every step (the Uno does this). --trim runs the left
track that fraction faster than the right (the original zumo_avr code used 20/200 = 0.10).
"""
import argparse
import csv
import json
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from rc import Robot, RobotError  # noqa: E402

BOARD_X_MM = 851.0
BOARD_Y_MM = 568.3
SENSOR_TO_CENTRE_MM = 70.0   # USS_DIST_ADJ_cm
X_MARGIN_MM = 160.0          # board half-width minus MOVE_X_LIMIT
Y_MARGIN_MM = 90.0           # board half-height minus MOVE_Y_LIMIT
STOP_SHORT_MM = 30.0         # also stay this far inside the x limit, so a step that overshoots its prediction cannot cross it
PROBE_MS = 300
MAX_STEPS = 80
MAX_STEP_MS = 1500


def median_pose(frames):
    """Median reading per sensor over a list of telemetry frames, plus the std per sensor."""
    pose, std = {}, {}
    for s in ("F", "R", "B", "L"):
        v = [f[s] for f in frames if f[s] >= 0]
        if len(v) < max(3, len(frames) // 2):
            raise RobotError(f"sensor {s} has too few valid readings ({len(v)}/{len(frames)}); aborting")
        pose[s] = statistics.median(v)
        std[s] = statistics.pstdev(v)
    pose["yaw"] = frames[-1]["yaw_deg"]
    return pose, std


def centre(pose):
    """Bot centre (x, y) in mm from the board centre, x toward the front wall, y toward the left (top) wall."""
    xl = -BOARD_X_MM / 2 + pose["B"] + SENSOR_TO_CENTRE_MM
    xr = BOARD_X_MM / 2 - pose["F"] - SENSOR_TO_CENTRE_MM
    yb = -BOARD_Y_MM / 2 + pose["R"] + SENSOR_TO_CENTRE_MM
    yt = BOARD_Y_MM / 2 - pose["L"] - SENSOR_TO_CENTRE_MM
    return (xl + xr) / 2.0, (yb + yt) / 2.0


def read_pose(robot):
    time.sleep(0.6)  # let the bot settle and the sonar cycle (each sensor updates every ~160 ms)
    robot.fetch()
    robot.frames()
    return median_pose(robot.telemetry(1.4))


def slope(xs, ys):
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    d = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / d if d else float("nan")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=os.environ.get("ROBOT_URL"))
    ap.add_argument("--legs", type=int, default=2, help="number of passes between the two x limits (default 2)")
    ap.add_argument("--speed", type=int, default=100)
    ap.add_argument("--step-mm", type=float, default=25.0)
    ap.add_argument("--ramp", type=int, default=0, help="ramp time in ms at the start of each step (default 0 = jump)")
    ap.add_argument("--trim", type=float, default=0.086, help="left track speed boost as a fraction (default 0.086, estimated from run 3)")
    ap.add_argument("--dry-run", action="store_true", help="read the start pose and print the plan; send no motion")
    a = ap.parse_args()
    if not a.url:
        print("Set --url or ROBOT_URL", file=sys.stderr)
        return 2

    # the Uno caps speeds at 200, so a trim that would push the left track past it slows the right track instead
    left_speed = round(a.speed * (1 + a.trim))
    right_speed = a.speed
    if left_speed > 200:
        left_speed, right_speed = 200, round(200 / (1 + a.trim))
    probe_ms = max(PROBE_MS, a.ramp + 200)
    x_lim = BOARD_X_MM / 2 - X_MARGIN_MM
    y_lim = BOARD_Y_MM / 2 - Y_MARGIN_MM
    robot = Robot(a.url, "t05_sweep")
    rows = []
    try:
        reply = robot.cmd("GYROZERO", timeout=4.0)
        if not reply.ok:
            raise RobotError(f"GYROZERO failed: {reply.text}")
        pose, std = read_pose(robot)
        x, y = centre(pose)
        print(f"left/right speed {left_speed}/{right_speed}, ramp {a.ramp} ms")
        print(f"start: x={x:+.0f} y={y:+.0f} mm (limits x +/-{x_lim:.0f}, y +/-{y_lim:.0f})  "
              f"F={pose['F']:.0f} R={pose['R']:.0f} B={pose['B']:.0f} L={pose['L']:.0f}")
        if abs(x) > x_lim or abs(y) > y_lim:
            raise RobotError("the bot starts outside the margins; move it toward the board centre first")
        if a.dry_run:
            print("dry run: no motion sent")
            return 0
        rows.append(dict(step=0, direction=0, ms=0, x=x, y=y, **pose, **{f"std_{k}": v for k, v in std.items()}))

        # probe with one short step to learn how far the bot goes per millisecond. The speed is not known yet, so the
        # probe only runs when there is at least 100 mm of room to the front limit.
        if x > x_lim - STOP_SHORT_MM - 100:
            raise RobotError("too close to the front limit for the probe step; move the bot toward the back first")
        direction = 1
        print("probe step...", flush=True)
        end = robot.drive(left_speed, right_speed, probe_ms, a.ramp)
        if not end.startswith("E done"):
            raise RobotError(f"probe step ended with '{end}'")
        pose, std = read_pose(robot)
        x1, y1 = centre(pose)
        mm_per_ms = abs(x1 - x) / probe_ms
        if mm_per_ms * probe_ms < 2.0:
            raise RobotError("the bot did not move on the probe step (battery off?)")
        if x1 < x:
            raise RobotError("forward moved the bot toward -x; the sensors or the orientation are not what the script assumes")
        x, y = x1, y1
        rows.append(dict(step=1, direction=1, ms=probe_ms, x=x, y=y, **pose, **{f"std_{k}": v for k, v in std.items()}))
        print(f"  moved {mm_per_ms * probe_ms:.1f} mm in {probe_ms} ms ({mm_per_ms * 1000:.0f} mm/s at speed {a.speed})")

        legs_done = 0
        for step in range(2, MAX_STEPS):
            ms = int(max(100, min(MAX_STEP_MS, a.step_mm / mm_per_ms)))
            predicted = x + direction * ms * mm_per_ms
            if abs(predicted) > x_lim - STOP_SHORT_MM and predicted * direction > 0:
                legs_done += 1
                print(f"end of leg {legs_done} at x={x:+.0f} mm")
                if legs_done >= a.legs:
                    break
                direction = -direction
                continue
            end = robot.drive(direction * left_speed, direction * right_speed, ms, min(a.ramp, ms))
            if not end.startswith("E done"):
                raise RobotError(f"step {step} ended with '{end}'")
            pose, std = read_pose(robot)
            new_x, y = centre(pose)
            if abs(new_x) > x_lim or abs(y) > y_lim:
                raise RobotError(f"left the margins: x={new_x:+.0f} y={y:+.0f} mm")
            if abs(new_x - x) > 0.5:
                mm_per_ms = 0.7 * mm_per_ms + 0.3 * abs(new_x - x) / ms
            x = new_x
            rows.append(dict(step=step, direction=direction, ms=ms, x=x, y=y, **pose,
                             **{f"std_{k}": v for k, v in std.items()}))
            print(f"  step {step:>2} dir {direction:+d} {ms:>4} ms  x={x:+6.0f} y={y:+5.0f}  "
                  f"F={pose['F']:.0f} R={pose['R']:.0f} B={pose['B']:.0f} L={pose['L']:.0f}  yaw={pose['yaw']:.1f}",
                  flush=True)
    except (RobotError, KeyboardInterrupt) as e:
        print(f"stopped: {e}", file=sys.stderr)
    finally:
        robot.stop()

    if len(rows) < 4:
        print("not enough positions to analyse", file=sys.stderr)
        robot.close()
        return 1

    with open(os.path.join(robot.run_dir, "positions.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    fb = [r["F"] + r["B"] for r in rows]
    rl = [r["R"] + r["L"] for r in rows]
    # F falls by gF*d while B rises by gB*d, so the ratio of their total travel is gF/gB. Totals, not a regression of
    # step against step: the steps are all about the same size, so noise in them would bias a slope toward zero.
    ratio = sum(abs(rows[i]["F"] - rows[i - 1]["F"]) for i in range(1, len(rows))) / \
        sum(abs(rows[i]["B"] - rows[i - 1]["B"]) for i in range(1, len(rows)))
    yaw = [r["yaw"] for r in rows]
    summary = {
        "positions": len(rows),
        "F+B_mm": {"mean": statistics.fmean(fb), "min": min(fb), "max": max(fb)},
        "R+L_mm": {"mean": statistics.fmean(rl), "min": min(rl), "max": max(rl)},
        "gain_ratio_F_over_B": ratio,
        "R_vs_x_mm_per_mm": slope([r["x"] for r in rows], [r["R"] for r in rows]),
        "yaw_change_deg": yaw[-1] - yaw[0],
        "x_range_mm": [min(r["x"] for r in rows), max(r["x"] for r in rows)],
    }
    with open(os.path.join(robot.run_dir, "summary.json"), "w") as fh:
        json.dump(summary, fh, indent=2)
    print("\n--- summary ---")
    print(f"{len(rows)} positions, x from {summary['x_range_mm'][0]:+.0f} to {summary['x_range_mm'][1]:+.0f} mm")
    print(f"F+B: mean {summary['F+B_mm']['mean']:.1f} mm, range {summary['F+B_mm']['min']:.0f}..{summary['F+B_mm']['max']:.0f} "
          f"(spread {summary['F+B_mm']['max'] - summary['F+B_mm']['min']:.0f})")
    print(f"R+L: mean {summary['R+L_mm']['mean']:.1f} mm, range {summary['R+L_mm']['min']:.0f}..{summary['R+L_mm']['max']:.0f} "
          f"(spread {summary['R+L_mm']['max'] - summary['R+L_mm']['min']:.0f})")
    print(f"gain ratio F/B: {ratio:.3f}  (1.000 = equal)")
    print(f"sideways drift: R changes {summary['R_vs_x_mm_per_mm']:+.3f} mm per mm of x travel; "
          f"yaw changed {summary['yaw_change_deg']:+.1f} deg")
    print(f"(run folder: {robot.run_dir})")
    robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
