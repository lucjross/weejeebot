#!/usr/bin/env python3
"""Test 1+2: sensors and IMU at rest. No motion commands are sent.

Precondition: the bot is on the board, level, and completely still (hands off) for the whole run.
Procedure: GYROZERO, then record telemetry for --seconds at 200 ms.
Measures:  per-sensor noise and dropouts, gyro offset after zeroing and yaw drift, magnetometer heading noise,
           and how the opposite-sensor sums compare with the known board size (implies the bot's effective size).
Writes:    runs/<timestamp>-t01_rest/{log.jsonl,summary.json,report.md}

Usage: t01_rest.py --url http://<nano-ip> [--seconds 20] 
"""
import argparse
import json
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from rc import Robot, RobotError, SENSORS  # noqa: E402

BOARD_X_MM = 851.0   # BOARD_W_cm 85.1 in the original code
BOARD_Y_MM = 568.3   # BOARD_H_cm 56.83, measured (22 3/8 in)

# Pass/fail limits (starting values; tune once we see real data)
SENSOR_STD_BASE_MM = 3.0          # allowed noise = base + fraction of the distance (ultrasonic noise grows with range)
SENSOR_STD_FRACTION = 0.02
MAX_TIMEOUT_FRACTION = 0.02
MAX_GYRO_Z_STD_COUNTS = 40        # about 0.35 deg/s
MAX_YAW_DRIFT_DEG = 2.0           # over the whole run, after GYROZERO
MAX_HEADING_STD_DEG = 1.5


def stats(values):
    return {
        "n": len(values),
        "mean": statistics.fmean(values) if values else None,
        "std": statistics.pstdev(values) if len(values) > 1 else None,
        "min": min(values) if values else None,
        "max": max(values) if values else None,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=os.environ.get("ROBOT_URL"))
    ap.add_argument("--seconds", type=float, default=20.0)
    a = ap.parse_args()
    if not a.url:
        print("Set --url or ROBOT_URL", file=sys.stderr)
        return 2

    robot = Robot(a.url, "t01_rest")
    try:
        reply = robot.cmd("GYROZERO", timeout=4.0)
        if not reply.ok:
            raise RobotError(f"GYROZERO failed: {reply.text}")
        bias = reply.text
        frames = robot.telemetry(a.seconds)
    except RobotError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    finally:
        robot.stop()
        robot.close()

    if len(frames) < 10:
        print(f"only {len(frames)} frames received; aborting", file=sys.stderr)
        return 1

    summary = {"frames": len(frames), "gyrozero": bias, "dropped_lines": robot.dropped}
    checks = []  # (name, passed, detail)

    # ---- sensors ----
    sens = {}
    for s in SENSORS:
        raw = [f[s] for f in frames]
        valid = [v for v in raw if 0 <= v < 8190]
        timeouts = sum(1 for v in raw if v < 0) / len(raw)
        far = sum(1 for v in raw if v >= 8190) / len(raw)
        st = stats(valid)
        st.update({"timeout_fraction": timeouts, "out_of_range_fraction": far})
        sens[s] = st
        ok = (st["std"] is not None
              and st["std"] <= SENSOR_STD_BASE_MM + SENSOR_STD_FRACTION * st["mean"]
              and timeouts <= MAX_TIMEOUT_FRACTION and far == 0)
        checks.append((f"sensor {s}", ok,
                       f"mean {st['mean']:.1f} mm, std {st['std']:.1f}, range {st['min']}-{st['max']}, "
                       f"timeouts {timeouts:.1%}, out-of-range {far:.1%}" if st["std"] is not None else "no valid readings"))
    summary["sensors"] = sens

    # ---- board consistency / implied bot size ----
    try:
        front = sens["F"]["mean"]
        x_sum = front + sens["B"]["mean"]
        y_sum = sens["R"]["mean"] + sens["L"]["mean"]
        summary["geometry"] = {
            "front_mean_mm": front, "x_sum_mm": x_sum, "y_sum_mm": y_sum,
            "implied_bot_length_mm": BOARD_X_MM - x_sum, "implied_bot_width_mm": BOARD_Y_MM - y_sum,
        }
    except TypeError:
        summary["geometry"] = None

    # ---- gyro ----
    gz = [f["gz"] for f in frames]
    yaw_drift = frames[-1]["yaw_deg"] - frames[0]["yaw_deg"]
    gz_st = stats(gz)
    summary["gyro_z"] = gz_st
    summary["yaw_drift_deg"] = yaw_drift
    checks.append(("gyro z noise", gz_st["std"] <= MAX_GYRO_Z_STD_COUNTS,
                   f"mean {gz_st['mean']:.0f} counts, std {gz_st['std']:.1f}"))
    checks.append(("yaw drift after zero", abs(yaw_drift) <= MAX_YAW_DRIFT_DEG,
                   f"{yaw_drift:+.2f} deg over {a.seconds:.0f} s"))

    # ---- magnetometer ----
    headings = [math.degrees(math.atan2(f["my"], f["mx"])) % 360 for f in frames]
    # unwrap around the circular mean so the std is meaningful near 0/360
    ref = math.degrees(math.atan2(statistics.fmean(math.sin(math.radians(h)) for h in headings),
                                  statistics.fmean(math.cos(math.radians(h)) for h in headings))) % 360
    dev = [((h - ref + 180) % 360) - 180 for h in headings]
    mag_mag = [math.sqrt(f["mx"] ** 2 + f["my"] ** 2 + f["mz"] ** 2) for f in frames]
    heading_std = statistics.pstdev(dev)
    summary["mag"] = {"raw_heading_mean_deg": ref, "raw_heading_std_deg": heading_std,
                      "field_magnitude_counts": stats(mag_mag)}
    checks.append(("compass heading noise (raw, uncalibrated)", heading_std <= MAX_HEADING_STD_DEG,
                   f"heading {ref:.1f} deg, std {heading_std:.2f}; field {statistics.fmean(mag_mag):.0f} counts"))

    # ---- report ----
    lines = ["# t01_rest report", "", f"{len(frames)} frames over {a.seconds:.0f} s; gyro zero: {bias}; "
             f"dropped log lines: {robot.dropped}", "", "| check | result | detail |", "|---|---|---|"]
    for name, ok, detail in checks:
        lines.append(f"| {name} | {'PASS' if ok else 'FAIL'} | {detail} |")
    g = summary.get("geometry")
    if g:
        lines += ["", "## Geometry (compare with a ruler)", "",
                  f"- front+back sum: {g['x_sum_mm']:.0f} mm, so the implied bot length incl. sensor offsets is "
                  f"{g['implied_bot_length_mm']:.0f} mm (board x = {BOARD_X_MM:.0f} mm)",
                  f"- right+left sum: {g['y_sum_mm']:.0f} mm, so the implied bot width incl. sensor offsets is "
                  f"{g['implied_bot_width_mm']:.0f} mm (board y = {BOARD_Y_MM:.0f} mm)"]
    report = "\n".join(lines) + "\n"
    with open(os.path.join(robot.run_dir, "report.md"), "w") as fh:
        fh.write(report)
    with open(os.path.join(robot.run_dir, "summary.json"), "w") as fh:
        json.dump(summary, fh, indent=2)
    print(report)
    print(f"(run folder: {robot.run_dir})")
    return 0 if all(ok for _, ok, _ in checks) else 3


if __name__ == "__main__":
    sys.exit(main())
