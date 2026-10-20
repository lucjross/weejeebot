#!/usr/bin/env python3
"""Walks through each HC-SR04 (front, right, back, left): press Enter, then
reads the sensor every 500 ms for 5 s. Needs sonar_test.ino on the Uno."""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
SIDES = [("F", "front"), ("R", "right"), ("B", "back"), ("L", "left")]
INTERVAL_S = 0.5
DURATION_S = 5

with serial.Serial(PORT, 9600, timeout=1) as ser:
    time.sleep(2)  # the Uno resets when the port opens
    ser.reset_input_buffer()
    for cmd, name in SIDES:
        input(f"\n{name.upper()} sensor: put something in front of it, then press Enter...")
        start = time.monotonic()
        for i in range(int(DURATION_S / INTERVAL_S)):
            ser.write(cmd.encode())
            reply = ser.readline().decode().strip()
            cm = int(reply) if reply.isdigit() else None
            print(f"  {name:5} {(i + 1) * INTERVAL_S:3.1f}s  " + (f"{cm} cm" if cm else "no echo"))
            time.sleep(max(0, start + (i + 1) * INTERVAL_S - time.monotonic()))
