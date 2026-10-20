# Remote test harness

Lets a host computer drive the Zumo and read its sensors over WiFi, for systematic testing on the real board.
Sensor and movement testing only; it is separate from the original `arduino/sketches/` code, which is kept for reference.

```
host (Python)  --HTTP-->  Nano 33 IoT (nano_relay)  --serial 19200-->  Uno + Zumo (uno_remote)
```

- `uno_remote/` - Uno firmware: command parser, ultrasonic sensors, IMU, motors, safety guards, telemetry.
- `nano_relay/` - Nano firmware: joins WiFi, serves HTTP, relays command lines to the Uno and buffers the Uno's lines.
- `host/` - Python client (`rc.py`) and the test scripts. Logs go to `host/runs/` (git-ignored).

Hardware and pin assignments are in `arduino/WIRING.md`.

## Safety rules (enforced on the Uno, not just the host)

- Every motion command carries a duration, capped at `MAX_DRIVE_MS` (2000 ms), and the motors stop on their own when it ends.
  A lost WiFi link therefore always ends in a stop. There is no command that leaves the motors running.
- Speeds are capped at `MAX_SPEED` (200).
- Wall guard: forward motion needs a fresh reading from the front sensor and refuses or aborts if it reads below
  `GUARD_TRANSLATE_MM`; backward uses the back sensor. Spinning or pivoting is refused if any sensor reads below `GUARD_SPIN_CLEAR_MM`.
- `STOP` always works. The battery switch is the physical kill.
- The host client always sends `STOP` on exit or error. A person should be watching during runs.
- No authentication: anyone on the LAN can drive the bot. Use it on a trusted network only.

## Uno <-> Nano serial protocol

ASCII lines ending in `\n`, 19200 baud, 8N1 (Uno side: NeoSWSerial on A0 RX / A3 TX; Nano side: Serial1, D0 RX / D1 TX).

Commands (Nano -> Uno), each optionally prefixed with `#<id> ` so replies can be matched:

| Command | Effect | Reply |
|---------|--------|-------|
| `PING` | liveness check | `#id OK pong` |
| `STOP` | motors off immediately | `#id OK` |
| `STATE` | send one telemetry line now | `#id OK`, then a `T` line |
| `TEL <ms>` | telemetry period, 0 = off, otherwise 200-5000 | `#id OK` |
| `DRIVE <left> <right> <ms> [ramp_ms]` | run both tracks, speeds -200..200, duration 1..2000 ms; with `ramp_ms` (0..ms) the speed rises linearly from 0 over that time, otherwise it jumps straight to the target | `#id OK`, later `E done` or `E guard <name>` |
| `GYROZERO` | measure gyro bias (~1 s, bot must be still), zero yaw | `#id OK bias=<x>` |

Errors: `#id ERR <reason>` (`unknown`, `range`, `guard`, `busy`, `stale`, ...).

Lines from the Uno (relayed to the host):

- `T <ms> <F> <R> <B> <L> <mx> <my> <mz> <ax> <ay> <az> <gx> <gy> <gz> <yaw> <ml> <mr>`
  - `ms`: Uno `millis()`. Distances in mm; `-1` means no echo (nothing in range, too close, or a wiring fault).
  - `mx..mz`: raw magnetometer, `ax..az`: raw accelerometer, `gx..gz`: raw gyro (8.75 mdps per count, +/-245 dps range).
  - `yaw`: gyro-integrated rotation about z in tenths of a degree since the last `GYROZERO` (bias-corrected).
  - `ml mr`: current motor commands.
- `E <event> [args]`: `boot`, `done`, `guard <sensor>`.

Sensor order everywhere: front, right, back, left (indices 0-3). They are pinged one at a time, 40 ms apart, so each sensor updates about every 160 ms.

## Nano HTTP API

| Request | Result |
|---------|--------|
| `POST /cmd` with form field `cmd` (or `GET /cmd?cmd=...`) | writes the line to the Uno, replies `queued` |
| `GET /log?since=<seq>` | first line `next <seq> dropped <n>`, then `<seq> <line>` for each buffered Uno line newer than `since` |
| `GET /status` | uptime, WiFi RSSI, buffer state |

The Nano keeps the last ~40 lines in RAM. Poll `/log` at least every second or two.

## Test catalogue (planned; scripts live in `host/tests/`)

Each test records raw telemetry to `host/runs/<timestamp>/` and prints pass/fail against stated limits.

1. Sensors at rest: noise per sensor, and opposite-sensor sums against the known board size.
2. IMU at rest: gyro bias, compass noise.
3. Compass calibration: spin on the board, fit the raw magnetometer circle, report offsets and residual.
4. Compass interference: heading with motors on vs off, and with WiFi traffic.
5. Gyro vs compass: a measured turn compared both ways.
6. Motors: speed (cm/s) from the wall-distance change, left/right drift while driving straight, turn rate vs command.
7. Position: sensor-derived x, y against the move the bot was told to make, and repeatability on return.
8. Link: latency and dropped lines.

## Setup

1. `nano_relay/`: copy `secrets.h.example` to `secrets.h` and fill in the WiFi name and password (git-ignored). Build and upload:
   `arduino-cli compile --fqbn arduino:samd:nano_33_iot arduino/remote_test/nano_relay` then
   `arduino-cli upload -p /dev/ttyACM1 --fqbn arduino:samd:nano_33_iot arduino/remote_test/nano_relay`.
   The Nano prints its IP address on its USB serial port at boot.
2. `uno_remote/`: `arduino-cli compile --fqbn arduino:avr:uno arduino/remote_test/uno_remote` then upload to `/dev/ttyACM0`.
   Needs the `NewPing`, `NeoSWSerial` and `ZumoShield` libraries (already installed).
3. Host: `python3 arduino/remote_test/host/rc.py --url http://<nano-ip> ping` (or set `ROBOT_URL`).
