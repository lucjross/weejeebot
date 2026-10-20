# Wiring (ultrasonic design)

Wiring for the Uno + Zumo bot with four HC-SR04 ultrasonic sensors, and the Nano 33 IoT + display on the planchette.
**Status:**
- Confirmed on the bench: the Nano 33 IoT is powered from the Uno's `Vin`, the display works on the pins below, and the Uno-Nano serial link works in both directions through the level converter.
- The ultrasonic sensors have not been tested in this arrangement yet.

Items marked **VERIFY** are assumptions or unknowns to check with a multimeter or the board itself.

## Power rails

| Rail | Source | Used by |
|------|--------|---------|
| `Vin` (~7.45 V) | Zumo shield boost regulator, on the Uno's Vin pin. Dead on USB-only power; the battery pack must be on. | Nano 33 IoT VIN |
| `5V` | Uno regulator | HC-SR04 sensors (about 15 mA each), level converter HV side |
| Uno's 3.3 V pin | Uno regulator | Level converter LV pin (a few mA at most) |
| 3.3 V from the Nano | Nano 33 IoT's 3V3 pin | GC9A01 display |
| `GND` | Common to every board | Everything |

The Uno's own 3.3 V pin also powers the shield's compass/IMU, so keep other loads off it. The converter's LV pin is only a voltage
reference and draws almost nothing, which is why it can share that pin.

**VERIFY:** the Zumo boost regulator's current rating is unknown. The Nano with WiFi is the main load added on `Vin`.

## Uno R3 + Zumo shield v1.3

| Pin | Function | Notes |
|-----|----------|-------|
| A0 | Serial RX (NeoSWSerial, 9600 baud) | From the Nano's TX, via level converter |
| A3 | Serial TX (NeoSWSerial, 9600 baud) | To the Nano's RX, via level converter |
| 4 | HC-SR04 front TRIG | |
| 11 | HC-SR04 right TRIG | |
| 5 | HC-SR04 back TRIG | |
| 2 | HC-SR04 left TRIG | |
| A2 | HC-SR04 x-axis ECHO (front + back, shared line, see below) | |
| A1 | HC-SR04 y-axis ECHO (left + right, shared line, see below) | |

Pin numbers come from `sketches/zumo_avr/zumo_avr.ino`.

## HC-SR04 ultrasonic sensors (x4)

| Sensor pin | Connects to |
|------------|-------------|
| VCC | Uno `5V` |
| GND | `GND` |
| TRIG | The sensor's own pin from the table above |
| ECHO | Its own diode (see below), then its axis's shared line: A2 for front/back, A1 for left/right |

They are 5 V devices like the Uno, so no level shifting is needed. Sensors face front, right, back and left of the bot,
which in the board frame (see "Orientation") means the front sensor measures along x, and left/right measure along y.

### Shared echo lines (diode-OR, one per axis)

Front and back share Uno pin A2 (x axis); left and right share A1 (y axis). Only one sensor fires at a time, but the idle sensor on a line would otherwise hold it low
while the active one drives it high, so each echo goes through a diode:

| Part | Wiring |
|------|--------|
| Diode on each ECHO | Anode (unmarked end) to the sensor's ECHO pin, cathode (striped end) to the shared line |
| Pull-down resistor (required, one per line) | From the shared line to `GND`, about 10 kOhm, on the Uno side of the diodes. Without it the line floats between echoes and the Uno reads it as permanently high, so every sensor on that line reports no echo |
| Shared lines | Front/back to Uno A2; left/right to Uno A1 |

The diode drops about 0.3-0.7 V, so a high echo reaches the Uno at roughly 4.3-4.7 V, still well above its ~3 V threshold.
**VERIFY:** the diode orientation and the resistor value as built.

## Level converter (4-channel, BSS138-style): serial link only

The Nano is 3.3 V logic and is not 5 V tolerant, so the Uno's serial lines go through the converter. Never connect them directly.

| Converter pin | Connects to |
|---------------|-------------|
| HV | Uno `5V` |
| LV | Uno 3.3 V pin |
| GND (both sides) | `GND` |
| HV3 / LV3 | Uno A3 (serial TX) / Nano RX (D0) |
| HV4 / LV4 | Uno A0 (serial RX) / Nano TX (D1) |
| HV1, HV2 / LV1, LV2 | Not used |

There must be no inline resistors on the Uno TX to Nano RX wire. They would divide the converter's 3.3 V output down below
what the Nano reads as high.

The LV pin goes to the Uno's 3.3 V pin.

## Arduino Nano 33 IoT (WiFi + display)

| Nano pin | Connects to | Notes |
|----------|-------------|-------|
| VIN | `Vin` | The Nano's VIN accepts 4.5-21 V, so 7.45 V is within spec |
| GND | `GND` | |
| D0 (RX) | Converter LV3 | Serial1 from the Uno |
| D1 (TX) | Converter LV4 | Serial1 to the Uno |
| D13 (SCK) | Display SCL | |
| D11 (MOSI) | Display SDA | |
| D9 | Display CS | |
| D8 | Display DC | |
| D7 | Display RES | |
| 3V3 | Display VCC | |

Pins come from `sketches/display_and_web/display_and_web.ino`. USB is for programming only.

## GC9A01 display (240x240 round, ZHONGJINGYUAN `ZJY_IPS128_8P`)

| Display pin | Connects to |
|-------------|-------------|
| VCC | Nano's 3V3 pin |
| GND | `GND` |
| SCL | Nano D13 |
| SDA | Nano D11 |
| CS | Nano D9 |
| DC | Nano D8 |
| RES | Nano D7 |
| BLK (backlight) | Not driven by the sketch; the backlight glows as wired |

I could not find a datasheet for the exact module, so the pin order along its 8-pin header is not documented here. Read the
board's silkscreen if you need to rewire it. The display's center is about 5 cm from the bot's center in +y (see "Orientation").

## Orientation

Heading 0 is the board's x-axis: the Zumo's front faces the board's right wall, and its back faces the left wall. The Zumo's left
and right sides face the board's top and bottom walls (+y is toward the top wall). The planchette is mounted at 270 degrees to the Zumo,
so its nose points toward the Zumo's left side (the board's top wall), and the planchette's right side is the Zumo's front.

The bot drives along x and reverses instead of turning around, so the front sensor always faces the right wall.

The display's center is about 5 cm from the bot's center in +y, so the bot targets each symbol's y minus 5 cm.

## Physical layout

**Planchette (detachable top):** the Nano 33 IoT and the GC9A01 display only, with the short display wiring staying on the planchette.

**Uno + Zumo side:** everything else: the four HC-SR04 sensors, the two diode-ORs and pull-downs, the level converter, and the Uno.

**Wires crossing between the Uno side and the planchette (4 total):**

| Wire | Uno side | Nano |
|------|----------|------|
| Power | Vin (~7.45 V) | VIN |
| Ground | GND | GND |
| Serial | Converter LV3 | D0 (RX) |
| Serial | Converter LV4 | D1 (TX) |

Put these four on one keyed connector (for example a 4-pin JST-style) near a mount rail, so the planchette detaches cleanly and
the 7.45 V wire can't be plugged into the wrong pin.

**Mounting notes:**
- The aluminum brackets won't take solder or hot glue. Use screws and nuts, slow-cure epoxy on a sanded and cleaned surface, or foam tape. Standard foam tape is not conductive.
- Use non-magnetic hardware (brass, nylon, aluminum) near the compass. Steel can disturb the magnetometer.
- Aluminum conducts. Put electrical tape or heat-shrink between any bare pads or solder joints and the brackets.
- Recheck the compass heading with the planchette on and off, since the Nano's WiFi and the display sit above it.

## Open items

- Confirm the diode orientation and pull-down value on both echo lines.
- Confirm the serial link works with the converter's LV pin on the Uno's 3.3 V pin.
- Zumo boost regulator current rating (unknown).
- Compass calibration on the floor with the battery pack on, then check the heading against a known direction.
- Sensor readings against a ruler at a few distances, and the nose-against-wall check.
- Fix `display_and_web.ino` so it builds (it needs a git-ignored `secrets.h` with your WiFi details, and line 194 has a stray `Serial1.`).
