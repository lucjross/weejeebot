#pragma once
#include <Arduino.h>

// ---- link to the Nano (NeoSWSerial) ----
#define LINK_RX_PIN A0
#define LINK_TX_PIN A3
#define LINK_BAUD 19200
#define LINE_MAX 48

// ---- HC-SR04 ultrasonic sensors ----
// sensor order everywhere: front, right, back, left
#define SONAR_COUNT 4
#define SONAR_F 0
#define SONAR_R 1
#define SONAR_B 2
#define SONAR_L 3
#define SONAR_F_TRIG_PIN 4
#define SONAR_R_TRIG_PIN 11
#define SONAR_B_TRIG_PIN 5
#define SONAR_L_TRIG_PIN 2
// front/back share the x-axis echo line, left/right share the y-axis echo line
#define SONAR_X_ECHO_PIN A2
#define SONAR_Y_ECHO_PIN A1
#define SONAR_MAX_CM 200
// Per-sensor correction fitted against tape measurements (host/tests/t02_accuracy.py, host/data/accuracy.csv):
// reading = gain * true + offset, so the firmware reports (reading - offset) / gain. Order: F R B L.
// accuracy.csv holds raw readings (taken before this correction existed); two points per sensor.
#define SONAR_GAIN {0.996f, 0.976f, 0.976f, 0.965f}
#define SONAR_OFFSET_MM {5.2f, 4.4f, 10.8f, 8.7f}
// pause between pings so one sensor's echo has died out before the next fires
#define SONAR_PING_GAP_MS 40

// ---- IMU ----
#define GYRO_DPS_PER_LSB 0.00875f   // +/-245 dps range

// ---- motion safety (see README) ----
#define MAX_SPEED 200
#define MAX_DRIVE_MS 2000
#define GUARD_TRANSLATE_MM 60   // stop if a sensor in the direction of travel reads below this
#define GUARD_SPIN_CLEAR_MM 50  // refuse to spin/pivot if any sensor reads below this
#define GUARD_STALE_MS 400      // refuse to move if a needed reading is older than this

// ---- telemetry ----
#define TEL_MIN_MS 200
#define TEL_MAX_MS 5000
