#pragma once

#define RAD2DEG 180.0 / M_PI

// program
#define SHOW_CHARACTER_WAIT_ms 1500

// planchette
#define PLANCH_CENTER_TO_SCREEN_cm 5

// board constants
#define BOARD_W_cm 85.1
#define BOARD_H_cm 56.83
#define BOARD_LEFT_WALL_X -(BOARD_W_cm / 2)
#define BOARD_RIGHT_WALL_X BOARD_W_cm / 2
#define BOARD_TOP_WALL_Y BOARD_H_cm / 2
#define BOARD_BOTTOM_WALL_Y -(BOARD_H_cm / 2)

// how close to the walls the bot's centre may go
#define MOVE_X_LIMIT ((BOARD_W_cm / 2.0) - 16)
#define MOVE_Y_LIMIT ((BOARD_H_cm / 2.0) - 9)

// "no position yet"; point stores int8_t, and board coordinates stay within +/-43 cm
#define COORD_UNDEF -128

// ultrasonic sensors
#define USS_F_TRIG_PIN 4
#define USS_B_TRIG_PIN 5
#define USS_L_TRIG_PIN 2
#define USS_R_TRIG_PIN 11
// front/back share the x-axis echo line, left/right share the y-axis echo line
#define USS_X_ECHO_PIN A2
#define USS_Y_ECHO_PIN A1
#define USS_F 0
#define USS_R 1
#define USS_B 2
#define USS_L 3
// distance from sensor to center of bot
#define USS_DIST_ADJ_cm 7
// each sensor reads this far long (cm), found against the bot's measured body size; order F R B L
#define USS_OFFSETS_cm {0.19, 0.30, 0.19, 0.30}
#define BOARD_W_DEV_THRESHOLD 5.0
#define BOARD_H_DEV_THRESHOLD 3.0

#define USS_PING_INTERVAL 50
#define FIND_POSITION_INTERVAL 300

// softwareserial link to the Nano
#define SS_RX A0
#define SS_TX A3
#define SS_BAUDRATE 9600

// motors
#define MOTOR_CALIB_SPEED 200
// Compass calibration spins in short pulses and reads the magnetometer in the pauses, with the motors off, because the
// motors disturb it. It stops once the gyro reports this much turning (a full circle plus margin), or after this long.
#define MOTOR_CALIB_PULSE_ON_MS 300
#define MOTOR_CALIB_SETTLE_MS 200
#define MOTOR_CALIB_MIN_TURN_DEG 420
#define MOTOR_CALIB_MAX_MS 30000
// pauses kept for working out which way the gyro counts
#define MOTOR_CALIB_MAX_SAMPLES 40
#define GYRO_BIAS_SAMPLES 50
#define GYRO_DEG_PER_LSB 0.00875  // +/-245 dps range
#define MOVE_INTERVAL 50
#define MOTOR_SPEED 200
#define TRACK_L_SPEED_ADJ 20
#define TURN_BASE_SPEED 150
//#define MOVE_cm_PER_s 8 // this is empirically determined with velocityTest()
#define MOVE_cm_PER_s 8.2
#define Y_POSITION_DEVIATION_THRESHOLD 3
#define X_POSITION_DEVIATION_THRESHOLD 2

// compass
#define FIND_ORIENTATION_INTERVAL 20
// The gyro tracks the heading while the tracks run. The compass is only trusted once the tracks have been off this
// long, and is then read at most this often.
#define MAG_SETTLE_MS 300
#define MAG_REFRESH_MS 100
// Settling to heading 0 at rest: far off, turn continuously; within this many degrees, turn in short gyro-limited steps
// (this fraction of the remaining error each time) with the motors off between them while the compass re-measures.
#define REST_FINE_ZONE_DEG 25
#define REST_STEP_FRACTION 0.6
#define REST_TURN_SPEED 150
// the calibration spin must have turned at least this far for the gyro to count as working
#define GYRO_WORKING_MIN_DEG 200
#define HEADING_DEVIATION_THRESHOLD 2.0
#define LAX_HEADING_DEVIATION_THRESHOLD 5.0
