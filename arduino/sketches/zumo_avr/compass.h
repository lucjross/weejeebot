#pragma once
#include <Arduino.h>
#include "types.h"

// Smallest turn from source to target, in degrees, and which way (1 or -1).
uint16_t angleDiff(int16_t source_deg, int16_t target_deg);
struct Turn shortestTurn(int16_t source_deg, int16_t target_deg);

// Compares global.theta (the bot's heading) with deg, within the normal or the lax threshold.
bool headingIsNot(uint16_t deg, bool lax);
bool headingIs(uint16_t deg, bool lax);

// Same as headingIs, but within the threshold on either side of deg (headingIs only accepts deg up to deg + threshold).
bool headingNear(uint16_t deg, bool lax);

uint16_t averageHeading();

// The tracked heading in degrees, and how far it must turn to reach target_deg: positive if it has to increase.
float headingDeg();
float angleError(float target_deg);
float wrap180(float a);

// Keeps global.theta up to date: the gyro tracks it while the tracks run, and the compass sets it once the tracks have
// been off for MAG_SETTLE_MS (the motors disturb the compass). Run it on a timer (FIND_ORIENTATION_INTERVAL).
bool updateHeading(void *);

// Start-up steps, in the order setup() runs them:
void compassBegin();      // initialise the IMU
void calibrateCompass();  // spin in place in pulses, recording the magnetometer range with the motors off
void orientToBoard();     // wait while the planchette is lined up with the board, then store its heading as the x axis
