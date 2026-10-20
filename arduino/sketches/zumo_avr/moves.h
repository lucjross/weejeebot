#pragma once
#include <Arduino.h>

// Drives the bot through the queued points using only moves along the board's axes. For each point it squares up to
// heading 0, reads its position, and if the y is off, turns to heading 90 and drives along y until the front/back sonar
// says it is there; then it turns back to 0 and does the same along x. Never diagonal, and the sonars are only
// believed while it is square to the walls. Run it on a timer (MOVE_INTERVAL).
bool runMotors(void *);

void turnToHeading(uint16_t deg);

// Bench helper: drives straight for 10 s so MOVE_cm_PER_s can be measured. Not called by default.
void velocityTest();
