#pragma once
#include <Arduino.h>

// Drives the bot through the queued points: turns to heading 0, positions itself with the sonars, then turns,
// goes straight and turns back for each point. Run it on a timer (MOVE_INTERVAL).
bool runMotors(void *);

// Ends the current step of the move queue and starts the next. Timer callback.
bool nextMove(void *);

void turnToHeading(uint16_t deg);

// Bench helper: drives straight for 10 s so MOVE_cm_PER_s can be measured. Not called by default.
void velocityTest();
