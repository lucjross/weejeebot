#pragma once
#include <Arduino.h>

// Timed motor commands with safety guards. No command can leave the motors running past its duration.
namespace Motion {

enum Event : uint8_t { NONE, DONE, GUARD };

void begin();

// Starts a move. rampMs > 0 raises the speed linearly from 0 to the target over that time (0 = jump straight to it).
// Returns nullptr on success, or a short error reason ("range", "busy", "stale", "guard F", ...).
const char *start(int left, int right, int durationMs, int rampMs);

void stop();
bool active();

// Call every loop. Returns DONE when the duration ended, GUARD if a guard stopped the move (see lastReason()).
Event update();
const char *lastReason();

int16_t left();
int16_t right();

}  // namespace Motion
