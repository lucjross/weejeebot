#include "globals.h"

Globals global;
Timer<> timer = timer_create_default();
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

static bool tracks_on = false;
static uint32_t tracks_off_since_ms = 0;

void setTracks(int16_t left, int16_t right) {
  motors.setSpeeds(left, right);
  if (left != 0 || right != 0) {
    tracks_on = true;
  } else if (tracks_on) {
    tracks_on = false;
    tracks_off_since_ms = millis();
  }
}

bool tracksIdleFor(uint16_t ms) {
  return !tracks_on && (uint32_t) (millis() - tracks_off_since_ms) >= ms;
}
