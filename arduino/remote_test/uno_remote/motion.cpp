#include "motion.h"
#include "config.h"
#include "sonar.h"
#include <ZumoShield.h>

namespace {

enum Mode : uint8_t { M_NONE, M_FORWARD, M_BACKWARD, M_SPIN };

ZumoMotors motors;
bool running = false;
Mode mode = M_NONE;
int16_t cmdLeft = 0, cmdRight = 0;
uint32_t endAt = 0;
const char *reason = "";

Mode modeOf(int left, int right) {
  if (left > 0 && right > 0) return M_FORWARD;
  if (left < 0 && right < 0) return M_BACKWARD;
  if (left == 0 && right == 0) return M_NONE;
  return M_SPIN;  // opposite signs, or one track only (pivot)
}

const char *const GUARD_TEXT[SONAR_COUNT] = {"guard F", "guard R", "guard B", "guard L"};

// Returns nullptr if the move is safe right now, otherwise the reason it is not.
const char *unsafeReason(Mode m) {
  switch (m) {
    case M_FORWARD:
      if (!Sonar::fresh(SONAR_F, GUARD_STALE_MS)) return "stale";
      if (Sonar::mm(SONAR_F) < GUARD_TRANSLATE_MM) return GUARD_TEXT[SONAR_F];
      return nullptr;
    case M_BACKWARD:
      if (!Sonar::fresh(SONAR_B, GUARD_STALE_MS)) return "stale";
      if (Sonar::mm(SONAR_B) < GUARD_TRANSLATE_MM) return GUARD_TEXT[SONAR_B];
      return nullptr;
    case M_SPIN:
      for (uint8_t i = 0; i < SONAR_COUNT; i++) {
        if (!Sonar::fresh(i, GUARD_STALE_MS)) return "stale";
        if (Sonar::mm(i) < GUARD_SPIN_CLEAR_MM) return GUARD_TEXT[i];
      }
      return nullptr;
    default:
      return nullptr;
  }
}

}  // namespace

namespace Motion {

void begin() {
  motors.setSpeeds(0, 0);
}

const char *start(int left, int right, int durationMs) {
  if (abs(left) > MAX_SPEED || abs(right) > MAX_SPEED) return "range";
  if (durationMs < 1 || durationMs > MAX_DRIVE_MS) return "range";
  Mode m = modeOf(left, right);
  if (m == M_NONE) return "range";
  if (running) return "busy";
  const char *why = unsafeReason(m);
  if (why) return why;

  mode = m;
  cmdLeft = left;
  cmdRight = right;
  endAt = millis() + durationMs;
  running = true;
  motors.setSpeeds(left, right);
  return nullptr;
}

void stop() {
  motors.setSpeeds(0, 0);
  running = false;
  mode = M_NONE;
  cmdLeft = 0;
  cmdRight = 0;
}

bool active() {
  return running;
}

Event update() {
  if (!running) return NONE;
  const char *why = unsafeReason(mode);
  if (why) {
    reason = why;
    stop();
    return GUARD;
  }
  if ((int32_t) (millis() - endAt) >= 0) {
    stop();
    return DONE;
  }
  return NONE;
}

const char *lastReason() {
  return reason;
}

int16_t left() {
  return cmdLeft;
}

int16_t right() {
  return cmdRight;
}

}  // namespace Motion
