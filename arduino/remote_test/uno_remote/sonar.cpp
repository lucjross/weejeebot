#include "sonar.h"
#include <NewPing.h>

namespace {

NewPing sensor[SONAR_COUNT] = {
  NewPing(SONAR_F_TRIG_PIN, SONAR_X_ECHO_PIN, SONAR_MAX_CM),
  NewPing(SONAR_R_TRIG_PIN, SONAR_Y_ECHO_PIN, SONAR_MAX_CM),
  NewPing(SONAR_B_TRIG_PIN, SONAR_X_ECHO_PIN, SONAR_MAX_CM),
  NewPing(SONAR_L_TRIG_PIN, SONAR_Y_ECHO_PIN, SONAR_MAX_CM)
};
int16_t value[SONAR_COUNT] = {-1, -1, -1, -1};
uint32_t stamp[SONAR_COUNT];
const float gain[SONAR_COUNT] = SONAR_GAIN;
const float offsetMm[SONAR_COUNT] = SONAR_OFFSET_MM;
uint8_t nextIdx = 0;
uint32_t lastPingMs = 0;

}  // namespace

namespace Sonar {

const char *const NAMES[SONAR_COUNT] = {"F", "R", "B", "L"};

void step() {
  if ((uint32_t) (millis() - lastPingMs) < SONAR_PING_GAP_MS) return;
  lastPingMs = millis();
  uint8_t i = nextIdx;
  nextIdx = (nextIdx + 1) % SONAR_COUNT;
  uint32_t us = sensor[i].ping();  // round trip, 0 = no echo
  if (us == 0) {
    value[i] = -1;
    return;
  }
  float mm = (us * 10.0f / 58.0f - offsetMm[i]) / gain[i];  // 58 us per cm round trip, then corrected
  value[i] = mm < 0 ? 0 : (int16_t) (mm + 0.5f);
  stamp[i] = millis();
}

int16_t mm(uint8_t i) {
  return value[i];
}

bool fresh(uint8_t i, uint16_t maxAgeMs) {
  return value[i] >= 0 && (uint32_t) (millis() - stamp[i]) <= maxAgeMs;
}

}  // namespace Sonar
