#include <NewPing.h>  // only for its 24 us timer interrupt; the timing of the two echoes is done here
#include "sonar.h"
#include "globals.h"

// The sensors fire in two slots, each a pair with one sensor on each echo line: F (x line) with R (y line), then B with L.
static const uint8_t SLOT_TRIG_PIN[2][2] = {{USS_F_TRIG_PIN, USS_R_TRIG_PIN}, {USS_B_TRIG_PIN, USS_L_TRIG_PIN}};
static const uint8_t SLOT_SENSOR[2][2] = {{USS_F, USS_R}, {USS_B, USS_L}};  // [slot][0: x line, 1: y line]

// subtracted from every reading; indexed F R B L
const float uss_offset_cm[4] = USS_OFFSETS_cm;

// The echo pin of each line as a bit of PORTC (A2 is PC2, A1 is PC1); read directly, the timer interrupt is short.
static const uint8_t ECHO_BIT[2] = {_BV(2), _BV(1)};

// the HC-SR04 raises its echo pin within a few hundred us of the trigger, and a result past the board is no result
#define US_PER_CM_ROUNDTRIP 57.0
#define MAX_RISE_WAIT_US 6000UL
#define MAX_ECHO_US ((uint32_t) (BOARD_W_cm * US_PER_CM_ROUNDTRIP))

static volatile uint8_t slot = 1;  // the slot fired last; flips before each ping
static volatile uint32_t fired_us;
static volatile uint32_t rise_us[2];
static volatile bool seen_rise[2], line_done[2];

void sonarBegin() {
  for (uint8_t s = 0; s < 2; s++) {
    for (uint8_t line = 0; line < 2; line++) {
      pinMode(SLOT_TRIG_PIN[s][line], OUTPUT);
      digitalWrite(SLOT_TRIG_PIN[s][line], LOW);
    }
  }
  pinMode(USS_X_ECHO_PIN, INPUT);
  pinMode(USS_Y_ECHO_PIN, INPUT);
}

static void storeReading(uint8_t uss, uint32_t width_us) {
  float cm = width_us / US_PER_CM_ROUNDTRIP - uss_offset_cm[uss];
  // not rounded here: uss_mm keeps the tenth of a cm the sensor has; the rounded ones serve the older code
  global.uss_mm[uss] = cm < 0 ? 0 : (int16_t) (cm * 10.0 + 0.5);
  global.uss_distances[uss] = cm < 0 ? 0 : (int8_t) (cm + 0.5);
  global.uss_adj_distances[uss] = global.uss_distances[uss] + USS_DIST_ADJ_cm;
}

// Timer interrupt every 24 us while a pair is out: times each echo line's high pulse.
static void echoPoll() {
  uint32_t now = micros();
  uint8_t pins = PINC;
  bool all_done = true;
  for (uint8_t line = 0; line < 2; line++) {
    if (line_done[line]) continue;
    bool high = pins & ECHO_BIT[line];
    if (!seen_rise[line]) {
      if (high) {
        seen_rise[line] = true;
        rise_us[line] = now;
      } else if (now - fired_us > MAX_RISE_WAIT_US) {
        line_done[line] = true;  // never answered
      }
    } else if (!high) {
      storeReading(SLOT_SENSOR[slot][line], now - rise_us[line]);
      line_done[line] = true;
    } else if (now - rise_us[line] > MAX_ECHO_US) {
      line_done[line] = true;  // farther than the board is long
    }
    if (!line_done[line]) all_done = false;
  }
  if (all_done) {
    NewPing::timer_stop();
  }
}

bool ussPingTimers(void *) {
  NewPing::timer_stop();
  if (global.turning) {
    return true;
  }

  slot ^= 1;
  for (uint8_t line = 0; line < 2; line++) {
    global.uss_distances[SLOT_SENSOR[slot][line]] = -1;
    seen_rise[line] = false;
    line_done[line] = false;
  }

  // both triggers together: 10 us high
  digitalWrite(SLOT_TRIG_PIN[slot][0], LOW);
  digitalWrite(SLOT_TRIG_PIN[slot][1], LOW);
  delayMicroseconds(4);
  digitalWrite(SLOT_TRIG_PIN[slot][0], HIGH);
  digitalWrite(SLOT_TRIG_PIN[slot][1], HIGH);
  delayMicroseconds(10);
  digitalWrite(SLOT_TRIG_PIN[slot][0], LOW);
  digitalWrite(SLOT_TRIG_PIN[slot][1], LOW);

  fired_us = micros();
  NewPing::timer_us(ECHO_TIMER_FREQ, echoPoll);
  return true;
}
