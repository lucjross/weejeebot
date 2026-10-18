#include <NewPing.h>
#include "sonar.h"
#include "globals.h"

NewPing uss_newping[4] = {
  NewPing(USS_F_TRIG_PIN, USS_X_ECHO_PIN, BOARD_W_cm),
  NewPing(USS_R_TRIG_PIN, USS_Y_ECHO_PIN, BOARD_W_cm),
  NewPing(USS_B_TRIG_PIN, USS_X_ECHO_PIN, BOARD_W_cm),
  NewPing(USS_L_TRIG_PIN, USS_Y_ECHO_PIN, BOARD_W_cm)
};

// subtracted from every reading; indexed like uss_newping (USS_F, USS_R, USS_B, USS_L)
const float uss_offset_cm[4] = USS_OFFSETS_cm;

void echoCheck() {
  uint8_t uss = global.uss_curr;
  if (uss_newping[uss].check_timer()) {
    // round to the nearest cm after the offset; truncating would bias every reading about 5 mm short
    float cm = (float) uss_newping[uss].ping_result / US_ROUNDTRIP_CM - uss_offset_cm[uss];
    global.uss_distances[uss] = cm < 0 ? 0 : (int8_t) (cm + 0.5);
    global.uss_adj_distances[uss] = global.uss_distances[uss] + USS_DIST_ADJ_cm;
  }
}

bool ussPingTimers(void *) {
  uss_newping[global.uss_curr].timer_stop();
  
  global.uss_curr = (global.uss_curr + 1) % 4;
  global.uss_distances[global.uss_curr] = -1;
  uss_newping[global.uss_curr].ping_timer(echoCheck);
  
  return true;
}
