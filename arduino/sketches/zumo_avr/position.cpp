#include "position.h"
#include "globals.h"
#include "compass.h"

bool findPosition(void *) {
  if (!headingNear(0, true)) {
    return true;
  }

  int8_t x = COORD_UNDEF, y = COORD_UNDEF,
      x_f, x_b, y_l, y_r;

  // x...
  int8_t dist_x_f = global.uss_adj_distances[USS_F],
      dist_x_b = global.uss_adj_distances[USS_B];
  if (dist_x_f != -1) {
    x_f = BOARD_RIGHT_WALL_X - dist_x_f;
    x = x_f;
  }
  
  if (dist_x_b != -1) {
    x_b = BOARD_LEFT_WALL_X + dist_x_b;
    x = x_b;
  }

  if (dist_x_f != -1 && dist_x_b != -1) {
    // use the more reliable shorter distance
    x = dist_x_f < dist_x_b ? x_f : x_b;
    
    if (abs((dist_x_f + dist_x_b) - BOARD_W_cm) > BOARD_W_DEV_THRESHOLD) {
//      Serial.print("x distances F="); Serial.print(dist_x_f);
//      Serial.print(" and B="); Serial.print(dist_x_b);
//      Serial.println(" don't add up");
    }
  }
  else if (dist_x_f == -1 && dist_x_b == -1) {
    Serial.println(F("x undef"));
    global.pos = {COORD_UNDEF, COORD_UNDEF};
    return true;
  }

  // y...
  int8_t dist_y_l = global.uss_adj_distances[USS_L],
      dist_y_r = global.uss_adj_distances[USS_R];
  if (dist_y_l != -1) {
    y_l = BOARD_TOP_WALL_Y - dist_y_l;
    y = y_l;
  }
  
  if (dist_y_r != -1) {
    y_r = BOARD_BOTTOM_WALL_Y + dist_y_r;
    y = y_r;
  }

  if (dist_y_l != -1 && dist_y_r != -1) {
    // use the more reliable shorter distance
    y = dist_y_l < dist_y_r ? y_l : y_r;
    
    if (abs((dist_y_l + dist_y_r) - BOARD_H_cm) > BOARD_H_DEV_THRESHOLD) {
//      Serial.print("y distances L="); Serial.print(dist_y_l);
//      Serial.print(" and R="); Serial.print(dist_y_r);
//      Serial.println(" don't add up");
    }
  }
  else if (dist_y_l == -1 && dist_y_r == -1) {
    Serial.println(F("y undef"));
    global.pos = {COORD_UNDEF, COORD_UNDEF};
    return true;
  }

  global.pos = {x, y};

  Serial.print(F("x=")); Serial.print(global.pos.x);
  Serial.print(F(", y=")); Serial.println(global.pos.y);
  
  return true;
}

bool axisPosition(bool y_axis, float &out, int8_t *sensor) {
  // the last echo of each sensor is kept while a new ping is out; too close to be real (or a missing echo that read 0)
  // does not count. Distances are to the sensor face; USS_DIST_ADJ_cm moves them to the bot's centre.
  // echoCheck updates these in an interrupt, and an int16 is read in two steps
  noInterrupts();
  int16_t f_mm = global.uss_mm[USS_F], b_mm = global.uss_mm[USS_B];
  interrupts();
  float df = f_mm / 10.0, db = b_mm / 10.0;
  bool f_ok = df >= USS_MIN_VALID_cm;
  bool b_ok = db >= USS_MIN_VALID_cm;

  int8_t use;
  if (sensor != nullptr && *sensor >= 0) {
    use = *sensor;
  } else if (f_ok && (!b_ok || df < db)) {
    use = USS_F;
  } else if (b_ok) {
    use = USS_B;
  } else {
    return false;
  }
  if (use == USS_F ? !f_ok : !b_ok) {
    return false;
  }
  if (sensor != nullptr) {
    *sensor = use;
  }

  float wall_f = y_axis ? BOARD_H_cm / 2 : BOARD_W_cm / 2;
  out = use == USS_F ? wall_f - (df + USS_DIST_ADJ_cm) : -wall_f + (db + USS_DIST_ADJ_cm);
  return true;
}
