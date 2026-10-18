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
