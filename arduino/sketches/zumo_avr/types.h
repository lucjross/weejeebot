#pragma once
#include <Arduino.h>

struct point {
  int8_t x;
  int8_t y;
};

struct Turn {
  uint16_t angle;
  int16_t direction;
};
