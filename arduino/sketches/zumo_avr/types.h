#pragma once
#include <Arduino.h>

struct point {
  int8_t x;
  int8_t y;
};

// one step of getting to a point of interest; the move queue holds pointers to these
struct intermMove {
  virtual void start() {}
  virtual void handle() {}
  virtual bool isDone() { return false; }
};

struct Turn {
  uint16_t angle;
  int16_t direction;
};
