#pragma once
#include <arduino-timer.h>
#include <ArduinoQueue.h>
#include <ZumoShield.h>
#include "config.h"
#include "types.h"

struct Globals {
  volatile int8_t uss_distances[4];
  volatile int8_t uss_adj_distances[4];
  // the same readings in mm with the offset applied but not rounded; 0 until the sensor has had a first echo
  volatile int16_t uss_mm[4];
  // true while the tracks turn the bot on the spot; no sonar is fired then, the readings are not squarely on the walls
  volatile bool turning = false;
  volatile point pos = {COORD_UNDEF, COORD_UNDEF};
  volatile int16_t theta;
  // move queue for POI on the board i.e. letters
  volatile ArduinoQueue<point> poi_queue = ArduinoQueue<point>(64);
};

extern Globals global;

// Every command to the tracks goes through here, so the compass code can tell when the motors are off.
void setTracks(int16_t left, int16_t right);
bool tracksIdleFor(uint16_t ms);

extern Timer<> timer;
extern ZumoMotors motors;
extern Pushbutton button;
