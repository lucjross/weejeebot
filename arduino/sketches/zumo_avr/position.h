#pragma once
#include <Arduino.h>

// Works out global.pos from the sonar distances. Run it on a timer (FIND_POSITION_INTERVAL).
bool findPosition(void *);

// The bot's coordinate along the axis it is facing, in cm from the board centre, from the F and B sonars: the x axis
// at heading 0, the y axis at heading 90 (the front sonar then looks at the top wall). Uses the sonar nearer a wall.
// Only meaningful while the bot is square to the walls. Returns false if neither sonar has a usable reading.
// If sensor is given: when it holds USS_F or USS_B only that sensor is used, so a drive does not jump between two
// sensors whose offsets differ a little; when it holds -1 the nearer one is chosen and written back.
bool axisPosition(bool y_axis, float &out, int8_t *sensor = nullptr);
