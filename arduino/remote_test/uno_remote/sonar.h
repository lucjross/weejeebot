#pragma once
#include <Arduino.h>
#include "config.h"

// Four HC-SR04 sensors, two per echo line (see ../../WIRING.md).
namespace Sonar {

// Pings ONE sensor (the next in round-robin order) if SONAR_PING_GAP_MS has passed. Call it every loop.
void step();

// Last distance in mm; -1 = no echo (nothing in range, too close, or a wiring fault).
int16_t mm(uint8_t i);

// True if sensor i produced a valid reading within the last maxAgeMs.
bool fresh(uint8_t i, uint16_t maxAgeMs);

extern const char *const NAMES[SONAR_COUNT];

}  // namespace Sonar
