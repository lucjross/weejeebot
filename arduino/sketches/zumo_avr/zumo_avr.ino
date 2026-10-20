// Uno on the Zumo: drives the planchette to the points of interest (letters) that the Nano sends it, finding its
// way with four ultrasonic sensors for position and the compass for heading. See ../../WIRING.md.
//
//   config.h    constants and pins          sonar.*     the four HC-SR04 sensors
//   types.h     point, move step, turn      position.*  x/y on the board from the sonars
//   globals.*   shared state, timer         compass.*   heading, turns, compass calibration
//   link.*      serial link to the Nano     moves.*     the move queue and the motors
#include <Wire.h>
#include "config.h"
#include "globals.h"
#include "sonar.h"
#include "position.h"
#include "compass.h"
#include "link.h"
#include "moves.h"

/**
 * Setup
 */
void setup() {

  Serial.begin(19200);
  Serial.println(F("Running"));

  linkBegin();

  sonarBegin();

  Wire.begin();

  compassBegin();

//  velocityTest();

  // compass calibration
  button.waitForButton();

  Serial.println(F("sending serial check"));
  sendCheck();

  Serial.println(F("waiting 5s"));
  delay(5000);

  calibrateCompass();

  orientToBoard();

  Serial.println(F("starting timers"));

  // start USS pings
  timer.every(USS_PING_INTERVAL, ussPingTimers);

  timer.every(FIND_ORIENTATION_INTERVAL, updateHeading);

  timer.every(FIND_POSITION_INTERVAL, findPosition);

  delay(500);
  timer.every(MOVE_INTERVAL, runMotors);

  timer.every(500, readNSS);
}

void loop() {
  timer.tick<void>();
}
