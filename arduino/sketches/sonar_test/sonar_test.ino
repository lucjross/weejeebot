// Bench test for the four HC-SR04 sensors. Send F, R, B or L over serial to
// take one reading from that side; replies with the distance in cm ("0" = no echo).
#include <NewPing.h>

#define USS_F_TRIG_PIN 4
#define USS_B_TRIG_PIN 5
#define USS_L_TRIG_PIN 2
#define USS_R_TRIG_PIN 11
// front/back share the x-axis echo line, left/right share the y-axis echo line
#define USS_X_ECHO_PIN A2
#define USS_Y_ECHO_PIN A1
#define MAX_DISTANCE_cm 200

NewPing front(USS_F_TRIG_PIN, USS_X_ECHO_PIN, MAX_DISTANCE_cm);
NewPing right(USS_R_TRIG_PIN, USS_Y_ECHO_PIN, MAX_DISTANCE_cm);
NewPing back(USS_B_TRIG_PIN, USS_X_ECHO_PIN, MAX_DISTANCE_cm);
NewPing left(USS_L_TRIG_PIN, USS_Y_ECHO_PIN, MAX_DISTANCE_cm);

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (!Serial.available()) return;
  NewPing *sensor = nullptr;
  switch (Serial.read()) {
    case 'F': sensor = &front; break;
    case 'R': sensor = &right; break;
    case 'B': sensor = &back; break;
    case 'L': sensor = &left; break;
  }
  if (sensor) Serial.println(sensor->ping_cm());
}
