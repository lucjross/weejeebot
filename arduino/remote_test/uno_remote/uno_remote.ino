// Uno side of the remote test harness. See ../README.md for the protocol and safety rules.
//
// Receives command lines from the Nano over serial, runs timed motor moves behind safety guards,
// reads the four ultrasonic sensors and the IMU, and sends telemetry lines back.
#include <Wire.h>
#include "config.h"
#include "link.h"
#include "sonar.h"
#include "imu.h"
#include "motion.h"

uint16_t telPeriodMs = 0;  // 0 = telemetry off
uint32_t lastTelMs = 0;

void reply(const char *id, const char *msg) {
  char out[LINE_MAX + 16];
  if (id[0]) {
    snprintf(out, sizeof out, "%s %s", id, msg);
  } else {
    snprintf(out, sizeof out, "%s", msg);
  }
  Link::sendLine(out);
}

void sendTelemetry() {
  Imu::readSlow();
  static char out[140];
  snprintf(out, sizeof out,
           "T %lu %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
           (unsigned long) millis(),
           Sonar::mm(SONAR_F), Sonar::mm(SONAR_R), Sonar::mm(SONAR_B), Sonar::mm(SONAR_L),
           Imu::imu.m.x, Imu::imu.m.y, Imu::imu.m.z,
           Imu::imu.a.x, Imu::imu.a.y, Imu::imu.a.z,
           Imu::imu.g.x, Imu::imu.g.y, Imu::imu.g.z,
           (int) (Imu::yawDeg() * 10.0f),
           Motion::left(), Motion::right());
  Link::sendLine(out);
}

// Parses an integer token; returns false if the token is missing or not a number.
bool parseInt(char *tok, long &out) {
  if (!tok) return false;
  char *end;
  out = strtol(tok, &end, 10);
  return *end == '\0';
}

void handleCommand(char *line) {
  char id[8] = "";
  char *tok = strtok(line, " ");
  if (!tok) return;
  if (tok[0] == '#') {
    strncpy(id, tok, sizeof id - 1);
    id[sizeof id - 1] = '\0';
    tok = strtok(nullptr, " ");
    if (!tok) {
      reply(id, "ERR empty");
      return;
    }
  }

  if (strcmp(tok, "PING") == 0) {
    reply(id, "OK pong");
  } else if (strcmp(tok, "STOP") == 0) {
    Motion::stop();
    reply(id, "OK");
  } else if (strcmp(tok, "STATE") == 0) {
    reply(id, "OK");
    sendTelemetry();
  } else if (strcmp(tok, "TEL") == 0) {
    long ms;
    if (!parseInt(strtok(nullptr, " "), ms)) {
      reply(id, "ERR args");
    } else if (ms != 0 && (ms < TEL_MIN_MS || ms > TEL_MAX_MS)) {
      reply(id, "ERR range");
    } else {
      telPeriodMs = (uint16_t) ms;
      lastTelMs = millis();
      reply(id, "OK");
    }
  } else if (strcmp(tok, "DRIVE") == 0) {
    long l, r, ms, ramp = 0;
    if (!parseInt(strtok(nullptr, " "), l) || !parseInt(strtok(nullptr, " "), r) ||
        !parseInt(strtok(nullptr, " "), ms)) {
      reply(id, "ERR args");
      return;
    }
    char *rampTok = strtok(nullptr, " ");  // optional 4th argument: ramp time in ms
    if (rampTok && !parseInt(rampTok, ramp)) {
      reply(id, "ERR args");
      return;
    }
    const char *err = Motion::start((int) l, (int) r, (int) ms, (int) ramp);
    if (err) {
      char msg[16];
      snprintf(msg, sizeof msg, "ERR %s", err);
      reply(id, msg);
    } else {
      reply(id, "OK");
    }
  } else if (strcmp(tok, "GYROZERO") == 0) {
    if (Motion::active()) {
      reply(id, "ERR busy");
    } else {
      char msg[24];
      snprintf(msg, sizeof msg, "OK bias=%d", Imu::zeroGyro());
      reply(id, msg);
    }
  } else {
    reply(id, "ERR unknown");
  }
}

void setup() {
  Wire.begin();
  Motion::begin();
  Link::begin();
  bool imuOk = Imu::begin();

  char msg[24];
  snprintf(msg, sizeof msg, "E boot imu=%d", imuOk ? 1 : 0);
  Link::sendLine(msg);
}

void loop() {
  Imu::update();

  Motion::Event ev = Motion::update();
  if (ev == Motion::DONE) {
    Link::sendLine("E done");
  } else if (ev == Motion::GUARD) {
    char msg[24];
    snprintf(msg, sizeof msg, "E %s", Motion::lastReason());
    Link::sendLine(msg);
  }

  Sonar::step();

  char line[LINE_MAX];
  if (Link::readLine(line)) {
    handleCommand(line);
  }

  if (telPeriodMs > 0 && (uint32_t) (millis() - lastTelMs) >= telPeriodMs) {
    lastTelMs = millis();
    sendTelemetry();
  }
}
