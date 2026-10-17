#include "imu.h"
#include "config.h"

namespace {

bool ok = false;
float yaw = 0;
float biasZ = 0;
uint32_t lastUs = 0;

}  // namespace

namespace Imu {

ZumoIMU imu;

bool begin() {
  ok = imu.init();
  if (ok) {
    imu.enableDefault();
    imu.configureForCompassHeading();
  }
  lastUs = micros();
  return ok;
}

void update() {
  if (!ok) return;
  imu.readGyro();
  uint32_t now = micros();
  float dt = (now - lastUs) * 1e-6f;
  lastUs = now;
  yaw += ((float) imu.g.z - biasZ) * GYRO_DPS_PER_LSB * dt;
}

void readSlow() {
  if (!ok) return;
  imu.readAcc();
  imu.readMag();
}

int16_t zeroGyro() {
  if (!ok) return 0;
  const uint8_t samples = 100;
  float sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    imu.readGyro();
    sum += imu.g.z;
    delay(10);
  }
  biasZ = sum / samples;
  yaw = 0;
  lastUs = micros();
  return (int16_t) biasZ;
}

float yawDeg() {
  return yaw;
}

}  // namespace Imu
