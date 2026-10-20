#include <Wire.h>
#include "compass.h"
#include "globals.h"
#include "link.h"

uint16_t x_heading;

ZumoIMU imu;
ZumoIMU::vector<int16_t> m_max, m_min;

// The heading is tracked by the gyro while the tracks run and set from the compass while they are off: the motors
// disturb the magnetometer, but the gyro only knows how far the bot has turned, not which way it faces.
static float theta_f = 0;      // degrees, same convention as global.theta
static float gyro_bias = 0;    // raw z counts at rest
static int8_t gyro_sign = 1;   // +1 if the gyro counts in the direction theta increases, -1 if the opposite
static bool gyro_ok = false;
static uint32_t last_gyro_us = 0;
static uint32_t last_mag_ms = 0;

uint16_t angleDiff(int16_t source_deg, int16_t target_deg) {
  int16_t diff = (target_deg - source_deg) % 360;
  if (diff < 0) diff += 360;
  return diff;
}

struct Turn shortestTurn(int16_t source_deg, int16_t target_deg) {
  uint16_t diff = angleDiff(source_deg, target_deg);
  return {
    min(diff, 360 - diff),
    diff < 180 ? 1 : -1
  };
}

bool headingIsNot(uint16_t deg, bool lax) {
  return angleDiff(deg, global.theta) > (lax ? LAX_HEADING_DEVIATION_THRESHOLD : HEADING_DEVIATION_THRESHOLD);
}

bool headingIs(uint16_t deg, bool lax) {
  return angleDiff(deg, global.theta) <= (lax ? LAX_HEADING_DEVIATION_THRESHOLD : HEADING_DEVIATION_THRESHOLD);
}

// Converts x and y components of a vector to a heading in degrees.
// This calculation assumes that the Zumo is always level.
template <typename T> uint16_t headingOf(ZumoIMU::vector<T> v)
{
  float x_scaled =  2.0 * (float)(v.x - m_min.x) / (m_max.x - m_min.x) - 1.0;
  float y_scaled =  2.0 * (float)(v.y - m_min.y) / (m_max.y - m_min.y) - 1.0;

  // a float, not uint16_t: atan2 is negative for headings past 180, and an unsigned angle wrapped to ~65000 there
  float angle = atan2(y_scaled, x_scaled) * RAD2DEG;
  if (angle < 0)
    angle += 360;

  return angle;
}

uint16_t averageHeading() {
  ZumoIMU::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    imu.readMag();
    avg.x += imu.m.x;
    avg.y += imu.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return headingOf(avg);
}

bool headingNear(uint16_t deg, bool lax) {
  return shortestTurn(global.theta, deg).angle <= (lax ? LAX_HEADING_DEVIATION_THRESHOLD : HEADING_DEVIATION_THRESHOLD);
}

float headingDeg() {
  return theta_f;
}

float angleError(float target_deg) {
  return wrap180(target_deg - theta_f);
}

// The bot's heading as the compass sees it. North is the x-axis; adjusted for the board's orientation.
static float compassTheta() {
  float t = 360 - averageHeading() + x_heading;
  if (t >= 360)
    t -= 360;
  return t;
}

bool updateHeading(void *) {
  if (gyro_ok) {
    imu.readGyro();
    uint32_t now_us = micros();
    theta_f += gyro_sign * (imu.g.z - gyro_bias) * GYRO_DEG_PER_LSB * (now_us - last_gyro_us) * 1e-6;
    last_gyro_us = now_us;
  }

  // the compass is only trusted with the motors off; without a working gyro it is all there is
  if (!gyro_ok) {
    theta_f = compassTheta();
  } else if (tracksIdleFor(MAG_SETTLE_MS) && millis() - last_mag_ms >= MAG_REFRESH_MS) {
    last_mag_ms = millis();
    theta_f = compassTheta();
  }

  while (theta_f >= 360) theta_f -= 360;
  while (theta_f < 0) theta_f += 360;
  global.theta = (int16_t) theta_f;

  return true;
}

void compassBegin() {
  imu.init();
  imu.enableDefault();
  imu.configureForCompassHeading();
}

// Adds up how far the gyro says the bot turned over ms, in the gyro's own positive direction.
static void integrateGyro(float &turned_deg, uint32_t &last_us, uint16_t ms) {
  uint32_t end = millis() + ms;
  while ((int32_t) (millis() - end) < 0) {
    imu.readGyro();
    uint32_t now_us = micros();
    turned_deg += (imu.g.z - gyro_bias) * GYRO_DEG_PER_LSB * (now_us - last_us) * 1e-6;
    last_us = now_us;
    delay(5);
  }
}

float wrap180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

struct CalibSample {
  int16_t x, y;
  int16_t turned_deci;  // gyro angle so far, in tenths of a degree
};

void calibrateCompass() {
  Serial.println(F("calibrating compass..."));

  ZumoIMU::vector<int16_t> running_min = {32767, 32767, 32767},
    running_max = {-32767, -32767, -32767};

  // the gyro reads a constant offset at rest; measure it while the bot is still
  float bias = 0;
  for (uint8_t i = 0; i < GYRO_BIAS_SAMPLES; i++) {
    imu.readGyro();
    bias += imu.g.z;
    delay(10);
  }
  gyro_bias = bias / GYRO_BIAS_SAMPLES;

  CalibSample samples[MOTOR_CALIB_MAX_SAMPLES];
  uint8_t n = 0;
  float turned_deg = 0;
  uint32_t started_ms = millis();
  uint32_t last_us = micros();

  // The magnetometer range is only complete after a full turn, so keep going until the gyro says the bot has made one.
  // Spin in pulses and read the compass in the pauses: the motors would disturb it.
  while (fabs(turned_deg) < MOTOR_CALIB_MIN_TURN_DEG && millis() - started_ms < MOTOR_CALIB_MAX_MS) {
    motors.setLeftSpeed(MOTOR_CALIB_SPEED);
    motors.setRightSpeed(-MOTOR_CALIB_SPEED);
    integrateGyro(turned_deg, last_us, MOTOR_CALIB_PULSE_ON_MS);

    motors.setLeftSpeed(0);
    motors.setRightSpeed(0);
    integrateGyro(turned_deg, last_us, MOTOR_CALIB_SETTLE_MS);  // the bot coasts to a stop, the motor fields die away

    int32_t sum_x = 0, sum_y = 0;
    for (uint8_t i = 0; i < 4; i++) {
      imu.readMag();
      sum_x += imu.m.x;
      sum_y += imu.m.y;
      delay(2);
    }
    int16_t mx = sum_x / 4, my = sum_y / 4;

    running_min.x = min(running_min.x, mx);
    running_min.y = min(running_min.y, my);

    running_max.x = max(running_max.x, mx);
    running_max.y = max(running_max.y, my);

    if (n < MOTOR_CALIB_MAX_SAMPLES) {
      samples[n++] = {mx, my, (int16_t) (turned_deg * 10)};
    }
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  Serial.print(F("turned ")); Serial.print(turned_deg); Serial.println(F(" deg"));
  Serial.print("max.x   ");
  Serial.print(running_max.x);
  Serial.println();
  Serial.print("max.y   ");
  Serial.print(running_max.y);
  Serial.println();
  Serial.print("min.x   ");
  Serial.print(running_min.x);
  Serial.println();
  Serial.print("min.y   ");
  Serial.print(running_min.y);
  Serial.println();

  m_max.x = running_max.x;
  m_max.y = running_max.y;
  m_min.x = running_min.x;
  m_min.y = running_min.y;

  // Work out which way the gyro counts relative to theta by comparing how both moved between the pauses.
  gyro_ok = fabs(turned_deg) >= GYRO_WORKING_MIN_DEG;
  if (gyro_ok && n >= 3) {
    float corr = 0;
    ZumoIMU::vector<int16_t> v = {samples[0].x, samples[0].y, 0};
    float prev_theta = -(float) headingOf(v);
    for (uint8_t i = 1; i < n; i++) {
      v = {samples[i].x, samples[i].y, 0};
      float t = -(float) headingOf(v);
      float d_theta = wrap180(t - prev_theta);
      float d_gyro = (samples[i].turned_deci - samples[i - 1].turned_deci) / 10.0;
      corr += d_theta * d_gyro;
      prev_theta = t;
    }
    gyro_sign = corr >= 0 ? 1 : -1;
  }
  Serial.print(F("gyro ")); Serial.print(gyro_ok ? F("ok") : F("NOT working, using the compass only"));
  Serial.print(F(", sign ")); Serial.println(gyro_sign);
}

void orientToBoard() {
  sendOrient();
  Serial.println(F("Orient planchette, waiting 5s"));
  delay(5000);
  x_heading = averageHeading();
  Serial.print(F("x_heading=")); Serial.println(x_heading);
  sendReady();

  // the planchette is lined up with the board's x axis, which is heading 0 by definition
  theta_f = 0;
  global.theta = 0;
  last_gyro_us = micros();
  last_mag_ms = millis();
}
