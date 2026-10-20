#pragma once
#include <Arduino.h>
#include <ZumoShield.h>

// Zumo shield v1.3 IMU (LSM6DS33 accelerometer + gyro, LIS3MDL magnetometer).
namespace Imu {

bool begin();

// Reads the gyro and integrates yaw. Call it every loop so the integration steps stay short.
void update();

// Reads the accelerometer and magnetometer (call just before sending telemetry).
void readSlow();

// Measures the gyro z bias over about 1 s (the bot must be still), then zeroes yaw. Returns the bias in raw counts.
int16_t zeroGyro();

// Rotation about z since the last zeroGyro(), in degrees (positive = counter-clockwise if the chip's z axis points up).
float yawDeg();

extern ZumoIMU imu;  // raw values: imu.a, imu.g, imu.m

}  // namespace Imu
