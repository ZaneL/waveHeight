#include <Teensy-ICM-20948.h>
#include "Quaternion.h"
#include <filters.h> // GitHub Repo: https://github.com/MartinBloedorn/libFilter

#define filterSamples   33 // filterSamples should  be an odd number, no smaller than 3

float xVelocity;
float xPosition;
float yVelocity;
float yPosition;
float zVelocity;
float zPosition;
unsigned long currentTime;
unsigned long lastTime;
float dt;
const float cutoff_freq   = 20;  //Cutoff frequency in Hz
const float sampling_time = 0.005; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD1; // Order (OD1 to OD4)

// Low-pass filter
Filter f(cutoff_freq, sampling_time, order);

// ICM Settings
TeensyICM20948 icm20948;

TeensyICM20948Settings icmSettings =
{
  .cs_pin = 10,                  // SPI chip select pin
  .mode = 1,                     // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = false,      // Enables gyroscope output
  .enable_accelerometer = true,  // Enables accelerometer output
  .enable_magnetometer = false,   // Enables magnetometer output
  .enable_quaternion = true,     // Enables quaternion output
  .gyroscope_frequency = 1,      // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 225,  // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 1,   // Max frequency = 70, min frequency = 1
  .quaternion_frequency = 225     // Max frequency = 225, min frequency = 50
};

/**************************
 * Setup
 *************************/
 
void setup()
{
  Serial.begin(115200);
  delay(5000);

  icm20948.init(icmSettings);

  currentTime = millis();
  lastTime = millis();
}

/************************
 * Loop
 ***********************/
 
void loop()
{
  float accel_x, accel_y, accel_z;
  float quat_w, quat_x, quat_y, quat_z;
  Quaternion quat;
  double newVec[3];

  // Must call this often in main loop -- updates the sensor values
  icm20948.task();

  if (icm20948.accelDataIsReady() && icm20948.quatDataIsReady())
  {
    currentTime = millis();
    dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Get most recent accel and quat values
    icm20948.readAccelData(&accel_x, &accel_y, &accel_z);
    icm20948.readQuatData(&quat_w, &quat_x, &quat_y, &quat_z);

    // Setup accel vector and convert g's to m/s^2
    double tempVec[3] = {accel_x * 9.81, accel_y * 9.81, accel_z * 9.81};

    // Setup quaternion
    Quaternion_set(quat_w, quat_x, quat_y, quat_z, &quat);

    // Get new accel vector that is rotated to world frame
    Quaternion_rotate (&quat, tempVec, newVec);

    // Smooth z acclerations in new vector
    float zAccelerationSmoothed = f.filterIn(newVec[2] - 9.93);

    // Integral of acceleration on each axis
    xVelocity += newVec[0] * dt;
    yVelocity += newVec[1] * dt;
    zVelocity += zAccelerationSmoothed * dt;

    // Smooth z velocity
    float zVelocitySmoothed = f.filterIn(zVelocity);

    // Integral of velocity on each axis
    xPosition += xVelocity * dt;
    yPosition += yVelocity * dt;
    zPosition += zVelocitySmoothed * dt;

    // Print
    Serial.println(zAccelerationSmoothed);

  }
}
