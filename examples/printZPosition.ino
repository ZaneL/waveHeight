#include "Teensy-ICM-20948.h"
#include "Quaternion.h"
#include <filters.h>

#define filterSamples   101 // filterSamples should  be an odd number, no smaller than 3

double xVelocity;
double xPosition;
double yVelocity;
double yPosition;
double zVelocity;
double zPosition;
float dt = 1.0 / 255.0;
float gravity = 9.81;

// Set filter parameters
const float cutoff_freq_lp   = 30.0;  //Cutoff frequency in Hz
const float sampling_time_lp = 0.005; //Sampling time in seconds.
IIR::ORDER  order_lp  = IIR::ORDER::OD2; // Order (OD1 to OD4)

const float cutoff_freq_hp   = 0.135;   //Cutoff frequency in Hz
const float sampling_time_hp = 0.005; //Sampling time in seconds.
IIR::ORDER order_hp = IIR::ORDER::OD2; // Order

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

// Create low-pass filter
Filter f(cutoff_freq_lp, sampling_time_lp, order_lp);

// Create high-pass filter
Filter fhp(cutoff_freq_hp, sampling_time_hp, order_hp, IIR::TYPE::HIGHPASS);

// Create function to combine filters
double bandPassFilter (double rawData)
{
  double lowPassFiltered = f.filterIn(rawData);
  double highPassFiltered = fhp.filterIn(lowPassFiltered);
  return highPassFiltered;
}

/**************************
   Setup
 *************************/

void setup()
{
  Serial.begin(115200);
  delay(5000);

  icm20948.init(icmSettings);
}

/************************
   Loop
 ***********************/

void loop()
{
  float accel_x, accel_y, accel_z;
  float quat_w, quat_x, quat_y, quat_z;
  Quaternion quat;
  double newVec[3];
  double xAccelFiltered, yAccelFiltered, zAccelFiltered;
  double accelMag;
  double maxZ;
  double avgHeight = 0.0;
  double readings[10]; 
  int i, j, n = 10;
  double value;

  // Must call this often in main loop -- updates the sensor values
  icm20948.task();

  if (icm20948.accelDataIsReady() && icm20948.quatDataIsReady())
  {
    // Get most recent accel and quat values
    icm20948.readAccelData(&accel_x, &accel_y, &accel_z);
    icm20948.readQuatData(&quat_w, &quat_x, &quat_y, &quat_z);

    // Setup accel vector and convert g's to m/s^2
    double tempVec[3] = {accel_x * gravity, accel_y * gravity, accel_z * gravity};

    // Setup quaternion
    Quaternion_set(quat_w, quat_x, quat_y, quat_z, &quat);

    // Get new acceleration vector that is rotated to world frame
    Quaternion_rotate(&quat, tempVec, newVec);

    // Smooth accelerations with band filter in new vector
    xAccelFiltered = bandPassFilter(newVec[0]);
    yAccelFiltered = bandPassFilter(newVec[1]);
    zAccelFiltered = bandPassFilter(newVec[2] - gravity);

    // Take magnitude of newVec
    accelMag = sqrt(xAccelFiltered * xAccelFiltered + yAccelFiltered * yAccelFiltered + zAccelFiltered * zAccelFiltered);

    // Reset position if stationary
    if (accelMag < 0.15)
    {
      xPosition = 0.0;
      yPosition = 0.0;
      zPosition = 0.0;
    }

    // Integral of acceleration on each axis
    xVelocity += xAccelFiltered * dt;
    yVelocity += yAccelFiltered * dt;
    zVelocity += zAccelFiltered * dt;

    // Integral of velocity on each axis
    xPosition += xVelocity * dt;
    yPosition += yVelocity * dt;
    zPosition += zVelocity * dt;

    // Print
    Serial.println(zPosition);
  }
}
