# waveHeight
A C++ algorithm for Arduino that attempts to measure significant wave height with a buoy mounted ICM-20948 breakout board connected to a Teensy 3.2

This algorithm measures the displacement of a ICM-20948 by constantly correcting for the orientation of the board and double integrating the accelerometer values.
The orientation is measured with this library https://github.com/ZaneL/Teensy-ICM-20948 which employs a Madgwick filter to acurrately express orientation with quaternion values.

The chip has it's own x, y and z axes that all motion is relative to and measuring wave height requires these axes to be corrected to a world frame so vertical acceleration will always be antiparallel to gravity. This correction is done with the help of a quaternion math library that converts the quaternion values to a rotation matrix. 

A filtering library employs a band-pass filter to eliminate much of the drift and high frequency noise from the accelerometer. The band-pass filter was chosen over a Kalman fitler because it is more tuned to the high f noise characteristics of the accelerometer. 

With any filtering mechanism, there must be a balance between signal stability and data loss. To limit integral drift, the high frequency accelerometer readings must be heavily filtered, resulting in significant data loss. 
![alt text](https://i.ibb.co/34X9ZZd/z-Accel015.png)

# Examples
The first example prints the real time vertical position values of the board to the serial port.

It uses a combination of a high-pass and low-pass filter from the <filters.h> library to create a band-pass filter.

```
// Set filter parameters
const float cutoff_freq_lp   = 30.0;  //Cutoff frequency in Hz
const float sampling_time_lp = 0.005; //Sampling time in seconds.
IIR::ORDER  order_lp  = IIR::ORDER::OD2; // Order (OD1 to OD4)

const float cutoff_freq_hp   = 0.135;   //Cutoff frequency in Hz
const float sampling_time_hp = 0.005; //Sampling time in seconds.
IIR::ORDER order_hp = IIR::ORDER::OD2; // Order

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
```
Wave height measurement is possible because we can limit noise accumulation by reseting the position at the trough of each wave. Experimentally, we decided this could be approximated by the point in time at which the magnitude of the acceleration vector is less than 0.15.

```
// Take magnitude of newVec
accelMag = sqrt(xAccelFiltered * xAccelFiltered + yAccelFiltered * yAccelFiltered + zAccelFiltered * zAccelFiltered);

// Reset position if stationary
if (accelMag < 0.15)
{
  xPosition = 0.0;
  yPosition = 0.0;
  zPosition = 0.0;
}    
```
