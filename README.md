# waveHeight
A C++ algorithm for Arduino that attempts to measure significant wave height with a buoy mounted ICM-20948 breakout board connected to a Teensy 3.2

This algorithm measures the displacement of a ICM-20948 by constantly correcting for the orientation of the board and double integrating the accelerometer values.
The orientation is measured with this library https://github.com/ZaneL/Teensy-ICM-20948 which employs a Madgwick filter to acurrately express orientation with quaternion values.

The chip has it's own x, y and z axes that all motion is relative to and measuring wave height requires these axes to be corrected to a world frame so vertical acceleration will always be antiparallel to gravity. This correction is done with the help of a quaternion math library that converts the quaternion values to a rotation matrix. 
![alt text](https://i.ibb.co/3kQRM8P/rotated-3-D-vector.png)

A filtering library employs a band-pass filter to eliminate much of the drift and high frequency noise from the accelerometer. The band-pass filter was chosen over a Kalman fitler because it is more tuned to the high frequency noise characteristics of the accelerometer. 

Unfortunately, even with these filtering mechanisms, noise compounds exponentially. The below graph is a experiment done by moving the chip up and down between two known points and graphing the actual height against what the chip was measuring.  
![alt text](https://i.ibb.co/nbWc3Y9/compounding-Noise.png)


By the time thes signal is filtered enough to be stable, too much data is ignored and the readings are innacurate. 
![alt text](https://i.ibb.co/34X9ZZd/z-Accel015.png)


Resetting the position at the trough of each wave allows for greater stability.
![alt text](https://i.ibb.co/BwhqqDy/position-Reset.png)


However, over longer periods of time the integral drift will still make the readings wildly inaccurate.
![alt text](https://i.ibb.co/YB7PPLq/integral-Drift.png)


Current development of the algorithm revolves around correcting this problem. The most promising avenues are using the trough and the peak of the wave (the points at which the chip is most stationary) to calcuate the rate of drift or employing machine learning techniques to validate the data.  

# Examples
The example prints the real time vertical position values of the board to the serial port.

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
