AT-START-F403 + GMP102 + MPU6050 Altitude Fusion example code
=============================================================

Requirements
-----------
- AT-START-F403 development board
- **Sensor Fusion Arduino Daughter Board V1.0**
  - Barometer: GMEMS GMP102
  - Gyro and Accelerometer: InvenSense MPU6050

I2C Connections
---------------
- Use I2C1
  - SCL: PB8
  - SDA: PB9
- Barometer
  - GMP102 I2C 7-bit slave address: 0x6C
- Gyro and Accelerometer
  - MPU6050 I2C 7-bit slave address: 0x68

Usage of AutoNil
----------------
 * The program will do an gyro and accelerometer offset AutoNil when executed. Hold the board steady and maintain in level facing up, then press **Key1** after the program prompt for input.

Brief Description
-----------------
MPU6050 operates at data rate 200Hz for attitude estimation. The altitude algorithm operates at 25Hz, at which the barometric pressure is measured and fused with the linear acceleration calculated from attitude and acceleration by the complimentary and Kalman filters respectively.

The fused altitudes of both the complimentary and Kalman filters are shown on LCM to be compared with the pressure altitude calculated directly from the barometer. The STD's of the fused and barometric altitude are calculated for 1 sec span and shown on LCM for comparison.

It can be observed that the Kalman filter result will settle down within secs to that of the complimentary filter. Actually the complimentary filter parameters can be chosen in such a way that related to the steady state Kalman gain, see [A Comparison of Complementary and Kalman Filtering](https://ieeexplore.ieee.org/document/4101411/) for detail. This example code set up the the complimentary filter parameters corresponding to the steady state Kalman gain.

