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
MPU6050 operates at data rate 200Hz for attitude estimation. The altitude algorithm operates at 25Hz, at which the barometric pressure is measured and fused with the linear acceleration calculated from attitude and acceleration.

The fused altitude is shown on LCM to be compared with the pressure altitude calculated directly from the barometer. The STD's of the fused and barometric altitude are calculated for 1 sec span and shown on LCM for comparison.

