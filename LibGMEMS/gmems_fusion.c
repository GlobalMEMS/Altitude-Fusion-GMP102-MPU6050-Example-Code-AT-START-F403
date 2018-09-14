/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : gmems_fusion.h
 *
 * Date : 2018/08/22
 *
 * Usage: Multi-sensor fusion algorithm
 *
 ****************************************************************************
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/

/*! @file gmems_fusion.c
 *  @brief  Multi-sensor fusion functions
 *  @author Joseph FC Tseng
 */

#include "gmems_fusion.h"
#include "string.h"
#include "math.h"

#define GTOMSEC2 9.80665f	 // standard gravity in m/s2
/*
 * Complementary filter parametrs KP1 and KP2 can be related to the steady state Kalman filter gain
 * by acceleration and pressure altitude noise STD.
 * See [A Comparison of Complementary and Kalman Filtering](https://ieeexplore.ieee.org/document/4101411/)
 *
 */
#define SIGMA_ACCEL     0.015f    // MPU6050 Accelerometer noise STD, g
//#define SIGMA_ACCEL     0.1f    // GMA303KU Accelerometer noise STD, g
#define SIGMA_PRESS_ALT 0.3f      // Pressure altitude noise STD, m
#define ALTITUDE_CF_KP1 (SIGMA_ACCEL*GTOMSEC2/SIGMA_PRESS_ALT) // Altitude complementary filter KP1
#define ALTITUDE_CF_KP2 sqrt(2.0f*SIGMA_ACCEL*GTOMSEC2/SIGMA_PRESS_ALT) // Altitude complementary filter KP2
//#define ALTITUDE_CF_KP1 0.55f
//#define ALTITUDE_CF_KP2 1.0f

#define ALTITUDE_CF_KI  0.f
//#define ALTITUDE_CF_KI  0.001f   // Altitude complementary filter KI
#define ALTITUDE_CF_MAX_INTEGRATED_ERROR 2500
#define ALTITUDE_CF_MIN_INTEGRATED_ERROR -2500

/*!
 * @brief Initialize the altitude state variable
 *
 * @param sv altitude fusion state variable
 *
 * @return None
 *
 */
void initAltitude(altitudeStateType* sv){

  memset(sv, 0, sizeof(altitudeStateType));
  sv->resetflag = 1;

}

/*!
 * @brief Estimate altitude by complementary filtering of
 * @brief pressure altitude and vertical acceleration
 * @brief Adapt from https://github.com/Tinkerforge/imu-barometer-fusion.git
 *
 * @param acc Vertical acceleration, g
 * @param hp  Pressure altitude, m
 * @param dt  Time step, sec
 * @param sv  Altitude fusion state variable
 *
 * @return Altitude in m
 *
 */
float altitudeByCompFilt(float acc, float hp, float dt, altitudeStateType* sv){

  const float KI = ALTITUDE_CF_KI;
  const float KP1 = ALTITUDE_CF_KP1;
  const float KP2 = ALTITUDE_CF_KP2;
  float eH;  // altitude error
  float a;   // instaneous acceleration, m/s^2
  float dV;

  // Initialization
  if(sv->resetflag){
    sv->Hhat = hp;
    sv->Vhat = 0.f;
    sv->eHi = 0.f;
    sv->resetflag = 0;
  }

  // Altitude error
  eH = hp - sv->Hhat;

  // Integrated altitude error
  sv->eHi += eH;
  if(sv->eHi > ALTITUDE_CF_MAX_INTEGRATED_ERROR)
    sv->eHi  = ALTITUDE_CF_MAX_INTEGRATED_ERROR;
  if(sv->eHi < ALTITUDE_CF_MIN_INTEGRATED_ERROR)
    sv->eHi = ALTITUDE_CF_MIN_INTEGRATED_ERROR;

  a = acc * GTOMSEC2 + sv->eHi * KI;       //m/s^2
  dV = a * dt + KP1 * dt * eH;  //m/s
  sv->Hhat += (sv->Vhat + dV / 2.0f) * dt + KP2 * dt * eH; //m
  sv->Vhat += dV;  //m/s

  return sv->Hhat;
}

static const float q30 = 1073741824.f;
static const float rad2deg = 57.2957795131f;

/*!
 * @brief Quaternion to Euler angle, Android coordinate system
 *
 * @param quat[] float array of quaternion
 * @param *phi roll in degree
 * @param *theta pitch in degree
 * @param *psi yaw in degree
 *
 * @return None
 *
 */
void quat2EulerAndroid(float quat[], float *phi, float *theta, float *psi){

  float q0, q1, q2, q3;

  q0 = quat[0];
  q1 = quat[1];
  q2 = quat[2];
  q3 = quat[3];

  *phi   =   asin(2*q1*q3 - 2*q0*q2)* rad2deg;                                  // roll
  *theta = - atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* rad2deg;         // pitch
  *psi   = - atan2(2*q1*q2 + 2*q0*q3, q0*q0 + q1*q1 - q2*q2 - q3*q3) * rad2deg; // yaw
}

/*!
 * @brief MPU6050 Quaternion to Euler angle, Android coordinate system
 *
 * @param quat[] long array of quaternion from MPU6050 eMPL
 * @param *phi roll in degree
 * @param *theta pitch in degree
 * @param *psi yaw in degree
 *
 * @return None
 *
 */
void mpuQuat2EulerAndroid(long quat[], float *phi, float *theta, float *psi){

  float q[4];

  q[0] = quat[0] / q30;
  q[1] = quat[1] / q30;
  q[2] = quat[2] / q30;
  q[3] = quat[3] / q30;

  quat2EulerAndroid(q, phi, theta, psi);
}

/*!
 * @brief MPU6050 calculate linear acceleration, Android coordinate system
 *
 * @param quat[] long array of quaternion from MPU6050 eMPL
 * @param s16Accel acceleration readings in sensor frame, code
 * @param fLinearAcc linear acceleration in global frame, g
 * @param accS accelerometer sensitivity, code/g
 *
 * @return None
 *
 */
void mpuCalcLinearAccelAndroid(long quat[], s16 s16Accel[], float fLinearAcc[], u16 accS){

  float q0, q1, q2, q3;
  float f2q;
  float f2q0q0, f2q0q1, f2q0q2, f2q0q3;
  float f2q1q1, f2q1q2, f2q1q3;
  float f2q2q2, f2q2q3;
  float f2q3q3;
  float R[3][3];
  float v[3];
  int i, j;

  q0 = quat[0] / q30;
  q1 = quat[1] / q30;
  q2 = quat[2] / q30;
  q3 = quat[3] / q30;

  // set f2q to 2*q0 and calculate products
  f2q = 2.0F * q0;
  f2q0q0 = f2q * q0;
  f2q0q1 = f2q * q1;
  f2q0q2 = f2q * q2;
  f2q0q3 = f2q * q3;
  // set f2q to 2*q1 and calculate products
  f2q = 2.0F * q1;
  f2q1q1 = f2q * q1;
  f2q1q2 = f2q * q2;
  f2q1q3 = f2q * q3;
  // set f2q to 2*q2 and calculate products
  f2q = 2.0F * q2;
  f2q2q2 = f2q * q2;
  f2q2q3 = f2q * q3;
  f2q3q3 = 2.0F * q3 * q3;

  // calculate the rotation matrix assuming the quaternion is normalized
  R[0][0] = f2q0q0 + f2q1q1 - 1.0F;
  R[0][1] = f2q1q2 + f2q0q3;
  R[0][2] = f2q1q3 - f2q0q2;
  R[1][0] = f2q1q2 - f2q0q3;
  R[1][1] = f2q0q0 + f2q2q2 - 1.0F;
  R[1][2] = f2q2q3 + f2q0q1;
  R[2][0] = f2q1q3 + f2q0q2;
  R[2][1] = f2q2q3 - f2q0q1;
  R[2][2] = f2q0q0 + f2q3q3 - 1.0F;

  // code to g
  for(i = 0; i < 3; ++i)
    v[i] = (float)s16Accel[i] / (float)accS;

  // transform from sensor to global coordinate
  for(i = 0; i < 3; ++i){

    fLinearAcc[i] = 0.f;

    for(j = 0; j < 3; ++j)
      fLinearAcc[i] += R[i][j]*v[j];
  }

  // remove the gravity
  fLinearAcc[2] -= 1.f;

  return;
}
