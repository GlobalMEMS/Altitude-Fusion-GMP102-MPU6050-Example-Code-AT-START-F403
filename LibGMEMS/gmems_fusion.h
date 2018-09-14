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

/*! @file gmems_fusion.h
 *  @brief  Multi-sensor fusion functions
 *  @author Joseph FC Tseng
 */

#ifndef __GMEMS_FUSION_H__
#define __GMEMS_FUSION_H__

#include "at32f4xx.h"

typedef struct
{
  float Hhat;     // estimated altitude
  float Vhat;     // estimated vertical velocity
  float eHi;      // integrated altitude error
  s8 resetflag;   // flag to request re-initialization on next pass
} altitudeStateType;

/*!
 * @brief Initialize the altitude state variable
 *
 * @param sv altitude fusion state variable
 *
 * @return None
 *
 */
void initAltitude(altitudeStateType* sv);

/*!
 * @brief Estimate altitude by complementary filtering of
 * @bried pressure altitude and vertical acceleration
 *
 * @param acc Vertical acceleration, m/s^2
 * @param hp  Pressure altitude, m
 * @param dt  Time step, sec
 * @param sv  Altitude fusion state variable
 *
 * @return Altitude in m
 *
 */
float altitudeByCompFilt(float acc, float hp, float dt, altitudeStateType* sv);

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
void quat2EulerAndroid(float quat[], float *phi, float *theta, float *psi);

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
void mpuQuat2EulerAndroid(long quat[], float *phi, float *theta, float *psi);

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
void mpuCalcLinearAccelAndroid(long quat[], s16 s16Accel[], float fLinearAcc[], u16 accS);

#endif //__GMEMS_FUSION_H__
