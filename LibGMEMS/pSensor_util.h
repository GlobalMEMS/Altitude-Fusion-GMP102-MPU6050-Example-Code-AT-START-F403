/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : pSensor_util.h
 *
 * Date : 2018/04/12
 *
 * Usage: Pressure Sensor Utility Function
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

/*! @file pSensor_util.h
 *  @brief  Pressure sensor utility function
 *  @author Joseph FC Tseng
 */

#ifndef __PSENSOR_UTIL_H__
#define __PSENSOR_UTIL_H__

#include "at32f4xx.h"

/*!
 * @brief Set sea level reference pressure base
 *        If not set, the default value is 101325 Pa
 *
 * @param fp_Pa Sea level pressure in Pa
 *
 * @return None
 *
 */
void set_sea_level_pressure_base(float fp_Pa);

/*!
 * @brief Pressure altitude conversion
 *        See https://en.wikipedia.org/wiki/Pressure_altitude
 *
 * @param fp_Pa Calibrated pressure in Pa
 *
 * @return Altitude in m
 *
 */
float pressure2Alt(float fp_Pa);

/*!
 * @brief Estimate the baro pressure average
 * @brief This average pressure can be used to set the base
 * @brief sea level pressure.
 *
 * @param readP_fcn The data function pointer to read the raw pressure datas
 * @param readT_fcn The data function pointer to read the raw temperature datas
 * @param comp_fcn The compensation function pointer to compensate P and T
 * @param fParam[] pressure calibration parameters
 * @param p The estimated baro pressure average
 *
 * @return 0 for Success
 * @return 1 for other errors
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 */
s8 pressureAve_f(s8(*readP_fcn)(s32*),
		 s8(*readT_fcn)(s16*),
		 void (*comp_fcn)(s16, s32, float[], float*, float*),
		 float fParam[],
		 float* p);

#endif //__PSENSOR_UTIL_H__
