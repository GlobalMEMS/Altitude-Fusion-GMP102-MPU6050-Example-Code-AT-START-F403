/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : pSensor_util.c
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

/*! @file pSensor_util.c
 *  @brief  Pressure sensor utility function
 *  @author Joseph FC Tseng
 */

#include <stdio.h>
#include <math.h>
#include "delay.h"
#include "pSensor_util.h"

#define DATA_AVE_NUM 32
#define POLLING_INTERVAL_MS 20
#define DELAY_MS(dt) delay_ms(dt)

static float fp_base_sea_level_Pa = 101325.f;

/*!
 * @brief Set sea level reference pressure base
 *        If not set, the default value is 101325 Pa
 *
 * @param fp_Pa Sea level pressure in Pa
 *
 * @return None
 *
 */
void set_sea_level_pressure_base(float fp_Pa){

  fp_base_sea_level_Pa = fp_Pa;

}

/*!
 * @brief Pressure altitude conversion
 *        See https://en.wikipedia.org/wiki/Pressure_altitude
 *
 * @param fp_Pa Calibrated pressure in Pa
 *
 * @return Altitude in m
 *
 */
float pressure2Alt(float fp_Pa){

  return 44307.694f * (1.0f - pow((fp_Pa / fp_base_sea_level_Pa), 0.190284f));

}

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
		 float* p)
{

  s8 i, comRslt = -1;
  float fT_Celsius, fP_Pa, v;
  s16 s16T;
  s32 s32P;

  //make sure the function point to somewhere
  if(readP_fcn == NULL || readT_fcn == NULL || comp_fcn == NULL){
    comRslt = 1;
    goto EXIT;
  }

  v = 0.f;
  //get the pressure readings
  for(i = 0; i < DATA_AVE_NUM; ++i){

    comRslt = readP_fcn(&s32P);
    if(comRslt < 0) goto EXIT;

    comRslt = readT_fcn(&s16T);
    if(comRslt < 0) goto EXIT;

    comp_fcn(s16T, s32P, fParam, &fT_Celsius, &fP_Pa);

    //accumulate the readout
    v += fP_Pa;

    DELAY_MS(POLLING_INTERVAL_MS); //delay between data polling
  }

  //Return 0 for success
  comRslt = 0;

  //The average
  *p = v / DATA_AVE_NUM;

 EXIT:
  return comRslt;
}
