/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : misc_util.h
 *
 * Usage: Miscellaneous utility Function
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

/*! @file misc_util.h
 *  @brief Miscellaneous utility Function
 *  @author Joseph FC Tseng
 */

#ifndef __MISC_UTIL_H__
#define __MISC_UTIL_H__

#include "type_support.h"

/***** Layout pattern ********************************************************/
typedef enum {
  PAT_INVALID = 0,
  PAT1,
  PAT2,
  PAT3,
  PAT4,
  PAT5,
  PAT6,
  PAT7,
  PAT8
} GMEMS_PATNO;

/***** Basic statistics ********************************************************/
typedef struct
{
  float ave;       // average
  float std;       // stdev
  float var;       // variance
  float max;       // maximum value
  float min;       // minmum value
  s32 n;           // sample number
  s8 resetflag;    // flag to request re-initialization on next pass
} basicStatsType;

/*!
 * @brief Rotate the axis according to layout pattern
 *
 * @param[in] pat Layout pattern number
 * @param[in/out] pi32Data pointer to raw int32_t data vectors
 *
 * @return 1 for Success
 * @return 0 for Error
 */
int16_t coord_rotate(const GMEMS_PATNO pat, raw_data_xyzt_t *pi32Data);

/*!
 * @brief Rotate the axis according to layout pattern
 *
 * @param[in] pat Layout pattern number
 * @param[in/out] pfData pointer to raw float data vectors
 *
 * @return 1 for Success
 * @return 0 for Error
 */
int16_t coord_rotate_f(const GMEMS_PATNO pat, float_xyzt_t *pfData);

/*!
 * @brief Initialize the basic statistics state variable
 *
 * @param sv basic statistics state variable
 *
 * @return None
 *
 */
void initBasicStats(basicStatsType* sv);

/*!
 * @brief Iterative algorithm to calculate the basic statistics
 * @brief avg_1 = x_1, var_1 = 0
 * @brief At step n+1 with ave_n and var_n available
 * @brief K = 1/(n+1)
 * @brief avg_n+1 = ave_n + K*(x_n+1 - ave_n)
 * @brief var_n+1 = (1-K)*(var_n + K * (x_n+1 - ave_n)^2)
 *
 * @param x new state value
 * @param sv basic statistics state variable
 *
 * @return None
 *
 */
void calcBasicStats(float x, basicStatsType* sv);

#endif //__MISC_UTIL_H__
