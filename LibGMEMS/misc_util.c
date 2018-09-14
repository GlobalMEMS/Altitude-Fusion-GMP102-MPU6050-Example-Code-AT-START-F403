/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : misc_util.c
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

/*! @file misc_util.c
 *  @brief Miscellaneous utility Function
 *  @author Joseph FC Tseng
 */

#include "misc_util.h"
#include "math.h"

int16_t _do_coord_rotate(const GMEMS_PATNO pat, float_xyzt_t *vec)
{
  float tmp;
  switch (pat) {
    /* Obverse */
  case PAT1:
    /* This is Android default */
    break;
  case PAT2:
    tmp = vec->u.x;
    vec->u.x = vec->u.y;
    vec->u.y = -tmp;
    break;
  case PAT3:
    vec->u.x = -(vec->u.x);
    vec->u.y = -(vec->u.y);
    break;
  case PAT4:
    tmp = vec->u.x;
    vec->u.x = -(vec->u.y);
    vec->u.y = tmp;
    break;
    /* Reverse */
  case PAT5:
    vec->u.x = -(vec->u.x);
    vec->u.z = -(vec->u.z);
    break;
  case PAT6:
    tmp = vec->u.x;
    vec->u.x = vec->u.y;
    vec->u.y = tmp;
    vec->u.z = -(vec->u.z);
    break;
  case PAT7:
    vec->u.y = -(vec->u.y);
    vec->u.z = -(vec->u.z);
    break;
  case PAT8:
    tmp = vec->u.x;
    vec->u.x = -(vec->u.y);
    vec->u.y = -tmp;
    vec->u.z = -(vec->u.z);
    break;
  default:
    return 0;
  }

  return 1;
}

/*!
 * @brief Rotate the axis according to layout pattern
 *
 * @param[in] pat Layout pattern number
 * @param[in/out] pi32Data pointer to raw int32_t data vectors
 *
 * @return 1 for Success
 * @return 0 for Error
 */
int16_t coord_rotate(const GMEMS_PATNO pat, raw_data_xyzt_t *pi32Data)
{
  float_xyzt_t vecTmp;
  int i;
  int16_t res;

  for(i = 0; i < 3; ++i)
    vecTmp.v[i] = pi32Data->v[i];

  res = _do_coord_rotate(pat, &vecTmp);

  for(i = 0; i < 3; ++i)
    pi32Data->v[i] = (int32_t) vecTmp.v[i];

  return res;
}


/*!
 * @brief Rotate the axis according to layout pattern
 *
 * @param[in] pat Layout pattern number
 * @param[in/out] pfData pointer to raw float data vectors
 *
 * @return 1 for Success
 * @return 0 for Error
 */
int16_t coord_rotate_f(const GMEMS_PATNO pat, float_xyzt_t *pfData)
{
  int16_t res;

  res = _do_coord_rotate(pat, pfData);

  return res;

}

/*!
 * @brief Initialize the basic statistics state variable
 *
 * @param sv basic statistics state variable
 *
 * @return None
 *
 */
void initBasicStats(basicStatsType* sv){
  sv->resetflag = 1;
}

/*!
 * @brief Iterative algorithm to calculate the basic statistics
 * @brief avg_1 = x_1, var_1 = 0
 * @brief At step n+1 with known ave_n and var_n
 * @brief Let K = 1/(n+1), K1 = 1/n
 * @brief avg_n+1 = ave_n + K*(x_n+1 - ave_n)
 * @brief var_n+1 = ((1-K1)*var_n + K * (x_n+1 - ave_n)^2)
 *
 * @param x new state value
 * @param sv basic statistics state variable
 *
 * @return None
 *
 */
void calcBasicStats(float x, basicStatsType* sv){

  float K, K1, e;

  if(sv->resetflag){
    sv->ave = x;
    sv->var = 0;
    sv->std = 0;
    sv->max = x;
    sv->min = x;
    sv->n = 1;
    sv->resetflag = 0;
    return;
  }

  K1 = 1.f / sv->n;
  sv->n += 1;
  K = 1.f / sv->n;
  e = x - sv->ave;

  sv->ave += K * e;
  sv->var *= (1 - K1);
  sv->var += K*e*e;
  sv->std = sqrt(sv->var);

  if(x > sv->max) sv->max = x;
  if(x < sv->min) sv->min = x;

}
