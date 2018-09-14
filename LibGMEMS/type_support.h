/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : type_support.h
 *
 * Date : 2018/04/03
 *
 * Usage: Define various support type
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

#ifndef __TYPE_SUPPORT_H__
#define __TYPE_SUPPORT_H__

#include <stdint.h>
#include "at32f4xx.h"

typedef union {
  struct{
    s32 x;
    s32 y;
    s32 z;
    s32 t;
  } u;
  s32 v[4];
} raw_data_xyzt_t;

typedef union {
  struct{
    float x;
    float y;
    float z;
    float t;
  } u;
  float v[4];
} float_xyzt_t;

typedef enum {X_AXIS = 1, Y_AXIS = 2, Z_AXIS = 4} m_axis_t;

#endif //__TYPE_SUPPORT_H__
