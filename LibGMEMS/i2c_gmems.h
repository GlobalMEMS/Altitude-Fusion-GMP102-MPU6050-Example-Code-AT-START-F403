/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : i2c_gmems.h
 *
 * Usage: I2C helper functions header file
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

/*! @file i2c_gmems.h
 *  @brief I2C helper functions
 *  @author Joseph FC Tseng
 */

#ifndef __I2C_GMEMS_H__
#define __I2C_GMEMS_H__

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "at32f4xx.h"

/* Private macro -------------------------------------------------------------*/
#define I2C1_REMAP
#define I2Cx    I2C1

#ifdef I2C1_REMAP
#define I2Cx_SCL_PIN    GPIO_Pins_8
#define I2Cx_SDA_PIN    GPIO_Pins_9
#define GPIOx           GPIOB
#else
#define I2Cx_SCL_PIN    GPIO_Pins_6
#define I2Cx_SDA_PIN    GPIO_Pins_7
#define GPIOx           GPIOB
#endif
#define I2Cx_peripheral_clock()  RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_I2C1, ENABLE)
#define I2Cx_scl_pin_clock()     RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE)
#define I2Cx_sda_pin_clock()     RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE)

#define I2C_Speed         400000

#define sI2C_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define sI2C_LONG_TIMEOUT         ((uint32_t)(100 * sI2C_FLAG_TIMEOUT))

#define TX_PIN_NUMBER GPIO_Pins_9
#define RX_PIN_NUMBER GPIO_Pins_10
#define TXRX_GPIOx    GPIOA

/* Exported types ------------------------------------------------------------*/
typedef enum i2c_state
  {
    COMM_DONE  = 0,  // done successfully
    COMM_PRE = 1,
    COMM_IN_PROCESS = 2,
    COMM_EXIT = 3 // exit since failure
  } I2C_STATE;

typedef enum i2c_direction
  {
    Transmitter = 0x00,
    Receiver = 0x01
  } I2C_DIRECTION;

void I2C1_Init(void);
u8 I2C1_ReadBuffer(u16 slaveAddr, u8* pBuffer, u16 ReadAddr, u16 NumByteToRead);
u8 I2C1_WriteBuffer(u16 slaveAddr, u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite);
void i2c1_evt_handle(void);
void i2c1_err_handle(void);

#endif  //__I2C_GMEMS_H__
