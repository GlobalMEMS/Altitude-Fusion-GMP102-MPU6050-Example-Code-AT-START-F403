/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : mpu6050.h
 *
 * Usage: MPU6050 I2C API
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

/*! @file mpu6050_i2c.h
 *  @brief MPU6050 I2C API
 *  @author Joseph FC Tseng
 */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "bus_support.h"
#include "type_support.h"

#define MPU6050_ADDRESS_AD0_LOW          0x68 // address pin low (GND)
#define MPU6050_ADDRESS_AD0_HIGH         0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_7BIT_I2C_ADDR    MPU6050_ADDRESS_AD0_LOW
#define MPU6050_DEFAULT_8BIT_I2C_ADDR    ((MPU6050_DEFAULT_7BIT_I2C_ADDR)<<1)

/*!
 * @brief Do nothing, for compatible to eMPL
 *
 */
void MPU_IIC_Init(void);

/*!
 * @brief Read multiple data from the starting regsiter address
 *
 * @param devAddr, device slave address, not used, only for compatible to eMPL
 * @param u8Addr Starting register address
 * @param u8Len Number of bytes to read
 * @param pu8Data The data array of values read
 *
 * @retval 0 Success
 * @retval -127 Error null bus
 * @retval -1   Bus communication error
 *
 */
s8 MPU_Read_Len(u8 devAddr, u8 u8Addr, u8 u8Len, u8* pu8Data);

/*!
 * @brief MPU Write multiple data to the starting regsiter address
 *
 * @param devAddr, device slave address, not used, only for compatible to eMPL
 * @param u8Addr Starting register address
 * @param u8Len Number of bytes to write
 * @param pu8Data The data array of values to write
 *
 * @retval 0 Success
 * @retval -127 Error null bus
 * @retval -1   Bus communication error
 *
 */
s8 MPU_Write_Len(u8 devAddr, u8 u8Addr, u8 u8Len, u8* pu8Data);


/*!
 * @brief MPU6050 initialize communication bus
 *
 * @param pbus Pointer to the I2C/SPI read/write bus support struct
 *
 * @return Result from bus communication function
 * @retval 0 Success
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 mpu6050_bus_init(bus_support_t* pbus);

#endif // __MPU6050_H__
