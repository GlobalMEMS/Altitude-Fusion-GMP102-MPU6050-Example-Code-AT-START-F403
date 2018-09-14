/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : mpu6050.c
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

/*! @file mpu6050.c
 *  @brief MPU6050 I2C API
 *  @author Joseph FC Tseng
 */

#include "stdio.h"
#include "mpu6050.h"

static bus_support_t* pBus_support = 0;

/*!
 * @brief Do nothing, for compatible to eMPL
 *
 */
void MPU_IIC_Init(void){
  //empty
}

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
s8 MPU_Read_Len(u8 devAddr, u8 u8Addr, u8 u8Len, u8* pu8Data){

  s8 comRslt = -1;
  if(pBus_support == NULL){
    return -127;
  }
  else{
    comRslt = pBus_support->bus_read(pBus_support->u8DevAddr, pu8Data, u8Addr, u8Len);
    if(comRslt == 0) //success, return 0
      comRslt = 0;
    else //return the error code
      comRslt = -comRslt;
  }

  return comRslt;
}

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
s8 MPU_Write_Len(u8 devAddr, u8 u8Addr, u8 u8Len, u8* pu8Data){

  s8 comRslt = -1;
  if(pBus_support == NULL){
    return -127;
  }
  else{
    comRslt = pBus_support->bus_write(pBus_support->u8DevAddr, pu8Data, u8Addr, u8Len);
    if(comRslt == 0) //success, return 0
      comRslt = 0;
    else //return the error code
      comRslt = -comRslt;
  }

  return comRslt;
}

/*!
 * @brief MPU6050 initialize communication bus
 *
 * @param pbus Pointer to the I2C/SPI read/write bus support struct
 *
 * @return
 * @retval 0 Success
 * @retval -127 Error null bus
 *
 */
s8 mpu6050_bus_init(bus_support_t* pbus){

  //assign the I2C/SPI bus
  if(pbus == NULL)
    return -127;
  else
    pBus_support = pbus;

  return 0;
}
