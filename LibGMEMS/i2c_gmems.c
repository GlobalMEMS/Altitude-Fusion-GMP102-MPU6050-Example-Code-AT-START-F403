/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : i2c_gmems.c
 *
 * Usage: I2C helper functions
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

/*! @file i2c_gmems.c
 *  @brief I2C helper functions
 *  @author Joseph FC Tseng
 */

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "at32f4xx.h"
#include "i2c_gmems.h"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static u8* I2C_pBuffer = 0;
static I2C_DIRECTION MasterDirection = Transmitter;
static u16 SlaveADDR;
static u16 DeviceOffset = 0x0;
static bool OffsetDone = false;
static I2C_STATE i2c_comm_state;
static u16 Int_NumByteToWrite = 0;
static u16 Int_NumByteToRead = 0;
static u32  sI2cTimeout = sI2C_LONG_TIMEOUT;
static uint32_t nextevent = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
 * Function Name  : I2C_WaitOperationIsCompleted
 * Description    : wait operation is completed
 * Input          : None
 * Output         : None
 * Return         : 1 if timeout on wait for COMM_DONE, 0 for normal COMM_DONE
 *******************************************************************************/
u8 I2C_WaitOperationIsCompleted(void)
{
  sI2cTimeout = sI2C_LONG_TIMEOUT;

  while(i2c_comm_state != COMM_DONE){
    if((sI2cTimeout--) == 0) return 1;
  }

  return 0;
}

/*******************************************************************************
 * Function Name  : I2C1_Init
 * Description    : Initializes I2C1 peripheral
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void  I2C1_Init(void)
{
  /******* GPIO configuration and clock enable *********/
  GPIO_InitType  GPIO_InitStructure;
  I2C_InitType  I2C_InitStructure;
  NVIC_InitType NVIC_InitStructure;
  I2Cx_peripheral_clock();
  I2C_DeInit(I2Cx);

  I2Cx_scl_pin_clock();
  I2Cx_sda_pin_clock();

  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_AFIO, ENABLE);
#ifdef I2C1_REMAP
  GPIO_PinsRemapConfig(GPIO_Remap_I2C1, ENABLE);
#endif

  GPIO_InitStructure.GPIO_Pins =  I2Cx_SCL_PIN | I2Cx_SDA_PIN;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOx, &GPIO_InitStructure);

  /*********** I2C periphral configuration **********/
  I2C_DeInit(I2Cx);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2CDevice;
  I2C_InitStructure.I2C_FmDutyCycle = I2C_FmDutyCycle_2_1;
  I2C_InitStructure.I2C_OwnAddr1 = 0xFF;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AddrMode = I2C_AddrMode_7bit;
  I2C_InitStructure.I2C_BitRate = I2C_Speed;
  I2C_Init(I2Cx, &I2C_InitStructure);

  /************** I2C NVIC configuration *************************/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
 * Function Name  : I2C1_WriteBuffer
 * Description    : Writes a block of data to the slave device on the I2C1 bus.
 * Input          : - slaveAddr : device slave address
 *                  - pBuffer : pointer to the buffer  containing the data to be
 *                    written to the device.
 *                  - WriteAddr : device's internal address to write to.
 *                  - NumByteToWrite : number of bytes to write to the device.
 * Output         : None
 * Return         : 0 if success, 1 otherwise
 *******************************************************************************/
u8 I2C1_WriteBuffer(u16 slaveAddr, u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite)
{

  I2C_pBuffer = pBuffer;
  MasterDirection = Transmitter;

  /*initialize static parameter according to input parameter*/
  SlaveADDR = slaveAddr;
  DeviceOffset = WriteAddr;
  OffsetDone = false;
  i2c_comm_state = COMM_PRE;
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  I2C_INTConfig(I2C1, I2C_INT_EVT | I2C_INT_BUF | I2C_INT_ERR, ENABLE);
  Int_NumByteToWrite = NumByteToWrite;

  sI2cTimeout = sI2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSYF)){
    if((sI2cTimeout--) == 0)  return 1;
  }
  nextevent = I2C_EVENT_MASTER_START_GENERATED;
  I2C_GenerateSTART(I2C1, ENABLE);

  return I2C_WaitOperationIsCompleted();
}

/*******************************************************************************
 * Function Name  : I2C1_ReadBuffer
 * Description    : Reads a block of data from slave device on the I2C1 bus.
 * Input          : - slaveAddr : device slave address
 *                  - pBuffer : pointer to the buffer that receives the data read
 *                    from the device.
 *                  - ReadAddr : device's internal address to read from.
 *                  - NumByteToRead : number of bytes to read from the device.
 * Output         : - pBuffer : data read from the device are stored in this buffer
 * Return         : 0 if success, 1 otherwise
 *******************************************************************************/
u8 I2C1_ReadBuffer(u16 slaveAddr, u8* pBuffer, u16 ReadAddr, u16 NumByteToRead)
{

  I2C_pBuffer = pBuffer;
  MasterDirection = Receiver;

  /*initialize static parameter according to input parameter*/
  SlaveADDR = slaveAddr;
  DeviceOffset = ReadAddr;
  OffsetDone = false;
  i2c_comm_state = COMM_PRE;
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  I2C_INTConfig(I2C1, I2C_INT_EVT | I2C_INT_BUF | I2C_INT_ERR, ENABLE);
  Int_NumByteToRead = NumByteToRead;

  sI2cTimeout = sI2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSYF)){
    if((sI2cTimeout--) == 0)  return 1;
  }
  nextevent = I2C_EVENT_MASTER_START_GENERATED;
  I2C_GenerateSTART(I2C1, ENABLE);

  return I2C_WaitOperationIsCompleted();
}

/**********************  Interrupt Service Routines	 **************************/

void i2c1_evt_handle(void)
{
  uint32_t lastevent = nextevent;

  //Check for redundant I2C_EVENT_MASTER_DATA_TRANSMITTED
  if(!I2C_CheckEvent(I2C1, lastevent)) return;

  switch (lastevent){
    /************************** Master Invoke**************************************/
  case I2C_EVENT_MASTER_START_GENERATED:        /* EV5 */
    i2c_comm_state = COMM_IN_PROCESS;
    if (MasterDirection == Receiver){
      if (!OffsetDone){
	nextevent = I2C_EVENT_MASTER_ADDRESS_WITH_TRANSMITTER;
	I2C_Send7bitAddress(I2C1, SlaveADDR, I2C_Direction_Transmit);
      }
      else{
	/* Send slave Address for read */
	nextevent = I2C_EVENT_MASTER_ADDRESS_WITH_RECEIVER;
	I2C_Send7bitAddress(I2C1, SlaveADDR, I2C_Direction_Receive);
	OffsetDone = false;
      }
    }
    else{
      /* Send slave Address for write */
      nextevent = I2C_EVENT_MASTER_ADDRESS_WITH_TRANSMITTER;
      I2C_Send7bitAddress(I2C1,  SlaveADDR,I2C_Direction_Transmit);
    }
    break;
    /********************** Master Receiver events ********************************/
  case I2C_EVENT_MASTER_ADDRESS_WITH_RECEIVER:  /* EV6 */
    nextevent = I2C_EVENT_MASTER_DATA_RECEIVED;
    if(Int_NumByteToRead == 1){
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2C1, DISABLE);
      I2C_GenerateSTOP(I2C1, ENABLE);
    }
    break;
  case I2C_EVENT_MASTER_DATA_RECEIVED:    /* EV7 */
    nextevent = I2C_EVENT_MASTER_DATA_RECEIVED;
    *I2C_pBuffer = I2C_ReceiveData(I2C1);
    I2C_pBuffer++;
    Int_NumByteToRead--;
    if(Int_NumByteToRead==1){
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2C1, DISABLE);
      I2C_GenerateSTOP(I2C1, ENABLE);
    }
    if(Int_NumByteToRead==0){
      I2C_INTConfig(I2C1, I2C_INT_EVT | I2C_INT_BUF |I2C_INT_ERR, DISABLE);
      i2c_comm_state = COMM_DONE;
    }
    break;
    /************************* Master Transmitter events **************************/
  case I2C_EVENT_MASTER_ADDRESS_WITH_TRANSMITTER:     /* EV8 just after EV6 */
    nextevent = I2C_EVENT_MASTER_DATA_TRANSMITTED;
    I2C_SendData(I2C1, DeviceOffset);
    OffsetDone = true;
    break;
  case I2C_EVENT_MASTER_DATA_TRANSMITTING:       /* EV8 I2C_EVENT_MASTER_DATA_TRANSMITTING*/
    break;
  case I2C_EVENT_MASTER_DATA_TRANSMITTED:       /* EV8-2 */
    if (MasterDirection == Transmitter){
      if(Int_NumByteToWrite == 0){
	nextevent = 0;
	i2c_comm_state = COMM_DONE;
	I2C_GenerateSTOP(I2C1, ENABLE);
      }
      else{
	nextevent = I2C_EVENT_MASTER_DATA_TRANSMITTED;
	I2C_SendData(I2C1, *I2C_pBuffer);
	I2C_pBuffer++;
	Int_NumByteToWrite--;
      }
    }
    else{
      nextevent = I2C_EVENT_MASTER_START_GENERATED;
      I2C_GenerateSTART(I2C1, ENABLE);
    }
    break;
  }
}

void i2c1_err_handle(void)
{
  if (I2C_GetFlagStatus(I2C1, I2C_FLAG_ACKFAIL)){
    if (I2C1->STS2 &0x01){ //real fail
      I2C_GenerateSTOP(I2C1, ENABLE);
      i2c_comm_state = COMM_EXIT;
    }
    I2C_ClearFlag(I2C1, I2C_FLAG_ACKFAIL);
  }

  if (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSERR)){
    if (I2C1->STS2 &0x01){
      I2C_GenerateSTOP(I2C1, ENABLE);
      i2c_comm_state = COMM_EXIT;
    }
    I2C_ClearFlag(I2C1, I2C_FLAG_BUSERR);
  }
}





