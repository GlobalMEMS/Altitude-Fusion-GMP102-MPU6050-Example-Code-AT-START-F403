#include "sys.h"
#include "spi.h"


uint8_t SPIy_Buffer_Tx[BufferSize] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                      0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
                                      0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
                                      0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C,
                                      0x1D, 0x1E, 0x1F, 0x20
                                     };
uint8_t SPIz_Buffer_Tx[BufferSize] = {0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
                                      0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E,
                                      0x5F, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65,
                                      0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C,
                                      0x6D, 0x6E, 0x6F, 0x70
                                     };
uint8_t SPIy_Buffer_Rx[BufferSize], SPIz_Buffer_Rx[BufferSize];
__IO uint8_t TxIdx = 0, RxIdx = 0, k = 0;

volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = FAILED;
volatile TestStatus TransferStatus3 = FAILED, TransferStatus4 = FAILED;

SPI_InitType   SPI_InitStructure;

/**
  * @brief  Configures the SPIy and SPIz.
  * @param  None
  * @retval None
  */
void SPI_Config(void)
{
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();

    /* 1st phase: SPIy Master and SPIz Slave */
    /* GPIO configuration ------------------------------------------------------*/
    SPI_PinConfig(SPI_MODE_MASTER, SPI_MODE_SLAVE);

    /* SPIy Config -------------------------------------------------------------*/
    SPI_InitStructure.SPI_TransMode = SPI_TRANSMODE_FULLDUPLEX;
    SPI_InitStructure.SPI_Mode = SPI_MODE_MASTER;
    SPI_InitStructure.SPI_FrameSize = SPI_FRAMESIZE_8BIT;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_LOW;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2EDGE;
    SPI_InitStructure.SPI_NSSSEL = SPI_NSSSEL_SOFT;
    SPI_InitStructure.SPI_MCLKP = SPI_MCLKP_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FIRSTBIT_LSB;
    SPI_InitStructure.SPI_CPOLY = 7;
    SPI_Init(SPIy, &SPI_InitStructure);

    /* SPIz Config -------------------------------------------------------------*/
    SPI_InitStructure.SPI_Mode = SPI_MODE_SLAVE;
    SPI_Init(SPIz, &SPI_InitStructure);
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
    /* Enable peripheral clocks --------------------------------------------------*/

    /* Enable SPIy clock and GPIO clock for SPIy and SPIz */
    RCC_APB2PeriphClockCmd(SPIy_GPIO_CLK | SPIz_GPIO_CLK | SPIy_CLK | RCC_APB2PERIPH_GPIOD, ENABLE);

    /* Enable SPIz Periph clock */
    RCC_APB1PeriphClockCmd(SPIz_CLK, ENABLE);
}

/**
  * @brief  Configures the different SPIy and SPIz GPIO ports.
  * @param  SPIy_Mode: Specifies the SPIy operating mode.
  *            This parameter can be:
  *              -  SPIy_Mode_Master
  *              -  SPIy_Mode_Slave
  * @param  SPIz_Mode: Specifies the SPIz operating mode.
  *            This parameter can be:
  *              -  SPIz_Mode_Master
  *              -  SPIz_Mode_Slave
  * @retval None
  */
void SPI_PinConfig(uint16_t SPIy_Mode, uint16_t SPIz_Mode)
{
    GPIO_InitType GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(SPIy_GPIO_CLK | SPIz_GPIO_CLK, ENABLE);

    /* Configure SPIy pins: SCK, MISO and MOSI ---------------------------------*/
    GPIO_InitStructure.GPIO_Pins = SPIy_PIN_SCK | SPIy_PIN_MOSI;
    GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;

    if(SPIy_Mode == SPI_MODE_MASTER)
    {
        /* Configure SCK and MOSI pins as Alternate Function Push Pull */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    }
    else
    {
        /* Configure SCK and MOSI pins as Input Floating */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }

    GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pins = SPIy_PIN_MISO;

    if(SPIy_Mode == SPI_MODE_MASTER)
    {
        /* Configure MISO pin as Input Floating  */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    else
    {
        /* Configure MISO pin as Alternate Function Push Pull */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    }

    GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

    /* Configure SPIz pins: SCK, MISO and MOSI ---------------------------------*/
    GPIO_InitStructure.GPIO_Pins = SPIz_PIN_SCK | SPIz_PIN_MOSI;

    if(SPIz_Mode == SPI_MODE_SLAVE)
    {
        /* Configure SCK and MOSI pins as Input Floating */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    else
    {
        /* Configure SCK and MOSI pins as Alternate Function Push Pull */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    }

    GPIO_Init(SPIz_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pins = SPIz_PIN_MISO;

    if(SPIz_Mode == SPI_MODE_SLAVE)
    {
        /* Configure MISO pin as Alternate Function Push Pull */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    }
    else
    {
        /* Configure MISO pin as Input Floating  */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }

    GPIO_Init(SPIz_GPIO, &GPIO_InitStructure);
}

/**
  * @brief  Full Duplex Test.
  * @param  None
  * @retval 1: Test passed
  *         0: Test failed
  */
uint8_t FullDuplex_Test(void)
{
    /* Enable SPIy */
    SPI_Enable(SPIy, ENABLE);
    /* Enable SPIz */
    SPI_Enable(SPIz, ENABLE);

    /* Transfer procedure */
    while (TxIdx < BufferSize)
    {
        /* Wait for SPIy Tx buffer empty */
        while (SPI_I2S_GetFlagStatus(SPIy, SPI_I2S_FLAG_TE) == RESET);

        /* Send SPIz data */
        SPI_I2S_TxData(SPIz, SPIz_Buffer_Tx[TxIdx]);
        /* Send SPIy data */
        SPI_I2S_TxData(SPIy, SPIy_Buffer_Tx[TxIdx++]);

        /* Wait for SPIz data reception */
        while (SPI_I2S_GetFlagStatus(SPIz, SPI_I2S_FLAG_RNE) == RESET);

        /* Read SPIz received data */
        SPIz_Buffer_Rx[RxIdx] = SPI_I2S_RxData(SPIz);

        /* Wait for SPIy data reception */
        while (SPI_I2S_GetFlagStatus(SPIy, SPI_I2S_FLAG_RNE) == RESET);

        /* Read SPIy received data */
        SPIy_Buffer_Rx[RxIdx++] = SPI_I2S_RxData(SPIy);
    }

    /* Check the correctness of written dada */
    TransferStatus1 = Buffercmp(SPIz_Buffer_Rx, SPIy_Buffer_Tx, BufferSize);
    TransferStatus2 = Buffercmp(SPIy_Buffer_Rx, SPIz_Buffer_Tx, BufferSize);
    /* TransferStatus1, TransferStatus2 = PASSED, if the transmitted and received data
     are equal */
    /* TransferStatus1, TransferStatus2 = FAILED, if the transmitted and received data
     are different */

    /* 2nd phase: SPIy Slave and SPIz Master */
    /* GPIO configuration ------------------------------------------------------*/
    SPI_PinConfig(SPI_MODE_SLAVE , SPI_MODE_MASTER);

    /* SPIy Re-configuration ---------------------------------------------------*/
    SPI_InitStructure.SPI_Mode = SPI_MODE_SLAVE;
    SPI_Init(SPIy, &SPI_InitStructure);

    /* SPIz Re-configuration ---------------------------------------------------*/
    SPI_InitStructure.SPI_Mode = SPI_MODE_MASTER;
    SPI_Init(SPIz, &SPI_InitStructure);

    /* Reset TxIdx, RxIdx indexes and receive tables values */
    TxIdx = 0;
    RxIdx = 0;

    for (k = 0; k < BufferSize; k++)
    {
        SPIz_Buffer_Rx[k] = 0;
    }

    for (k = 0; k < BufferSize; k++)
    {
        SPIy_Buffer_Rx[k] = 0;
    }

    /* Enable SPIy */
    SPI_Enable(SPIz, ENABLE);
    /* Enable SPIz */
    SPI_Enable(SPIy, ENABLE);

    /* Transfer procedure */
    while (TxIdx < BufferSize)
    {
        /* Wait for SPIz Tx buffer empty */
        while (SPI_I2S_GetFlagStatus(SPIz, SPI_I2S_FLAG_TE) == RESET);

        /* Send SPIy data */
        SPI_I2S_TxData(SPIy, SPIy_Buffer_Tx[TxIdx]);
        /* Send SPIz data */
        SPI_I2S_TxData(SPIz, SPIz_Buffer_Tx[TxIdx++]);

        /* Wait for SPIy data reception */
        while (SPI_I2S_GetFlagStatus(SPIy, SPI_I2S_FLAG_RNE) == RESET);

        /* Read SPIy received data */
        SPIy_Buffer_Rx[RxIdx] = SPI_I2S_RxData(SPIy);

        /* Wait for SPIz data reception */
        while (SPI_I2S_GetFlagStatus(SPIz, SPI_I2S_FLAG_RNE) == RESET);

        /* Read SPIz received data */
        SPIz_Buffer_Rx[RxIdx++] = SPI_I2S_RxData(SPIz);
    }

    /* Check the correctness of written dada */
    TransferStatus3 = Buffercmp(SPIz_Buffer_Rx, SPIy_Buffer_Tx, BufferSize);
    TransferStatus4 = Buffercmp(SPIy_Buffer_Rx, SPIz_Buffer_Tx, BufferSize);

    if ((TransferStatus1 == PASSED) && (TransferStatus2 == PASSED) && \
            (TransferStatus3 == PASSED) && (TransferStatus4 == PASSED))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *         FAILED: pBuffer1 differs from pBuffer2
  */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            return FAILED;
        }

        pBuffer1++;
        pBuffer2++;
    }

    return PASSED;
}
