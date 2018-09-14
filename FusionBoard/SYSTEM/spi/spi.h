#ifndef __SPI_H
#define __SPI_H
#include "stdio.h"
#include "sys.h"


typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

#define BufferSize 32

#define SPIy                   SPI1
#define SPIy_CLK               RCC_APB2PERIPH_SPI1
#define SPIy_GPIO              GPIOA
#define SPIy_GPIO_CLK          RCC_APB2PERIPH_GPIOA
#define SPIy_PIN_SCK           GPIO_Pins_5
#define SPIy_PIN_MISO          GPIO_Pins_6
#define SPIy_PIN_MOSI          GPIO_Pins_7

#define SPIz                    SPI2
#define SPIz_CLK                RCC_APB1PERIPH_SPI2
#define SPIz_GPIO               GPIOB
#define SPIz_GPIO_CLK           RCC_APB2PERIPH_GPIOB
#define SPIz_PIN_SCK            GPIO_Pins_13
#define SPIz_PIN_MISO           GPIO_Pins_14
#define SPIz_PIN_MOSI           GPIO_Pins_15

void RCC_Configuration(void);
void SPI_PinConfig(uint16_t SPIy_Mode, uint16_t SPIz_Mode);
void SPI_Config(void);
uint8_t FullDuplex_Test(void);
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
#endif


