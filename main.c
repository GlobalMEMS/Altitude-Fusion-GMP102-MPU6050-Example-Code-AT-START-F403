/*
 *
 ****************************************************************************
 * Copyright (C) 2018 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : main.c
 *
 * Usage: main function
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

/*! @file main.c
 *  @brief main program
 *  @author Joseph FC Tseng
 */

/* Includes ------------------------------------------------------------------*/
#include "i2c_gmems.h"
#include "mpu6050.h"
#include "gmp102.h"
#include "Lcd_Driver.h"
#include "GUI.h"
#include "usart.h"
#include "delay.h"
#include "key.h"
#include "string.h"
#include "math.h"
#include "gmems_fusion.h"  //altitude fusion
#include "misc_util.h"
#include "pSensor_util.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/* Private macro -------------------------------------------------------------*/
#define LED2_GPIO_PIN    GPIO_Pins_13
#define LED3_GPIO_PIN    GPIO_Pins_14
#define LED4_GPIO_PIN    GPIO_Pins_15
#define LED_GPIO_PORT    GPIOD
#define SMPLRT_TMR_TICK_FREQ_kHZ      (5)               // Sensor sampling timer tick@5kHz
#define SENSOR_SAMPLING_RATE_HZ       (200)        // Sensor sampling rate, Hz
#define DMP_DATA_RATE_HZ              (25)              // DMP data rate 25Hz DEFAULT_MPU_HZ
#define OUTPUT_PRINTOUT_RATE_HZ       (1)               // Output printout rate
#define OUTPUT_PRINTOUT_OSR_PRESCALER (1)               // Real output printout rate=Output printout rate/prescaler
#define MPU6050_GYRO_FS_DEFAULT       250               // MPU6050 gyro fr +-250 dps
#define MPU6050_ACC_FS_DEFAULT        2                 // MPU6050 accel fr +-2g
#define GMP102_OSR_DEFAULT            GMP102_P_OSR_8192  //Max data rate 85Hz

/* global variables ---------------------------------------------------------*/
u8 ui8PeriodicMeasureFlag = 0;

/* Private variables ---------------------------------------------------------*/
static signed char gyro_orientation[9] = { -1,  0, 0,
                                            0, -1, 0,
                                            0,  0, 1};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  GPIO Initialize For LED.
 * @param  None
 * @retval None
 */
void LED_Init(void)
{
  GPIO_InitType GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOD, ENABLE);

  /*PD13->LED2 PD14->LED3 PD15->LED4*/
  GPIO_InitStructure.GPIO_Pins = LED2_GPIO_PIN | LED3_GPIO_PIN | LED4_GPIO_PIN;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP;
  GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);

  GPIO_SetBits(LED_GPIO_PORT, LED2_GPIO_PIN);
  GPIO_SetBits(LED_GPIO_PORT, LED3_GPIO_PIN);
  GPIO_SetBits(LED_GPIO_PORT, LED4_GPIO_PIN);

}

/**
 * @brief  Configures the nested vectored interrupt controller.
 * @param  None
 * @retval None
 */
void NVIC_Configuration(void)
{
  NVIC_InitType NVIC_InitStructure;

  /* Enable the TMR6 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TMR6_GLOBAL_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void TMR6Init(){

  RCC_ClockType RccClkSource;
  TMR_TimerBaseInitType  TIM_TimeBaseStructure;

  /* TMR6 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR6, ENABLE);

  /* Init the TMR6 configuration */
  RCC_GetClocksFreq(&RccClkSource);
  TIM_TimeBaseStructure.TMR_Period = 1000 * SMPLRT_TMR_TICK_FREQ_kHZ / DMP_DATA_RATE_HZ - 1;
  TIM_TimeBaseStructure.TMR_DIV = RccClkSource.AHBCLK_Freq / SMPLRT_TMR_TICK_FREQ_kHZ / 1000 - 1;
  TIM_TimeBaseStructure.TMR_ClockDivision = 0;
  TIM_TimeBaseStructure.TMR_CounterMode = TMR_CounterDIR_Down;
  TMR_TimeBaseInit(TMR6, &TIM_TimeBaseStructure);

  /* Enable TMR6 interrupt */
  TMR_INTConfig(TMR6, TMR_INT_Overflow, ENABLE);

  /* TMR6 Enable */
  TMR_Cmd(TMR6, ENABLE);
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
    {}
}

#endif

const u16 RESOLUTION_X = 128;
const u16 RESOLUTION_Y = 120;
const u16 FONT_HEIGHT = 16;
const u16 LINE_HEIGHT = FONT_HEIGHT + 2;
const u16 MAX_DISPLAY_ITEM = 9;
void showMsg(u16 x, u16 line, u8* str, u16 color, u8 reDraw){

  int i;
  char* subStr;

  if(reDraw) Lcd_Clear(GRAY0);

  subStr = strtok((char*)str, "\n");

  for(i = line; subStr; ++i){
    Gui_DrawFont_GBK16(x, LINE_HEIGHT * i, color, GRAY0, (u8*)subStr);
    subStr = strtok(NULL, "\n");
  }
}

void floatCatToStr(float fIn, u8 precision, u8* outStr){

  s32 i = 0;
  float fTmp;
  s32 s32Dec, s32Dig;

  if(fIn < 0){
    fIn = -fIn;
    strcat((char*)outStr, "-");
  }

  s32Dec = (s32)fIn;
  fTmp = fIn - s32Dec;
  for(i = 0; i < precision; ++i)
    fTmp *= 10;
  s32Dig = (s32)(fTmp + 0.5f);

  itoa(s32Dec, &outStr[strlen((const char*)outStr)]);
  strcat((char*)outStr, ".");

  fTmp = 1;
  for(i = 0; i < precision; ++i)
    fTmp *= 10;
  for(i = 0; i < precision; ++i){
    fTmp /= 10;
    if(s32Dig < fTmp){
      strcat((char*)outStr, "0");
    }
    else{
      itoa(s32Dig, &outStr[strlen((const char*)outStr)]);
      break;
    }
  }
}

altitudeStateType thisSV_alt;
altitudeStateType thisSV_altKalman;

/**
 * @brief   Main program
 * @param  None
 * @retval None
 */
int main(void)
{
  const float algDt = 1.f / DMP_DATA_RATE_HZ; //delta t, sec
  const s32 PRINTOUT_OSR = (s32)(((float)DMP_DATA_RATE_HZ) * OUTPUT_PRINTOUT_OSR_PRESCALER / ((float)OUTPUT_PRINTOUT_RATE_HZ)  + 0.5f);
  float gyroS;
  u16 accS;
  bus_support_t gmp102_bus, mpu6050_bus;
  s16 s16T;
  s32 s32P;
  s32 i, printIcounter = 0;
  float fCalibParam[GMP102_CALIBRATION_PARAMETER_COUNT], fT_Celsius, fP_Pa, fBaroAlt_m, fAlt_m, fAltKalman_m, fTmp;
  u8 str[64];
  basicStatsType baroAltStats, fusAltStats, kalmanAltStats;
  s32 i32Res;
  long lGyro[3], lAccel[3], lGyro_off[3], lAccel_off[3], quat[4];
  s16 s16Gyro[3], s16Accel[3], sensors;
  unsigned long timestamp;
  unsigned char more;
  float fLinearAcc[3];

  /* NVIC configuration */
  NVIC_Configuration();

  /* User LED initialization */
  LED_Init();

  /* I2C1 initialization */
  I2C1_Init();

  /* TMR6 initialization */
  TMR6Init();

  /* Init Key */
  KEY_Init();

  /* Initialize the LCD */
  uart_init(19200);
  delay_init();
  Lcd_Init();

  /* GMP102 I2C bus setup */
  bus_init_I2C1(&gmp102_bus, GMP102_8BIT_I2C_ADDR);  //Initialize bus support to I2C1
  gmp102_bus_init(&gmp102_bus);  //Initailze GMP102 bus to I2C1

  /* MPU6050 I2C bus setup */
  bus_init_I2C1(&mpu6050_bus, MPU6050_DEFAULT_8BIT_I2C_ADDR);  //Initialize bus support to I2C1
  mpu6050_bus_init(&mpu6050_bus);  //Initailze MPU6050 bus to I2C1

  /* GMP102 Initialization */
  gmp102_soft_reset();
  delay_ms(100);
  gmp102_get_calibration_param(fCalibParam);
  gmp102_initialization();
  gmp102_set_P_OSR(GMP102_OSR_DEFAULT);

  /* MPU6050 Initialization */
  mpu_init();
  mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL); //
  mpu_set_sample_rate(SENSOR_SAMPLING_RATE_HZ); //set data rate and auto set lpf bw
  mpu_set_gyro_fsr(MPU6050_GYRO_FS_DEFAULT);
  mpu_set_accel_fsr(MPU6050_ACC_FS_DEFAULT);
  //enable gyro and accel, auto select clock source, no more setting after the sensors is enabled
  mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);
  /* MPU6050 dmp */
  dmp_load_motion_driver_firmware();
  dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
  dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|
		     DMP_FEATURE_SEND_RAW_ACCEL|
		     DMP_FEATURE_SEND_CAL_GYRO|
		     DMP_FEATURE_GYRO_CAL);
  dmp_set_fifo_rate(DMP_DATA_RATE_HZ);  //DMP output data rate

 AUTONIL:
  /* User message: press Key1 to start offset AutoNil */
  strcpy((char*)str, "Hold sensor in\nlevel for offset\nAutoNil.");
  showMsg(0, 0, str, BLACK, 1);
  strcpy((char*)str, "Press Key1 when\nready.");
  showMsg(0, 4, str, RED, 0);

  do{
    delay_ms(10);
  }while(KEY_Scan() != KEY1_PRES);

  Lcd_Clear(GRAY0);

  delay_ms(200);

  //Conduct MPU6050 self-test, offset will be estimated as well
  i32Res = mpu_run_self_test(lGyro, lAccel);
  if(i32Res < 3){
    /* User error message */
    strcpy((char*)str, "selftest fail:");
    itoa(i32Res, &str[strlen((const char*)str)]);
    showMsg(0, 0, str, 0xCCC0, 1);
    strcpy((char*)str, "Press Key1 to\ncontinue.");
    showMsg(0, 2, str, RED, 0);
    do{
      delay_ms(10);
    }while(KEY_Scan() != KEY1_PRES);
    goto AUTONIL;
  }

  // Get the gyro and accel sensitivity
  mpu_get_gyro_sens(&gyroS);
  mpu_get_accel_sens(&accS);

  // DMP compensate the gyro offset
  for(i = 0; i < 3; ++i)
    lGyro_off[i] = (long)(lGyro[i] * gyroS + 0.5f);
  dmp_set_gyro_bias(lGyro_off);

  // DMP compensate accel offset
  for(i = 0; i < 3; ++i)
    lAccel_off[i] = -lAccel[i];
  mpu_set_accel_bias(lAccel_off);

  //Conduct pressure AutoNil
  pressureAve_f(gmp102_measure_P, gmp102_measure_T, gmp102_compensation, fCalibParam, &fTmp);
  set_sea_level_pressure_base(fTmp);

  /* User message: show offset */
  strcpy((char*)str, "acc offset(mg)\n");
  floatCatToStr(1000*lAccel[0]/65536.f, 1, &str[strlen((const char*)str)]);
  strcat((char*)str, ",");
  floatCatToStr(1000*lAccel[1]/65536.f, 1, &str[strlen((const char*)str)]);
  strcat((char*)str, ",");
  floatCatToStr(1000*lAccel[2]/65536.f, 1, &str[strlen((const char*)str)]);
  showMsg(0, 0, str, BLACK, 1);

  strcpy((char*)str, "gyro offset(dps)\n");
  floatCatToStr(lGyro[0]/65536.f, 2, &str[strlen((const char*)str)]);
  strcat((char*)str, ",");
  floatCatToStr(lGyro[1]/65536.f, 2, &str[strlen((const char*)str)]);
  strcat((char*)str, ",");
  floatCatToStr(lGyro[2]/65536.f, 2, &str[strlen((const char*)str)]);
  showMsg(0, 2, str, BLACK, 0);

  strcpy((char*)str, "pressure (Pa)\n");
  floatCatToStr(fTmp, 1, &str[strlen((const char*)str)]);
  showMsg(0, 4, str, BLACK, 0);

  strcpy((char*)str, "Press Key1 to\ncontinue");
  showMsg(0, 7, str, RED, 0);

  do{
    delay_ms(10);
  }while(KEY_Scan() != KEY1_PRES);

  // Initialize the altitude fusion state
  initAltitude(&thisSV_alt);

  /* finally */
  mpu_set_dmp_state(1);                 //This will automatically mpu_set_sample_rate to DMP_SAMPLE_RATE

  /* User message: header */
  strcpy((char*)str, "Data,Alg=");
  itoa(SENSOR_SAMPLING_RATE_HZ, &str[strlen((const char*)str)]);
  strcat((char*)str, ",");
  itoa(DMP_DATA_RATE_HZ, &str[strlen((const char*)str)]);
  strcat((char*)str, "Hz");
  showMsg(0, 0, str, BLACK, 1);
  strcpy((char*)str, "Alt,cm=");
  showMsg(0, 1, str, GRAY1, 0);
  strcpy((char*)str, "AltSTD,mm=");
  showMsg(0, 2, str, GRAY1, 0);
  strcpy((char*)str, "AltK,cm=");
  showMsg(0, 3, str, GRAY1, 0);
  strcpy((char*)str, "AltKSTD,mm=");
  showMsg(0, 4, str, GRAY1, 0);
  strcpy((char*)str, "hp,cm=");
  showMsg(0, 5, str, GRAY1, 0);
  strcpy((char*)str, "hpSTD,mm=");
  showMsg(0, 6, str, GRAY1, 0);

  // Initialize the altitude fusion state
  initAltitude(&thisSV_alt);
  initAltitude(&thisSV_altKalman);

  /* Init the statistics state variables */
  initBasicStats(&baroAltStats);
  initBasicStats(&fusAltStats);
  initBasicStats(&kalmanAltStats);

  while (1){

    if(ui8PeriodicMeasureFlag){

      // Algorithm loop
      ui8PeriodicMeasureFlag = 0;

      // Read barometer and calculate altitude
      gmp102_measure_P(&s32P);
      gmp102_measure_T(&s16T);
      gmp102_compensation(s16T, s32P, fCalibParam, &fT_Celsius, &fP_Pa);
      fBaroAlt_m = pressure2Alt(fP_Pa); // Pressure Altitude

      while(1){ //get the lastest results
	dmp_read_fifo(s16Gyro, s16Accel, quat, &timestamp, &sensors, &more);
	if(more == 0)
	  break;
      }

      if((sensors & INV_XYZ_GYRO) == 0 || (sensors & INV_XYZ_ACCEL) == 0 || (sensors & INV_WXYZ_QUAT) == 0){
	continue;
      }

      mpuCalcLinearAccelAndroid(quat, s16Accel, fLinearAcc, accS);

      LED_GPIO_PORT->OPTDT ^= LED2_GPIO_PIN;

      // Altitude fusion by complimentary filter
      fAlt_m = altitudeByCompFilt(fLinearAcc[2], fBaroAlt_m, algDt, &thisSV_alt);

      // Altitude fusion by Kalman filter
      fAltKalman_m = altitudeByKalmanFilt(fLinearAcc[2], fBaroAlt_m, algDt, &thisSV_altKalman);

      // Calculate statistics
      calcBasicStats(fBaroAlt_m, &baroAltStats);
      calcBasicStats(fAlt_m, &fusAltStats);
      calcBasicStats(fAltKalman_m, &kalmanAltStats);

      /* User message output */
      if(++printIcounter < PRINTOUT_OSR) continue;
      printIcounter = 0; //reset the counter

      //Altitude from fusion of GMP102 and MPU6050 by complimentary filter
      strcpy((char*)str, "");
      itoa(fAlt_m < 0?(fAlt_m*100.f-0.5f):(fAlt_m*100.f+0.5f), str);
      strcat((char*)str, "       ");
      showMsg(65, 1, str, BLUE, 0);
      strcpy((char*)str, "");
      itoa(fusAltStats.std*1000.f+0.5f, str);
      strcat((char*)str, "       ");
      showMsg(80, 2, str, BLUE, 0);

      //Altitude from fusion of GMP102 and MPU6050 by Kalman filter
      itoa(fAltKalman_m < 0?(fAltKalman_m*100.f-0.5f):(fAltKalman_m*100.f+0.5f), str);
      strcat((char*)str, "       ");
      showMsg(65, 3, str, BLUE, 0);
      strcpy((char*)str, "");
      itoa(kalmanAltStats.std*1000.f+0.5f, str);
      strcat((char*)str, "       ");
      showMsg(85, 4, str, BLUE, 0);

      //Altitude directly from GMP102
      strcpy((char*)str, "");
      itoa(fBaroAlt_m < 0?(fBaroAlt_m*100.f-0.5f):(fBaroAlt_m*100.f+0.5f), str);
      strcat((char*)str, "       ");
      showMsg(65, 5, str, BLUE, 0);
      strcpy((char*)str, "");
      itoa(baroAltStats.std*1000.f+0.5f, str);
      strcat((char*)str, "       ");
      showMsg(80, 6, str, BLUE, 0);

      /* Init the statistics state variables */
      initBasicStats(&baroAltStats);
      initBasicStats(&fusAltStats);
      initBasicStats(&kalmanAltStats);

      LED_GPIO_PORT->OPTDT ^= LED4_GPIO_PIN;

    }
  }
}

