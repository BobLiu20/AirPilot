/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                          (c) Copyright 2003-2009; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                           MASTER INCLUDES
*
*                                     ST Microelectronics STM32
*                                              with the
*                                   STM3210E-EVAL Evaluation Board
*
* Filename      : includes.h
* Version       : V1.00
* Programmer(s) : EHS
*********************************************************************************************************
*/

#ifndef  INCLUDES_PRESENT
#define  INCLUDES_PRESENT

/*
*********************************************************************************************************
*                                         STANDARD LIBRARIES
*********************************************************************************************************
*/

#include  <stdarg.h>
#include  <stdio.h>
#include  <stdlib.h>
#include  <stdbool.h>
//#include  <math.h>
#include <arm_math.h>


/*
*********************************************************************************************************
*                                              LIBRARIES
*********************************************************************************************************
*/

#include  <cpu.h>
#include  <lib_def.h>
#include  <lib_ascii.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <lib_str.h>


/*
*********************************************************************************************************
*                                              APP / BSP
*********************************************************************************************************
*/

#include  <app_cfg.h>
#include  <os_cfg_app.h>
#include  <bsp.h>


/*
*********************************************************************************************************
*                                                 OS
*********************************************************************************************************
*/

#include  <os.h>


/*
*********************************************************************************************************
*                                                 ST
*********************************************************************************************************
*/
#include <stm32f4xx.h>

/*×Ô¼ºÌí¼ÓµÎ¡£¡£¡£*/
#include "define.h"
#include "config.h"
#include "System.h"
#include "AP_PID.h"
/*dir*/
#include "AP_Flash.h"
#include "AP_USART.h"
#include "AP_I2C.h"
#include "AP_MPU6050.h"
#include "AP_HMC5883.h"
#include "AP_MS5611.h"
#include "AP_LED.h"
#include "AP_PWM.h"
#include "AP_XbeePro.h"
#include "AP_Sonar.h"
#include "AP_SonarAvoid.h"
#include "AP_Battery.h"


/*module*/
#include "AP_Serial.h"
#include "AP_Sensor.h"
#include "AP_IMU.h"
#include "AP_Altitude.h"
#include "AP_Actuator.h"
#include "AP_RC.h"
#include "AP_Mixer.h"
#include "AP_Daemon.h"
#include "AP_GPS.h"
#include "AP_Telemetry.h"
#include "AP_OpticalFlow.h"
#include "AP_EasyCtrl.h"
#include "AP_VisionLanding.h"
#include "AP_Avoid.h"


/*
*********************************************************************************************************
*                                            INCLUDES END
*********************************************************************************************************
*/


#endif

