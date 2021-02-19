/*
*******************************************************************************
*
*                              Hexar Systems, Inc.
*                      104, Factory Experiment Bldg, No41.55
*              Hanyangdaehak-ro, Sangnok-gu, Ansan, Gyeonggi-do, Korea
*
*                (c) Copyright 2017, Hexar Systems, Inc., Sangnok-gu, Ansan
*
* All rights reserved. Hexar Systems¡¯s source code is an unpublished work and the
* use of a copyright notice does not imply otherwise. This source code contains
* confidential, trade secret material of Hexar Systems, Inc. Any attempt or participation
* in deciphering, decoding, reverse engineering or in any way altering the 
source
* code is strictly prohibited, unless the prior written consent of Hexar Systems, Inc.
* is obtained.
*
* Filename		: hx_CAN.h
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/05/18
* Description		: CAN Driver
*******************************************************************************
*
*/

#ifndef _CAN_H_
#define _CAN_H_

/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx.h"

/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
extern void CANDrv_Init(void);
extern void CANDrv_SetFilter16bit(uint32_t a);
extern int CANDrv_WriteFile(uint8_t *buf, uint8_t cnt);
extern void CANDrv_SetStdID(uint32_t id);

#endif //_CAN_H_
