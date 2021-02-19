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
* in deciphering, decoding, reverse engineering or in any way altering the source
* code is strictly prohibited, unless the prior written consent of Hexar Systems, Inc.
* is obtained.
*
* Filename		: hx_CANOpen.c
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/02/21
* Description		: CAN Open protocol
*******************************************************************************
*
*/

#ifndef _CAN_OPEN_H_
#define _CAN_OPEN_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private define ------------------------------------------------------------*/
//#define RIGHT_SIDE					
#ifdef RIGHT_SIDE
#define CAN_RX_STD_ID				0x60A		//right
#define CAN_TX_STD_ID				0x58A		//right
#else
#define CAN_RX_STD_ID				0x61A		//left
#define CAN_TX_STD_ID				0x59A		//left
#endif
#define CAN_INSOL_SENSOR1		(CAN_TX_STD_ID+1)
#define CAN_INSOL_SENSOR2		(CAN_INSOL_SENSOR1+1)
#define CAN_INSOL_SENSOR3		(CAN_INSOL_SENSOR2+1)
#define CAN_INSOL_SENSOR4		(CAN_INSOL_SENSOR3+1)
#define CAN_INSOL_SENSOR5		(CAN_INSOL_SENSOR4+1)
#define CAN_INSOL_SENSOR6		(CAN_INSOL_SENSOR5+1)


/* Private typedef -----------------------------------------------------------*/
#pragma pack(1)
typedef struct tagCanOpenCommand {
	unsigned char	m_sdo;
	unsigned short m_inx;
	unsigned char	m_subinx;
	unsigned char	data[4];
} CanOpenCommand;
#pragma pack()


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
extern void CANOpen_Process(void);

#endif //_CAN_OPEN_H_

