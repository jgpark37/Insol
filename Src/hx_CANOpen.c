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
* Filename		: hx_API.h
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/02/21
* Description		: Message box
*******************************************************************************
*
*/

/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx.h"
#include "hx_CANOpen.h"
#include "CAN_OPEN_DSP402.h"
#include "hx_InsolSensor.h"
#include "hx_CAN.h"
#include "hx_mcuFlash.h"

/* Private define ------------------------------------------------------------*/
#define WRITE_REQUEST_NDEF					0x22
#define WRITE_REQUEST_2BYTE				0x2B
#define WRITE_REQUEST_1BYTE				0x2F
#define READ_REQUEST						0x40
//=========================================================
//SDO Transfer Protocol - Legend [BYTE 0]
//=========================================================
//Ref. Maxon Document ID:rel2501, page 10-148
#define	WRITE_REQUEST_NDEF		0x22
#define	WRITE_REQUEST_3BYTE		0x23
#define	WRITE_REQUEST_2BYTE		0x2B
#define	WRITE_REQUEST_1BYTE		0x2F
#define	WRITE_RESPONSE			0x60

#define	READ_REQUEST				0x40
#define	READ_RESPONSE_3BYTE		0x43
#define	READ_RESPONSE_2BYTE		0x4B
#define	READ_RESPONSE_1BYTE		0x4F
//=========================================================

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern char CommonBuf[FLASH_PAGE_SIZE1*2];

/* Private function prototypes -----------------------------------------------*/
//extern static void MX_CAN_Init(void);


////////////////////////////////////////////////////////////////////////////////////////
// CAN OPEN
uint16_t CANOpen_GetIndex(uint16_t index , uint8_t subindex)
{
	return index;
}

void CANOpen_Control(CanOpenCommand *pCoc)
{
	uint8_t buf[5];
	uint8_t cnt;

	cnt = 0;
	CANDrv_SetStdID(Insol.can.txid);
	
	switch (pCoc->m_subinx) {
	case CAN_OPEN_SHUTDOWN:
		break;
	case CAN_OPEN_SWITCH_ON:
		break;
	case CAN_OPEN_SWITCH_OFF:
		break;
	case CAN_OPEN_ENABLE_OPERATION:
		if (Insol.data.size == DS_4BIT) 
			Insol_SetRunMode(RM_NORMAL_4BIT);
		else if (Insol.data.size == DS_8BIT) 
			Insol_SetRunMode(RM_NORMAL_8BIT);
		else if (Insol.data.size == DS_12BIT) 
			Insol_SetRunMode(RM_NORMAL_12BIT);

		buf[cnt++] = WRITE_RESPONSE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_ENABLE_OPERATION;
		buf[cnt++] = 0;
		//cnt = 5;
		CANDrv_WriteFile(buf, cnt);
		break;
	case CAN_OPEN_DISABLE_OPERATION:
		Insol_SetRunMode(RM_STANDBY);
		buf[cnt++] = WRITE_RESPONSE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_DISABLE_OPERATION;
		buf[cnt++] = 0;
		//cnt = 5;
		CANDrv_WriteFile(buf, cnt);
		break;
	case CAN_OPEN_SET_INTERVAL_SEND_DATA:
		Insol.data.sendTime = (pCoc->data[1]<<8) | pCoc->data[0];
		if (Insol.data.sendTime < 1) Insol.data.sendTime = 6;
		buf[cnt++] = WRITE_RESPONSE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_SET_INTERVAL_SEND_DATA;
		buf[cnt++] = 0;
		//cnt = 5;
		CANDrv_WriteFile(buf, cnt);
		break;
	case CAN_OPEN_RUN_CALIBRATION:
		Insol_RunMode_Calibration();
		buf[cnt++] = WRITE_RESPONSE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_RUN_CALIBRATION;
		buf[cnt++] = 0;
		//cnt = 5;
		CANDrv_WriteFile(buf, cnt);
		break;
	case CAN_OPEN_SET_DEVICE_NUM:
		Insol.can.txid = CAN_TX_STD_ID + pCoc->data[0];
		Insol.can.rxid = CAN_RX_STD_ID + pCoc->data[0];
		buf[cnt++] = WRITE_RESPONSE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_SET_DEVICE_NUM;
		if (Insol_SaveParamToFlash(CommonBuf)) buf[cnt++] = 0;
		else buf[cnt++] = 1;
		//cnt = 5;
		CANDrv_WriteFile(buf, cnt);
		break;
	case CAN_OPEN_SET_DATA_OUTPUT_TYPE:
		Insol.data.outputType = pCoc->data[0];
		buf[cnt++] = WRITE_RESPONSE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_SET_DATA_OUTPUT_TYPE;
		buf[cnt++] = 0;
		//cnt = 5;
		CANDrv_WriteFile(buf, cnt);
		break;
	case CAN_OPEN_SET_DATA_SIZE:
		Insol.data.size = pCoc->data[0];
		Insol.data.shift = pCoc->data[1];
		if (Insol.runMode > RM_SLEEP && Insol.runMode < RM_MAX) {
			if (Insol.data.size == DS_4BIT) 
				Insol_SetRunMode(RM_NORMAL_4BIT);
			else if (Insol.data.size == DS_8BIT) {
				Insol_SetRunMode(RM_NORMAL_8BIT);
				if (Insol.data.shift < 4 || Insol.data.shift > 11) Insol.data.shift = 4;
			}
			else if (Insol.data.size == DS_12BIT)
				Insol_SetRunMode(RM_NORMAL_12BIT);
		}
		buf[cnt++] = WRITE_RESPONSE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_ENABLE_OPERATION;
		buf[cnt++] = 0;
		//cnt = 5;
		CANDrv_WriteFile(buf, cnt);
		break;
	case CAN_OPEN_SEL_SENSOR_POS:
		Insol.sensorONPos = pCoc->data[0]<<24|pCoc->data[1]<<16|pCoc->data[2]<<8|pCoc->data[3];
		//Insol_GetSensorONNum(Insol.sensorONPos);
		Insol_SetBufPtr(Insol.data.size, Insol.sensorONPos);
		buf[cnt++] = WRITE_RESPONSE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_SEL_SENSOR_POS;
		buf[cnt++] = 0;
		//cnt = 5;
		CANDrv_WriteFile(buf, cnt);
		break;
	default:
		break;
	}
	
}

void CANOpen_Status(CanOpenCommand *pCoc)
{
	uint8_t buf[8];
	uint8_t cnt;

 	cnt = 0;
	switch (pCoc->m_subinx) {
	case CAN_OPEN_GET_INFORMATION:
		buf[cnt++] = FW_VER;
		buf[cnt++] = HW_VER;
		buf[cnt++] = FW_USER;
		buf[cnt++] = IMN_SELF;
		buf[cnt++] = FW_CREATE_YEAR;
		buf[cnt++] = FW_CREATE_MONTH;
		buf[cnt++] = FW_CREATE_DAY;
		buf[cnt++] = FW_CREATE_HOUR;
		//buf[cnt++] = FW_CREATE_MIN;
		//cnt = 8;
		CANDrv_WriteFile(buf, cnt);
		break;
	case CAN_OPEN_GET_STATUS:
		break;
	default:
		break;
	}
	
}

void CANOpen_Ack(void)
{
	
}

void CANOpen_Process(void)
{
	CanOpenCommand *pCoc;
	
	
	if (CanInfo.rx.head == CanInfo.rx.tail) return;
	
	pCoc = (CanOpenCommand *)&CanInfo.rx.buf[CanInfo.rx.tail];
	if (pCoc->m_inx == CANOpen_GetIndex(CAN_OPEN_CONTROL)) {
		CANOpen_Control(pCoc);
	}
	else if (pCoc->m_inx == CANOpen_GetIndex(CAN_OPEN_STATUS)) {
		CANOpen_Status(pCoc);
	}
	else {
		CANOpen_Ack();
	}

	CanInfo.rx.tail++;
	if (CanInfo.rx.tail >= CAN_RX_BUF_NUM) CanInfo.rx.tail = 0;
}


