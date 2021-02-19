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
* Filename		: hx_CAN.c
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/05/18
* Description		: CAN Driver
*******************************************************************************
*
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "hx_InsolSensor.h"
#include "hx_CANOpen.h"

/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static CanTxMsgTypeDef        TxMessage; 
static CanRxMsgTypeDef        RxMessage; 
static CAN_FilterConfTypeDef myFilter; 

extern CAN_HandleTypeDef hcan;


/* Private function prototypes -----------------------------------------------*/
extern void Error_Handler(void);
extern void MX_CAN_Init(void);

void CANDrv_SetStdID(uint32_t id)
{
	hcan.pTxMsg->StdId = id;
}

int CANDrv_WriteFile(uint8_t *buf, uint8_t cnt)
{
	//uint16_t *pbuf;
	int i;
	HAL_CAN_StateTypeDef status;

	//if (Insol.can.complete == 0) return 0;
	//while (hcan.State == HAL_CAN_STATE_BUSY);
	
	//if (hcan.Lock == HAL_LOCKED) return 0;
//	if (cnt > 4) return;
	status = HAL_CAN_GetState(&hcan);
	//aa = status;
	switch (status) {
	case HAL_CAN_STATE_READY:		// normal
		//if (aa == 50 || aa == 18) {
		//	MX_CAN_Init();
		//	Insol_CAN_Init();
		//	return 0;
		//}
		break;
	case HAL_CAN_STATE_BUSY_RX:
		break;
	case HAL_CAN_STATE_BUSY_TX:	//host no run
	case HAL_CAN_STATE_BUSY_TX_RX: // can connector plug out
		return -1;
	case HAL_CAN_STATE_RESET:
		return 0;
	case HAL_CAN_STATE_ERROR:
		return -1;
	case HAL_CAN_STATE_TIMEOUT:
		Insol.can.timeout++;
		if (Insol.can.timeout > 100) {
			Insol.can.timeout = 0;
			return -1;
		}
		return 0;
	default:
		return 0;
	}
	
	//hcan.pTxMsg->StdId = cmdID;
	
	//pbuf = (uint16_t *)hcan.pTxMsg->Data;
	for (i = 0; i < cnt; i++) {
		hcan.pTxMsg->Data[i] = buf[i];
	}
	hcan.pTxMsg->DLC = cnt;
	/*##-3- Start the Transmission process ###############################*/
	Insol.can.complete = 0;
	//if (HAL_CAN_Transmit_IT(&hcan) != HAL_OK)
	if (HAL_CAN_Transmit(&hcan, 10) != HAL_OK)
	{
		/* Transmition Error */
		//Error_Handler();
		return -1;
	}

	Insol.can.timeout = 0;
	Insol.led.canCnt++;
	return 1;
}

uint32_t CANDrv_FilterID_Generator_16Bits(uint32_t ID, uint32_t RTR, uint32_t IDE)
{
	uint32_t ret;

	if (IDE == CAN_ID_STD)
	{
		 ret = (ID & 0x7FF) << 5 | (RTR << 3) | (IDE << 1);
	}
	else // IDE == CAN_ID_EXT
	{
		 ret = ((ID & 0x1FFC0000) >> 13) | (RTR << 3) | (IDE << 1) | ((ID & 0x38000) >> 15);
	}

	return ret;
}

void CANDrv_SetFilter16bit(uint32_t a)
{
	myFilter.FilterIdHigh = 0;//CAN_STD_ID<<5;//0x0000; 
	myFilter.FilterIdLow = 0x0000; 
	myFilter.FilterMaskIdHigh = 0;//(CAN_STD_ID<<3)>>16;//0x0000; 
	myFilter.FilterMaskIdLow = 0;//((CAN_STD_ID<<3)&0xffff)|(0x1<<2);//0x0000; 
	HAL_CAN_ConfigFilter(&hcan, &myFilter);
}

void CANDrv_Init(void)
{
	myFilter.FilterNumber = 0; 
	myFilter.FilterMode = CAN_FILTERMODE_IDMASK; 
	myFilter.FilterScale = CAN_FILTERSCALE_16BIT; 
	myFilter.FilterIdHigh = 0;//CAN_STD_ID<<5;//0x0000; 
	myFilter.FilterIdLow = 0x0000; 
	myFilter.FilterMaskIdHigh = 0;//(CAN_STD_ID<<3)>>16;//0x0000; 
	myFilter.FilterMaskIdLow = 0;//((CAN_STD_ID<<3)&0xffff)|(0x1<<2);//0x0000; 
	myFilter.FilterFIFOAssignment = 0; 
	myFilter.FilterActivation = ENABLE; 
	HAL_CAN_ConfigFilter(&hcan, &myFilter);

	hcan.pTxMsg = &TxMessage; 
	hcan.pRxMsg = &RxMessage; 
	
	hcan.pTxMsg->StdId = CAN_TX_STD_ID&0x7FF; 
	hcan.pTxMsg->ExtId = 0;//CAN_STD_ID&0x1FFFFFFF; 
	hcan.pTxMsg->RTR = CAN_RTR_DATA; 
	hcan.pTxMsg->IDE = CAN_ID_STD; 
	hcan.pTxMsg->DLC = 2; 
	/* Set the data to be tranmitted */
	hcan.pTxMsg->Data[0] = 'H';
	hcan.pTxMsg->Data[1] = 'X';

	if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK)
	{
		//  * Reception Error *
	   	Error_Handler();
	}

	//Insol.can.chkTime = 1;
	Insol.can.rv = 0;
	Insol.can.timeout = 0;
	Insol.can.txid = CAN_TX_STD_ID&0x7FF;
	Insol.can.rxid = CAN_RX_STD_ID&0x7FF;
	Insol.can.complete = 0;
	
	CanInfo.rx.head = 0;
	CanInfo.rx.tail = 0;

}

