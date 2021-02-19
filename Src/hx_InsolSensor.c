/*
*******************************************************************************
*
*                              Hexar Systems, Inc.
*                      104, Factory Experiment Bldg, No41.55
*              Hanyangdaehak-ro, Sangnok-gu, Ansan, Gyeonggi-do, Korea
*
*                (c) Copyright 2017, Hexar Systems, Inc., Sangnok-gu, Ansan
*
* All rights reserved. Hexar Systems’s source code is an unpublished work and the
* use of a copyright notice does not imply otherwise. This source code contains
* confidential, trade secret material of Hexar Systems, Inc. Any attempt or participation
* in deciphering, decoding, reverse engineering or in any way altering the 
source
* code is strictly prohibited, unless the prior written consent of Hexar Systems, Inc.
* is obtained.
*
* Filename		: hx_InsolSensor.c
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F103C8T6
* Compiler		: IAR
* Created      	: 2017/02/21
* Description		: Insol Sensor
*******************************************************************************
*
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "hx_InsolSensor.h"
#include "hx_CAN.h"
#include "hx_mcuFlash.h"

/* Private define ------------------------------------------------------------*/
#define LED_ADC_RUN_BLINK_TIME					3500
#define LED_CAN_TX_BLINK_TIME					30
#define CAN_ERR_RECHECK_TIME					1000
#define USE_PULLUP								1 		//reduce nois than floating

#define LED_ADC_RUN_BLINK 				HAL_GPIO_TogglePin(GPIOB, LED_ORANGE)
#define LED_ADC_RUN_OFF 				HAL_GPIO_WritePin(GPIOB, LED_ORANGE, SET)
#define LED_CAN_TX_BLINK 				HAL_GPIO_TogglePin(GPIOB, LED_GREEN)
#define LED_CAN_TX_OFF 					HAL_GPIO_WritePin(GPIOB, LED_GREEN, SET)
#define LED_CAN_TX_ON 					HAL_GPIO_WritePin(GPIOB, LED_GREEN, RESET)

/* Private typedef -----------------------------------------------------------*/
struct _tagSETUP_INFO{	//셋업(옵션)창의 변수 저장
	uint32_t canRxID;					
	uint32_t canTxID;					
}Setup, SetupOld;

typedef struct _tagFLASH_SAVE_SYSTEM_INFO {
	char tag[2];
	char company[COMPANY_LENGTH];
	char model[MODEL_LENGTH];
	char swVer;
	char hwVer;
	char createDate[3];
	char createTime[3];
	char modifyDate[3];
	char modifyTime[3];
	uint32_t writeTime;	//flash re-write times(max 10,000)
	uint32_t nextNode;
	uint32_t replaceNode;
	struct _tagSETUP_INFO setup;
}SYSTEM_INFO;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t AdcDmaBuf[DMABUFSIZE] = {0xAA, 0, 0, 0, 0, 0, 0, 0, 0xBB};
uint16_t AdcCalBuf[INSOL_SENSOR_NUM];	//base data with no load
uint16_t ForceConstant[INSOL_SENSOR_NUM];
CAN_INFO CanInfo;
INSOL Insol;
INSOL_TIMER InsolTimer;
SYSTEM_STATUS SysStatus;
char CommonBuf[FLASH_PAGE_SIZE1*2];	//for mcu flash page size
const char COMPANY_t[COMPANY_LENGTH] = {"HEXAR SYSTEMS"};
const char MODEL_t[MODEL_LENGTH] = {"INSOL"};
//const uint8_t FW_VER[4] = {'1', '0', 'K', 'G'};		//v1.0.KG
//const uint8_t CREATE_DATE[3] = {17, 3, 24};		//17y, 3m, 24d
//const uint8_t CREATE_TIME[3] = {16, 31, 55};	//hour, min, sec

extern ADC_HandleTypeDef hadc1;
extern IWDG_HandleTypeDef hiwdg;


/* Private function prototypes -----------------------------------------------*/
void (*fnADCDrv_Copy2Buf)(void);
void (*fnInsol_RunMode)(void);
void Insol_CopyADC2Buf(void);


void ADCDrv_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
	//GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
 	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	

}

void ADCDrv_Copy2CalBuf(void)
{
	switch (Insol.sourcePos) {
	case 0:
		AdcCalBuf [0] = ((AdcCalBuf [0]*(Insol.adc.cnt-1)) + AdcDmaBuf[1])/Insol.adc.cnt;
		AdcCalBuf [1] = ((AdcCalBuf [1]*(Insol.adc.cnt-1)) + AdcDmaBuf[2])/Insol.adc.cnt;
		AdcCalBuf [2] = ((AdcCalBuf [2]*(Insol.adc.cnt-1)) + AdcDmaBuf[3])/Insol.adc.cnt;
		AdcCalBuf [3] = ((AdcCalBuf [3]*(Insol.adc.cnt-1)) + AdcDmaBuf[4])/Insol.adc.cnt;
		AdcCalBuf [4] = ((AdcCalBuf [4]*(Insol.adc.cnt-1)) + AdcDmaBuf[5])/Insol.adc.cnt;
		break;
	case 1:
		AdcCalBuf [5] = ((AdcCalBuf [5]*(Insol.adc.cnt-1)) + AdcDmaBuf[1])/Insol.adc.cnt;
		AdcCalBuf [6] = ((AdcCalBuf [6]*(Insol.adc.cnt-1)) + AdcDmaBuf[2])/Insol.adc.cnt;
		AdcCalBuf [7] = ((AdcCalBuf [7]*(Insol.adc.cnt-1)) + AdcDmaBuf[3])/Insol.adc.cnt;
		break;
	case 2:
		AdcCalBuf [8] = ((AdcCalBuf [8]*(Insol.adc.cnt-1)) + AdcDmaBuf[3])/Insol.adc.cnt;
		AdcCalBuf [9] = ((AdcCalBuf [9]*(Insol.adc.cnt-1)) + AdcDmaBuf[4])/Insol.adc.cnt;
		AdcCalBuf [10] = ((AdcCalBuf [10]*(Insol.adc.cnt-1)) + AdcDmaBuf[5])/Insol.adc.cnt;
		AdcCalBuf [11] = ((AdcCalBuf [11]*(Insol.adc.cnt-1)) + AdcDmaBuf[6])/Insol.adc.cnt;
		AdcCalBuf [12] = ((AdcCalBuf [12]*(Insol.adc.cnt-1)) + AdcDmaBuf[7])/Insol.adc.cnt;
		break;
	case 3:
		AdcCalBuf [13] = ((AdcCalBuf [13]*(Insol.adc.cnt-1)) + AdcDmaBuf[4])/Insol.adc.cnt;
		AdcCalBuf [14] = ((AdcCalBuf [14]*(Insol.adc.cnt-1)) + AdcDmaBuf[5])/Insol.adc.cnt;
		AdcCalBuf [15] = ((AdcCalBuf [15]*(Insol.adc.cnt-1)) + AdcDmaBuf[6])/Insol.adc.cnt;
		AdcCalBuf [16] = ((AdcCalBuf [16]*(Insol.adc.cnt-1)) + AdcDmaBuf[7])/Insol.adc.cnt;
		break;
	case 4:
		AdcCalBuf [17] = ((AdcCalBuf [17]*(Insol.adc.cnt-1)) + AdcDmaBuf[4])/Insol.adc.cnt;
		AdcCalBuf [18] = ((AdcCalBuf [18]*(Insol.adc.cnt-1)) + AdcDmaBuf[5])/Insol.adc.cnt;
		AdcCalBuf [19] = ((AdcCalBuf [19]*(Insol.adc.cnt-1)) + AdcDmaBuf[6])/Insol.adc.cnt;
		AdcCalBuf [20] = ((AdcCalBuf [20]*(Insol.adc.cnt-1)) + AdcDmaBuf[7])/Insol.adc.cnt;
		break;
	default:
		break;
	}	
}

void ADCDrv_Copy2Buf()
{
	uint16_t temp;
	
	switch (Insol.sourcePos) {
	case 0: //GPIO1
		if (!Insol.data.processType) {		
			Insol.sensorValueBuf[0] = AdcDmaBuf[1]; //A1/G1
			Insol.sensorValueBuf[1] = AdcDmaBuf[2];
			Insol.sensorValueBuf[2] = AdcDmaBuf[3];
			Insol.sensorValueBuf[3] = AdcDmaBuf[4];
			Insol.sensorValueBuf[4] = AdcDmaBuf[5];
		}
		else if (Insol.data.processType&0x80) {	
			if (AdcCalBuf[0] > AdcDmaBuf[1]) 
				temp = AdcCalBuf[0] - AdcDmaBuf[1];
			else temp = 0;
			Insol.sensorValueBuf[0] = temp;
			if (AdcCalBuf[1] > AdcDmaBuf[2]) 
				temp = AdcCalBuf[1] - AdcDmaBuf[2];
			else temp = 0;
			Insol.sensorValueBuf[1] = temp;
			if (AdcCalBuf[2] > AdcDmaBuf[3]) 
				temp = AdcCalBuf[2] - AdcDmaBuf[3];
			else temp = 0;
			Insol.sensorValueBuf[2] = temp;
			if (AdcCalBuf[3] > AdcDmaBuf[4]) 
				temp = AdcCalBuf[3] - AdcDmaBuf[4];
			else temp = 0;
			Insol.sensorValueBuf[3] = temp;
			if (AdcCalBuf[4] > AdcDmaBuf[5]) 
				temp = AdcCalBuf[4] - AdcDmaBuf[5];
			else temp = 0;
			Insol.sensorValueBuf[4] = temp;
		}
		else if (Insol.data.processType&0x40) {
			Insol.sensorValueBuf[0] = ForceConstant[0]/AdcDmaBuf[1];
			Insol.sensorValueBuf[1] = ForceConstant[1]/AdcDmaBuf[2];
			Insol.sensorValueBuf[2] = ForceConstant[2]/AdcDmaBuf[3];
			Insol.sensorValueBuf[3] = ForceConstant[3]/AdcDmaBuf[4];
			Insol.sensorValueBuf[4] = ForceConstant[4]/AdcDmaBuf[5];
		}
		break;
	case 1: //GPIO2
		if (!Insol.data.processType) {		
			Insol.sensorValueBuf[5] = AdcDmaBuf[1]; //A1/G2
			Insol.sensorValueBuf[6] = AdcDmaBuf[2];
			Insol.sensorValueBuf[7] = AdcDmaBuf[3];
		}
		else if (Insol.data.processType&0x80) {	
			if (AdcCalBuf[5] > AdcDmaBuf[1]) 
				temp = AdcCalBuf[5] - AdcDmaBuf[1];
			else temp = 0;
			Insol.sensorValueBuf[5] = temp;
			if (AdcCalBuf[6] > AdcDmaBuf[2]) 
				temp = AdcCalBuf[6] - AdcDmaBuf[2];
			else temp = 0;
			Insol.sensorValueBuf[6] = temp;
			if (AdcCalBuf[7] > AdcDmaBuf[3]) 
				temp = AdcCalBuf[7] - AdcDmaBuf[3];
			else temp = 0;
			Insol.sensorValueBuf[7] = temp;
		}
		else if (Insol.data.processType&0x40) {
			Insol.sensorValueBuf[5] = ForceConstant[5]/AdcDmaBuf[1];
			Insol.sensorValueBuf[6] = ForceConstant[6]/AdcDmaBuf[2];
			Insol.sensorValueBuf[7] = ForceConstant[7]/AdcDmaBuf[3];
		}
		break;
	case 2: //GPIO3
		if (!Insol.data.processType) {		
			Insol.sensorValueBuf[8] = AdcDmaBuf[3];
			Insol.sensorValueBuf[9] = AdcDmaBuf[4];
			Insol.sensorValueBuf[10] = AdcDmaBuf[5];
			Insol.sensorValueBuf[11] = AdcDmaBuf[6];
			Insol.sensorValueBuf[12] = AdcDmaBuf[7];
		}
		else if (Insol.data.processType&0x80) {	
			if (AdcCalBuf[8] > AdcDmaBuf[3]) 
				temp = AdcCalBuf[8] - AdcDmaBuf[3];
			else temp = 0;
			Insol.sensorValueBuf[8] = temp;
			if (AdcCalBuf[9] > AdcDmaBuf[4]) 
				temp = AdcCalBuf[9] - AdcDmaBuf[4];
			else temp = 0;
			Insol.sensorValueBuf[9] = temp;
			if (AdcCalBuf[10] > AdcDmaBuf[5]) 
				temp = AdcCalBuf[10] - AdcDmaBuf[5];
			else temp = 0;
			Insol.sensorValueBuf[10] = temp;
			if (AdcCalBuf[11] > AdcDmaBuf[6]) 
				temp = AdcCalBuf[11] - AdcDmaBuf[6];
			else temp = 0;
			Insol.sensorValueBuf[11] = temp;
			if (AdcCalBuf[12] > AdcDmaBuf[7]) 
				temp = AdcCalBuf[12] - AdcDmaBuf[7];
			else temp = 0;
			Insol.sensorValueBuf[12] = temp;
		}
		else if (Insol.data.processType&0x40) {
			Insol.sensorValueBuf[8] = ForceConstant[8]/AdcDmaBuf[3];
			Insol.sensorValueBuf[9] = ForceConstant[9]/AdcDmaBuf[4];
			Insol.sensorValueBuf[10] = ForceConstant[10]/AdcDmaBuf[5];
			Insol.sensorValueBuf[11] = ForceConstant[11]/AdcDmaBuf[6];
			Insol.sensorValueBuf[12] = ForceConstant[12]/AdcDmaBuf[7];
		}
		break;
	case 3: //GPIO4
		if (!Insol.data.processType) {		
			Insol.sensorValueBuf[13] = AdcDmaBuf[4];
			Insol.sensorValueBuf[14] = AdcDmaBuf[5];
			Insol.sensorValueBuf[15] = AdcDmaBuf[6];
			Insol.sensorValueBuf[16] = AdcDmaBuf[7];
		}
		else  if (Insol.data.processType&0x80) {	
			if (AdcCalBuf[13] > AdcDmaBuf[4]) 
				temp = AdcCalBuf[13] - AdcDmaBuf[4];
			else temp = 0;
			Insol.sensorValueBuf[13] = temp;
			if (AdcCalBuf[14] > AdcDmaBuf[5]) 
				temp = AdcCalBuf[14] - AdcDmaBuf[5];
			else temp = 0;
			Insol.sensorValueBuf[14] = temp;
			if (AdcCalBuf[15] > AdcDmaBuf[6]) 
				temp = AdcCalBuf[15] - AdcDmaBuf[6];
			else temp = 0;
			Insol.sensorValueBuf[15] = temp;
			if (AdcCalBuf[16] > AdcDmaBuf[7]) 
				temp = AdcCalBuf[16] - AdcDmaBuf[7];
			else temp = 0;
			Insol.sensorValueBuf[16] = temp;
		}
		else if (Insol.data.processType&0x40) {
			Insol.sensorValueBuf[13] = ForceConstant[13]/AdcDmaBuf[4];
			Insol.sensorValueBuf[14] = ForceConstant[14]/AdcDmaBuf[5];
			Insol.sensorValueBuf[15] = ForceConstant[15]/AdcDmaBuf[6];
			Insol.sensorValueBuf[16] = ForceConstant[16]/AdcDmaBuf[7];
		}
		break;
	case 4: //GPIO5
		if (!Insol.data.processType) {		
			Insol.sensorValueBuf[17] = AdcDmaBuf[4];
			Insol.sensorValueBuf[18] = AdcDmaBuf[5];
			Insol.sensorValueBuf[19] = AdcDmaBuf[6];
			Insol.sensorValueBuf[20] = AdcDmaBuf[7];
		}
		else if (Insol.data.processType&0x80) {	
			if (AdcCalBuf[17] > AdcDmaBuf[4]) 
				temp = AdcCalBuf[17] - AdcDmaBuf[4];
			else temp = 0;
			Insol.sensorValueBuf[17] = temp;
			if (AdcCalBuf[18] > AdcDmaBuf[5]) 
				temp = AdcCalBuf[18] - AdcDmaBuf[5];
			else temp = 0;
			Insol.sensorValueBuf[18] = temp;
			if (AdcCalBuf[19] > AdcDmaBuf[6]) 
				temp = AdcCalBuf[19] - AdcDmaBuf[6];
			else temp = 0;
			Insol.sensorValueBuf[19] = temp;
			if (AdcCalBuf[20] > AdcDmaBuf[7]) 
				temp = AdcCalBuf[20] - AdcDmaBuf[7];
			else temp = 0;
			Insol.sensorValueBuf[20] = temp;
		}
		else if (Insol.data.processType&0x40) {
			Insol.sensorValueBuf[17] = ForceConstant[17]/AdcDmaBuf[4];
			Insol.sensorValueBuf[18] = ForceConstant[18]/AdcDmaBuf[5];
			Insol.sensorValueBuf[19] = ForceConstant[19]/AdcDmaBuf[6];
			Insol.sensorValueBuf[20] = ForceConstant[20]/AdcDmaBuf[7];
		}
		break;
	default:
		break;
	}	
}

void ADCDrv_SelectSensorSource(void)
{
#ifdef SUPPORT_HW_V1_2
	Insol.sourcePos++;
	if (Insol.sourcePos >= INSOL_SOURCE_TOTLAL_NUM) Insol.sourcePos = 0;

	switch (Insol.sourcePos) {
	case 0:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);	//S3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);	//S2
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	//S1
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);	//S3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);	//S2
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);		//S1
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);	//S3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);		//S2
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	//S1
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);	//S3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);		//S2
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);		//S1
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);		//S3
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);	//S2
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	//S1
		break;
	default:
		break;
	}
#else	//h/w ver1.0
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	Insol.sourcePos++;
	if (Insol.sourcePos >= INSOL_SOURCE_TOTLAL_NUM) Insol.sourcePos = 0;

	switch (Insol.sourcePos) {
	case 0:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = GPIO_PIN_7;
		#ifdef USE_PULLUP
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		#else
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		#endif
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		GPIO_InitStruct.Pin = GPIO_PIN_4;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		#ifdef USE_PULLUP
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		#else
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		#endif
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = GPIO_PIN_4;
		#ifdef USE_PULLUP
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		#else
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		#endif
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		#ifdef USE_PULLUP
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		#else
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		#endif
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		GPIO_InitStruct.Pin = GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		#ifdef USE_PULLUP
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		#else
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		#endif
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		break;
	default:
		break;
	}

#endif

	//LED_CAN_TX_BLINK;
}

void Insol_RunMode_Calibration(void)
{
	int i, j, k;

	Insol_SetRunMode(RM_CALIBRATION);
	for (i = 0; i < 20; i++) {	//avg
		for (j = 0; j < INSOL_SOURCE_TOTLAL_NUM; j++) {
			k = 0;
			while (Insol.adc.complete == 0) {
				HAL_Delay(2);
				k++;
				if (k > 100) {
					HAL_ADC_Stop_DMA(&hadc1);
					SysStatus.mcu1 |= SSM_ADC_ERR;
					Insol_SetRunMode(RM_STANDBY);
					return;
				}
				HAL_IWDG_Refresh(&hiwdg);
			}
			HAL_IWDG_Refresh(&hiwdg);
			Insol.adc.complete = 0;
			HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&AdcDmaBuf[1], NumOfAdcChan );
		}
		Insol.adc.cnt++;
	}

	for (i = 0; i <  INSOL_SENSOR_NUM; i++) {
		//AdcCalBuf[i] = AdcCalBuf[i] + (INSOL_SENSOR_MAX_VALUE - AdcCalBuf[i]);
	}
	
	HAL_ADC_Stop_DMA(&hadc1);
}

void Insol_RunMode_Standby(void)
{

}

void Insol_RunMode_Normal_12bit(void)
{
	int i;//, j;

  	if (Insol.adc.complete == 0) return;

	if (InsolTimer.tick < Insol.data.sendTime) return;
	
	if (Insol.data.outputType&DOT_UART) {
		//if (!Insol.sendOrder) 
		{
#if 1		
			#if 1 //21 sensor data send
			printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d,\r\n", 
				Insol.sensorValueBuf[0], Insol.sensorValueBuf[1],
				Insol.sensorValueBuf[2], Insol.sensorValueBuf[3],
				Insol.sensorValueBuf[4], 
				Insol.sensorValueBuf[5], Insol.sensorValueBuf[6], 
				Insol.sensorValueBuf[7],
				Insol.sensorValueBuf[8], Insol.sensorValueBuf[9],
				Insol.sensorValueBuf[10], Insol.sensorValueBuf[11],
				Insol.sensorValueBuf[12], 
				Insol.sensorValueBuf[13], Insol.sensorValueBuf[14], 
				Insol.sensorValueBuf[15], Insol.sensorValueBuf[16], 
				Insol.sensorValueBuf[17], Insol.sensorValueBuf[18], 
				Insol.sensorValueBuf[19], Insol.sensorValueBuf[20] 
				);
			#else //5 sensor data send
			printf("%d,%d\r\n", 
				4096-Insol.sensorValueBuf[0], 
				//4096-Insol.sensorValueBuf[5], 
				//4096-Insol.sensorValueBuf[8],
				//4096-Insol.sensorValueBuf[13], 
				4096-Insol.sensorValueBuf[18]
				);
			#endif
			Insol.led.canCnt++;
			if (Insol.led.canCnt > LED_CAN_TX_BLINK_TIME) {
				LED_CAN_TX_BLINK;
				Insol.led.canCnt = 0;
			}
#endif
		}
		InsolTimer.tick = 0;
	}

	
	Insol.can.rv = 0;
	if (Insol.data.outputType&DOT_CAN) {
		//if (Insol.can.complete > 0) Insol.can.complete--;    
		//if (Insol.can.complete != 0) return;
		CANDrv_SetStdID(Insol.can.txid + Insol.sendOrder);

		for (i = (Insol.sendOrder<<2); i < (Insol.sendOrder<<2) + (CanInfo.tx.cnt>>1); i++) {
			CanInfo.tx.buf[i<<1] = (uint8_t)((*CanInfo.tx.ptr[i])&0xff);
			CanInfo.tx.buf[(i<<1)+1] = (uint8_t)((*CanInfo.tx.ptr[i])>>8);
		}
		
		Insol.can.rv = CANDrv_WriteFile(CanInfo.tx.buf + (Insol.sendOrder<<3), CanInfo.tx.cnt);
		//if (!Insol.can.rv) return;
		Insol.sendOrder++;
		//if (Insol.sendOrder > INSOL_SOURCE_TOTLAL_NUM) Insol.sendOrder = 0;
		
		if (Insol.sensorONNum > ((Insol.sendOrder+1)<<2)) {
			CanInfo.tx.cnt = 8;
		}
		else {
			if (Insol.sensorONNum > (Insol.sendOrder<<2)) {
				CanInfo.tx.cnt = (Insol.sensorONNum - ((Insol.sendOrder<<2)))<<1;
			}
			else {
				if (Insol.sensorONNum < 4) {
					CanInfo.tx.cnt = Insol.sensorONNum<<1;
				}
				else {
					CanInfo.tx.cnt = 8;
				}
				Insol.sendOrder = 0;
				InsolTimer.tick = 0;
			}
		}
		
		Insol.led.canCnt++;
		if (Insol.led.canCnt > LED_CAN_TX_BLINK_TIME) {
			LED_CAN_TX_BLINK;
			Insol.led.canCnt = 0;
		}
		InsolTimer.tick = 0;
	}
	
	if (Insol.can.rv == -1) {
		if (!Insol.data.sendTimeBackup) Insol.data.sendTimeBackup = Insol.data.sendTime;
		Insol.data.sendTime = CAN_ERR_RECHECK_TIME;
	}
	else {
		if (Insol.data.sendTimeBackup > 0) {
			Insol.data.sendTime = Insol.data.sendTimeBackup;
			Insol.data.sendTimeBackup = 0;
		}
	}

	if (Insol.led.adcCnt > LED_ADC_RUN_BLINK_TIME) {
		LED_ADC_RUN_BLINK;
		Insol.led.adcCnt = 0;
	}

}

void Insol_RunMode_Normal_8bit(void)
{
	int i;//, j;
	
  	if (Insol.adc.complete == 0) return;

	if (InsolTimer.tick < Insol.data.sendTime) return;
	
	if (Insol.data.outputType&DOT_UART) {
		//if (!Insol.sendOrder) 
		{	
			printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d,\r\n", 
				Insol.sensorValueBuf[0], Insol.sensorValueBuf[1],
				Insol.sensorValueBuf[2], Insol.sensorValueBuf[3],
				Insol.sensorValueBuf[4], 
				Insol.sensorValueBuf[5], Insol.sensorValueBuf[6], 
				Insol.sensorValueBuf[7],
				Insol.sensorValueBuf[8], Insol.sensorValueBuf[9],
				Insol.sensorValueBuf[10], Insol.sensorValueBuf[11],
				Insol.sensorValueBuf[12], 
				Insol.sensorValueBuf[13], Insol.sensorValueBuf[14], 
				Insol.sensorValueBuf[15], Insol.sensorValueBuf[16], 
				Insol.sensorValueBuf[17], Insol.sensorValueBuf[18], 
				Insol.sensorValueBuf[19], Insol.sensorValueBuf[20] 
				);
		}
		InsolTimer.tick = 0;
	}

	
	Insol.can.rv = 0;
	if (Insol.data.outputType&DOT_CAN) {

		CANDrv_SetStdID(Insol.can.txid + Insol.sendOrder);

		for (i = (Insol.sendOrder<<3); i < (Insol.sendOrder<<3) + CanInfo.tx.cnt; i++) {
			CanInfo.tx.buf[i] = (*CanInfo.tx.ptr[i])>>Insol.data.shift;
		}

		Insol.can.rv = CANDrv_WriteFile(CanInfo.tx.buf + (Insol.sendOrder<<3), CanInfo.tx.cnt);

		Insol.sendOrder++;
		//if (Insol.sendOrder > INSOL_SOURCE_TOTLAL_NUM) Insol.sendOrder = 0;

		if (Insol.sensorONNum > ((Insol.sendOrder+1)<<3)) {
			CanInfo.tx.cnt = 8;
		}
		else {
			if (Insol.sensorONNum > (Insol.sendOrder<<3)) {
				CanInfo.tx.cnt = Insol.sensorONNum - (Insol.sendOrder<<3);
			}
			else {
				if (Insol.sensorONNum < 8) {
					CanInfo.tx.cnt = Insol.sensorONNum;
				}
				else {
					CanInfo.tx.cnt = 8;
				}
				Insol.sendOrder = 0;
				InsolTimer.tick = 0;
			}
		}

		Insol.led.canCnt++;
		if (Insol.led.canCnt > LED_CAN_TX_BLINK_TIME) {
			LED_CAN_TX_BLINK;
			Insol.led.canCnt = 0;
		}

		if (Insol.can.rv == -1) {
			Insol.data.sendTimeBackup = Insol.data.sendTime;
			Insol.data.sendTime = CAN_ERR_RECHECK_TIME;
		}
		else {
			if (Insol.data.sendTimeBackup > 0) {
				Insol.data.sendTime = Insol.data.sendTimeBackup;
				Insol.data.sendTimeBackup = 0;
			}
		}
	}


	if (Insol.led.adcCnt > LED_ADC_RUN_BLINK_TIME) {
		LED_ADC_RUN_BLINK;
		Insol.led.adcCnt = 0;
	}
	
}

void Insol_RunMode_Normal_4bit(void)
{
	int i;//, j;
	
  	if (Insol.adc.complete == 0) return;

	if (InsolTimer.tick < Insol.data.sendTime) return;
	
	if (Insol.data.outputType&DOT_UART) {
		//if (!Insol.sendOrder) 
		{	
			printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d,\r\n", 
				Insol.sensorValueBuf[0], Insol.sensorValueBuf[1],
				Insol.sensorValueBuf[2], Insol.sensorValueBuf[3],
				Insol.sensorValueBuf[4], 
				Insol.sensorValueBuf[5], Insol.sensorValueBuf[6], 
				Insol.sensorValueBuf[7],
				Insol.sensorValueBuf[8], Insol.sensorValueBuf[9],
				Insol.sensorValueBuf[10], Insol.sensorValueBuf[11],
				Insol.sensorValueBuf[12], 
				Insol.sensorValueBuf[13], Insol.sensorValueBuf[14], 
				Insol.sensorValueBuf[15], Insol.sensorValueBuf[16], 
				Insol.sensorValueBuf[17], Insol.sensorValueBuf[18], 
				Insol.sensorValueBuf[19], Insol.sensorValueBuf[20] 
				);
		}
		InsolTimer.tick = 0;
	}

	
	Insol.can.rv = 0;
	if (Insol.data.outputType&DOT_CAN) {

		CANDrv_SetStdID(Insol.can.txid + Insol.sendOrder);
		
		for (i = Insol.sendOrder<<4; i < (Insol.sendOrder<<4)+(CanInfo.tx.cnt<<1); i++) {
			CanInfo.tx.buf[(i>>1)] = ((*CanInfo.tx.ptr[i])>>8) & 0x0f;
			CanInfo.tx.buf[(i>>1)] |= ((((*CanInfo.tx.ptr[i+1])>>8) & 0x0f) <<4);
		}
		
		Insol.can.rv = CANDrv_WriteFile(&CanInfo.tx.buf[Insol.sendOrder<<3], CanInfo.tx.cnt);

		Insol.sendOrder++;
		//if (Insol.sendOrder > INSOL_SOURCE_TOTLAL_NUM) Insol.sendOrder = 0;

		if (Insol.sensorONNum > ((Insol.sendOrder+1)<<4)) {
			CanInfo.tx.cnt = 8;
		}
		else {
			if (Insol.sensorONNum > (Insol.sendOrder<<4)) {
				CanInfo.tx.cnt = ((Insol.sensorONNum - (Insol.sendOrder<<4))>>1);
			}
			else {
				if (Insol.sensorONNum < 16) {
					CanInfo.tx.cnt = (Insol.sensorONNum>>1);
				}
				else {
					CanInfo.tx.cnt = 8;
				}
				Insol.sendOrder = 0;
				InsolTimer.tick = 0;
			}
		}

		Insol.led.canCnt++;
		if (Insol.led.canCnt > LED_CAN_TX_BLINK_TIME) {
			LED_CAN_TX_BLINK;
			Insol.led.canCnt = 0;
		}
	}

	
	if (Insol.can.rv == -1) {
		Insol.data.sendTimeBackup = Insol.data.sendTime;
		Insol.data.sendTime = CAN_ERR_RECHECK_TIME;
	}
	else {
		if (Insol.data.sendTimeBackup > 0) {
			Insol.data.sendTime = Insol.data.sendTimeBackup;
			Insol.data.sendTimeBackup = 0;
		}
	}

	if (Insol.led.adcCnt > LED_ADC_RUN_BLINK_TIME) {
		LED_ADC_RUN_BLINK;
		Insol.led.adcCnt = 0;
	}
}

void Insol_SetRunMode(uint8_t runmode)
{
	Insol.sourcePos = 0;
	Insol.adc.complete = 0;

	if (Insol.runMode == runmode) return;

	switch (runmode) {
	case RM_STANDBY:
	case RM_SLEEP:
		//Insol.runMode = RM_STANDBY;
		fnADCDrv_Copy2Buf = ADCDrv_Copy2Buf;
		fnInsol_RunMode = Insol_RunMode_Standby;
		//HAL_ADC_Stop_DMA(&hadc1);
		LED_ADC_RUN_OFF;
		LED_CAN_TX_OFF;
		break;
	case RM_NORMAL_4BIT:
		//Insol.runMode = RM_NORMAL_4BIT;
		fnADCDrv_Copy2Buf = ADCDrv_Copy2Buf;
		fnInsol_RunMode = Insol_RunMode_Normal_4bit;
		Insol_SetBufPtr(Insol.data.size, Insol.sensorONPos);
		HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&AdcDmaBuf[1], NumOfAdcChan );
		if (Insol.sensorONNum > 15) CanInfo.tx.cnt = 8;
		else CanInfo.tx.cnt = Insol.sensorONNum>>1;
		break;
	case RM_NORMAL_8BIT:
		//Insol.runMode = RM_NORMAL_8BIT;
		fnADCDrv_Copy2Buf = ADCDrv_Copy2Buf;
		fnInsol_RunMode = Insol_RunMode_Normal_8bit;
		//CanInfo.tx.ptr8 = (uint8_t *)CanInfo.tx.buf;
		Insol_SetBufPtr(Insol.data.size, Insol.sensorONPos);
		HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&AdcDmaBuf[1], NumOfAdcChan );
		if (Insol.sensorONNum > 7) CanInfo.tx.cnt = 8;
		else CanInfo.tx.cnt = Insol.sensorONNum;
		break;
	case RM_NORMAL_12BIT:
		//Insol.runMode = RM_NORMAL_12BIT;
		fnADCDrv_Copy2Buf = ADCDrv_Copy2Buf;
		fnInsol_RunMode = Insol_RunMode_Normal_12bit;
		Insol_SetBufPtr(Insol.data.size, Insol.sensorONPos);
		HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&AdcDmaBuf[1], NumOfAdcChan );
		if (Insol.sensorONNum > 3) CanInfo.tx.cnt = 8;
		else CanInfo.tx.cnt = Insol.sensorONNum<<1;
		break;
	case RM_CALIBRATION:
		Insol.adc.cnt = 1;		// avg varible
		//Insol.runMode = RM_CALIBRATION;
		fnADCDrv_Copy2Buf = ADCDrv_Copy2CalBuf;
		HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&AdcDmaBuf[1], NumOfAdcChan );
		break;
	default:
		break;
	}
	
	//Insol.oldRunMode = runmode;//Insol.runMode;
	Insol.runMode = runmode;
	
}

uint32_t Insol_LoadParamFromFlash(char *buf)
{
	SYSTEM_INFO *pFSSysInfo;
	uint32_t addr;
	int i;

#ifdef SUPPORT_MCU_FLASH
	addr = FLASH_SYSTEM_INFO_ADDR;
	for (i = 0; i < FLASH_USER_END_ADDR; i += FLASH_PAGE_SIZE1) {
		FlashDrv_Read(addr, buf, sizeof(SYSTEM_INFO));
		pFSSysInfo = (SYSTEM_INFO *)buf;
		if (pFSSysInfo->company[0] != 'H' ||
			pFSSysInfo->company[1] != 'E' ||
			pFSSysInfo->company[2] != 'X') {
			return 0;
		}
		else {
			if (pFSSysInfo->writeTime > FLASH_ENDURANCE) {
				if (pFSSysInfo->replaceNode == 0xffffffff || 
					pFSSysInfo->replaceNode == 0) return 0;
				else {
					addr = pFSSysInfo->replaceNode;
				}
			}
			else {
				break;
			}
		}
	}

	return addr;
#else 
	return 0;
#endif
}

int Insol_SaveParamToFlash(char *buf)
{
	SYSTEM_INFO *pFSSysInfo;
	int i;
	uint32_t addr;

#ifdef SUPPORT_MCU_FLASH
	addr = Insol_LoadParamFromFlash(buf);

	pFSSysInfo = (SYSTEM_INFO *)buf;
	//pFSSysInfo->setup.vol = Setup.vol;
	//pFSSysInfo->setup.bright = Setup.bright;
	//pFSSysInfo->setup.angle = Setup.angle;
	//pFSSysInfo->setup.speed = Setup.speed;
	//pFSSysInfo->setup.repeat = Setup.repeat;
	//pFSSysInfo->setup.lsrpt_en = Setup.lsrpt_en;
	//pFSSysInfo->setup.sndGuid = Setup.sndGuid;
	//pFSSysInfo->setup.hiAngle = Setup.hiAngle;

	pFSSysInfo->tag[0] = 'H';
	pFSSysInfo->tag[1] = 'X';
	for (i = 0; i < COMPANY_LENGTH; i++) pFSSysInfo->company[i] = COMPANY_t[i];
	for (i = 0; i < MODEL_LENGTH; i++) pFSSysInfo->model[i] = MODEL_t[i];
	pFSSysInfo->swVer = FW_VER;
	pFSSysInfo->hwVer = HW_VER;
	pFSSysInfo->createDate[0] = FW_CREATE_YEAR;
	pFSSysInfo->createDate[1] = FW_CREATE_MONTH;
	pFSSysInfo->createDate[2] = FW_CREATE_DAY;
	pFSSysInfo->createTime[0] = 16;
	pFSSysInfo->createTime[1] = 20;
	pFSSysInfo->createTime[2] = 40;
	pFSSysInfo->modifyTime[0] = 'a';
	pFSSysInfo->modifyTime[1] = 'b';
	pFSSysInfo->modifyTime[2] = 'c';
	
	pFSSysInfo->nextNode = 0;
	pFSSysInfo->replaceNode = 0;
	pFSSysInfo->replaceNode = 0x12345678;

	pFSSysInfo->setup.canTxID = Insol.can.txid;
	pFSSysInfo->setup.canRxID = Insol.can.rxid;
		
	if (addr >= FLASH_SYSTEM_INFO_ADDR) {
		pFSSysInfo->writeTime++;
		if (!FlashDrv_Write(addr, buf, sizeof(SYSTEM_INFO))) return 0;
	}
	else {
		pFSSysInfo->writeTime = 1;
		if (!FlashDrv_Write(FLASH_SYSTEM_INFO_ADDR, 
					buf, sizeof(SYSTEM_INFO))) return 0;
	}

	return 1;
#else
	return 0;
#endif
}

/*
uint8_t Insol_GetSensorONNum(uint32_t data)
{
	int i;
	uint8_t cnt;

	cnt = 0;
	for (i = 0; i < 32; i++) {
		if (data & 0x1) cnt++;
		data >>= 1;
	}
	return cnt;
}
*/

void Insol_SetBufPtr(uint8_t data_size, uint32_t data)
{
	int i;
	uint8_t cnt;
	//uint8_t *ptr;
	
	cnt = 0;
	//ptr = (uint8_t *)Insol.sensorValueBuf;
	
	for (i = 0; i < 32; i++) {
		if (data & 0x1) {
			//ptr = (uint8_t *)&Insol.sensorValueBuf[i];
			//CanInfo.tx.ptr[cnt] = ptr+1;
			CanInfo.tx.ptr[cnt] = &Insol.sensorValueBuf[i];
			cnt++;
			if (cnt >= INSOL_SENSOR_NUM) break;
		}
		data >>= 1;
	}
	Insol.sensorONNum = cnt;
	
/*
	switch (data_size) {
	case DS_4BIT:
		for (i = 0; i < 32; i++) {
			if (data & 0x1) {
				//ptr = (uint8_t *)&Insol.sensorValueBuf[i];
				//CanInfo.tx.ptr[cnt] = ptr+1;
				CanInfo.tx.ptr[cnt] = &Insol.sensorValueBuf[i];
				cnt++;
			}
			data >>= 1;
		}
		Insol.sensorONNum = cnt;
		break;
	case DS_8BIT_MSB8:
		for (i = 0; i < 32; i++) {
			if (data & 0x1) {
				//ptr = (uint8_t *)&Insol.sensorValueBuf[i];
				//CanInfo.tx.ptr[cnt] = ptr+1;
				CanInfo.tx.ptr[cnt] = &Insol.sensorValueBuf[i];
				cnt++;
			}
			data >>= 1;
		}
		Insol.sensorONNum = cnt;
		break;
	case DS_8BIT_256STEP:
		for (i = 0; i < 32; i++) {
			if (data & 0x1) {
				//ptr = (uint8_t *)&Insol.sensorValueBuf[i];
				//CanInfo.tx.ptr[cnt] = ptr+1;
				CanInfo.tx.ptr[cnt] = &Insol.sensorValueBuf[i];
				cnt++;
			}
			data >>= 1;
		}
		Insol.sensorONNum = cnt;
		break;
	case DS_12BIT:
		for (i = 0; i < 32; i++) {
			if (data & 0x1) {
				//ptr = (uint8_t *)&Insol.sensorValueBuf[i];
				//CanInfo.tx.ptr[cnt] = ptr+1;
				CanInfo.tx.ptr[cnt] = &Insol.sensorValueBuf[i];
				cnt++;
			}
			data >>= 1;
		}
		Insol.sensorONNum = cnt;
		break;
	default:
		break;
	}
*/
}

void Insol_Print2Uart_SystemInfo(void)
{
	SYSTEM_INFO *psi;
	
	if (Insol_LoadParamFromFlash(CommonBuf)) {
		psi = (SYSTEM_INFO *)CommonBuf;
		if (psi->setup.canTxID < 0x1FFFFFFF) Insol.can.txid = psi->setup.canTxID;
		if (psi->setup.canRxID < 0x1FFFFFFF) Insol.can.rxid = psi->setup.canRxID;
	}
	
	printf("------- HEXAR SYSTEMS -------\r\n");
	printf("F/W Version : %d.%d\r\n", (FW_VER&0x0)>>4, FW_VER&0x0f);
	printf("User : %c\r\n", FW_USER);
	printf("CAN ID : Tx : %Xh, Rx : %Xh\r\n", Insol.can.txid, Insol.can.rxid);
	printf("Create date : %d/%d/%d %d:%d\r\n", 
		FW_CREATE_YEAR, FW_CREATE_MONTH, FW_CREATE_DAY, 
		FW_CREATE_HOUR, FW_CREATE_MIN);
	printf("------- Insol Sensor -------\r\n");
}

void Insol_Init(void)
{
	int i;
	
	LED_ADC_RUN_BLINK;
	LED_CAN_TX_BLINK;

	Insol.sendOrder = 0;
	Insol.led.adcCnt = 0;
	Insol.led.canCnt = 0;
	Insol.data.sendTime = 10;
	Insol.data.sendTimeBackup = 0;
	fnInsol_RunMode = Insol_RunMode_Standby;
	Insol.runMode = Insol.oldRunMode = RM_STANDBY;
	#ifdef SUPPORT_UART_PRINT
	Insol.data.outputType = DOT_UART;
	#else
	Insol.data.outputType = DOT_CAN;
	#endif

	//Insol.data.sendTime = 50;
	#ifdef SUPPORT_ALL_SENSOR
	Insol.data.size = DS_12BIT;
	Insol.sensorONPos = 0x001fffff; // all sensor
	#else
	Insol.data.size = DS_8BIT;	//for h4
	Insol.sensorONPos = 0x001e0087;	//for h4
	//Insol.sensorONPos = 0x0000000f;
	#endif
	Insol.data.shift = 4;	//256 level
	Insol_SetBufPtr(Insol.data.size, Insol.sensorONPos);
	if (Insol.sensorONNum < 9) {
		CanInfo.tx.cnt = Insol.sensorONNum;
	}
	else {
		CanInfo.tx.cnt = 8;
	}

	Insol.data.processType = 0;//0x40;
	
	for (i = 0; i < INSOL_SENSOR_NUM; i++) {
		ForceConstant[i] = 1;
	}

	LED_ADC_RUN_OFF;
	LED_CAN_TX_ON;
	
#ifdef SUPPORT_MCU_FLASH
	FlashDrv_SetTempBuf(&CommonBuf[FLASH_PAGE_SIZE1]);
	FlashDrv_SetParam(FLASH_PAGE_SIZE1);
#endif
}

#if 0
#include "stdarg.h"
#include "hal_uart.h"
#include 
#include "hal_types.h"

void printf(char *format, ...);

static void sendByte(unsigned char byte)
{
    HalUARTWrite(HAL_UART_PORT_0, &byte, 1); //change port to suit your needs
}

static void putc(unsigned char c)
{
    sendByte(c);
}

static void puts(uint8 *str)
{
    HalUARTWrite(HAL_UART_PORT_0, str, strlen((const char*)str)); //change port to suit your needs
}

static const unsigned long dv[] =
{
    // 4294967296 // 32 bit unsigned max
    1000000000,// +0
    100000000, // +1
    10000000, // +2
    1000000, // +3
    100000, // +4
    // 65535 // 16 bit unsigned max
    10000, // +5
    1000, // +6
    100, // +7
    10, // +8
    1, // +9
};

static void xtoa(unsigned long x, const unsigned long *dp)
{
    char c;
    unsigned long d;

    if (x)
    {
        while (x < *dp)
            ++dp;
        do
        {
            d = *dp++;
            c = '0';
            while (x >= d)
                ++c, x -= d;
            putc(c);
        } while (!(d & 1));
    } else
        putc('0');
}

static void puth(unsigned n)
{
    static const char hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8',
        '9', 'A', 'B', 'C', 'D', 'E', 'F' };

    putc(hex[n & 15]);
}

void printf(char *format, ...)
{
    char c;
    int i;
    long n;

    va_list a;
    va_start(a, format);
    while(c = *format++)
    {
        if(c == '%')
        {
            switch(c = *format++)
            {
                case 's': // String
                    puts(va_arg(a, char*));
                    break;

                case 'c':// Char
                    putc(va_arg(a, char));
                    break;

                case 'i':// 16 bit Integer
                case 'u':// 16 bit Unsigned
                    i = va_arg(a, int);
                    if(c == 'i' && i < 0)
                        i = -i, putc('-');
                    xtoa((unsigned)i, dv + 5);
                    break;

                case 'l':// 32 bit Long
                case 'n':// 32 bit uNsigned loNg
                    n = va_arg(a, long);
                    if(c == 'l' && n < 0)
                        n = -n, putc('-');
                    xtoa((unsigned long)n, dv);
                    break;

                case 'x':// 16 bit heXadecimal
                    i = va_arg(a, int);
                    puth(i >> 12);
                    puth(i >> 8);
                    puth(i >> 4);
                    puth(i);
                    break;

                case 0:
                    return;

                default:
                    goto bad_fmt;
            }
        }
        else
            bad_fmt: putc(c);
    }

    va_end(a);
}
//출처: http://ingorae.tistory.com/1423 [잉고래의 잇다이어리]
#endif

