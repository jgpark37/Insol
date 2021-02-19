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
* Filename		: hx_InsolSensor.h
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/02/21
* Description		: Insol Sensor
*******************************************************************************
*
*/

#ifndef _INSOL_SENSOR_H_
#define _INSOL_SENSOR_H_

/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx.h"

/* Private define ------------------------------------------------------------*/
#define CAN_RX_BUF_NUM		(20)
//#define CAN_TX_BUF_NUM			(21)
#define CAN_DATA_SEND_NUM		6
#define NumOfAdcChan 			7 // num of scanned ADC channels
#define DMABUFSIZE 				NumOfAdcChan + 2 // total elements of DMABuf-Array

#define INSOL_SOURCE_TOTLAL_NUM		5
#define INSOL_SOURCE1_NUM				5
#define INSOL_SOURCE2_NUM				3
#define INSOL_SOURCE3_NUM				5
#define INSOL_SOURCE4_NUM				4
#define INSOL_SOURCE5_NUM				4
#define INSOL_SENSOR_NUM				(INSOL_SOURCE1_NUM+\
										INSOL_SOURCE2_NUM+\
										INSOL_SOURCE3_NUM+\
										INSOL_SOURCE4_NUM+\
										INSOL_SOURCE5_NUM)

#define INSOL_SENSOR_MAX_VALUE		4096
#define LED_ORANGE						GPIO_PIN_12
#define LED_GREEN						GPIO_PIN_13

#define COMPANY_LENGTH				14
#define MODEL_LENGTH					10
#define FW_VER						0x12	//v1.0
#define HW_VER						0x01	//v0.1(17.03.19)
#define FW_USER						'H'		//user : hexar
#define FW_CREATE_YEAR				17
#define FW_CREATE_MONTH			4
#define FW_CREATE_DAY				17
#define FW_CREATE_HOUR			15
#define FW_CREATE_MIN				59
//extern const char COMPANY_t[COMPANY_LENGTH];// = {"HEXAR SYSTEMS"};
//extern const char MODEL_t[MODEL_LENGTH];// = {"INSOL"};
//extern const uint8_t FW_VER[4];// = {'1', '0', 'K', 'G'};		//v1.0.KG
//extern const uint8_t CREATE_DATE[3];// = {17, 3, 24};		//17y, 3m, 24d
//extern const uint8_t CREATE_TIME[3];// = {16, 31, 55};	//hour, min, sec

enum INSOL_MODEL_NUM {
	IMN_SELF,
	IMN_SMART,
	IMN_MAX
};

enum eSSYTEM_STATUS_CAN {
	SSC_RX_ERR				= 0x01,
	SSC_TX_ERR				= 0x02,
};

enum eSSYTEM_STATUS_MCU {
	SSM_ADC_ERR			= 0x01,
	SSM_CAN_ERR			= 0x02,
};

/* Private typedef -----------------------------------------------------------*/
typedef struct tag_INSOL{
	uint8_t sourcePos;
	uint8_t sendOrder;
	uint16_t sensorValueBuf[INSOL_SENSOR_NUM];
	struct {
		uint8_t complete;
		uint8_t cnt;
	}adc;
	struct {
		uint16_t adcCnt;
		uint16_t canCnt;
	}led;
	uint8_t runMode;
	uint8_t oldRunMode;
	struct {
		uint16_t sendTime;
		uint16_t sendTimeBackup;
		uint8_t outputType;
		uint8_t size;					//4:4bit, 8: 8bit, 12:12bit
		uint8_t shift;					//divide value (2 ^y)
		uint8_t processType;
	}data;
	uint32_t sensorONPos;
	uint8_t sensorONNum;
	struct {
		uint32_t rxid;
		uint32_t txid;
		//uint16_t chkTime;
		int rv;
		uint8_t timeout;
		uint8_t complete;
	}can;
	struct {
		uint8_t ok;
	}cal;
}INSOL;

typedef struct tag_INSOL_Timer{
	uint32_t tick;
	uint8_t temp;
}INSOL_TIMER;

typedef union tag_SYSTEM_STATUS{
	uint64_t all;
	uint8_t can1;
	uint8_t can2;
	uint8_t sensor1;
	uint8_t sensor2;
	uint8_t mcu1;
	uint8_t mcu2;
	uint8_t etc1;
	uint8_t etc2;
}SYSTEM_STATUS;

typedef struct tagCAN_INFO{
	struct {
		uint8_t head;
		uint8_t tail;
		uint8_t buf[CAN_RX_BUF_NUM][8];
	}rx;
	struct {
		uint8_t buf[INSOL_SENSOR_NUM*2+1];	// 2 is 12bit  
		uint16_t *ptr[INSOL_SENSOR_NUM+1];
		uint8_t cnt;
	}tx;
}CAN_INFO;

enum _RUN_MODE {
	RM_STANDBY,
	RM_SLEEP,
	RM_CALIBRATION,
	RM_NORMAL_4BIT,
	RM_NORMAL_8BIT,
	RM_NORMAL_12BIT,
	RM_MAX
};

enum _DATA_OUTPUT_TYPE {
	DOT_NONE,
	DOT_CAN,
	DOT_UART,
	DOT_CAN_UART
};

enum _DATA_SIZE {
	DS_4BIT = 4,
	DS_8BIT = 8,
	DS_12BIT = 12,
};

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern volatile uint16_t AdcDmaBuf[DMABUFSIZE];
extern uint16_t AdcCalBuf[INSOL_SENSOR_NUM];
extern INSOL Insol;
extern INSOL_TIMER InsolTimer;
extern SYSTEM_STATUS SysStatus;
extern CAN_INFO CanInfo;

/* Private function prototypes -----------------------------------------------*/
extern void (*fnADCDrv_Copy2Buf)(void);
extern void (*fnInsol_RunMode)(void);
extern void ADCDrv_Init();
extern void ADCDrv_SelectSensorSource(void);
extern void Insol_Init();
extern void Insol_RunMode_Calibration(void);
//extern void Insol_RunMode_Standby(void);
//extern void Insol_RunMode_Normal(void);
extern void Insol_SetRunMode(uint8_t runmode);
extern void Insol_Print2Uart_SystemInfo(void);
extern int Insol_SaveParamToFlash(char *buf);
//extern uint8_t Insol_GetSensorONNum(uint32_t data);
extern void Insol_SetBufPtr(uint8_t data_size, uint32_t data);

#endif //_INSOL_SENSOR_H_

