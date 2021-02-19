#pragma once

#define	CAN_OPEN_DSP402										0x00020192		//	Draft Standard Proposal 402

#define	CAN_OPEN_CONTROL									0x6040	//	Unsigned 16bit, RW
	#define	CAN_OPEN_SWITCH_ON									0x0001
	#define	CAN_OPEN_ENABLE_VOLTAGE								0x0002
	#define	CAN_OPEN_QUICK_STOP									0x0004
	#define	CAN_OPEN_ENABLE_OPERATION							0x0008
	#define	CAN_OPEN_FAULT_RESET									0x0080
	//pjg++170420
	#define	CAN_OPEN_SWITCH_OFF									0x0010
	#define	CAN_OPEN_DISABLE_OPERATION							0x0012
	#define	CAN_OPEN_SHUTDOWN									0x0013
	#define	CAN_OPEN_SET_INTERVAL_SEND_DATA						0x0014
	#define	CAN_OPEN_GET_INFORMATION							0x0015
	#define	CAN_OPEN_RUN_CALIBRATION								0x0016
	#define	CAN_OPEN_SET_DEVICE_NUM								0x0017
	#define	CAN_OPEN_SET_DATA_SIZE								0x0018
	#define	CAN_OPEN_SET_DATA_PROC_TYPE							0x0019
	#define	CAN_OPEN_SET_DATA_OUTPUT_TYPE						0x001A
	#define	CAN_OPEN_SEL_SENSOR_POS								0x001B

	#define	CAN_OPEN_CONTROL_SHUTDOWN							0x0006
	#define	CAN_OPEN_CONTROL_SWITCH_ON							0x0007		
	#define	CAN_OPEN_CONTROL_DISABLE_VOLTAGE					0x0000
	#define	CAN_OPEN_CONTROL_QUICK_STOP							0x0010
	#define	CAN_OPEN_CONTROL_DISABLE_OPERATION					0x0007
	#define	CAN_OPEN_CONTROL_ENABLE_OPERATION					0x000F
	#define	CAN_OPEN_CONTROL_FAULT_RESET						0x0080

#define	CAN_OPEN_STATUS										0x6041	//	Unsigned 16bit,	RO
	#define	CAN_OPEN_READY_TO_SWITCH_ON						0x0001
	#define	CAN_OPEN_SWITCHED_ON								0x0002
	#define	CAN_OPEN_OPERATION_ENABLE							0x0004
	#define	CAN_OPEN_FAULT										0x0008
	#define	CAN_OPEN_VOLTAGE_ENABLED							0x0010
	#define	CAN_OPEN_IS_QUICK_STOP								0x0020
	#define	CAN_OPEN_SWITCH_ON_DISABLE							0x0040
	#define	CAN_OPEN_WARNING										0x0080
	#define	CAN_OPEN_OFFSET_CURRENT_MEASURED					0x0100
	#define	CAN_OPEN_TARGET_REACHED								0x0200
	#define	CAN_OPEN_REFRESH_CYCLE_OF_POWER_STAGE			0x4000
	#define	CAN_OPEN_POSITION_REFERECED_TO_HOME_POSITION		0x8000
	//pjg++170420
	#define	CAN_OPEN_GET_STATUS									0x0003

	#define	CAN_OPEN_STATUS_BOOTUP								0x0000			//	Bootup
	#define	CAN_OPEN_STATUS_NOT_READY_TO_SWITCH_ON			0x0100			//	The current offset will be measured. The drive function is disabled.
	#define	CAN_OPEN_STATUS_SWITCH_ON_DISABLED					0x0140			//	The drive initialization is complete. The driver parameters may be changed. The driver function is disabled.
	#define	CAN_OPEN_STATUS_READY_TO_SWITCH_ON				0x0121			//	The drive parameters may be changed. The drive function is disabled.
	#define	CAN_OPEN_STATUS_SWITCH_ON							0x0123			//	The drive function is disabled.
	#define	CAN_OPEN_STATUS_REFRESH								0x4123			//	Refresh power stage.
	#define	CAN_OPEN_STATUS_MEASURE_INIT						0x4133			//	The power is applied to motor. The motor resistance or the communtation delay is measured.
	#define	CAN_OPEN_STATUS_OPERATION_ENABLE					0x0137			//	No faults have been detected. The drive function is enabled and power is applied to motor.
	#define	CAN_OPEN_STATUS_QUICK_STOP_ACTIVE					0x0117			//	The quick stop function is being executed. The drive function is enabled and power is applied to motor.
	#define	CAN_OPEN_STATUS_FAULT_REACTION_ACTIVE_DISABLED	0x010F			//	A fault has occurred in the drive. The drive function is disabled.
	#define	CAN_OPEN_STATUS_FAULT_REACTION_ACTIVE_ENABLED		0x011F			//	A fault has occurred in the drive. The quick stop function is being executed. The drive function is enabled and power is applied to motor
	#define	CAN_OPEN_STATUS_FAULT								0x0108			//	A fault has occurred in the drive. The drive parameters may be changed. The drive function is disabled.

#define	CAN_OPEN_MODES_OF_OPERATION							0x6060,	0x00	//	Signed 8bit,	RW
	#define	CAN_OPEN_INTERPOLATED_POSITION_MODE				0x07
	#define	CAN_OPEN_HOMING_MODE								0x06
	#define	CAN_OPEN_PROFILE_VELOCITY_MODE						0x03
	#define	CAN_OPEN_PROFILE_POSITION_MODE						0x01
	#define	CAN_OPEN_POSITION_MODE								0xFF
	#define	CAN_OPEN_VELOCITY_MODE								0xFE
	#define	CAN_OPEN_CURRENT_MODE								0xFD
	#define	CAN_OPEN_CURRENT_AUTO_TUNE_MODE						0xFC
	#define	CAN_OPEN_VELOCITY_AUTO_TUNE_MODE					0xFB
	#define	CAN_OPEN_POSITION_AUTO_TUNE_MODE					0xFA

#define	CAN_OPEN_ACTUAL_POSITION							0x6064,	0x00	//	Signed 32Bit,	RO	Unit	(-2147483648 ~ 2147483647)

#define	CAN_OPEN_ACTUAL_VELOCITY							0x606C,	0x00	//	Signed 32Bit,	RO	RPM		(-2147483648 ~ 2147483647)

#define	CAN_OPEN_ACTUAL_CURRENT								0x6078,	0x00	//	Signed 16Bit,	RO	mA		(-32768 ~ 32767)


#define	CAN_OPEN_POSITION_MODE_SETTING_VALUE				0x2062,	0x00	//	Signed 32Bit,	RW	Unit	(-2147483648 ~ 2147483647)

#define	CAN_OPEN_TARGET_POSITION							0x607A,	0x00	//	Signed 32Bit,	RW	Unit	(-2147483648 ~ 2147483647)
#define	CAN_OPEN_POSITION_RANGE_LIMIT						0x607B,	0x00
#define	CAN_OPEN_MINIMAL_POSITION_RANGE_LIMIT				0x607B,	0x01	//	Signed 32Bit,	RW	Unit	(-2147483648 ~ 2147483647)
#define	CAN_OPEN_MAXIMAL_POSITION_RANGE_LIMIT				0x607B,	0x02	//	Signed 32Bit,	RW	Unit	(-2147483648 ~ 2147483647)
#define	CAN_OPEN_POSITION_SW_LIMIT							0x607D,	0x00
#define	CAN_OPEN_MINIMAL_POSITION_SW_LIMIT					0x607D,	0x01	//	Signed 32Bit,	RW	Unit	(-2147483648 ~ 2147483647)
#define	CAN_OPEN_MAXIMAL_POSITION_SW_LIMIT					0x607D,	0x02	//	Signed 32Bit,	RW	Unit	(-2147483648 ~ 2147483647)
#define	CAN_OPEN_MAXIMAL_PROFILE_VELOCITY					0x607F,	0x00	//	Unsigned 32Bit,	RW	RPM		(1 ~ 25000)
#define	CAN_OPEN_PROFILE_VELOCITY							0x6081,	0x00	//	Unsigned 32Bit, RW	RPM		(1 ~ 25000)
#define	CAN_OPEN_PROFILE_ACCELERATION						0x6083,	0x00	//	Unsigned 32Bit, RW	RPM/s	(1 ~ 1000000000)
#define	CAN_OPEN_PROFILE_DECELERATION						0x6084,	0x00	//	Unsigned 32Bit, RW	RPS/s	(1 ~ 1000000000)
#define	CAN_OPEN_QUICK_STOP_DECELERATION					0x6085,	0x00	//	Unsigned 32Bit, RW	RPM/s	(1 ~ 1000000000)
#define	CAN_OPEN_MOTION_PROFILE_TYPE						0x6086,	0x00	//	Signed 16Bit,	RW
#define	CAN_OPEN_MAX_ACCELERATION							0x60C5,	0x00	//	Unsigned 32Bit, RW	RPM/s	(1 ~ 1000000000)
	#define	CAN_OPEN_TRAPEZOIDAL_PROFILE						0x0000
	#define	CAN_OPEN_SINUSOIDAL_PROFILE							0x0001			//	Not yet implemented


#define	CAN_OPEN_POSITION_NOTATION_INDEX					0x6089, 0x00	//	Signed 8Bit,	RW	
#define	CAN_OPEN_POSITION_DIMENSION_INDEX					0x608A, 0x00	//	Unsigned 8Bit,	RW
#define	CAN_OPEN_VELOCITY_NOTATION_INDEX					0x608B,	0x00	//	Signed 8Bit,	RW	
#define	CAN_OPEN_VELOCITY_DIMENSION_INDEX					0x608C,	0x00	//	Unsigned 8Bit,	RW
#define	CAN_OPEN_ACCELERATION_NOTATION_INDEX				0x608D,	0x00	//	Signed 8Bit,	RW
#define	CAN_OPEN_ACCELERATION_DIMENSION_INDEX				0x608E,	0x00	//	Unsigned 8Bit,	RW


#define	CAN_OPEN_CURRENT_PARAMETER							0x60F6,	0x00	//
#define	CAN_OPEN_CURRENT_P_GAIN								0x60F6,	0x01	//	Unsigned 16Bit,	RW	10^-3	(0 ~ 65535)
#define	CAN_OPEN_CURRENT_I_GAIN								0x60F6,	0x02	//	Unsigned 16Bit,	RW	10^0	(0 ~ 65535)

#define	CAN_OPEN_VELOCITY_PARAMETER							0x60F9,	0x00
#define	CAN_OPEN_VELOCITY_P_GAIN							0x60F9,	0x01	//	Unsigned 16Bit,	RW	10^-4	(0 ~ 65535)
#define	CAN_OPEN_VELOCITY_I_GAIN							0x60F9,	0x02	//	Unsigned 16Bit,	RW	10^-4	(0 ~ 65535)

#define	CAN_OPEN_POSITION_PARAMETER							0x60FB,	0x00
#define	CAN_OPEN_POSITION_P_GAIN							0x60FB,	0x01	//	Unsigned 16Bit,	RW	10^-3	(0 ~ 65535)
#define	CAN_OPEN_POSITION_I_GAIN							0x60FB,	0x02	//	Unsigned 16Bit,	RW	10^-3	(0 ~ 65535)
#define	CAN_OPEN_POSITION_D_GAIN							0x60FB,	0x03	//	Unsigned 16Bit,	RW	10^-3	(0 ~ 65535)
#define	CAN_OPEN_POSITION_VFF_FACTOR						0x60FB,	0x04	//	Unsigned 16Bit,	RW	10^-3	(0 ~ 65535)
#define	CAN_OPEN_POSITION_AFF_FACTOR						0x60FB,	0x05	//	Unsigned 16Bit,	RW	10^-3	(0 ~ 65535)

#define	CAN_OPEN_TARGET_VELOCITY							0x60FF,	0x00	//	Signed 32Bit,	RW	RPM		(-2147483648 ~ 2147483647)

#define	CAN_OPEN_MOTOR_TYPE									0x6402, 0x00	//	Unsigned 16Bit,	RW
	#define	CAN_OPEN_DC_MOTOR									0x0001			//	Brushed DC Motor
	#define	CAN_OPEN_PMSM_MOTOR									0x000A			//	Sinusoidal PM BL Motor
	#define	CAN_OPEN_BLDC_MOTOR									0x000B			//	Trapezoidal PM BL Motor

#define	CAN_OPEN_MOTOR_DATA									0x6410,	0x00	//	Unsigned 8bit,	RO
	#define	CAN_OPEN_CONTINUOUS_CURRENT_LIMIT					0x6410,	0x01	//	Unsigned 16Bit,	RW
	#define	CAN_OPEN_OUTPUT_CURRENT_LIMIT						0x6410, 0x02	//	Unsigned 16Bit, RW
	#define	CAN_OPEN_POLE_PAIR_NUMBER							0x6410, 0x03	//	Unsigned 8Bit,	RW


