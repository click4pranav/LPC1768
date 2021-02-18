#ifndef __OV2640_H 
	#define __OV2640_H
	
	#include "GPIO_LPC17xx.h"
	#include "Driver_I2C.h"
		
	#define OV2640_ADR 			0x30
	
	#define REG_PID         0x0A    				// Product ID MSB
	#define RA_DLMT					0xFF						// Register BANK select, 0 =  DSP, and 1  = Sensor Address
	#define REG_CLKRC				0x12
	#define REG_COM10				0x15
		
	struct SensorReg {
		uint8_t reg; 
		uint8_t val;
	};
		
	extern ARM_DRIVER_I2C Driver_I2C0;				// OV7670 SCCB interface
		
	extern uint32_t OV2640init (void);
	extern uint32_t OV2640WriteReg (uint8_t RegAddress, uint8_t RegValue);
	extern uint32_t OV2640ReadReg (uint8_t RegAdress, uint8_t *RegVal);
	extern void 		OV2640WriteRegS (const struct SensorReg RegList[]);
	extern uint32_t OV2640Setup (uint32_t CamMode);
	
	extern uint8_t CamBuf[10];
	
#endif
