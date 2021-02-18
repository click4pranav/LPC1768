 #ifndef __PAC1710_H_
	#define __PAC1710_H_
	
	#include "GPIO_LPC17xx.h"
	#include "Driver_I2C.h"
	
	#define PAC1710_ADR 0x2A					// with 2k7 external R, 0101010(r/w#)
	
	#define PAC_CONFIG_REG 				0x00 
	#define PAC_CONV_REG					0x01
	#define PAC_OS_REG						0x02
	#define PAC_MAN_ID 						0xFD
	
	// CHANNEL 1 VSENSE RESULT REGISTER (ADDRESSES 0DH AND 0EH)
	#define C1SR_HIGH					0x0D					// C1SR11:C1SR:4
	#define C1SR_LOW					0x0E					// C1SR3:C1SR0	
	// CHANNEL 1 VSOURCE RESULT REGISTER (ADDRESSES 11H AND 12H)
	#define C1VR_HIGH					0x11
	#define C1VR_LOW					0x12
	// CHANNEL 1 POWER RATIO REGISTER  (ADDRESSES 15H AND 16H) (16bit output)
	#define C1P_HIGH					0x15
	#define C1P_LOW						0x16
	// CHANNEL 1 VSENSE SAMPLING CONFIGURATION REGISTER (ADDRESS 0BH)
	#define C1SR_SMP_CFG 			0x0B
	// VSOURCE SAMPLING CONFIGURATION REGISTER (ADDRESS 0AH)
	#define C1VR_SMP_CFG 			0x0A
	
	typedef struct PAC1710 {
		uint32_t vSenseCh1;
		uint32_t vSourceCh1;
		uint32_t PowerRatioCh1;
	} PAC1710interface;
	
	extern ARM_DRIVER_I2C Driver_I2C1;
	extern PAC1710interface PAC1710Regs;
	
	uint32_t InitPAC1710 (void);
	
	extern uint32_t InitPAC1710 (void);
	extern uint32_t PAC1710ReadReg(uint8_t RegAdress, uint8_t *RegVal);
	extern uint32_t PAC1710WriteReg(uint8_t RegAdress, uint8_t RegVal);
	extern uint32_t PAC1710OneShotMeasure (void);
	
#endif 
