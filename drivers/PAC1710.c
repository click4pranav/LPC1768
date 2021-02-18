#include "PAC1710.h"

static ARM_DRIVER_I2C *I2C1drv = &Driver_I2C1;
PAC1710interface PAC1710Regs;

uint32_t InitPAC1710 (void)
{
	uint8_t RegVal;
	
	I2C1drv->Initialize(NULL);
	I2C1drv->PowerControl(ARM_POWER_FULL);
	I2C1drv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);			// uint32_t control, uint32_t arg &  max speed 100khz
	I2C1drv->Control(ARM_I2C_BUS_CLEAR, 0);
	
	PAC1710ReadReg(PAC_MAN_ID, &RegVal);
	
	if(RegVal != 0x58)
		return 1;
	
	PAC1710WriteReg(PAC_CONV_REG, 0x01);					// 01b = 4 Samples/Second
	PAC1710WriteReg(PAC_CONFIG_REG, 0x63);				// ALERT OFF, Device measuring with one shot command only
	PAC1710WriteReg(C1VR_SMP_CFG, 0x0F); 
	PAC1710WriteReg(C1SR_SMP_CFG, 0x7F); 
	
	return 0;
}

uint32_t PAC1710ReadReg(uint8_t RegAdress, uint8_t *RegVal)
{
	I2C1drv->MasterTransmit(PAC1710_ADR, &RegAdress, 1, false);
	while(I2C1drv->GetStatus().busy)
		continue;
	
	I2C1drv->MasterReceive(PAC1710_ADR, RegVal, 1, false);
	while(I2C1drv->GetStatus().busy)
		continue;
	
	if(I2C1drv->GetDataCount())
		return 0;
	else
		return 1;
}

uint32_t PAC1710WriteReg(uint8_t RegAdress, uint8_t RegVal)
{
	uint8_t I2CwBuf[3];
	
	I2CwBuf[0]= RegAdress;
	I2CwBuf[1]= RegVal;
	
	I2C1drv->MasterTransmit(PAC1710_ADR, I2CwBuf, 2, false);
	while(I2C1drv->GetStatus().busy)
		continue;
	
	if(I2C1drv->GetDataCount() == 2)
		return 0;
	else
		return 1;
}

uint32_t PAC1710OneShotMeasure (void) 
{
	uint8_t ConfigRegs;
	uint8_t	RegValLow, RegValHigh;
	
	PAC1710WriteReg(PAC_OS_REG, 0x00); 		//  When the device is in the Standby state, writing to the One-Shot Register will initiate a
																				// conversion cycle and update all measurements.
	do {
		PAC1710ReadReg(PAC_CONFIG_REG, &ConfigRegs);
		} while (!(ConfigRegs & 0x03));					// C1IDS:C1VDS
	
	//V-Sense Result (10bit)
	PAC1710ReadReg(C1SR_HIGH, &RegValHigh);	
	PAC1710Regs.vSenseCh1 = RegValHigh;
	PAC1710Regs.vSenseCh1 <<= 8;
	PAC1710ReadReg(C1SR_LOW, &RegValLow);
	PAC1710Regs.vSenseCh1 |= RegValLow;
	PAC1710Regs.vSenseCh1 >>= 4;
		
	// V-Source Result (10bit)
	PAC1710ReadReg(C1VR_HIGH, &RegValHigh);
	PAC1710Regs.vSourceCh1 = RegValHigh;
	PAC1710Regs.vSourceCh1 <<= 8;
	PAC1710ReadReg(C1VR_LOW, &RegValLow);
	PAC1710Regs.vSourceCh1 |= RegValLow;
	PAC1710Regs.vSourceCh1 >>= 5;
	
	// Power Ratio Result (10bit)
	PAC1710ReadReg(C1P_HIGH, &RegValHigh);
	PAC1710Regs.PowerRatioCh1 = RegValHigh;
	PAC1710Regs.PowerRatioCh1 <<= 8;
	PAC1710ReadReg(C1P_LOW, &RegValLow);
	PAC1710Regs.PowerRatioCh1 |= RegValLow;		
	
	return 0;
}
