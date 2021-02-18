#include "ArduCAM.h"

static ARM_DRIVER_SPI *SPI0drv = &Driver_SPI0;

void ArdCAMChipSelect(uint32_t CSsignal) 
{
	(CSsignal)? SPI0drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE) : SPI0drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
}

uint32_t ArduCamInit (void)
{
	if(OV2640init())
		return 2;
	
	SPI0drv->Initialize(NULL);
	SPI0drv->PowerControl(ARM_POWER_FULL);
	// Master Mode, SPI Mode 00, MSB to LSB, SS controlled by s/w, 8 Bit data, 4e6 bps
	SPI0drv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_SW | ARM_SPI_DATA_BITS(8), 4000000); 
	// SS line: INACTIVE = HIGH 
	SPI0drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	
	// Write 0xA5 to Test register
	ArduCAMtransfer(ARDC_REG_TEST | 0x80, 0xA5);					// return value from MISO is don't care
	// Read test register
	if( ArduCAMtransfer(ARDC_REG_TEST, 0) != 0xA5)		
		return 1;
	
	return 0;
}


uint8_t 	ArduCAMtransfer (uint8_t CommandPhase, uint8_t DataPhase)
{
		uint8_t OutData;
	
		// CS_ is now asserted
		SPI0drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
		// Command Phase
		SPI0drv->Transfer(&CommandPhase, &OutData, 1);				// Out-data is dummy value in Command Phase
		while(SPI0drv->GetStatus().busy)											// Check if SSP module busy
			continue;
		if(!SPI0drv->GetDataCount())													// Check if 1 byte has been transferred
			return 1;
		// Data Phase
		SPI0drv->Transfer(&DataPhase, &OutData, 1);						// OutData should be valid if in read mode
		while(SPI0drv->GetStatus().busy)
			continue;
		if(!SPI0drv->GetDataCount())
			return 1;
		// CS_ line: INACTIVE = HIGH
		SPI0drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
		
		if(!SPI0drv->GetDataCount())
			return 1;
		else
			return OutData;
}
void 		FlushFIFO (void)
{
	ArduCAMtransfer(ARDC_REG_FIFO_CTRL | 0x80, FIFO_CLEAR_MASK);					// ARDC_REG_FIFO_CTRL in write mode, B0 = 1
}

void 		StartCapture (void)
{
	ArduCAMtransfer(ARDC_REG_FIFO_CTRL | 0x80, FIFO_START_MASK);					// B1 = 1
}

uint32_t ReadFIFOlen (void)	
{
	uint32_t LenB0_B7, LenB8_B15, LenB16_B18;	
	
	LenB0_B7		= ArduCAMtransfer (ARDC_REG_CAM_W_FIFO_0, 	0x00);
	LenB8_B15		= ArduCAMtransfer (ARDC_REG_CAM_W_FIFO_1, 	0x00);
	LenB16_B18	= ArduCAMtransfer	(ARDC_REG_CAM_W_FIFO_2, 	0x00);
	
	return (LenB0_B7 | (LenB8_B15 << 8) | (LenB16_B18 << 16));
}

uint32_t CheckFIFOdone (void)
{
	uint32_t FiFoStatus;
	
	FiFoStatus = ArduCAMtransfer(ARDC_REG_CAM_VS_WR_FIFO, 0x00);
	
	return (FiFoStatus & (1 << 3));
}

uint32_t ReadFIFO (uint8_t *ImgBufferPtr, uint32_t Length)
{
	// CS_ is now asserted
	SPI0drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	
	ArduCAMBurstRead (ARDC_REG_BURST_FIFO);														// Burst FIFO read operation
	while(Length) {
		*ImgBufferPtr++ = ArduCAMBurstRead(0x00);												// Dummy write
		Length--;
	}
	
	// SS line: INACTIVE = HIGH
	SPI0drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	
	return 0;
}

uint8_t		ArduCAMBurstRead (uint8_t CommandPhase)
{
	uint8_t OutData;
	
	SPI0drv->Transfer(&CommandPhase, &OutData, 1);				// Out-data is valid after 1st command
	while(SPI0drv->GetStatus().busy)											// Check if SSP module busy
		continue;
	if(!SPI0drv->GetDataCount())													// Check if 1 byte has been transferred
		return 0;
	else
		return OutData;
}
