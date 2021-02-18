/**
 *  \file FlashMem.c
 *  \brief Header file for N25Q interface.
 *  \Author Arun K G
 */
#include "FlashMem.h"
#include "N25Q.h"

static ARM_DRIVER_SPI *SPI1drv = &Driver_SPI1;
// global flash device object
static FLASH_DEVICE_OBJECT *fdo;


uint32_t FlashMemInit(FLASH_DEVICE_OBJECT *flash_device_object)
{
	uint32_t DevId;
	uint8_t IDdataOut[4];						// {Memory Capacity ,Memory Type, Manufacturer ID}
	
	fdo = flash_device_object;
	
	SPI1drv->Initialize(NULL);
	SPI1drv->PowerControl(ARM_POWER_FULL);
	// Master Mode, SPI Mode 00, MSB to LSB, SS controlled by s/w, 8 Bit data, 4e6 bps
	SPI1drv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_SW | ARM_SPI_DATA_BITS(8), 50000000); 
	// SS line: INACTIVE = HIGH 
	SPI1drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	
	FlashMemReadReg(0x9E, &IDdataOut[0], 3);
	DevId = IDdataOut[0] << 16 | IDdataOut[1] << 8 | IDdataOut[2];
	
	if(DevId == MEM_TYPE_N25Q64) {
		// device shape
		fdo->Desc.FlashSize 							= 0x800000;					// 83,88,608 Bytes (8MB)
		fdo->Desc.FlashSectorCount 				= 0x80;							// 128 Sector
		fdo->Desc.FlashSectorSize 				= 0x10000;					// 65,536 Sector Size
		fdo->Desc.FlashSectorSize_bit			= 16;									
		fdo->Desc.FlashSubSectorCount 		= 0x800;						// 2048 Sub sectors
		fdo->Desc.FlashSubSectorSize 			= 0x1000;
		fdo->Desc.FlashSubSectorSize_bit 	= 12;
		fdo->Desc.FlashPageCount 					= 0x8000;						// 32768
		fdo->Desc.FlashPageSize 					= 0x100;						// 256 bytes
		fdo->Desc.FlashOTPSize 						= 0x40;							// 64 OTP bytes
		fdo->Desc.FlashAddressMask 				= 0x00FF;						// 0xFF
		
		return Flash_Success;
	}
	else
		return Flash_WrongType;																								// no known device dectected
}

uint32_t 	FlashMemReadReg (uint8_t Command, uint8_t *RxData, uint8_t RxLength)
{
	// CS_ is now asserted
	SPI1drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	// Command Phase
	SPI1drv->Send(&Command, 1);																
	while(SPI1drv->GetStatus().busy)													// Check if SSP module busy
		continue;
	if(!SPI1drv->GetDataCount())															// Check if 1 byte has been transffered
		return 1;
	
	Command = DUMMY_BYTE;
	// Data Phase
	SPI1drv->Transfer(&Command, RxData, RxLength);						// OutData should be valid if in read mode
	while(SPI1drv->GetStatus().busy)
		continue;
	if(!SPI1drv->GetDataCount())
		return 1;
	// CS_ line: INACTIVE = HIGH
	SPI1drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	if(!SPI1drv->GetDataCount())
		return 1;
	else
		return 0;
}

uint32_t		FlashDataRead (uint32_t StartAddr, uint8_t *ReadElements, uint32_t NoOfElements)
{
	uint8_t Command;
	uint8_t InsAddress[3];								// max address could be 4 Byte, i.e. 32bit
	
	if(StartAddr > fdo->Desc.FlashSize)
		return 1;
	
	InsAddress[0] = StartAddr >> 16; 
	InsAddress[1] = StartAddr >> 8;
	InsAddress[2] = StartAddr;

	Command = SPI_FLASH_INS_READ;
	// CS_ is now asserted
	SPI1drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	SPI1drv->Send(&Command, 1);																			//0x03, Read Command followed by address A[MAX:MIN] of A[23:0]
	while(SPI1drv->GetStatus().busy)
		continue;
	SPI1drv->Send(&InsAddress[0], 3);																// Address A[MAX:MIN] of A[23:0]
	while(SPI1drv->GetStatus().busy)
		continue;
		// Read data, Command is don't care value
	SPI1drv->Receive(ReadElements, NoOfElements);										// OutData should be valid if in read mode
	while(SPI1drv->GetStatus().busy)
		continue;
	// CS_ line: INACTIVE = HIGH
	SPI1drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

	if(!SPI1drv->GetDataCount())
		return 1;
	
	return 0;
}

uint32_t 	FlashWriteEnable (void)
{
	uint8_t Command;
	
	Command = SPI_FLASH_INS_WREN;
	// CS_ is now asserted
	SPI1drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	SPI1drv->Send(&Command, 1);																			//0x03, Read Command followed by address A[MAX:MIN] of A[23:0]
	while(SPI1drv->GetStatus().busy)
		continue;
	// CS_ line: INACTIVE = HIGH
	SPI1drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	if(!SPI1drv->GetDataCount())
		return 1;
	// Now read status Regs. check if our op is successfull one
	do {
			FlashMemReadReg(SPI_FLASH_INS_RDSR, &Command, 1);					// Here the command var is used a return variable for getting status regs. 
	}while(!(Command & SPI_FLASH_WEL));														// B1 should be set to indicate Write enable
	
	return Flash_Success;
}

uint32_t PageProgram(uint32_t Addr, uint8_t *Array , uint32_t NrOfElementsInArray, uint8_t SpiInstruction)
{
	uint8_t Command;
	uint8_t InsAddress[3];								// max address could be 4 Byte, i.e. 32bit
	
	if(Addr > fdo->Desc.FlashSize)
		return Flash_AddressInvalid;
	
	InsAddress[0] = Addr >> 16; 
	InsAddress[1] = Addr >> 8;
	InsAddress[2] = Addr;
	
	// Check whether any previous Write, Program or Erase cycle is on-going
	if(IsFlashBusy())
		return Flash_OperationOngoing;
	// Disable Write protection
	FlashWriteEnable();
	// Send in the command
	Command = SpiInstruction;
	// CS_ is now asserted
	SPI1drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	SPI1drv->Send(&Command, 1);														//0x03, Read Command followed by address A[MAX:MIN] of A[23:0]
	while(SPI1drv->GetStatus().busy)
		continue;
	SPI1drv->Send(&InsAddress[0], 3);												// Address A[MAX:MIN] of A[23:0]
	while(SPI1drv->GetStatus().busy)
		continue;
	// Write data, 
	SPI1drv->Send(Array, NrOfElementsInArray);										// OutData should be valid if in read mode
	while(SPI1drv->GetStatus().busy)
		continue;
	// CS_ line: INACTIVE = HIGH
	SPI1drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	if(!SPI1drv->GetDataCount())
		return 1;
	// Wait until operation is complete, or timeout??
	while(IsFlashBusy())
		continue;
	
	// read 'flag status' register @0x70
	FlashMemReadReg(SPI_FLASH_INS_RFSR, &Command, 1);								// returns Flag Status in Command var
	// Check if region protected or Program failure
	if( (Command & SPI_FSR_PROT) || (Command & SPI_FSR_PROGRAM)  )
		return Flash_SectorProtected;
	
	return Flash_Success;
}

uint32_t		IsFlashBusy (void)
{
	uint8_t	OutData;
	FlashMemReadReg(SPI_FLASH_INS_RDSR, &OutData, 1);					// Here the OutData var is used a return variable for getting status regs.
	/*
	Status Register: B0 Indicates whether a PROGRAM, ERASE, WRITE STATUS REGISTER, 
	or WRITE NONVOLATILE CONFIGURATION command cycle is in progress
	*/
	if(OutData & SPI_FLASH_WIP)
		return 1;
	else
		return 0;
}

/*******************************************************************************
									Function:     	FlashDataProgram()
*******************************************************************************/
uint32_t FlashDataProgram(uint32_t Addr, uint8_t *Array , uint32_t NrOfElementsInArray, uint8_t SpiInstruction)
{
	uint16_t dataOffset;

	// Enabling the Write
	FlashWriteEnable();

	// Computing the starting alignment, i.e. the distance from the 64 bytes boundary
	dataOffset = (fdo->Desc.FlashPageSize - (Addr & fdo->Desc.FlashAddressMask) ) & fdo->Desc.FlashAddressMask;
	if (dataOffset > NrOfElementsInArray)
		dataOffset = NrOfElementsInArray;
	if (dataOffset > 0)
	{
		if(PageProgram(Addr, Array, dataOffset, SpiInstruction))
			return 1;
	}

	for ( ; (dataOffset+fdo->Desc.FlashPageSize) < NrOfElementsInArray; dataOffset += fdo->Desc.FlashPageSize)
	{
		if(PageProgram(Addr+dataOffset, Array+dataOffset, fdo->Desc.FlashPageSize, SpiInstruction))
			return 1;
	}

	if (NrOfElementsInArray > dataOffset) {
		if(PageProgram(Addr + dataOffset, Array + dataOffset, (NrOfElementsInArray - dataOffset), SpiInstruction))
			return 1;
	}
	return Flash_Success;
}

uint32_t 	FlashSectorErase(uint16_t SectorNr)
{
	uint8_t Command;
	uint32_t SectorAdr;
	uint8_t InsAddress[3];								// max address could be 4 Byte, i.e. 32bit
	
	if(SectorNr > fdo->Desc.FlashSectorCount)
		return Flash_SectorNrInvalid;
	
	// Convert Sector number to sector address
	SectorAdr = SectorNr << fdo->Desc.FlashSectorSize_bit;
	
	InsAddress[0] = SectorAdr >> 16; 
	InsAddress[1] = SectorAdr >> 8;
	InsAddress[2] = SectorAdr;
	
	// Check whether any previous Write, Program or Erase cycle is on-going
	if(IsFlashBusy())
		return Flash_OperationOngoing;
	// Disable Write protection
	FlashWriteEnable();

	// Send in the command
	Command = SPI_FLASH_INS_SE;
	// CS_ is now asserted
	SPI1drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	SPI1drv->Send(&Command, 1);														//0x03, Read Command followed by address A[MAX:MIN] of A[23:0]
	while(SPI1drv->GetStatus().busy)
		continue;
	SPI1drv->Send(&InsAddress[0], 3);												// Address A[MAX:MIN] of A[23:0]
	while(SPI1drv->GetStatus().busy)
		continue;
	// CS_ line: INACTIVE = HIGH
	SPI1drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	if(!SPI1drv->GetDataCount())
		return 1;
	// Wait until operation is complete, or timeout??
	while(IsFlashBusy())
		continue;
	// read 'flag status' register @0x70
	FlashMemReadReg(SPI_FLASH_INS_RFSR, &Command, 1);								// returns Flag Status in Command var
	// Check if region protected or Program failure
	if( (Command & SPI_FSR_PROT) || (Command & SPI_FSR_ERASE)  )
		return Flash_SectorProtected;
	
	return Flash_Success;
}

/*******************************************************************************
Function:     ReturnType FlashSubSectorErase( uSectorType uscSectorNr )
Arguments:    uSectorType is the number of the subSector to be erased.

Return Values:
   Flash_SectorNrInvalid
   Flash_OperationOngoing
   Flash_OperationTimeOut
   Flash_Success

Description:  This function erases the SubSector (4k) specified in uscSectorNr by sending an
              SPI_FLASH_INS_SSE Instruction.
              The function checks that the sub sector number is within the valid range
              before issuing the erase Instruction. Once erase has completed the status
              Flash_Success is returned.
Note:
              This function does not check whether the target memory area is in a Software
              Protection Mode(SPM) or Hardware Protection Mode(HPM), in which case the PP
              Instruction will be ignored.
              The function assumes that the target memory area has previously been unprotected at both
              the hardware and software levels.
              To unprotect the memory, please call FlashWriteStatusRegister(NMX_uint8 ucStatusRegister),
              and refer to the datasheet to set a proper ucStatusRegister value.

Pseudo Code:
   Step 1: Validate the sub sector number input
   Step 2: Check whether any previous Write, Program or Erase cycle is on going
   Step 3: Disable Write protection (the Flash memory will automatically enable it
           again after the execution of the Instruction)
   Step 4: Initialize the data (Instruction & address) packet to be sent serially
   Step 5: Send the packet (Instruction & address) serially
   Step 6: Wait until the operation completes or a timeout occurs.
*******************************************************************************/

uint32_t  FlashSubSectorErase(uint16_t uscSectorNr)
{
	uint8_t Command;
	uint32_t SubSectorAddr;
	uint8_t InsAddress[3];								// max address could be 4 Byte, i.e. 32bit
	
	// Step 1: Validate the sector number input
	if(uscSectorNr > fdo->Desc.FlashSubSectorCount)
		return Flash_SectorNrInvalid;

	SubSectorAddr = uscSectorNr << fdo->Desc.FlashSubSectorSize_bit;
	InsAddress[0] = SubSectorAddr >> 16; 
	InsAddress[1] = SubSectorAddr >> 8;
	InsAddress[2] = SubSectorAddr;
	
	// Step 2: Check whether any previous Write, Program or Erase cycle is on going
	if(IsFlashBusy())
		return Flash_OperationOngoing;
	
	// Disable Write protection
	FlashWriteEnable();

	// Step 4: Initialize the Instruction packet to be sent serially
	// Send in the command
	Command = SPI_FLASH_INS_SSE;
	
	// CS_ is now asserted
	SPI1drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	SPI1drv->Send(&Command, 1);														//0x03, Read Command followed by address A[MAX:MIN] of A[23:0]
	while(SPI1drv->GetStatus().busy)
		continue;
	SPI1drv->Send(&InsAddress[0], 3);												// Address A[MAX:MIN] of A[23:0]
	while(SPI1drv->GetStatus().busy)
		continue;
	// CS_ line: INACTIVE = HIGH
	SPI1drv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	if(!SPI1drv->GetDataCount())
		return 1;
	// Wait until operation is complete, or timeout??
	while(IsFlashBusy())
		continue;
	// read 'flag status' register @0x70
	FlashMemReadReg(SPI_FLASH_INS_RFSR, &Command, 1);								// returns Flag Status in Command var
	// Check if region protected or Program failure
	if( (Command & SPI_FSR_PROT) || (Command & SPI_FSR_ERASE)  )
		return Flash_SectorProtected;
	
	return Flash_Success;
}
