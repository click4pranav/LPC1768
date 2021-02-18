#ifndef __FLASH_MEM_
	#define __FLASH_MEM_
	
	#include <LPC17xx.h>
	#include <Driver_SPI.h>
	#include "N25Q.h"

	extern ARM_DRIVER_SPI Driver_SPI1;
	
	extern uint32_t 	FlashMemInit (FLASH_DEVICE_OBJECT *flash_device_object); 
	extern uint32_t 	FlashMemReadReg (uint8_t Command, uint8_t *Data, uint8_t RxLength);
	extern uint32_t		FlashDataRead (uint32_t StartAddr, uint8_t *ReadElements, uint32_t NoOfElements);
	extern uint32_t 	FlashWriteEnable (void);
	extern uint32_t		IsFlashBusy (void); 
	extern uint32_t 	PageProgram(uint32_t Addr, uint8_t *Array , uint32_t NrOfElementsInArray, uint8_t SpiInstruction);
	extern uint32_t 	FlashDataProgram(uint32_t Addr, uint8_t *Array , uint32_t NrOfElementsInArray, uint8_t SpiInstruction);
	extern uint32_t  	FlashSubSectorErase(uint16_t uscSectorNr);
	extern uint32_t 	FlashSectorErase(uint16_t SectorNr);
	
#endif	
