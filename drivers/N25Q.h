#ifndef __N25Q_H_	
	#define __N25Q_H_
	
	#include <LPC17xx.h>
	
	#define DUMMY_BYTE      0x00
	
	/*******************************************************************************
	Device Constants
	*******************************************************************************/

	// manufacturer id + mem type + mem capacity
	#define 	MEM_TYPE_N25Q64				0x20BA17			// ID for N25Q  64M device
	// Read Op
	#define 	SPI_FLASH_INS_READ		0x03					// Read data Bytes
	// Write Op
	#define 	SPI_FLASH_INS_WREN		0x06					// Enable write
	#define		SPI_FLASH_INS_WRDI		0x04					// Disable write
	// Program Operation
	#define 	SPI_FLASH_INS_PP			0x02					// Page Program
	// REGISTER operations
	#define		SPI_FLASH_INS_RDSR 		0x05					// read status register
	#define 	SPI_FLASH_INS_RFSR		0x70					// read flag status register
	// Erase Sector
	#define 	SPI_FLASH_INS_SE			0xD8					// Sector Erase command
	#define		SPI_FLASH_INS_SSE			0x20					// Sub sector erase
	
	
	/*******************************************************************************
												Status Register Definitions
	*******************************************************************************/
	enum
	{
		SPI_FLASH_SRWD	= 0x80,				// status Register Write Protect
		SPI_FLASH_BP3		= 0x40,				// block Protect Bit3
		SPI_FLASH_TB		= 0x20,				// top/Bottom bit
		SPI_FLASH_BP2		= 0x10,				// block Protect Bit2
		SPI_FLASH_BP1		= 0x08,				// block Protect Bit1
		SPI_FLASH_BP0		= 0x04,				// block Protect Bit0
		SPI_FLASH_WEL		= 0x02,				// write Enable Latch
		SPI_FLASH_WIP		= 0x01				// write/Program/Erase in progress bit
	};
	/*******************************************************************************
								flag Status Register Definitions
	*******************************************************************************/
	enum
	{
		SPI_FSR_PROG_ERASE_CTL		= 0x80,				// Program or erase controller busy
		SPI_FSR_ERASE_SUSP				= 0x40,				// Erase suspend in effect
		SPI_FSR_ERASE							= 0x20,				// Erase failure or protection error
		SPI_FSR_PROGRAM						= 0x10,				// Program failure
		SPI_FSR_VPP								= 0x08,				// Vpp Enabled
		SPI_FSR_PROG_SUSP					= 0x04,				// Program suspend in effect
		SPI_FSR_PROT							= 0x02,				// Protection error
		SPI_FSR_ADDR_MODE					= 0x01				// Reserved
	};

	typedef struct _FLASH_DESCRIPTION
	{
		uint32_t		FlashId;
		uint8_t			FlashType;
		uint32_t 		StartingAddress;						// Start Address of the Flash Device
		uint32_t 		FlashAddressMask;
		uint32_t 		FlashSectorCount;
		uint32_t 		FlashSubSectorCount;
		uint32_t		FlashSubSectorSize_bit;
		uint32_t 		FlashPageSize;
		uint32_t		FlashPageCount;
		uint32_t		FlashSectorSize;
		uint32_t		FlashSectorSize_bit;
		uint32_t		FlashSubSectorSize;
		uint32_t 		FlashSize;
		uint32_t		FlashOTPSize;
		uint8_t			FlashDieCount;
		uint32_t		FlashDieSize;
		uint32_t		FlashDieSize_bit;
		uint32_t		Size;												// The density of flash device in bytes
		uint32_t		BufferSize;									// In bytes */
		uint8_t			DataWidth;									// In bytes */
	} 
	FLASH_DESCRIPTION, *PFLASH_DESCRIPTION;
	/******** ReturnType ********/
	typedef enum
	{
		Flash_Success,
		Flash_AddressInvalid,
		Flash_MemoryOverflow,
		Flash_PageEraseFailed,
		Flash_PageNrInvalid,
		Flash_SubSectorNrInvalid,
		Flash_SectorNrInvalid,
		Flash_FunctionNotSupported,
		Flash_NoInformationAvailable,
		Flash_OperationOngoing,
		Flash_OperationTimeOut,
		Flash_ProgramFailed,
		Flash_SectorProtected,
		Flash_SectorUnprotected,
		Flash_SectorProtectFailed,
		Flash_SectorUnprotectFailed,
		Flash_SectorLocked,
		Flash_SectorUnlocked,
		Flash_SectorLockDownFailed,
		Flash_WrongType
	} ReturnType;
	
	typedef struct
	{
		FLASH_DESCRIPTION 	Desc;
		//FLASH_OPERATION   	GenOp;

	} FLASH_DEVICE_OBJECT;
	
#endif

