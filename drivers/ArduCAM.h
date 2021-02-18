#ifndef __ARDU_CAM_H
	#define __ARDU_CAM_H
	
	#include <LPC17xx.h>
	#include <Driver_SPI.h>
	#include "OV2640.h"
	
	#define MAX_FIFO_SIZE						0x5FFFF			//384KByte
	
	#define ARDC_REG_TEST						0x00
	#define ARDC_REG_CAP_CTRL				0x01
	#define ARDC_REG_SENSOR_TM			0x03
	
	#define ARDC_REG_FIFO_CTRL 			0x04
		#define FIFO_CLEAR_MASK    		0x01			// B0 = 1
		#define FIFO_START_MASK    		0x02			// B1 = 1
		#define FIFO_RDPTR_RST_MASK   0x10
		#define FIFO_WRPTR_RST_MASK   0x20

	#define ARDC_REG_GPIO_DIR				0x05
	#define ARDC_REG_GPIO_WR				0x06
	
	#define ARDC_REG_BURST_FIFO			0x3C
	#define ARDC_REG_SING_FIFO			0x3D
	#define ARDC_REG_CHIP_VER				0x40
	#define ARDC_REG_CAM_VS_WR_FIFO	0x41
	#define ARDC_REG_CAM_W_FIFO_0		0x42			// Camera write FIFO size[7:0] for burst to read
	#define ARDC_REG_CAM_W_FIFO_1		0x43			// [15:8]
	#define ARDC_REG_CAM_W_FIFO_2		0x44			// [18:16]
	#define ARDC_REG_GPIO_READ			0x45
	
	
	#define GPIO_RESET_MASK			0x01  				//0 = Sensor reset,							1 =  Sensor normal operation
	#define GPIO_PWDN_MASK			0x02  				//0 = Sensor normal operation, 	1 = Sensor standby
	#define GPIO_PWREN_MASK			0x04					//0 = Sensor LDO disable, 			1 = sensor LDO enable
	
	extern uint32_t 	ArduCamInit (void);
	extern void 			FlushFIFO (void);
	extern void 			StartCapture (void);
	extern uint32_t 	ReadFIFOlen (void);
	extern uint32_t 	CheckFIFOdone (void);
	extern uint32_t 	ReadFIFO (uint8_t *ImgBufferPtr, uint32_t Length);
	extern uint8_t 		ArduCAMtransfer (uint8_t CommandPhase, uint8_t DataPhase);
	extern uint8_t		ArduCAMBurstRead (uint8_t CommandPhase);
	extern void				ArdCAMChipSelect(uint32_t CSsignal);
	
	extern ARM_DRIVER_SPI Driver_SPI0;

#endif
