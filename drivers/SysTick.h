#ifndef __SYSTICK
	#define __SYSTICK
	
	#include "LPC17xx.h"
	#include "GPIO_LPC17xx.h"
	
	extern void SysTick_Handler (void);
	extern void msDelay (uint32_t);
	extern volatile uint32_t SysTicks, SysTimeOut, BeeperTimeout, ModemRefreshRate;               // Counts 1ms timeTicks
	
#endif
