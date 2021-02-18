#include "SysTick.h"

volatile uint32_t SysTicks, SysTimeOut, BeeperTimeout, ModemRefreshRate;

// SysTick_Handler, 1mS
void SysTick_Handler (void)  {
	SysTicks++;                                // Decrement counter
	if(SysTimeOut)
		SysTimeOut--;
	if(ModemRefreshRate)
		ModemRefreshRate--;	
	if(BeeperTimeout) {
		GPIO_PinWrite (1, 18, 1);
		BeeperTimeout--;
	}else
		GPIO_PinWrite (1, 18, 0);
}

void msDelay (uint32_t mSdelayCnt)
{
	SysTimeOut = mSdelayCnt;
	while(SysTimeOut)
		continue;
}
