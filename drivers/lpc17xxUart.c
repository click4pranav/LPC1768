#include "lpc17xxUart.h"

void Uart2Init(uint32_t baudrate)
{
	uint32_t var_UartPclk_u32, var_Pclk_u32, var_RegValue_u32;
	uint32_t SystemFrequency = SystemCoreClock;							
	
	LPC_SC->PCONP	 = LPC_SC->PCONP | (1 << 24);			// Uart 2 Power Block
	//LPC_PINCON->PINSEL0 &= ~0x000000F0;
	//LPC_PINCON->PINSEL0 |= 0x00000050;            // Enable TxD2 P0.10 and P0.11 
	
	PIN_Configure (0, 10, PIN_FUNC_1, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL);  		// TXD2
	PIN_Configure (0, 11, PIN_FUNC_1, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL);			// RXD2


	LPC_UART2->FCR = (1<<SBIT_FIFO) | (1<<SBIT_RxFIFO) | (1<<SBIT_TxFIFO); // Enable FIFO and reset Rx/Tx FIFO buffers    
	LPC_UART2->LCR = (0x03<<SBIT_WordLenght) | (1<<SBIT_DLAB); // 8bit data, 1Stop bit, No parity


    /** Baud Rate Calculation :
       PCLKSELx registers contains the PCLK info for all the clock dependent peripherals.
       Bit6,Bit7 contains the Uart Clock(ie.UART_PCLK) information.
       The UART_PCLK and the actual Peripheral Clock(PCLK) is calculated as below.
       (Refer data sheet for more info)
       
       UART_PCLK    PCLK
         0x00       SystemFreq/4        
         0x01       SystemFreq
         0x02       SystemFreq/2
         0x03       SystemFreq/8   
     **/

    var_UartPclk_u32 = (LPC_SC->PCLKSEL1 >> 16) & 0x03;

    switch( var_UartPclk_u32 )
    {
          case 0x00:
            var_Pclk_u32 = SystemFrequency/4;
            break;
          case 0x01:
            var_Pclk_u32 = SystemFrequency;
            break; 
          case 0x02:
            var_Pclk_u32 = SystemFrequency/2;
            break; 
          case 0x03:
            var_Pclk_u32 = SystemFrequency/8;
            break;
    }    

    var_RegValue_u32 = ( var_Pclk_u32 / (16 * baudrate )); 
    LPC_UART2->DLL =  var_RegValue_u32 & 0xFF;
    LPC_UART2->DLM = (var_RegValue_u32 >> 0x08) & 0xFF;
		
    LPC_UART2->LCR =  LPC_UART2->LCR & ~(uint32_t)(1 << 7);
}

void Uart2TxChar (char txData)
{
	while(!(LPC_UART2->LSR & (1 << 5)))				// Wait until TSR Set
		continue;
	LPC_UART2->THR = txData;                        // Load the data to be transmitted
}

char Uart2RxChar (void)
{
 char rxData;
	
 while( !(LPC_UART2->LSR & 1))
			continue;
    rxData = LPC_UART2->RBR;                              // Read received data
 
 return rxData;
}

void Uart2TxString (const char *TxString)
{
	while (*TxString)
		Uart2TxChar(*TxString++);
}	

void Uart2Send(const char *TxString, uint32_t Len)
{
	uint32_t i;
	
	for(i = 0; i < Len; i++)
		Uart2TxChar(*TxString++);
}
