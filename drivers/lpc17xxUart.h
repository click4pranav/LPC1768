/****************************************************************************************************************************
* uC = LPC1769 (32 ARM Cortex M3 bit controller)																																						*
* Device clock		: 100.000000 MHz, i.e 16MIPS																																							*
* Specials			: Standard WDT, Internal Fast RC OSC (8Mhz) with PLL x4					  																					*
*****************************************************************************************************************************
* Author						: Arun K.G (The Lone Ranger)																																						*
* Company						: Inomterics Technologies PVT Ltd.																																			*
* Armed since	 			: January-02, 2015										  																																*
* Last modified	 		: January-03, 2016									  																																	*
* MCU-Programmer 		: Pickit-3  																																														*
* Compiler 		 			: MDK 5.06, uV 5.22 (Pro Version)																																				*	
* IDE & Text Editor	: Keil MDK 5.22 & Notepad++ v7.2 Running on an Windows 7 x64 (Home Basic)																*
* Debugger					: J-LInk																																																*
* Project code	 		: 								  																																										*
* Code rev		 			: 1.0 Beta	(Debug)																																											*
* Code name					: 																																																			*
* Change-log				: 																																																			*
*****************************************************************************************************************************/
#ifndef __LPC_UART_
	#define __LPC_UART_
  
	#include "LPC17xx.h"
	#include "PIN_LPC17xx.h"

	#define SBIT_WordLenght    0x00u
	#define SBIT_DLAB          0x07u
	#define SBIT_FIFO          0x00u
	#define SBIT_RxFIFO        0x01u
	#define SBIT_TxFIFO        0x02u

	#define SBIT_RDR           0x00u
	#define SBIT_THRE          0x05u
	
	#define IER_RBR						0x01u
	#define IER_THRE					0x02u
	#define IER_RLS						0x04u

	#define IIR_PEND					0x01u
	#define IIR_RLS						0x03u
	#define IIR_RDA						0x02u
	#define IIR_CTI						0x06u
	#define IIR_THRE					0x01u

	#define LSR_RDR						0x01u
	#define LSR_OE						0x02u
	#define LSR_PE						0x04u
	#define LSR_FE						0x08u
	#define LSR_BI						0x10u
	#define LSR_THRE					0x20u
	#define LSR_TEMT					0x40u
	#define LSR_RXFE					0x80u
	
	extern void Uart2TxChar (char);
	extern char Uart2RxChar (void);
	extern void Uart2Init (uint32_t);
	extern void Uart2TxString (const char *);
	extern void Uart2Send(const char *TxString, uint32_t Len);
	
#endif
