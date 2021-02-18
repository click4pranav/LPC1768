/************************************************************************************
* HD44780(or clones) 4 bit LCD interface routines, (C) Arun K.G, 2017								*
* Specs:		# 4-bit interface, Higher LCD nibbles on to MCU's higher nibbles				*
*           	# 16x2 LCD module!									 																	*
*           	# Implements Busy checking & Optional				 													*
*           	# Code upgraded to support Keil ARM CMSIS			 												*
* Dependencies	: Fcy has to be defined																							*
* Code rev		: v1.0																																*
* Last Modified : 01, Feb 2017																											*			
*************************************************************************************/
#ifndef __LCD_H
	#define __LCD_H
	
	#include "LPC17xx.h"
	#include "GPIO_LPC17xx.h"
	#define BSY_CHECK
	// #define BKLITE_CTRL

	/**************************  LCD Interface *************************************
	DB4  	=		P1.14
	DB5		=		P1.15
	DB6		= 	P1.16
	DB7		=		P1.17
	********************************************************************************/
	#define 		LCD_DATA 				LPC_GPIO1->FIOPIN 				// Corresponds to pin P1.0 thru pin P1.31 (P1.2, P1.3, P1.7:5, P1.13:11 NA)
	#define 		LCD_DATA_MASK		LPC_GPIO1->FIO1MASK3									// Mask bits, Clearing bits will inhibits write operations
	#define			PORTDIR					LPC_GPIO1->FIODIR

	#define 		LCD_RS					LPC_GPIO1->FIOPIN & ( 1 << 0)					// Reg. Select (Command or Data), P1.0
	#define 		LCD_EN					LPC_GPIO1->FIOPIN & ( 1 << 4)					// enable, Read and Write strobe, P1.4

	#ifdef BSY_CHECK
		#define 		LCD_RW				LPC_GPIO1->FIOPIN & ( 1 << 1)					// Read write Sel, P1.1
	#endif

	#ifdef BKLITE_CTRL
		#define			BKLITE 				LATBbits.LATB14			// LCD backlight
	#endif
		
	/********************************************************************************/

	#define CGRAM_ADR_MASK 	0x40
	#define DDRAM_ADR_MASK	0x80
	// #define tDDR (unsigned int)(3 * Fcy/20000000L)
	
	#define MAX_ROW		2
	#define MAX_COL		16
	
	#define DIGIT_JUSTIFY_NONE	0x00
	#define DIGIT_JUSTIFY_2D	0x01	
	#define DIGIT_JUSTIFY_3D	0x02
	#define DIGIT_JUSTIFY_4D	0x03	
	#define DIGIT_JUSTIFY_5D	0x04

	#define LCDdataMaskEN() { \
		LCD_DATA_MASK = 0xF0;	\
	}
	#define LCDdataMaskDI() { \
		LCD_DATA_MASK = 0;	\
	}
	/*
	// for HD4470 tDDR = 160nS, after close examination in simulation it shud be least 300nS
	#define DataDelayTime() {\
		uint16 Dcnt = tDDR;	\
		while(--Dcnt)		\
			continue;		\
	}
	*/

	extern void LcdInit(void);					// Init LCD
	extern void ReadStrobe(void);
	extern void WriteStrobe(void);
	extern void LcdClear(void);					// Clears the display
	extern void LcdReady(void);   				// checks for LCD busy condition, returns only after busy over
	extern void LcdCMD(uint8_t);					// Sends the command
	extern void LcdPutch(char);					// Writes a char onto LCD
	extern void LcdWrite(uint8_t);				// Writes to LCD cmd Reg.
	extern void LcdPutNum(uint16_t);			// Displays Byte values on to LCD
	extern void LcdPuts(const char*);			// Puts a string
	extern void LcdWrt2CGRAM(uint16_t, uint8_t *);
	extern uint8_t LcdPutsXY(uint8_t X, uint8_t Y,const char *String);
	extern uint8_t LcdGotoXY(uint8_t X, uint8_t Y);
	extern void Num2CharLcd(uint16_t val, uint8_t DigitJustify);
#endif
