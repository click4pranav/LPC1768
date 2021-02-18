#include "lcd.h"
#include "SysTick.h"

extern void ReadStrobe()
{
	GPIO_PinWrite(1, 4, 0);
	GPIO_PinWrite(1, 4, 1);
}
void WriteStrobe(void)
{
	GPIO_PinWrite(1, 4, 1);
	GPIO_PinWrite(1, 4, 0);
}

void LcdReady(void)
{
	#ifdef BSY_CHECK
		uint8_t Status = 0x00;						// Var used to read Status Reg & DD-ADDRESS (BF.AC6.AC5.AC4.AC3.AC2.AC1.AC0)
		// LCD Datalines back to output mode
		LPC_GPIO1->FIODIRL |= (LPC_GPIO1->FIODIRL & 0x3FFF);						// P1.14 thru P1.15 as i/p
		LPC_GPIO1->FIODIRH |= (LPC_GPIO1->FIODIRH & 0xFFFC); 						// P1.16 thru P1.17 as i/p
		//LCD_RS = 0;    									// select instruction regs.
		GPIO_PinWrite(1, 0, 0);
		//LCD_RW = 1;   									// lcd in read mode
		GPIO_PinWrite(1, 1, 1);
		// Address set-up time is Min:40nS
		do {
			ReadStrobe();										// Read out Higher Nibble
			msDelay(1);											// Without this NOP, data wud never be read at all
			Status = (LCD_DATA & 0x3C000) >> 14;				// read higher nibble, in higher nibble of Status var.
			ReadStrobe();										// Perform another read, and grab the Lower Nibble
			msDelay(1);							// Wait for it
			Status += ((LCD_DATA & 0x3C000) >> 18);		// Lower Nibble, and we discard it :D  Why? (yeah skip the ddaddress)
		} while(Status & 0x80);									// if Busy Flag, loop until busy over
						 
		// LCD_RW = 0;  				 	// lcd back to write mode
		GPIO_PinWrite(1, 1, 0);
		// LCD Datalines back to output mode
		LPC_GPIO1->FIODIRL |= 0xC000;							// P1.14 thru P1.15 as o/p
		LPC_GPIO1->FIODIRH |= 0x0003; 						// P1.16 thru P1.17 as o/p
	#else
		msDelay(2);
	#endif
	
}
 
void LcdWrite(uint8_t data)
{
	LCD_DATA = (LCD_DATA & 0xFFFC3FFF) | ( (data >> 4) << 14);					  										// send the higher nibbles
	WriteStrobe();
	LCD_DATA = (LCD_DATA & 0xFFFC3FFF) | ( (data & 0x0F) << 14);															// send the lower nibbles
	WriteStrobe();
}

// Sends command to LCD
// (0x00 ~ 0x13) + 0x80 = 1st Line
// (0x40 ~ 0x53) + 0x80 = 2nd Line
// (0x14 ~ 0x27) + 0x80 = 3rd Line
// (0x54 ~ 0x67) + 0x80 = 4th Line
void LcdCMD(uint8_t cmd)
{
	LcdReady();
	//LCD_RS = 0;     		// select command regs.
	GPIO_PinWrite(1, 0, 0);
	LcdWrite (cmd);
}

// Clears the LCD screen
void LcdClear(void)
{
	LcdReady();
	//LCD_RS = 0;
	GPIO_PinWrite(1, 0, 0);
	LcdWrite(0x01);			// Clears entire display and sets DDRAM address 0 in address counter.
}
/* 0b0000001x = Sets DDRAM address 0 in address counter. Also returns display from being shifted to original position.
DDRAM contents remain unchanged. */
/* 0b000001(ID)S = Sets cursor move direction and specifies display shift. These operations are performed
during data write and read.
Increments (I/D = 1) or decrements (I/D = 0) */

// Write data to CGRAM or DDRAM
void LcdPutch(char c)
{
	LcdReady();
  //LCD_RS = 1;   					  // select data regs.
	GPIO_PinWrite(1, 0, 1);
  LcdWrite(c);
}

// Writes an array of character (string)
void LcdPuts(const char *s) 
{
   while(*s)
		LcdPutch(*(s++));   		// Warning nested function calls
}

void LcdInit(void)
{
	// GPIO Direction Set.
	// make corresponding higher(0~7) nibble (4 bit) i/o as o/p without upsetting other bits, we will deal with i/p (busy chk) later
	LPC_GPIO1->FIODIRL |= 0xC000;							// P1.14 thru P1.15 as o/p
	LPC_GPIO1->FIODIRH |= 0x0003; 						// P1.16 thru P1.17 as o/p
	// RS, EN, RW Direction Set
	GPIO_SetDir(1, 0, GPIO_DIR_OUTPUT);				// LCD_RS
	GPIO_SetDir(1, 1, GPIO_DIR_OUTPUT);				// LCD_RW_
	GPIO_SetDir(1, 4, GPIO_DIR_OUTPUT);				// LCD_EN
	
	// LCD_RW = 0															// LCD in Write Mode
	GPIO_PinWrite(1, 1, 0);
	//LCD_RS = 0;   													// LCD in command mode
	GPIO_PinWrite(1, 0, 0);
	//LCD_EN = 0;   													// LCD o/p latch disable 
	GPIO_PinWrite(1, 4, 0);
	
	msDelay(100);		// wait 15mSec after power applied,
	
	// FUNCTION SET
	/* Sets to 8-bit operation and selects ?-line display and 5x8 dot character font.
	Number of display lines and character fonts cannot be changed after!
	************************ Function Set ***************************
	-----------------------------------------------------------------
	|	DB7	|	DB6	|	DB5	|	DB4	|	DB3	|	DB2	|	DB1	|	DB0	|
	|	0	|	0	|	1	|	DL	|	N 	|	F 	|	x	|	x	|
	-----------------------------------------------------------------
	DL = Sets interface data length (1 for 8bit)
	N = Sets no. of display lines	(0 for single line)
	F = Font						(0 for 5x8 font)
	0b0011NFxx		;N = Sets the number of display lines. F = Sets the character font.
	-----------------------------
		N	|	F	|	Remark	|
	-----------------------------	
		0	|	0	|	5x8 Font, 1-Line
		0	|	1	|	5x10 Font
		1	|	x	|	5x8 Font, cuz 5x10 2-Line not possible 	
	* *
	0b001100xx 	= 8-bit, 1-Line, 5x8 font	(0x30)
	0b001110xx	= 8-bit, 2-Line, 5x8 font	(0x38)
	ob001010xx	= 4-bit, 1-Line, 5x8 font	(0x22)		
	
	*/
	// Send higher nibble 1st
	LCD_DATA = (LCD_DATA & 0xFFFC3FFF) | (0x03 << 14); 			  				// Function set: 4-bit,  2 Line 5x8 font
	WriteStrobe();            		 	// function set (1)
	msDelay(5);   									// no busy chk possible, so deliberate delay
	
	LCD_DATA = (LCD_DATA & 0xFFFC3FFF) | (0x02 << 14);								// Four bit mode, function set, Busy Flag can be checked now on
	WriteStrobe(); 
		
	msDelay(5);
	LcdWrite(0x28);									// function set (2), // 2-Line, 5x8 font
	msDelay(0);
	//usDelay(200);
	
	// Display ON/OFF Control
	/*
	-----------------------------------------------------------------
	|	DB7	|	DB6	|	DB5	|	DB4	|	DB3	|	DB2	|	DB1	|	DB0	|
	|	0	|	0	|	0	|	0	|	1 	|	D 	|	C	|	B	|
	-----------------------------------------------------------------
	D = 1 ; Display ON & vice-Versa
	C = 1 ; Cursor ON
	B = 1 ; Blink 	*/
	//Display On, Cursor On, Cursor Blink)
	LcdReady();
	LcdWrite(0x0C);
	
	/* Entry Mode Set
	// 0b000001(ID)S
	Increments (I/D = 1) or decrements (I/D = 0)
 	Shifts the entire display either to the right (I/D = 0) or to the left (I/D = 1) when S is 1.
	The display does not shift if S is 0.
	*/
	LcdReady();
	LcdWrite(0x06);      				// Increment cursor to right
	
	// Clear display
	LcdClear();
}

void LcdPutNum(uint16_t val)
{
	uint8_t s[4];
	uint16_t i, j;
	i = 0;

	while(val!=0) {
		s[i] = val%10;
		val = val/10;
		i++;
	  }
	
	j = 3 - i;
	
	if(j)
		while(--j)
			LcdPutch(0x30);
		
	while(i)
	  LcdPutch(s[--i] + 0x30);
}

	
void LcdWrt2CGRAM(uint16_t Loc, uint8_t *Font)
{
	uint16_t i;
	/* The DDRAM Address:
	-----------------------------------------------------------------
	|	DB7	|	DB6	|	DB5	|	DB4	|	DB3	|	DB2	|	DB1	|	DB0	|
	|	0	|	1	|	ACG	|	ACG	|	ACG |	ACG |	ACG	|	ACG	|
	-----------------------------------------------------------------
	*/
	LcdCMD(CGRAM_ADR_MASK + (Loc << 3));			// i.e Loc *= 8;
	for(i = 0; i < 7; i++) {
		LcdPutch((char)*Font++);
	}
	//LcdCMD(DDRAM_ADR_MASK);
}	

uint8_t LcdGotoXY(uint8_t X, uint8_t Y)
{
	// (0x00 ~ 0x13) + 0x80 = 1st Line, 16x2
	// (0x40 ~ 0x53) + 0x80 = 2nd Line, 16x2
	// (0x14 ~ 0x27) + 0x80 = 3rd Line, 16x4
	// (0x54 ~ 0x67) + 0x80 = 4th Line, 16x4
	uint8_t LcdRowLookUp[4] = {0x80, 0xC0, 0x94, 0xD4};
	// Sanity check
	if(X > MAX_ROW || Y > MAX_COL || !X || !Y)
		return 1;
	
	X--;								// actual range starts from 0
	Y--;
	
	LcdCMD(LcdRowLookUp[X] + Y);
	
	return 0;
}

uint8_t LcdPutsXY(uint8_t X, uint8_t Y, const char *String)
{
	if(LcdGotoXY(X, Y))
		return 1;
	LcdPuts(String);
	
	return 0;
}

void Num2CharLcd(uint16_t val, uint8_t DigitJustify)
{
	uint8_t s[6], j = 0, i = 0;

	while(val!=0) {
		s[i] = val%10;
		val = val/10;
		i++;
	  }
	
	switch(DigitJustify) {
		case DIGIT_JUSTIFY_NONE:
			j = 0;
			break;
		case DIGIT_JUSTIFY_2D:
			j = 3 - i;
			break;
		case DIGIT_JUSTIFY_3D:
			j = 4 - i;
			break;
		case DIGIT_JUSTIFY_4D:
			j = 5 - i;
			break;		
		case DIGIT_JUSTIFY_5D:
			j = 6 - i;
			break;
		default:
			j = 0;
			
	}
	
	
	if(j)
		while(--j)
			LcdPutch(0x30);

	while(i)
	  LcdPutch(s[--i] + 0x30);
}
