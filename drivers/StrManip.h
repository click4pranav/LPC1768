#ifndef __STR_MANIP_H
	#define __STR_MANIP_H
	
	#include "LPC17xx.h"

	extern uint32_t StrLen(char *);
	extern char *ParseBuff(char *, uint32_t);
	extern uint32_t Str2Num(uint8_t *StrVal);
	extern uint8_t *Num2Str(uint32_t NumVal);
	extern uint32_t StrCmp(char *, char *, uint32_t);
	extern char *StrCat(char *dest, const char *src);
	extern void StrCpy(char *, const char *, char, uint32_t);
 #endif

