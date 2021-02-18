#include "StrManip.h"

uint32_t StrLen(char *p)
{
	uint32_t i = 0;
	
	while(*p++){
		if((uint8_t)*p > 0x7F)					// Skips if non-standard ASCII characters are encountered
			break;
		i++;
	}	
		
	return (i);	
}
// Parse up the buffer for character c, return with very next address
char *ParseBuff(char *P, uint32_t c)
{
	//Parse up buffer for c symbol
	while(*P)												// do until '\0', a risky loop, what if , if there's no '\0'
		if(*P++ == c)									// now look for c until end
			return (P);									// returns the next address..
	
	return 0;												// sorry we couldn't find the symbol 'c'
}

// Compares str1 with str2 non-Null terminated, for given length 'len'
// returns 1 on successful comparison
uint32_t StrCmp(char *str1, char *str2, uint32_t len)
{
	uint32_t j;
	uint32_t flag = 1;	

	for(j = 0; j < len; j++)
		if( str1[j] != str2[j] )			// 
				flag = 0;
	return (flag);
}

// Copies str2 to str1, for given length 'len'
void StrCpy(char *dStr, const char *sStr, char DeLim, uint32_t len)
{
	uint32_t j;
	
	for(j = 0; j < len; j++) {
		if( sStr[j] == DeLim || sStr[j] == '\0' ) {
			dStr[j] = '\0';
			break;
		}
		dStr[j] = sStr[j];			// 
	}
}

char *StrCat(char * dest, const char * src)
{
	uint32_t SourceLen;
	char *Rptr = dest;

	SourceLen = StrLen(dest);
	dest += SourceLen;

	while(*src)
		*dest++ = *src++;
	
	*dest = '\0';

	return Rptr;
}

// Convert Str to Num, limited to 4 digits currently
uint32_t Str2Num(uint8_t *StrVal)
{
	uint32_t Num = 0;
	uint32_t i = 0, j;
	uint32_t Pow10[4] = {1, 10, 100, 1000};

	while (*StrVal > 0x2F && *StrVal < 0x3A) {
		StrVal++;
		i++;
	}
	if (i && (i < 5))
		for (j = 1; j <= i; j++)
			Num += ((*(StrVal - j)) - 0x30) *  Pow10[j - 1];
	return Num;
}

uint8_t *Num2Str(uint32_t NumVal)
{
	uint8_t StrNum[11], *StrNumPtr; 						// max size possible digits for unsigned long is 10 digit
	static volatile uint8_t RevStrNum[11];
	uint32_t i, j, k;
	

	StrNumPtr = &StrNum[0];
	RevStrNum[0] = 0x30;
	RevStrNum[1] = '\0';
	
	i = 0;
	if (NumVal == 0 || NumVal > 999999999)
		return (uint8_t *)RevStrNum;
	while (NumVal) {
		*StrNumPtr++ = ((NumVal % 10) | 0x30);
		NumVal /= 10;
		++i;
	}
	i--;
	k = i;
	for (j = 0; j <= k; i--, j++)
		RevStrNum[j] = StrNum[i];

	RevStrNum[j] = '\0';

	return	(uint8_t *)RevStrNum;
}
