#ifndef __SIMCOM_H
	#define __SIMCOM_H
	
	#include "LPC17xx.h"
	
	#define GSM_UPDATE_INTERVAL	30000
	// GSM Params
	typedef struct GSMSpec {
		char GSMOptrName[14];					// Airtel, CellOne etc..
		char MEID[16];
		struct GSMStat {
			char SignalQ[3];	 					// rssi in string
			char GSMBand[8];						// biggest possible value is GSM850_PCS_MODE..
			char MEModel[16];
			char IPaddress[16];					// Maximum possible IPv4: 255.255.255.255
			uint32_t BatVoltmV;					// e.g. 3940mV, 4 digits, cant go above 4300mv if it does then you are talking to a dead modem
			uint32_t Rssi;							// rssi in numerical format
			uint32_t ModemStatus:4;
		} GSMStatus;
	} GSMinterface;
	
	typedef struct SIMCOMtime {
		uint16_t Year;
		uint8_t Month;
		uint8_t	Day;
		uint8_t Hour;
		uint8_t Minute;
		uint8_t	Seconds;
		char 	TZone[4];
		uint8_t Ctime[7];
	}SimComCurrentTime;
	
	extern char *DateTime2String(void);
		
	enum {GSM_TURN_OFF, GSM_TURN_ON};
	enum {GSM_STATE_NO_MODEM, GSM_STATE_NO_SIM, GSM_STATE_NO_NETWORK, GSM_STATE_MODEM_NW_SR, GSM_STATE_MODEM_READY, GSM_STATE_MODEM_ROAMING};
	enum {GSM_ERROR_NONE, GSM_CME_ERROR, GSM_CMD_TIMEOUT, GSM_UNKNOWN_ERROR};
#endif
