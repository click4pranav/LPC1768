#ifndef __INTRUSION_DECT_H
	#define __INTRUSION_DECT_H
	
	#define 	CCLK							100e6
	#define		WDT_TIMEOUT     	300e6																	// Watchodog time out in 5 mins (300s)
	#define 	VERSION_INFO			"0.30b"
	#define 	VERBOSE
	
	#define 	BUZZER_EN 				LPC_GPIO1->FIOPIN & ( 1 << 18)				// P1.18
	#define 	RELAY_EN 					LPC_GPIO1->FIOPIN & ( 1 << 19)				// P1.19
	#define 	GSM1_PWR_KEY 			LPC_GPIO1->FIOPIN & ( 1 << 21)				// P1.21
	#define 	GSM2_PWR_KEY 			LPC_GPIO1->FIOPIN & ( 1 << 24)				// P1.24

	#define 	UART2_BUF_SIZE 		300																		// for the love of GSM2
	#define 	UART3_BUF_SIZE 		250																		// GSM1 Rx Buffer Size
	
	#define 	IMAGE_BUF_SIZE 		30000
	
	#define		FTP_XFER_MIN 			1000
	#define		FTP_XFER_MAX 			1500
	
	#define FTP_SRVR_NAME_LEN		50
	#define FTP_PORT_NAME_LEN		5
	#define FTP_USER_NAME_LEN		25
	#define FTP_PSWD_LEN				25
	#define APN_LEN							15
	
	#define	IMAGE_NAME_LEN			11
	
	// Phone number length inclusive country code (+91)
	#define PHONE_NO_LEN				13	
	
	typedef volatile struct System {
		uint32_t SystemMode:3;
		uint32_t GsmStatus:3;
		uint32_t GPRSup:1;
		uint32_t CameraOk:1;
		uint32_t CurrentSensorOK:1;	
		uint32_t BatCharging:1;
		uint32_t RoninMode:1;
		volatile uint32_t Sensor1Trigger:1;
		volatile uint32_t Sensor2Trigger:1;
		volatile uint32_t Sensor3Trigger:1;
		volatile uint32_t BatLowFlag:1;
		volatile uint32_t BatteryStatus;
	}SystemStatusInd;
	
	// Battery voltage level:	 12.9v,			 		11.7v,				 10v
	enum SYSTEM_BAT_LEVEL {BAT_HIGH = 650, BAT_MID = 600, BAT_LOW = 515};
	
	enum {LOW_BAT, ALERT_1, ALERT_2, ALERT_3, NOTIFY_PHNO, NOTIFY_PHNO_FAIL, NOTIFY_MOD_CHANGE, REMOTE_ALARM, DEBUG_TXT};
	enum {SENSOR_ALIGN, SMS_MODE, CAM_MODE, LIVE_MODE};
	enum {DEBUG_TXT_OFF, DEBUG_TXT_ON, DUMP_CONFIG};
	
	// Functions Prototypes

	// System Related
	extern 	void 		initGPIO(void);
	extern 	void 		initPWM(void);
	extern 	void 		Screen(void);
	extern uint32_t UpdateSystemConfig(void);
	extern uint32_t ReloadSystemConfig(void);
	extern uint32_t VerifyPhoneNo(char *PhNo);
	extern uint32_t isPrintableASCIInoSpace(char *StrParam, uint32_t ParamSize);
	// SIMCOM Related
	extern uint32_t PowerSIM908(uint32_t);
	extern uint32_t GetGSM_SSI(void);
	extern uint32_t ProbModemRspns(void);
	extern char *		RetrieveSIMCOMtime (void);
	extern uint32_t RunUSSD(void);
	extern void			GSMntsUpdate(void);
	// UART2 Related
	extern uint32_t Uart2Transfer (const void *, uint32_t, uint32_t);
	extern uint32_t Uart2TransferChar (const char Ch, uint32_t TimeOut, uint32_t NoTermChara);
	extern void 		FlushUart2Buffer (void);
	// SMS related
	extern uint32_t ChkNuMsg (void);
	extern uint32_t ParseSMS (uint32_t);
	extern uint32_t StatusReport(char *CstrNoPtr, char *MsgBody, uint32_t ExtraInfo);
	// WWW Related
	extern uint32_t InitWWW (void);
	extern uint32_t HttpGetStatusReport(void);
	extern uint32_t HttpRetrieveRecipient (void);
	extern uint32_t FtpPutReady (uint8_t *BufferPtr);
	extern uint32_t HttpGetIntrusion (char *Message, char *UniqueResp);
	extern uint32_t FtpUpload (uint8_t *ImageDPtr, uint32_t ImgSize, char *ImgUniqName);
	extern uint32_t FtpUploadFlash(uint32_t FlashAddr, uint32_t ImgSize, char *ImgUniqName);
	// MMS Related
	extern uint32_t InitMMS (void);
	extern uint32_t SendMMS (char *CstrNoPtr, char *TitleTextPtr, char *TextPtr, char *PicPtr, uint32_t PicSize);
	// Timer
	extern uint32_t getPrescalarForUs (uint8_t timerPclkBit);
	extern void 		initTimer1 (void);
	// Image & Flash Memory
	extern uint32_t ImageFIFO2Flash(uint32_t Addr, uint32_t Imglength);
#endif
