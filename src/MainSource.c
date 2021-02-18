/**
*  \file 		MainSource.c
*  \Pranavkumar K.V
****************************************************************************************************************************
*	System Clock
* Fcco = (2 * M * Fin) / N
*	N is PLL0 Pre-divider , M is PLL0 Multiplier
*	where 32khz < Fin <50Mhz, 275Mhz < Fcco < 550Mhz
*	(2 * 16 * 25e6) / 2 = 400Mhz
*	Cclk (CPU Clock divider)  = 4, i.e. Cclk = PLLclk/4 =  100Mhz
*	Pclk (Peripheral Clock)  = Cclk/PCD = 25Mhz (**Peripheral Clock divider = 4)
* 	For PWM1 its PCD = 2, i.e. 50Mhz
*
*	Data Stored in Flash :
*	#1 Master Phone Number, #2 Remote Dev 1 Phone, #3 Remote Dev 2 Phone, #3 APN
*	$4 APN, #5 FTP Credentials,
*						+-----------------------------------------------------+
*						|		LED				|	Status				|	info.					|																																	*
*						+-----------------------------------------------------+																													*
*						|	LED 1				|	Steady				|	Arducam Up & running	|																													*
*						|	LED 2				| Steady				|	Flash mem check OK		|																													*
*						|	LED 3				|	Steady				|	Current Sensor OK			|																													*
*						+-----------------------------------------------------+																													*
*																																																					*
*****************************************************************************************************************************
* Inometrics Technologies PVT Ltd. ("Inometrics") and Pranavkumar KV retains all ownership and  intellectual property rights						*
* in the code accompanying this message and in all derivatives hereto. 
* 																																														*
*					# Updated Time syncing with NTP Server 																									*
*					# Power Saving mode partial implemented:																								*
*						currently restricted to just Modem turn OFF only  																	*
*					# GSM reinitialization if found dead (OFF) during operational mode										  *
*					# Added Status reporting to Web, Image Capture request from Web SMS gateway 					  *
* 																																															*
*					# Added Texting option to check current configuration																		*
*					# Device Text now includes firmware version																							*
*					# Added FTP credentials modification via SMS																						*
*					# Added APN modification via SMS																												*
*					# Added Phone# length modification																											*
*					# Added WatchDog reset to circumvent system failure 																		*
*					# Added progress display whilst system Uploading image via FTP													*
*					# Bug Fix																																								*
*					$ Network strength display																				  								*
*					$ Intermittent FTP upload fails																			  							*
*					$ System getting stuck if SMS distribution list not updated via web						  		*
*					System now detects FTP login error, although no fall-back mechanism implemented  	*
*					# Known Limitation																																			*
*					Distribution list are not supplied with country codes, so we assume										*
*					default country code as +91																													*
*					# Possible data corruption with uploaded image quality of 2MP is selected with ArduCAM	*
*****************************************************************************************************************************/
// CMSIS Driver
#include <LPC17xx.h>
#include <PIN_LPC17xx.h>
#include <GPIO_LPC17xx.h>
#include <Driver_USART.h>									

#include "lpc17xxWDT.h"										// WDT driver of Mine!
#include "lpc17xxUart.h"									// USART Driver Mine!
#include "lpc17xxADC.h"										// ADC Driver Mine!

#include <stdio.h>												// sprintf
#include <string.h>

#include "IntruDect.h"
#include "SysTick.h"
#include "ServiceProvider.h"
#include "StrManip.h"
#include "lcd.h"
#include "ArduCAM.h"
#include "FlashMem.h"
#include "PAC1710.h"
#include "SIMCOM.h"

#define UART2_BAUDRATE 9600

#define		PCLK_TIMER1 	4
#define		SBIT_MR0I 		0
#define		SBIT_MR0R 		1

#define SBIT_CNTEN 0

// The interface to CMSIS
extern ARM_DRIVER_USART Driver_USART0;
static ARM_DRIVER_USART *Uart2drv = &Driver_USART0;

// GSM, SIMCOM800
GSMinterface GSM;
SimComCurrentTime NetworkTime;

// System status flags
SystemStatusInd SystemStatus;

// UART 2 related
volatile bool BufferFill = 0;
volatile uint8_t UART2TxEmpty = 1;
volatile uint32_t UART2Status, TermCharCnt;
uint8_t RxBuffer[UART2_BUF_SIZE], *RxBuffPtr;

// Back-light and Beeper time-out
extern volatile uint32_t BeeperTimeout, ModemRefreshRate;
volatile uint32_t BkLiteLength, AlertStatus, WebRequest;

// Big data, Image Buffer
uint8_t FlashBuffer[FTP_XFER_MAX];

// Phone number storages, extra 1 for NULL character
char CSTR_NUM[PHONE_NO_LEN + 1];											// Master Sender number	
char REM_DEV_NUM_1[PHONE_NO_LEN + 1];									// Remote Device-1
char REM_DEV_NUM_2[PHONE_NO_LEN + 1];									// Remote Device-2
char RecipientNo[11][PHONE_NO_LEN + 1];								// Distribution list
// FTP Credentials
char FtpPort[FTP_PORT_NAME_LEN];											// The value of FTP Control port, from 1 to 65535. Default value is 21
char FtpServerName[FTP_SRVR_NAME_LEN];								// Alphanumeric ASCII text string up to 49 characters if DNS is available (SIM800 limitation)
char FtpUserName[FTP_USER_NAME_LEN];
char FtpUserPassword[FTP_PSWD_LEN];
// ISP configurations, Access Point Name
char APN[APN_LEN];
// Image Name, this used to be a local variable but had to change it in last moment
char UniqRespName[IMAGE_NAME_LEN];

// Hitachi HD44780 5x7 custom font
uint8_t NWfont[] 			=	{0x1F, 0x1F, 0x0E, 0x04, 0x04, 0x04, 0x04};
uint8_t NWStrFull[] 	= {0x01, 0x01, 0x05, 0x05, 0x15, 0x15, 0x15};
uint8_t NWStrMid[] 		= {0x00, 0x00, 0x04, 0x04, 0x14, 0x14, 0x14};
uint8_t NWStrLow[] 		= {0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10};
uint8_t ClearPixel[] 	= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BattFull[]		=	{0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F};
uint8_t BattMid[]			=	{0x0E, 0x11, 0x11, 0x11, 0x1F, 0x1F, 0x1F};
uint8_t BattLow[]			=	{0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F};
uint8_t BattBad[]			=	{0x0E, 0x1F, 0x1B, 0x15, 0x1B, 0x15, 0x1F};

void EINT0_IRQHandler (void) {
	NVIC_ClearPendingIRQ(EINT0_IRQn);
	LPC_SC->EXTINT |= 1;												// Clear EINT0
	AlertStatus |= 8;
}

// UART2 Interrupt handler
void UART2_IRQHandler (void) {

	uint8_t IIRValue, LSRValue;

  IIRValue = LPC_UART2->IIR;										// IntId (Interrupt identification) 010 0 (IntStatus)

  IIRValue >>= 1;																// skip pending bit in IIR
  IIRValue &= 0x07;															// check bit 1~3, interrupt identification
  if ( IIRValue == IIR_RLS ) {									// Receive Line Status
		LSRValue = LPC_UART2->LSR;
		// Receive Line Status
		if ( LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI) ) {
			// There are errors or break interrupt, Reading LSR will clear the interrupt
			UART2Status = LSRValue;
			LPC_UART2->RBR;														// Dummy read on RX to clear interrupt, then bail out
			return;
		}
		if ( LSRValue & LSR_RDR )	 { 								// Receive Data Ready
			// If no error on RLS, normal read, save into the data buffer, read RBR will clear the interrupt
			*RxBuffPtr++ = LPC_UART2->RBR;
			if ( (RxBuffPtr - RxBuffer)  == UART2_BUF_SIZE) {
				BufferFill = 1;														// Buffer overflow
				RxBuffPtr	 = &RxBuffer[0];
			}
		}
  }
  else if ( IIRValue == IIR_RDA ) {									// Receive Data Available
		*RxBuffPtr++ = LPC_UART2->RBR;
		if ( (RxBuffPtr - RxBuffer)  == UART2_BUF_SIZE) {
			BufferFill = 1;																// Buffer overflow
			RxBuffPtr	 = &RxBuffer[0];
		}
  }
	else if ( IIRValue == IIR_CTI )										// Character timeout indicator
		UART2Status |= 0x100;														// Bit 9 as the CTI error
	else if ( IIRValue == IIR_THRE ) {								// THRE, transmit holding register empty
		LSRValue = LPC_UART2->LSR;											// Check status in the LSR to see if valid data in U0THR or not
	if ( LSRValue & LSR_THRE )
		UART2TxEmpty = 1;
	else
	  UART2TxEmpty = 0;
  }
	if(*(RxBuffPtr - 1) == '\r')
		TermCharCnt++;
	NVIC_ClearPendingIRQ(UART2_IRQn);
}

void TIMER1_IRQHandler(void)
{
  uint32_t isrMask;

  isrMask = LPC_TIM1->IR;
	LPC_TIM1->IR = isrMask;         // Clear the Interrupt Bit

	if(BkLiteLength) {
		LPC_PWM1->MR2  	= 99;
		LPC_PWM1->LER		= 0x05;				// Latch Last value written to the PWM Match 2
		BkLiteLength--;
	}
	else {
		if(LPC_PWM1->MR2) {
			LPC_PWM1->MR2 	-= 1;
			LPC_PWM1->LER		= 0x05;				// Latch Last value written to the PWM Match 2
		}
	}
}

// EINT3
void EINT3_IRQHandler(void) {
	  if(LPC_GPIOINT->IO2IntStatF & (1 << 3)) {					// We have Sensor 1 failing
			LPC_GPIOINT->IO2IntClr |= (1 << 3);
			SystemStatus.Sensor1Trigger = 1;
			AlertStatus |= 1;
	}
	if(LPC_GPIOINT->IO2IntStatF & (1 << 4)) {						// We have Sensor 2 failing
			LPC_GPIOINT->IO2IntClr |= (1 << 4);
			SystemStatus.Sensor2Trigger = 1;
			AlertStatus |= 2;
	}
	if(LPC_GPIOINT->IO2IntStatF & (1 << 5)) {						// We have Sensor 3 failing
			LPC_GPIOINT->IO2IntClr |= (1 << 5);
			SystemStatus.Sensor3Trigger = 1;
			AlertStatus |= 4;
	}

	NVIC_ClearPendingIRQ(EINT3_IRQn);
	LPC_SC->EXTINT |= (1 << 3);													// Clear EINT3
	//NVIC_DisableIRQ(EINT3_IRQn);

	BeeperTimeout = 500;
}

int main()
{
	uint32_t i, ImgLength, RecipientCount;
	
	FLASH_DEVICE_OBJECT fdo;

	ImgLength											= 0;
	BufferFill 										= 0;
	WebRequest										= 0;
	TermCharCnt 									= 0;
	RecipientCount								= 0;
	UniqRespName[0]								= '\0';
	SystemStatus.CameraOk 				= 0;
	SystemStatus.CurrentSensorOK 	= 0;
	SystemStatus.BatLowFlag 			= 0;
	SystemStatus.BatCharging			= 0;
	SystemStatus.SystemMode				= LIVE_MODE;
	RxBuffPtr 										= &RxBuffer[0];

	// 1mS SysTick
	SysTick_Config (SystemCoreClock/1000);									// RELOAD = (cclk / ticks) - 1

	// WDT interrupt priority set (but not enabling it), Configure WDT mode, enable with timer
	NVIC_SetPriority(WDT_IRQn, 0x10);												// default exception# is 0x10 (16)  
	WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_RESET);
	WDT_Start(WDT_TIMEOUT);
	
	// Initialize the GPIOs
	initGPIO();
	// Turn ON all LEDs, for a short while
	GPIO_PinWrite(1, 8,  1);	
	GPIO_PinWrite(1, 9,  1);
	GPIO_PinWrite(1, 10, 1);
	GPIO_PinWrite (1, 19, 1);					// Turn ON Speed-Light 
	
	// Timer 1 initialization
	initTimer1();

	// UART:2
	Uart2Init(UART2_BAUDRATE);
	LPC_UART2->IER = 1;																				// Enable Uart2 Rx Interrupt
	NVIC_EnableIRQ(UART2_IRQn);

	#ifdef VERBOSE
		Uart2drv->Initialize(NULL);
		Uart2drv->PowerControl(ARM_POWER_FULL);
		Uart2drv->Control(ARM_USART_MODE_ASYNCHRONOUS |
												ARM_USART_DATA_BITS_8			|
												ARM_USART_PARITY_NONE			|
												ARM_USART_STOP_BITS_1			|
												ARM_USART_FLOW_CONTROL_NONE, 9600);
		Uart2drv->Control(ARM_USART_CONTROL_TX, 1);												// Tx Enabled
		Uart2drv->Control(ARM_USART_CONTROL_RX, 1);												// Rx Enabled
		//Uart2drv->Send("DeBug Port Up\r\n", 14);
	#endif

	// PWM Initialization
	initPWM();
	
	// Turn OFF all LEDs
	msDelay(2000);
	GPIO_PinWrite(1, 8,  0);	
	GPIO_PinWrite(1, 9,  0);
	GPIO_PinWrite(1, 10, 0);
	GPIO_PinWrite(1, 19, 0);					// Turn OFF Speed-Light 	

	// This statement specifically check for Micron N25Q 64Mb flash, if you are using different flash please change accordingly
	if(!FlashMemInit(&fdo)) {
		GPIO_PinWrite(1, 9, 1);																					// Status LED2 indicate Flash memory presence
		ReloadSystemConfig();
		if( (uint8_t)SystemStatus.SystemMode > 3 ) 
			SystemStatus.SystemMode = LIVE_MODE;													// Default mode is always live mode
	}		
	// Check PAC1710 presence
	if(!InitPAC1710()) {
		GPIO_PinWrite(1, 10, 1);																				// Status LED3 ON to indicate Current Sensor functionality
		SystemStatus.CurrentSensorOK = 1;
	}
	// Check if we do have a valid Master phone number
	if(!VerifyPhoneNo(&CSTR_NUM[0]))
		SystemStatus.RoninMode = 1;
	else
		SystemStatus.RoninMode = 0;

	// Cam OV2620 ArduCAM 2MP + I2C0
	if(!ArduCamInit()) {
			GPIO_PinWrite(1, 8, 1);																				// Status LED1 ON to indicate the presence of ArduCam
			SystemStatus.CameraOk = 1;
	}
	// Initialize LCD to 16x2, and then write our custom CG to CGRAM
	LcdInit();
	LcdWrt2CGRAM(0, &NWfont[0]);
	LcdWrt2CGRAM(1, &NWStrMid[0]);
	LcdWrt2CGRAM(2, &NWStrLow[0]);
	LcdWrt2CGRAM(3, &NWStrFull[0]);
	LcdWrt2CGRAM(4, &ClearPixel[0]);

	BkLiteLength = 100;
	LcdCMD(0x83);
	LcdPuts("INOMETRICS");
	LcdCMD(0x80 | 0x43);
	LcdPuts("Technology");
	msDelay(1000);
	
	if(SystemStatus.CurrentSensorOK)
		PAC1710OneShotMeasure();

	//ArduCAMtransfer(ARDC_REG_GPIO_WR + 0x80, GPIO_PWDN_MASK);
	PowerSIM908(GSM_TURN_ON);

	RetrieveSIMCOMtime();
	WDT_Feed();
	//WDTOF Watchdog time-out flag. Set when the watchdog timer times out, cleared by software. and is 0 when Only after POR
	if ((LPC_WDT->WDMOD >> 2) & 1) {
		StatusReport(&CSTR_NUM[0], "System Rebooted from Bug Check", DEBUG_TXT_ON);
		LPC_WDT->WDMOD &= ~(1 << 2);
	}

	LcdClear();
	i = 3;
	do {
		RecipientCount = HttpRetrieveRecipient();
		i--;
	}while(!RecipientCount && i);
	//ADC_Initialize();																	// AD0.0, i.e. P[0].23
	
	NVIC_EnableIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(EINT0_IRQn);
	AlertStatus = 0;

	LcdClear();
	
	while(1)
	{
		PAC1710OneShotMeasure();
		if(!SystemStatus.BatLowFlag) {
			if(!ModemRefreshRate) {
				if(GetGSM_SSI() || !GSM.GSMStatus.ModemStatus) {
					PowerSIM908(GSM_TURN_ON);
				}
				if(GSM.GSMStatus.ModemStatus > GSM_STATE_NO_NETWORK) {
					i = ChkNuMsg();																					// Check for the arrival of new messages
					if(i)
						ParseSMS(i);
				}
				ModemRefreshRate = GSM_UPDATE_INTERVAL;
			}
			// Update the screen
			Screen();
			// If any alert status true
			if(AlertStatus) {
				if(SystemStatus.CameraOk && (SystemStatus.SystemMode == LIVE_MODE || SystemStatus.SystemMode == CAM_MODE) ) {
					// Take a picture
					GPIO_PinWrite (1, 19, 1);					// Turn ON Speed-Light 
					msDelay(2000);
					FlushFIFO();
					StartCapture();
					while(!CheckFIFOdone())
						continue;
					ImgLength = ReadFIFOlen();																						// Read the 'FIFO length' in var Status
					GPIO_PinWrite (1, 19, 0);				// Turn OFF Speed-Light
				}					// Picture loop
				BkLiteLength = 100;
				if(SystemStatus.Sensor1Trigger) {
					SystemStatus.Sensor1Trigger = 0;
					if(SystemStatus.SystemMode == SMS_MODE || SystemStatus.SystemMode == LIVE_MODE) {
						StatusReport((char *)&CSTR_NUM[0], "Sensor 1 Tripped!!", DEBUG_TXT_ON);
						StatusReport(&REM_DEV_NUM_1[0], "ONEID", DEBUG_TXT_OFF);
						StatusReport(&REM_DEV_NUM_2[0], "ONEID", DEBUG_TXT_OFF);
						for(i = 0; i < RecipientCount; i++)
							StatusReport(RecipientNo[i], "Sensor 1 Tripped!!", DEBUG_TXT_ON);
					}
					if(SystemStatus.SystemMode == LIVE_MODE)
						HttpGetIntrusion("Alarm: Sensor 1 Triggered", &UniqRespName[0]);
					AlertStatus &= 0xFFFFFFFE;
				}
				if(SystemStatus.Sensor2Trigger) {
					SystemStatus.Sensor2Trigger = 0;
					if(SystemStatus.SystemMode == SMS_MODE || SystemStatus.SystemMode == LIVE_MODE) {
						StatusReport((char *)&CSTR_NUM[0], "Sensor 2 Tripped!!", DEBUG_TXT_ON);
						StatusReport(&REM_DEV_NUM_1[0], "ONEID", DEBUG_TXT_OFF);
						StatusReport(&REM_DEV_NUM_2[0], "ONEID", DEBUG_TXT_OFF);
						for(i = 0; i < RecipientCount; i++)
							StatusReport(RecipientNo[i], "Sensor 2 Tripped!!", DEBUG_TXT_ON);
					}
					if(SystemStatus.SystemMode == LIVE_MODE)
						HttpGetIntrusion("Alarm: Sensor 2 Triggered", &UniqRespName[0]);
					AlertStatus &= 0xFFFFFFFD;
				}
				// Sensor 3 Trigger
				if(SystemStatus.Sensor3Trigger) {
					SystemStatus.Sensor3Trigger = 0;
					if(SystemStatus.SystemMode == SMS_MODE || SystemStatus.SystemMode == LIVE_MODE) {
						StatusReport((char *)&CSTR_NUM[0], "Sensor 3 Tripped!!", DEBUG_TXT_ON);
						StatusReport(&REM_DEV_NUM_1[0], "ONEID", DEBUG_TXT_OFF);
						StatusReport(&REM_DEV_NUM_2[0], "ONEID", DEBUG_TXT_OFF);
						for(i = 0; i < RecipientCount; i++)
							StatusReport(RecipientNo[i], "Sensor 3 Tripped!!", DEBUG_TXT_ON);
					}
					if(SystemStatus.SystemMode == LIVE_MODE)
						HttpGetIntrusion("Alarm: Sensor 3 Triggered", &UniqRespName[0]);
					AlertStatus &= 0xFFFFFFFB;
				}
				// External Key trigger to take a Picture
				if(AlertStatus & 0x08) {
					LcdClear();
					BeeperTimeout = 500;
					LcdPutsXY(1, 5, "Toast !!");
					AlertStatus &= 0xFFFFFFF7;
					msDelay(500);
				}
				if(ImgLength) {
					// Store JPEG Image in FIFO to Flash Memory
					ImageFIFO2Flash(0x10000, ImgLength);
					i = 3;
					if(!strlen(UniqRespName))
						strcpy(UniqRespName, RetrieveSIMCOMtime());
					// Uploaded the stored image from flash to FTP server
					do {
						i--;	
					} while ( FtpUploadFlash(0x10000, ImgLength, UniqRespName) && i);
					
					ImgLength = 0;
					strcpy(UniqRespName, "\0");	
				}
			}
			// Got request to update SMS distribution list
			if(WebRequest & 0x0001) {
				RecipientCount = HttpRetrieveRecipient();
				WebRequest &= 0xFFFFFFFE;
			}
			// Got request to update system status to Web
			if(WebRequest & 0x0002) {
				HttpGetStatusReport();
				WebRequest &= 0xFFFFFFFD;
			}
			if(PAC1710Regs.vSourceCh1 < BAT_LOW) {
				SystemStatus.BatLowFlag = 1;
				BkLiteLength = 0;
				StatusReport((char *)&CSTR_NUM[0], "Battery Low Shutdown!!", DEBUG_TXT_ON);
				PowerSIM908(GSM_TURN_OFF);
				LcdClear();
				LcdPuts("Low Battery!");
			}
		}  // SystemStatus.BatLowFlag True
		else {
			if(PAC1710Regs.vSourceCh1 > BAT_MID) {
				SystemStatus.BatLowFlag = 0;
				PowerSIM908(GSM_TURN_ON);
				AlertStatus = 0;
			}
		}
		WDT_Feed();
		msDelay(750);
	}						// While (1)
	return 0;
}							// Main() ends here

void initGPIO(void)
{
	GPIO_PortClock(1);

	// Status LEDs
	GPIO_SetDir   (1, 8,  GPIO_DIR_OUTPUT);																					// LED 1	@P1.8
	GPIO_SetDir   (1, 9,  GPIO_DIR_OUTPUT);																					// LED 2	@P1.9
	GPIO_SetDir   (1, 10, GPIO_DIR_OUTPUT);																					// LED 3	@P1.10
	// Charger Status Input
	GPIO_SetDir		(2, 6, 	GPIO_DIR_INPUT);																					// CHG_STAT1
	GPIO_SetDir		(2, 7, 	GPIO_DIR_INPUT);																					// CHG_STAT2
	// Buzzer Enable
	GPIO_SetDir 	(1, 18, GPIO_DIR_OUTPUT);
	// Relay Enable
	GPIO_SetDir 	(1, 19, GPIO_DIR_OUTPUT);																					// Relay Enable P1.19
	GPIO_PinWrite (1, 19, 0);																												// Relay is OFF	
	// LCD Backlight
	GPIO_SetDir 	(1, 20, GPIO_DIR_OUTPUT);																					// LCD Backlight P1.20
	// IR Sensor, again will be using internal pull-up
	GPIO_SetDir		(2, 3, 	GPIO_DIR_INPUT);
	GPIO_SetDir		(2, 4, 	GPIO_DIR_INPUT);
	GPIO_SetDir		(2, 5, 	GPIO_DIR_INPUT);
	PIN_Configure	(2, 3,  PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);			// Sensor 1 @P2.3
	PIN_Configure	(2, 4,  PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);			// Sensor 2 @P2.4
	PIN_Configure	(2, 5,  PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);			// Sensor 3 @P2.5
	// Test Switch /w external interrupt
	GPIO_SetDir		(2, 10,	GPIO_DIR_INPUT);																					// Test In @P2.10
	PIN_Configure	(2, 10,	PIN_FUNC_1, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);			// EINT0 /w Pull-up

	// DIP switch 2x2, Configuration
	GPIO_SetDir		(2, 0, 	GPIO_DIR_INPUT);																					// @P2.0
	GPIO_SetDir		(2, 1, 	GPIO_DIR_INPUT);																					// @P2.1
	PIN_Configure	(2, 0,  PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
	PIN_Configure	(2, 1,  PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);

	// SD card Dect
	GPIO_SetDir		(0, 4, 	GPIO_DIR_INPUT);																					// @P0.4
	PIN_Configure	(0, 4,  PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);

	//NOR flash, Chip selection not allowed, So active CS# for the time being (this will be later controlled by SSP Module)
	GPIO_SetDir		(0, 6, 	GPIO_DIR_OUTPUT);
	GPIO_PinWrite	(0, 6, 1);																												//CS# is deactivated

	// PWM1
	GPIO_SetDir		(1, 20,	GPIO_DIR_OUTPUT);
	PIN_Configure	(1, 20, PIN_FUNC_2, PIN_PINMODE_NORMAL, PIN_PINMODE_NORMAL);			// LCD Backlight

	// GSM 2 GPIO set-ups
	// Power Button
	GPIO_SetDir		(1, 24, 	GPIO_DIR_OUTPUT);																				// SIMCOM-2 Power Key
	GPIO_PinWrite	(1, 24, 0);																												// make sure its clear (Off)
	
	// Status Pin
	GPIO_SetDir		(1, 25, 	GPIO_DIR_INPUT);																				// SIMCOM-2 Status input
	PIN_Configure	(1, 25, PIN_FUNC_0, PIN_PINMODE_PULLDOWN, PIN_PINMODE_NORMAL);
	// DCD Pin
	GPIO_SetDir		(1, 26, 	GPIO_DIR_INPUT);																				// SIMCOM-2 DCD o/p
	PIN_Configure	(1, 26, PIN_FUNC_0, PIN_PINMODE_PULLDOWN, PIN_PINMODE_NORMAL);

	//I2C
	//PIN_Configure	(0, 27, PIN_FUNC_1, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
	//PIN_Configure	(0, 28, PIN_FUNC_1, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);

	// External INT0
	// Level sensitive, EINT0 is low-active
	// GPIO Interrupt enable on Sensor 1, Sensor 2, Sensor 3
	LPC_GPIOINT->IO2IntEnF =  (1 << 3) | (1 << 4) | (1 << 5);
}

/**
\fn
\brief Initializes Timer 1
\return void
**/
void initTimer1(void)
{
	LPC_TIM1->MCR  = (1 << SBIT_MR0I) | (1 << SBIT_MR0R); 		  // Clear TC on MR0 match and Generate Interrupt
  LPC_TIM1->PR   = getPrescalarForUs(PCLK_TIMER1);	     			// Pre-scalar for 1us
  LPC_TIM1->MR0  = 500000;							                 			// Load timer value to generate 500ms delay
  LPC_TIM1->TCR  = (1 << SBIT_CNTEN);    	  	           			// Start timer by setting the Counter Enable
	NVIC_EnableIRQ(TIMER1_IRQn); 													 			// Enable Timer1 Interrupt
}

unsigned int getPrescalarForUs(uint8_t timerPclkBit)
{
    uint32_t pclk,prescalarForUs;
    pclk = (LPC_SC->PCLKSEL0 >> timerPclkBit) & 0x03;  // get the pclk info for required timer

    switch ( pclk )                                    // Decode the bits to determine the pclk
    {
    case 0x00:
        pclk = SystemCoreClock/4;
        break;

    case 0x01:
        pclk = SystemCoreClock;
        break;

    case 0x02:
        pclk = SystemCoreClock/2;
        break;

    case 0x03:
        pclk = SystemCoreClock/8;
        break;

    default:
        pclk = SystemCoreClock/4;
        break;
    }

    prescalarForUs = pclk/1000000 - 1;                    // Pre-scalar for 1us (1000000Counts/sec)

    return prescalarForUs;
}

/**
\fn
\brief Initializes PWM
\return void
**/
void initPWM()
{
	/**
	Fpwm Calculation
	Pclk = Cclk/4, i.e. 100Mhz
	Fpwm = Pclk/PR/MRx
	e.g. PR = 25, MR0 = 1e6 (0xF4240) & MR2 = MR0/2 (0x7A120), yields 1 Hz rate, i.e. 25e6/25/1e6
	**/
	// PWM1.2
	LPC_PWM1->TCR		= 2;					// Timer Control Register: Counter disabled (b0), Counter Reset (b1), PWM disable (b3)
	LPC_PWM1->PR		= 0;					// Pre-scale Register, The TC is incremented every PR+1 cycles of PCLK
	LPC_PWM1->MCR		= 2;					// Reset on PWMMR0: the PWMTC will be reset if PWMMR0 matches it
	LPC_PWM1->MR0 	= 100;				// PWM rate, MR0 can be enabled in the MCR to reset the TC
	LPC_PWM1->MR2 	= 98;
	LPC_PWM1->LER		= 0x05;				// Latch Last value written to the PWM Match 2
	LPC_PWM1->PCR		= 1 << 10;		// PWMENA2 (b10) PWM2 enabled

	LPC_SC->PCONP		|=  1 << 6;		// Power to PWM1, On reset, the PWM is enabled (PCPWM1 = 1), default condition is enabled
	LPC_PWM1->TCR		= 9;					// Timer Control Register, Counter Enable, PWM Enable
}

/**
 *  \brief Brief description
 *  
 *  \return Return description
 *  
 *  \details More details
 */
uint32_t UpdateSystemConfig(void)
{
	uint8_t CurrentSystemMode;
	uint32_t FlashStartAddress = 0;

	CurrentSystemMode = (uint8_t) SystemStatus.SystemMode;
	
	// Erase first Sub-sector, i.e. 1st 4KB
	FlashSubSectorErase(0);
	
	FlashDataProgram(FlashStartAddress, (uint8_t *)&CSTR_NUM[0], sizeof(CSTR_NUM), SPI_FLASH_INS_PP);
	
	FlashStartAddress += sizeof(CSTR_NUM);
	FlashDataProgram(FlashStartAddress, (uint8_t *)&REM_DEV_NUM_1[0], sizeof(REM_DEV_NUM_1), SPI_FLASH_INS_PP);
	
	FlashStartAddress += sizeof(REM_DEV_NUM_1);
	FlashDataProgram(FlashStartAddress, (uint8_t *)&REM_DEV_NUM_2[0], sizeof(REM_DEV_NUM_2), SPI_FLASH_INS_PP);
	
	FlashStartAddress += sizeof(REM_DEV_NUM_2);
	FlashDataProgram(FlashStartAddress, (uint8_t *)&FtpServerName[0], sizeof(FtpServerName), SPI_FLASH_INS_PP);
	
	FlashStartAddress += sizeof(FtpServerName);
	FlashDataProgram(FlashStartAddress, (uint8_t *)&FtpPort[0], sizeof(FtpPort), SPI_FLASH_INS_PP);
	
	FlashStartAddress += sizeof(FtpPort);
	FlashDataProgram(FlashStartAddress, (uint8_t *)&FtpUserName[0], sizeof(FtpUserName), SPI_FLASH_INS_PP);
	
	FlashStartAddress += sizeof(FtpUserName);
	FlashDataProgram(FlashStartAddress, (uint8_t *)&FtpUserPassword[0], sizeof(FtpUserPassword), SPI_FLASH_INS_PP);
	
	FlashStartAddress += sizeof(FtpUserPassword);
	FlashDataProgram(FlashStartAddress, (uint8_t *)&APN[0], sizeof(APN), SPI_FLASH_INS_PP);
	
	FlashStartAddress += sizeof(APN);
	FlashDataProgram(FlashStartAddress, &CurrentSystemMode, 1, SPI_FLASH_INS_PP);

	return 0;
}

/**
 *  \brief Brief description
 *  
 *  \return Return description
 *  
 *  \details More details
 */
 uint32_t ReloadSystemConfig(void)
{
	uint32_t FlashStartAddress = 0;
	uint8_t CurrentSystemMode;
	
	// Address range 0x00:0x0C (length 13)
	FlashDataRead(FlashStartAddress, (uint8_t *)&CSTR_NUM[0], sizeof(CSTR_NUM));
	if( !VerifyPhoneNo(&CSTR_NUM[0]) )
		strcpy(&CSTR_NUM[0], "");
	
	FlashStartAddress += sizeof(CSTR_NUM);
	// Address range 0x0D:0x19 (length 13)
	FlashDataRead(FlashStartAddress, (uint8_t *)&REM_DEV_NUM_1[0], sizeof(REM_DEV_NUM_1));
	if( !VerifyPhoneNo(&REM_DEV_NUM_1[0]) )
		strcpy(&REM_DEV_NUM_1[0], "");
	
	FlashStartAddress += sizeof(REM_DEV_NUM_1);
	// Address range 0x1A:0x26 (length 13)
	FlashDataRead(FlashStartAddress, (uint8_t *)&REM_DEV_NUM_2[0], sizeof(REM_DEV_NUM_2));
	if( !VerifyPhoneNo(&REM_DEV_NUM_2[0]) )
		strcpy(&REM_DEV_NUM_2[0], "");
	
	FlashStartAddress += sizeof(REM_DEV_NUM_2);
	// Address range 0x27:0x59 (length 50)
	FlashDataRead(FlashStartAddress, (uint8_t *)&FtpServerName[0], sizeof(FtpServerName));
	if( !isPrintableASCIInoSpace(&FtpServerName[0], sizeof(FtpServerName)) )
		strcpy(FtpServerName, "");
	
	FlashStartAddress += sizeof(FtpServerName);
	// Address range 0x5A:0x5F (length 5)
	FlashDataRead(FlashStartAddress, (uint8_t *)&FtpPort[0], sizeof(FtpPort));
	if( !isPrintableASCIInoSpace(&FtpPort[0], sizeof(FtpPort)) )
		strcpy(&FtpPort[0], "21");
	
	FlashStartAddress += sizeof(FtpPort);
	// Address range 0x60:0x6F (length 15)
	FlashDataRead(FlashStartAddress, (uint8_t *)&FtpUserName[0], sizeof(FtpUserName));
	if( !isPrintableASCIInoSpace(&FtpUserName[0], sizeof(FtpUserName)) )
		strcpy(&FtpUserName[0], "");
	
	FlashStartAddress += sizeof(FtpUserName);
	// Address range 0x70:0x7F (length 15)
	FlashDataRead(FlashStartAddress, (uint8_t *)&FtpUserPassword[0], sizeof(FtpUserPassword));
	if( !isPrintableASCIInoSpace(&FtpUserPassword[0], sizeof(FtpUserPassword)) )
		strcpy(&FtpUserPassword[0], "");
	
	FlashStartAddress += sizeof(FtpUserPassword);
	FlashDataRead(FlashStartAddress, (uint8_t *)&APN[0], sizeof(APN));
	if( !isPrintableASCIInoSpace(&APN[0], sizeof(APN)) )
		strcpy(&APN[0], "");
		
	FlashStartAddress += sizeof(APN);	
	FlashDataRead(FlashStartAddress, &CurrentSystemMode, 1);
	
	SystemStatus.SystemMode = CurrentSystemMode;
	
	return 0;
}
/**
 *  \brief Verifies if the phone number is in proper order or not
 *  
 *  \param [in] PhNo Input Phone of 13 digits length
 *  \return Return 1 on success
 *  
 *  \details More details
 */
uint32_t VerifyPhoneNo(char *PhNo)
{
	uint32_t PhNoLen, i, VerifyOkFlag;

	VerifyOkFlag = 1;
	PhNoLen = StrLen(PhNo);

	if(PhNoLen != 13)
		return 0;

	for(i = 1; i < PhNoLen; i++)												// We skip first value as its '+'
		if( *(PhNo + i) < 0x30 || *(PhNo + i) > 0x39)			// between 0 and 9
			VerifyOkFlag = 0;

	return VerifyOkFlag;
}
/**
* \Fn			:	uint32_t PowerSIM8x(uint32_t Action)
*																					
* \Brief		:	Turns On or OFF SIM90x Module
*										
* \param		: 
*								
* \return		:  	0 ; On success
*								1 ; If Error is received from MODEM
**/
uint32_t PowerSIM908(uint32_t Action)
{
	char *PtrIndex;
	uint32_t i;					// Retry count is 3
	
	LcdClear();
	
	switch (Action){
		case GSM_TURN_ON:
									i = 4;
									LcdPuts("Init Modem");
									// loops around until we get a response from Modem
									do {
										GPIO_PinWrite (1, 24, 1);													// Modem ON
										msDelay(1200);																		// User can power up\down SIM908 by pulling down the PWRKEY < 1s
										GPIO_PinWrite (1, 24, 0);													// Modem OFF

										SysTimeOut = 2000;
										while(SysTimeOut && !GPIO_PinRead(1, 25))					// Modem Power ON (Status high) or time-out
											continue;
										i--;
										LcdPutch('.');
									}while( (Uart2Transfer("AT\r", 0xC8, 4)) && i );		// Try AT\r, if modem not replied in time: with \r\nAT\r\n\r\nOK\r\n, will do it again..

									// Turn OFF Echo
									Uart2Transfer("ATE0\r", 0xC8, 4);
									
									if(!i) {
										StrCpy(&GSM.GSMOptrName[0], "GSM ERR", '\0', 7);
										GSM.GSMStatus.ModemStatus = GSM_STATE_NO_MODEM;
										LcdPuts("FAIL");
										return 1;
									}
									LcdPuts("OK");
									
									msDelay(2000);
									// Get ME Model
									Uart2Transfer("AT+GMM\r", 0xC8, 2);
									PtrIndex = ParseBuff((char *)&RxBuffer[0], '\r');				// look for CR
									if(*PtrIndex == '\n') {																	// Next one has to be LF
										PtrIndex++;
										StrCpy(&GSM.GSMStatus.MEModel[0], PtrIndex, '\r', 15);
										LcdPutsXY(2, 1, GSM.GSMStatus.MEModel);
									}
									msDelay(2000);
									LcdClear();
									// Get MEid etc
									Uart2Transfer("AT+GSN\r", 0xC8, 2);
									PtrIndex = ParseBuff((char *)&RxBuffer[0], '\r');				// look for CR
									if(*PtrIndex == '\n') {																	// Next one has to be LF
										PtrIndex++;
										StrCpy(&GSM.MEID[0], PtrIndex, '\r', 15);
										LcdPutsXY(2, 1, GSM.MEID);
									}
									msDelay(2500);
									// Now check for SIM Regs.
									LcdPutsXY(1, 1, "Network");
									i = 8;
									do {
										LcdPutch('.');
										// Network Registration
										Uart2Transfer("AT+CREG?\r", 0xC8, 2);
										/** 	+CREG: 0,0 (ME is not registered, currently not searching')
													+CREG: 0,1 (ME registered to Home n/w)
													+CREG: 0,2 (ME is not registered, currently searching')
										**/
										PtrIndex = ParseBuff((char *)&RxBuffer[0], '+');		// look for colon
										if(StrCmp(PtrIndex, "CREG: 0", 7)) {
											switch(PtrIndex[8]) {
													case '0':
																StrCpy(&GSM.GSMOptrName[0], "No SIM!!!", '\0', 6 );
																GSM.GSMStatus.ModemStatus = GSM_STATE_NO_SIM;
																i = 0;																										// This will free us from the loop
																break;
													case '1':
													case '5':
																i = 0;
																// Time to get the operator name
																msDelay(200);																							// wait few mSec
																Uart2Transfer("AT+COPS?\r", 0xC8, 2);											// +COPS: 0,0,"Bharat Kerala"
																PtrIndex = ParseBuff((char *)&RxBuffer[0], '"');					// Parse up buffer for " symbol
																if(PtrIndex)												// Okay we found it
																	StrCpy(&GSM.GSMOptrName[0], PtrIndex, '"', 9);					// copy the name
																else {														// Na...ptr ---> NULL
																	StrCpy(&GSM.GSMOptrName[0], "Emergency", '\0', 10 );
																	GSM.GSMStatus.ModemStatus = GSM_STATE_NO_NETWORK;
																}
																// Probe Modem current working band, EDGE or GSM
																Uart2Transfer("AT+CBAND?\r", 0xC8, 2);										// +CBAND: "EGSM_DCS_MODE"
																PtrIndex = ParseBuff((char *)&RxBuffer[0], '"');					// Parse up buffer for " symbol
																if(PtrIndex)												// found it
																	StrCpy(&GSM.GSMStatus.GSMBand[0], PtrIndex, '"', 1);
																GSM.GSMStatus.ModemStatus = GSM_STATE_MODEM_READY;
																break;
													case '2':
																StrCpy(&GSM.GSMOptrName[0], "Searching", '\0', 10 );
																GSM.GSMStatus.ModemStatus = GSM_STATE_MODEM_NW_SR;
																break;
													default:
																break;
											} 	// switch-case
										}		// if CREG
										// 3.93 sec time delay, before for probing a network coverage again.
										msDelay(2500);
									}while(i--);
									
									// Display Operator name on screen
									LcdClear();
									LcdPuts(&GSM.GSMOptrName[0]);
									msDelay(2500);
																			
									// Configure Modem  for SMS text Mode with SIM Memory, Clear all SMS and Save current configuration in Modem Flash memory
									if(GSM.GSMStatus.ModemStatus > GSM_STATE_NO_SIM) {
										// Switch Modem SMS mode to 'Text mode'
										Uart2Transfer("AT+CMGF=1\r", 0xC8, 2);
										// Switch to SIM Memory
										Uart2Transfer("AT+CPMS=\"SM\"\r", 0xC8, 2);
										// Delete all SMS in memory
										Uart2Transfer("AT+CMGDA=\"DEL ALL\"\r", 0xC8, 2);

										// GSM 7bit alphabet selection
										Uart2Transfer("AT+CSCS=\"GSM\"\r", 0xC8, 2);

										//Uart2Transfer("AT&W0\r", 0x1388, 2);
										// Get SSI
										GetGSM_SSI();
									}
									GSMntsUpdate();
									ModemRefreshRate = GSM_UPDATE_INTERVAL;
									break;
		case GSM_TURN_OFF:
				msDelay(1000);
				return (Uart2Transfer("AT+CPOWD=1\r", 0xC8, 2));							// Careful what you wish for
	}		// Switch case
	return 0;
}

/**
	\function
	\brief
	\param
	\return
**/
uint32_t Uart2Transfer (const void *TxData, uint32_t TimeOut, uint32_t NoTermChara)
{
	FlushUart2Buffer();
	Uart2TxString(TxData);

	SysTimeOut = TimeOut;
	// Now for data reception or until timeout
	while( SysTimeOut && !BufferFill && !(TermCharCnt == NoTermChara) )
		continue;

	if(!TermCharCnt && !SysTimeOut)
		return 1;

 return (ProbModemRspns());
}

uint32_t Uart2TransferChar (const char Ch, uint32_t TimeOut, uint32_t NoTermChara)
{
	FlushUart2Buffer();
	Uart2TxChar(Ch);

	SysTimeOut = TimeOut;
	// Now for data reception or until timeout
	while( SysTimeOut && !BufferFill && !(TermCharCnt == NoTermChara) )
		continue;

	if(!TermCharCnt && !SysTimeOut)
		return GSM_CMD_TIMEOUT;

	return (ProbModemRspns());
}

void FlushUart2Buffer(void)
{
	uint32_t i;
	RxBuffPtr = &RxBuffer[0];

	for ( i = 0; i <  UART2_BUF_SIZE; i++)
		*RxBuffPtr++ = '\0';

	RxBuffPtr = &RxBuffer[0];
	TermCharCnt = 0;
	BufferFill = 0;
}

/**
 *  \brief Analysis the response provided by the modem
 *  
 *  \return (0)GSM_ERROR_NONE			; Got OK from Modem	
 *  				(1)GSM_CME_ERROR			; ERROR received
 *  				(2)GSM_UNKNOWN_ERROR	; Any other error
 *  \details More details
 */
uint32_t ProbModemRspns(void)
{
	//reset pointer to beginning.
	if(strstr( (const char *)(RxBuffer), "OK"))
		return GSM_ERROR_NONE;
	else if(strstr( (const char *)(RxBuffer), "ERROR"))
		return GSM_CME_ERROR;
	
		return GSM_UNKNOWN_ERROR;
}

/**
 *  \brief Reads GSM Signal Strength index, and updates the same in 'GSM.GSMStatus.Rssi'
 *  
 *  \return Return description
 *  
 *  \details More details
 */
uint32_t GetGSM_SSI(void)
{
	char *Index;
	GSM.GSMStatus.Rssi = 0;																								// Received signal Strength Indication

	if(!Uart2Transfer("AT+CSQ\r", 0xC8, 4))	{															// +CSQ: 17,0
		Index  = strstr((char *)&RxBuffer[0], "+CSQ: ");
		if(Index) {
			Index += 6;
			StrCpy(&GSM.GSMStatus.SignalQ[0], Index, ',', 3);
			GSM.GSMStatus.SignalQ[2] = '\0';																					// Max value is 99
			GSM.GSMStatus.Rssi = Str2Num((uint8_t *)&GSM.GSMStatus.SignalQ[0]);				// String value to Num
			if(GSM.GSMStatus.Rssi > 100)																							// in case op is +CSQ: 7,0, we might over-flow
				GSM.GSMStatus.Rssi = 0;
		}
	}
	else
		return 1;
	
	return 0;
}

/**
 *  \brief Brief description
 *  
 *  \return None
 *  
 *  \details More details
 */
void Screen(void)
{
	static uint32_t LoopCnt;
	//uint32_t	AdConvResult;

	/** Read battery Status, P2.6 CHG-STAT1 and P2.7 CHG-STAT2
	+-----------+-----------+-------------------------------------------------------+
	|		STAT2		|		STAT1		|	Remark																								|
	+-----------+-----------+-------------------------------------------------------+
	|		OFF	(1)	|		ON	(0)	| Charging																							|
	|		ON	(0)	|		OFF	(1)	|	Done																									|
	|	 	OFF	(1)	|		OFF	(1)	|	Sleep, Over voltage, Charge suspend, Battery absent		|
	+-----------+-----------+-------------------------------------------------------+
	|
	**/
	// ADC_StartConversion();																		// Start AD CH0 conversion
	//SystemStatus.BatteryStatus = GPIO_PinRead(2, 7);					// Stat-2
	//SystemStatus.BatteryStatus <<= 1;
	//SystemStatus.BatteryStatus |= GPIO_PinRead(2, 6);					// Stat-1

	RetrieveSIMCOMtime();

	/*
	// True if conversion in progress
	while(ADC_ConversionDone())
		continue;
	AdConvResult = ADC_GetValue();
	*/
	if(PAC1710Regs.vSenseCh1 & 0x800) {
		if(!SystemStatus.BatCharging) {
			SystemStatus.BatCharging = 1;
			BkLiteLength = 50;
			BeeperTimeout = 300;
		}
	}
	else if(SystemStatus.BatCharging) {
			SystemStatus.BatCharging = 0;
			BkLiteLength = 50;
			BeeperTimeout = 300;
		}
	// LcdClear();
	LcdPutsXY(1, 3, "            ");
	LcdPutsXY(2, 6, " ");
	LcdGotoXY(1, 1);
	LcdPutch(0);												// Display n/w symbol on screen (or Marginal strength (-95dBm), blank display)
	if(GSM.GSMStatus.ModemStatus < GSM_STATE_MODEM_READY)
		LcdPuts("x ");
	
	/**
	-109 	< dBm < -95 (2 < CSQ <9)		: Marginal
	-93 	< dBm	< -85 (10 < CSQ <14)	: OK
	-83 	< dBm < -75 (15 < CSQ <19)	: Good
	-73		< dBm < -53 (20 < CSQ <30)	: Excellent
	**/
	if(GSM.GSMStatus.ModemStatus > GSM_STATE_NO_SIM) {
		if(GSM.GSMStatus.Rssi < 10)
				LcdPutch(4);								// Marginal strength (-95dBm), blank display
				else if(GSM.GSMStatus.Rssi < 14)
					LcdPutch(1);							// Okay, I can live with this
					else if(GSM.GSMStatus.Rssi < 20)				// -85 to -75 dBm or greater (Signal Quality Good)
							LcdPutch(2);					// Good
						else										// Above -75dBm
							LcdPutch(3);					// Excellent
	}
	if(SystemStatus.GPRSup)
			LcdPutch(0xAB);							// display special character from built in CG-ROM
	else
			LcdPutch(' ');

	LcdPutch(' ');
	LcdPuts(&GSM.GSMOptrName[0]);				// GSMOptrName contains Error Message like no SIM, Modem Error etc., for normal case the operator name might be large and could eat up display space
	//LcdPutNum(AdConvResult);

	// HH:MM
	LcdGotoXY(2, 1);
	Num2CharLcd(NetworkTime.Hour, DIGIT_JUSTIFY_2D);
	if(NetworkTime.Seconds % 2)
		LcdPutch(':');
	else
		LcdPutch(' ');
	Num2CharLcd(NetworkTime.Minute, DIGIT_JUSTIFY_2D);

	// DD/MM/YYYY
	LcdGotoXY(2, 7);
	Num2CharLcd(NetworkTime.Day, DIGIT_JUSTIFY_2D);
	LcdPutch('/');
	Num2CharLcd(NetworkTime.Month, DIGIT_JUSTIFY_2D);
	LcdPutch('/');
	Num2CharLcd(NetworkTime.Year, DIGIT_JUSTIFY_4D);
	
	// Battery Charging condition
	if(SystemStatus.BatCharging) {
		LoopCnt++;
		switch (LoopCnt) {
			case 1:
					LcdWrt2CGRAM(2, &BattLow[0]);
					break;
			case 2:
					LcdWrt2CGRAM(2, &BattMid[0]);
					break;
			case 3:
					LoopCnt = 0;
					LcdWrt2CGRAM(2, &BattFull[0]);
					break;
		}
	}
	// Not charging, so check if current is drawn from the same	
	else if(PAC1710Regs.vSenseCh1) {
		if(PAC1710Regs.vSourceCh1 > BAT_HIGH)				// Vbat > 12.7v
			LcdWrt2CGRAM(2, &BattFull[0]);
		else if(PAC1710Regs.vSourceCh1 > BAT_MID)		// Vbat > 11.72
			LcdWrt2CGRAM(2, &BattMid[0]);
		else
			LcdWrt2CGRAM(2, &BattLow[0]);
	}
	// Not Charging and no current is drawn from battery	
	else
		LcdWrt2CGRAM(2, &BattBad[0]);
	
	LcdGotoXY(1, 15);
	LcdPutch(SystemStatus.SystemMode | 0x30);
	LcdGotoXY(1, 16);
	LcdPutch(2);												// Display Battery symbol
}

/**
 *  \brief NTP to sync time
 *  
 *  \return None
 *  
 *  \details More details
 */
void GSMntsUpdate(void) 
{
	if(InitWWW()) {
		LcdClear();
		LcdPuts("NTS fail");
	}
	LcdClear();
	
	LcdPuts("NTS Sync");
	LcdGotoXY(2,1);
	// Set NTP use bearer profile 1
	Uart2Transfer("AT+CNTPCID=1\r", 0xC8, 2);
	// 216.239.35.0, time.google.com
	Uart2Transfer("AT+CNTP=\"216.239.35.0\",22\r", 0x1388, 2);
	if(Uart2Transfer("AT+CNTP\r", 0x1388, 4))
		LcdPuts("Fail");
	else
		LcdPuts("Ok");
	msDelay(2000);	
}

char *RetrieveSIMCOMtime(void)
{
	char *Index, *DateTimeStringPtr;
	static char DateTimeString[17];												//yymmddhhmm
	
	DateTimeStringPtr = &DateTimeString[0];		
	
	if(GSM.GSMStatus.ModemStatus < GSM_STATE_MODEM_NW_SR)
		return NULL;

	// yy/mm/dd,hh:mm:ss±zz
	Uart2Transfer("AT+CCLK?\r", 0xC8, 2);
	Index = ParseBuff((char *)&RxBuffer[0], '+');

	if (Index) {
		if (StrCmp(Index, "CCLK:", 5)) {
			Index += 7;
			DateTimeString[0] = *Index++;				//1
			DateTimeString[1] = *Index++;				//7
			NetworkTime.Year = Str2Num((uint8_t *)(Index - 2));
			NetworkTime.Year += 2000;						// Yeah But why
			Index++;														///
			DateTimeString[2] = *Index++;				//0
			DateTimeString[3] = *Index++;				//5
			NetworkTime.Month = Str2Num((uint8_t *)(Index - 2));
			Index++;														///
			DateTimeString[4] = *Index++;				//0
			DateTimeString[5] = *Index++;				//2
			NetworkTime.Day = Str2Num((uint8_t *)(Index - 2));
			Index++;														//,
			DateTimeString[6] = *Index++;				//2
			DateTimeString[7] = *Index++;				//2
			NetworkTime.Hour = Str2Num((uint8_t *)(Index - 2));
			Index++;														//:
			DateTimeString[8] = *Index++;				//1
			DateTimeString[9] = *Index++;				//0
			NetworkTime.Minute = Str2Num((uint8_t *)(Index - 2));
			Index++;
			DateTimeString[10] = *Index++;			//2
			DateTimeString[11] = *Index++;			//2
			NetworkTime.Seconds = Str2Num((uint8_t *)(Index - 2));
			//Index++;
			DateTimeString[11] = '\0';					// Append NULL to string end for proper termination
			
			return DateTimeStringPtr;
		}
	}
	return NULL;
}

/**
 *  \brief Brief description
 *  
 *  \param [in] CstrNoPtr Number to which the SMS is sent 
 *  \param [in] MsgBody Message body, the content
 *  \param [in] ExtraInfo Description for ExtraInfo
 *  \return [0] on successful texting, else 1
 *  
 *  \details More details
 */
uint32_t StatusReport(char *CstrNoPtr, char *MsgBody, uint32_t ExtraInfo)
{
	char *PtrIndex, *PtrIndex_2;
	uint32_t Vsense;

	LcdClear ();
	LcdPuts(MsgBody);
	LcdPutsXY(2, 1, "Texting:");

	if(GSM.GSMStatus.ModemStatus < GSM_STATE_NO_NETWORK)
		return 2;

	if (!VerifyPhoneNo(&CstrNoPtr[0]))
		return 1;

	msDelay(1000);
	// Now compose message, MODEM is already in text mode
	Uart2TxString("AT+CMGS=\"");									// send message, AT+CMGS="number"<C.R>
	Uart2TxString(CstrNoPtr);											// cell, number
	Uart2Transfer("\"\r", 0x3E8, 3);

	// Message Body
	Uart2TxString(MsgBody);
	Uart2TxString("\r\n");

	switch (ExtraInfo) {
		case DEBUG_TXT_ON:
			// Debug Text On
			Uart2TxString(GSM.GSMStatus.MEModel);					// Model#
			Uart2TxString("\r\n");
			Uart2TxString(&GSM.MEID[0]);									// IMEI
			Uart2TxString("\r\n");
			Uart2TxString(&GSM.GSMOptrName[0]);						// Operator Name
			Uart2TxString("\r\nrssi @");
			Uart2TxString(&GSM.GSMStatus.SignalQ[0]);
			Uart2TxString("\r\nFW: ");
			Uart2TxString(VERSION_INFO);
		// Print current mode on SMS
			Uart2TxString("\r\nMode:");
			switch (SystemStatus.SystemMode) {
				case SENSOR_ALIGN:
					Uart2TxString("Sensor Align");
					break;
				case SMS_MODE:
					Uart2TxString("SMS");
					break;
				case CAM_MODE:
					Uart2TxString("CAM");
					break;
				case LIVE_MODE:
					Uart2TxString("L!ve");
			}
			Uart2TxString("\r\nBattery:\r\n");
			Uart2TxString((const char *)Num2Str((uint32_t) ( (float)PAC1710Regs.vSourceCh1 * 19.5)));			// 40/(2^11)
			Uart2TxString("mV ");
			// Now punch in the measured Current (I)
			if(PAC1710Regs.vSenseCh1 & 0x800){								// oh oh -ve values
				Vsense = PAC1710Regs.vSenseCh1 - 1;							// 2s'
				Vsense = (~Vsense) & 0x7FF ;										// Mask off extra bits as our o/p range is 14bit
				Uart2TxChar('-');
			}
			else
				Vsense = PAC1710Regs.vSenseCh1;

			Uart2TxString((const char *)Num2Str((uint32_t)( (float)Vsense * 1.45) ));
			Uart2TxString("mA");
			if(SystemStatus.BatCharging)
				Uart2TxString("\r\nCharging");
			else
				Uart2TxString("\r\nNot Charging");
			Uart2TxString("\r\nIP:");
			Uart2TxString(&GSM.GSMStatus.IPaddress[0]);				// Current IP address
			break;
		case DUMP_CONFIG:
			Uart2TxString("APN:");
			Uart2TxString(APN);
			Uart2TxString("\r\nFTP\r\n");
			Uart2TxString(FtpServerName);
			Uart2TxChar(':');
			Uart2TxString(FtpPort);
			Uart2TxString("\r\n");
			Uart2TxString(FtpUserName);
			Uart2TxString("\r\n");
			Uart2TxString(FtpUserPassword);
			break;
		default:
			break;	
	}
	Uart2TxString("\r\n");
	Uart2TxString((char *)Num2Str(NetworkTime.Day));
	Uart2TxChar('/');
	Uart2TxString((char *)Num2Str(NetworkTime.Month));
	Uart2TxChar('/');
	Uart2TxString((char *)Num2Str(NetworkTime.Year));
	Uart2TxString(" @ ");
	Uart2TxString((char *)Num2Str(NetworkTime.Hour));
	Uart2TxChar(':');
	Uart2TxString((char *)Num2Str(NetworkTime.Minute));
	Uart2TxString("\r\n");
	
	msDelay(1000);
	Uart2TransferChar(0x1A, 0x1388, 4);														// send ^Z ....Message away...

	PtrIndex = ParseBuff((char *)&RxBuffer[0], '+');							// Parse up buffer for " symbol
	if(StrCmp(PtrIndex, "CMGS:", 5)) {
		PtrIndex_2 = ParseBuff(PtrIndex, '\r');
		*(PtrIndex_2 - 1) = '\0';
		PtrIndex += 5;
		LcdPuts(PtrIndex);
		BeeperTimeout = 250;
		msDelay(500);
		return 0;										// Success; Message sent
	}
	Uart2Transfer("ATE0\r", 0xC8, 2);

	return 1;											//  Getting here is no good
}
/**
 *  \brief Brief description
 *  
 *  \return Return description
 *  
 *  \details More details
 */
uint32_t InitWWW(void)
{
	uint32_t i, RetryCount  = 5, OctectCount = 0;

	char *Index, *IPptr;

	SystemStatus.GPRSup = 0;
	IPptr = &GSM.GSMStatus.IPaddress[0];
	Index = (char *)&RxBuffer[0];

	if(GSM.GSMStatus.ModemStatus < GSM_STATE_NO_NETWORK)
			return 2;

	LcdClear();
	LcdPuts("GPRS.");
	// Check if gprs context is already activated
	Uart2Transfer("AT+SAPBR=2,1\r", 0x1388, 4);
	/* /r/nOK/r/n/r/n+SAPBR 1:DEACT/r/n
		+SAPBR: 1,3,"0.0.0.0"
	*/
	if(strstr( (const char *) &RxBuffer[0], "+SAPBR: 1,3") ) {									// context deactivated, then re-activate 		
		Uart2Transfer("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r", 0xC8, 2);
		Uart2TxString("AT+SAPBR=3,1,\"APN\",");
		Uart2TxString(APN);
		Uart2Transfer("\r",0xC8, 2);
		Uart2Transfer("AT+SAPBR=1,1\r", 0xC8, 2);																	// Activate context 1
	}
	do {
		// Query connection status, read your IP address
		LcdPutch('.');
		Uart2Transfer("AT+SAPBR=2,1\r", 0x1388, 4);															// +SAPBR: 1,1,"10.89.193.1" e.g.
		if(strstr((const char *)&RxBuffer[0], "+SAPBR: 1,1" )) {								// +SAPBR: 1,1,"10.89.193.1"
			Index = ParseBuff((char *)&RxBuffer[0], '"');
			if(Index) {		// get the IP length and copy the same to the variable
				i = 0;
				OctectCount = 0;
				while(*Index != '"') {
					*IPptr++ = *Index;
					if(*Index++ == '.')
						OctectCount++;
					if(i++ > 15)
						break;
				}
			}
		}
		msDelay(750);
		RetryCount --;
	} while(OctectCount != 3 && RetryCount);

	*IPptr = '\0';

	if(OctectCount != 3)
		return 1;

	SystemStatus.GPRSup = 1;
	LcdPutsXY(2,1, GSM.GSMStatus.IPaddress);			// display IP address on 2nd line
	msDelay(1000);

	return 0;
}
/**
 *  \brief Brief description
 *  
 *  \return Return description
 *  
 *  \details More details
 */
uint32_t HttpRetrieveRecipient(void)
{
	char *Index;
	uint32_t Row, RecipientCount = 0;

	Index = (char *)&RxBuffer[0];

	if(InitWWW()) {
		LcdPuts ("fail");
		return 0;
	}

	LcdClear();
	LcdPuts("SMS GET:");

	Uart2Transfer("AT+HTTPINIT\r", 0xC8, 2);
	Uart2Transfer("AT+HTTPPARA=\"CID\",1\r", 0xC8, 2);
	Uart2TxString("AT+HTTPPARA=\"URL\",\"");
	Uart2TxString(SMS_RCP_URL);
	Uart2TxString("imei=");
	//Uart2TxChar('1');
	Uart2TxString(GSM.MEID);
	Uart2Transfer("\"\r", 0xC8, 2);
	Uart2Transfer("AT+HTTPACTION=0\r", 0x1388, 5);
	/*
		\r\nOK\r\n
		\r\n+HTTPACTION: 0,200,xxxx\r\n
	*/
	if(strstr((const char *)&RxBuffer[0], "+HTTPACTION: 0,200")) {
		LcdPuts("OK");
		msDelay(500);
		Uart2Transfer("AT+HTTPREAD\r", 0x1388, 4);
		/*
			\r\nOK\r\n
			\r\n+HTTPREAD: xxxx\r\n
		*/
		// {"6":["9847540368","8848583393","9995324154","9037261532","9048775044",8954110738]}
		Index = ParseBuff((char *)&RxBuffer[0], '{');
		if (*Index == '"') {
			Index++;
			RecipientCount = Str2Num((uint8_t *)Index);
			if (RecipientCount) {
				Index = ParseBuff((char *)&RxBuffer[0], '[');
				if (*Index++ == '"') {
					for (Row = 0; Row < RecipientCount; Row++) {
						RecipientNo[Row][0] = '+';
						RecipientNo[Row][1] = '9';
						RecipientNo[Row][2] = '1';
						StrCpy(&RecipientNo[Row][3], Index, '"', 10);
						Index += 13;
						if (*(Index - 1) != '"')
							break;
					}
				}
			}
		}
	}
	Uart2Transfer("AT+HTTPTERM\r", 0x1F4, 4);

	return RecipientCount;
}
/**
 *  \brief Brief description
 *  
 *  \return Return description
 *  
 *  \details More details
 */
uint32_t HttpGetStatusReport(void)
{
	char *Index;
	char *numStrPtr;
	uint32_t RespLength = 0;
	
	Index = (char *)&RxBuffer[0];
		
	if(InitWWW()) {
			LcdPuts ("fail");
			return 0;
	}
	LcdClear();
	LcdPuts("GET Status:");
	Uart2Transfer("AT+HTTPINIT\r", 0xC8, 2);
	Uart2Transfer("AT+HTTPPARA=\"CID\",1\r", 0xC8, 2);
	Uart2TxString("AT+HTTPPARA=\"URL\",\"");
	Uart2TxString(GET_STATUS_URL);
	Uart2TxString("vsrc=");
	//Uart2TxString("1");
	numStrPtr = (char *)Num2Str(PAC1710Regs.vSourceCh1);
	Uart2TxString(numStrPtr);
	Uart2TxString("&vsns=");
	//Uart2TxString("2");
	numStrPtr = (char *) Num2Str(PAC1710Regs.vSenseCh1);
	Uart2TxString(numStrPtr);
	Uart2TxString("&pord=");
	//Uart2TxString("3");
	numStrPtr = (char *)Num2Str(PAC1710Regs.PowerRatioCh1);
	Uart2TxString(numStrPtr);
	Uart2TxString("&cam_stat=");
	if(SystemStatus.CameraOk)
		Uart2TxChar('1');
	else
		Uart2TxChar('0');
	Uart2TxString("&powr_sns_stat=");
	if(SystemStatus.CurrentSensorOK)
		Uart2TxChar('1');
	else
		Uart2TxChar('0');
	Uart2TxString("&imei=");
	Uart2TxString(GSM.MEID);
	
	Uart2Transfer("\"\r", 0xC8, 2);
	Uart2Transfer("AT+HTTPACTION=0\r", 0x1388, 5);
	/*
		\r\nOK\r\n
		\r\n+HTTPACTION:0:200,xxxx\r\n
	*/
	if(strstr((const char *)&RxBuffer[0], "+HTTPACTION: 0,200")) {
		LcdPuts("OK");
		Uart2Transfer("AT+HTTPREAD\r", 0x1388, 4);
		/*
			\r\nOK\r\n
			\r\n+HTTPREAD: xxxx\r\n
		*/
		// {"data":{"Message":"success","image_id":307}}
		Index = ParseBuff((char *)&RxBuffer[0], 'u');
			if( StrCmp(Index, "ccess", 5) ) {
				Index = ParseBuff(Index, ':');
				if(Index) {
					BeeperTimeout = 500;
				}
			}
	}

	Uart2Transfer("AT+HTTPTERM\r", 0x1F4, 4);

	return RespLength;
	
}
/**
 *  \brief Brief description
 *  
 *  \param [in] Message Description for Message
 *  \param [in] UniqueResp Description for UniqueResp
 *  \return Return description
 *  
 *  \details More details
 */
uint32_t HttpGetIntrusion(char *Message, char *UniqueResp)
{
	char *Index;
	uint32_t RespLength = 0;

	Index = (char *)&RxBuffer[0];

	msDelay(5000);

	if(InitWWW()) {
		LcdPuts ("Fail");
		return 0;
	}

	LcdClear();
	LcdPuts("GET Int:");

	Uart2Transfer("AT+HTTPINIT\r", 0xC8, 2);
	Uart2Transfer("AT+HTTPPARA=\"CID\",1\r", 0xC8, 2);
	Uart2TxString("AT+HTTPPARA=\"URL\",\"");
	Uart2TxString(GET_INTRUSION_URL);
	Uart2TxString("imei=");
	//Uart2TxChar('1');
	Uart2TxString(GSM.MEID);
	Uart2TxString("&messge=");															// spelling mistake in server side programming
	while(*Message) {
		if(*Message == ' ')
			Uart2TxString("%20");
		else
			Uart2TxChar(*Message);
		Message++;
	}
	Uart2TxString("&range=");
	Uart2TxString("trivandrum&voltage=");
	Index = (char *)Num2Str(PAC1710Regs.vSourceCh1);
	Uart2TxString(Index);
	Uart2Transfer("\"\r", 0xC8, 2);
	Uart2Transfer("AT+HTTPACTION=0\r", 0x1388, 5);
	/*
		\r\nOK\r\n
		\r\n+HTTPACTION:0,200,xxxx\r\n
	*/
	if(strstr((const char *)&RxBuffer[0], "+HTTPACTION: 0,200")) {
		Uart2Transfer("AT+HTTPREAD\r", 0x1388, 4);
		/*
			\r\nOK\r\n
			\r\n+HTTPREAD: xxxx\r\n
		*/
		// {"data":{"Message":"success","image_id":307}}
		Index = ParseBuff((char *)&RxBuffer[0], 'u');
			if( StrCmp(Index, "ccess", 5) ) {
				Index = ParseBuff(Index, ':');
				if(Index) {
					StrCpy(UniqueResp, Index, '}', 10);
					LcdPuts(UniqueResp);
					BeeperTimeout = 500;
				}
				else
					LcdPuts("NaN");
			}
	}
	else
		LcdPuts("Fail");
		
	Uart2Transfer("AT+HTTPTERM\r", 0x1F4, 4);
	msDelay(500);
	
	return RespLength;
}
/**
 *  \brief Brief description
 *  
 *  \param [in] FlashAddr Description for FlashAddr
 *  \param [in] ImgSize Description for ImgSize
 *  \param [in] ImgUniqName Description for ImgUniqName
 *  \return Return description
 *  
 *  \details More details
 */
uint32_t FtpUploadFlash(uint32_t FlashAddr, uint32_t ImgSize, char *ImgUniqName)
{
	uint32_t i = 0;
	uint8_t *Index;
	char FtpTransferLim[6];								// Possible values are 1360, i.e 5 digits
	char DispProgressVal[7];
	uint32_t FtpXerCount = 0;
	uint32_t OffSet = ImgSize;
	
	Index = &RxBuffer[0];
	FlashAddr++;													// Skip first value as its 0x00

	if(InitWWW()) {
		LcdPuts ("Fail");
		return 0;
	}

	LcdClear();
	LcdPuts("FTP");

	Uart2Transfer ("AT+FTPQUIT\r", 0x1388, 4);

	Uart2Transfer ("AT+FTPCID=1\r", 0xC8, 2);
	// FTP Server
	Uart2TxString ("AT+FTPSERV=");
	Uart2TxString (FtpServerName);
	Uart2Transfer ("\r", 0x1F4, 2);

	// FTP Credentials
	Uart2TxString ("AT+FTPUN=");												// Set FTP User Name
	Uart2TxString (FtpUserName);
	Uart2Transfer ("\r", 0x1F4, 2);
	Uart2TxString ("AT+FTPPW=");												// Set FTP Password
	Uart2TxString (FtpUserPassword);
	Uart2Transfer ("\r", 0x1F4, 2);

	// File Upload
	// Create a file first with .jpg extension
	if(!strstr(ImgUniqName, ".jpg"))
		StrCat(ImgUniqName, ".jpg");
	Uart2TxString("AT+FTPPUTNAME=\"");									// Set Upload File Name
	Uart2TxString(ImgUniqName);
	Uart2Transfer("\"\r", 0x1F4, 2);
	
	//Uart2Transfer ("AT+FTPPUTOPT=\"APPE\"\r", 0x1F4, 2);													// Append mode please
	Uart2Transfer ("AT+FTPPUTPATH=\"/\"\r", 0x1F4, 2);														// Root
	msDelay(1000);
	Uart2Transfer ("AT+FTPPUT=1\r", 0x9C40, 4);																		// Put Session
	/* \r\nOK\r\n\r\n+FTPPUT:1,1,1280\r\n, returns the maximum transfer limit
		Max Size possible 1280!, depends on network!!! */
	Index = (uint8_t *)ParseBuff((char *)&RxBuffer[0], '+');
	if (Index) {
		Index = (uint8_t *)ParseBuff((char *)&RxBuffer[0], ',');
		if (Index) {
			if (StrCmp((char *)Index, "1,", 2)) {
				Index += 2;
				FtpXerCount = Str2Num(Index);																						// we will have the value 1280 in integer
			}
		}
	}
	// Check for session errors, if so abort abort
	if(!FtpXerCount || (FtpXerCount > FTP_XFER_MAX) )		{													// > 1500
		Uart2Transfer ("AT+FTPQUIT\r", 0x1388, 4);	
		return 1;																																		
	}

	LcdPutsXY(1, 6, "Uploading");
	FtpXerCount -= 2;
	sprintf(&FtpTransferLim[0], "%d", (FtpXerCount + 1));
	// FtpTransferLim = Num2Str(FtpXerCount + 1);
	/* We have the max image size, we have the ftp limits, now if the image size is greater than ftp limit
			we split and upload	*/
	if(FtpXerCount) {																																// if not true, Alarm, alarm we don't have the ftp limit
		if(OffSet < FtpXerCount) {
					Uart2TxString("AT+FTPPUT=2,");																					// No need to divide and upload
					sprintf(&FtpTransferLim[0], "%d", OffSet);															// convert the current image size to string
										
					Uart2TxString((const char *)FtpTransferLim);
					Uart2Transfer("\r", 0x1F40, 2);																					// Size send OK
					// \r\n+FTPPUT:2,OffSet\r\n ......
					if( strstr((const char *)&RxBuffer[0], "+FTPPUT: 2") ) {								// Check if modem to ready to accept data
						FlashDataRead(FlashAddr, &FlashBuffer[0], OffSet);										// Offset is ImageLength, hence read the same length of data from memory
						Uart2Send((const char *)&FlashBuffer[0], OffSet);											// Send in the cavalry
						Uart2TransferChar(FlashBuffer[OffSet], 0x2710, 4);
					}	// and \r\nOK\r\n when data received
		}
		else {
			do {
				if (i + FtpXerCount > ImgSize)
					break;
				Uart2TxString("AT+FTPPUT=2,");																		
				Uart2TxString((const char *)FtpTransferLim);																// Client request to send <FtpTransferLim> bytes of data
				Uart2Transfer("\r", 0x1F40, 2);
				if( strstr((const char *)&RxBuffer[0], "+FTPPUT: 2") ) {
					FlashDataRead(FlashAddr, &FlashBuffer[0], FtpXerCount + 1);								// read FtpXerCount + 1 Length of image from flash
					Uart2Send((const char *)&FlashBuffer[0], FtpXerCount);										// Send FtpXerCount to FTP Put, except last byte
					Uart2TransferChar(FlashBuffer[FtpXerCount], 0x2710, 4);										// Send last byte
					i += FtpXerCount + 1;
					OffSet -= FtpXerCount;
					FlashAddr += FtpXerCount + 1;
					// Display progress on LCD
					sprintf(&DispProgressVal[0], "%d", i);
					LcdPutsXY(2, 4, (const char * )DispProgressVal);
					LcdPutch('/');
					sprintf(&DispProgressVal[0], "%d", ImgSize);
					LcdPuts((const char * )DispProgressVal);
				}
				else if( strstr((const char *)&RxBuffer[0], "ERROR") ) {
					return 1;
				}
			}while(OffSet > FtpXerCount);
			if (OffSet) {
				// i to ImgSize
				Uart2TxString("AT+FTPPUT=2,");
				sprintf(&FtpTransferLim[0], "%d", (ImgSize - i));
				Uart2TxString((const char *)FtpTransferLim);
				Uart2Transfer("\r", 0x1388, 2);
				if( strstr((const char *)&RxBuffer[0], "+FTPPUT: 2") ) {
					FlashDataRead(FlashAddr, &FlashBuffer[0], (ImgSize - i));
					Uart2Send((const char *)&FlashBuffer[0], (ImgSize - i));	
					Uart2TransferChar(FlashBuffer[ImgSize - i + 1], 0x1F40, 4);
					// Display last progress on LCD
					sprintf(&DispProgressVal[0], "%d", ImgSize);
					LcdPutsXY(2, 4, (const char * )DispProgressVal);
					LcdPutch('/');
					LcdPuts((const char * )DispProgressVal);
				}
				else if( strstr((const char *)&RxBuffer[0], "ERROR") ) {
					return 1;
				}
			}
		}
	}
	// No more data to upload, session will be closed
	Uart2Transfer ("AT+FTPPUT=2,0\r", 0x1388, 4);
	return 0;
}

uint32_t FtpPutReady(uint8_t *BufferPtr)
{
	uint8_t *Index = (uint8_t *)ParseBuff((char *)BufferPtr, '+');
	if(Index)
		if(StrCmp( (char *)Index, "FTPPUT:2", 8) )														// Check if modem is ready to accept the incoming data
			return 1;
	return 0;
}

/****************************************************************************************************************
* \Function		:	uint32_t ChkNuMsg(void)																																					*
*																																																								*
* \Overview		:	Check if any new texts has been received by the modem																						*
* 						(i.e MODEM or GPS in this case), and waits until a 'Termination Character' (nTerm) 								*
*						or until Wait time is expired or which ever comes first.																						*
*																																																								*
* \Parameters	:   none																																													*
*																																																								*
* \Returns		:  	0 ; if there's no new messages																																*
*									x ; a non zero values if there's a new message where this non zero value is the msg index			*
* Side Effects	:	So far none																																										*
*****************************************************************************************************************/
uint32_t ChkNuMsg(void)
{
	uint8_t *PtrIndex;
	uint32_t MsgIndx = 0;

	if(GSM.GSMStatus.ModemStatus < GSM_STATE_NO_NETWORK)
			return 2;

	Uart2Transfer("AT+CMGL=\"ALL\"\r",0x1388, 2);
	/*******************	Modem response would be as follows...	*********
	*	+CMGL: 2,"REC UNREAD","+91xxxxxxxxxx","","17/02/10,11:15:27+22"		*
	*	\r\n																															*
	*	Blah blah																													*
	*	\r\n																															*
	*	\r\n																															*
	*	OK																																*
	*	\r\n																															*
	*********************************************************************/
	PtrIndex = (uint8_t *)ParseBuff((char *)&RxBuffer[0], '+');					// Parse up buffer for " symbol

	if(StrCmp((char *)PtrIndex, "CMGL: ", 6)) {
		MsgIndx = *(PtrIndex + 6) - 0x30;								// i.e. +CMGL: x
		if(*(PtrIndex + 7) != ',') {
			MsgIndx = 0;
			Uart2Transfer("AT+CMGDA=\"DEL ALL\"\r",0xC8, 2);
		 }
	}
	return (MsgIndx);
}

/**
	\function: ParseSMS(uint32_t MsgIndx)
	\brief
	\param
	\return	GSM_ERROR_NONE on success 
**/
/**
\function
\param

**/
uint32_t ParseSMS(uint32_t MsgIndx)
{
	uint32_t c, SystemConfigUpdateFlag = 0;
	char *Index;
	char Command[6];
	char SenderNo[PHONE_NO_LEN + 1];
	char NewCstrNo[PHONE_NO_LEN + 1];
	char NewRemDevNo_1[PHONE_NO_LEN + 1];
	char NewRemDevNo_2[PHONE_NO_LEN + 1];
	char NewFtpServerName[FTP_SRVR_NAME_LEN];									
	char NewFtpUserName[FTP_USER_NAME_LEN];
	char NewFtpUserPassword[FTP_PSWD_LEN];
	char UssCode[10];
	char *CmdPtr = &Command[0];

	msDelay(2500);																			// Previously 5sec
	Uart2TxString("AT+CMGR=");											
	Uart2TxChar(MsgIndx | 0x30);												// Convert to ASCII
	Uart2Transfer("\r", 0x1388, 5);											// read-index = 0
	 // o/p e.g +CMGR: "REC UNREAD","+91xxxxxxxxxx",,"12/09/02,14:14:21+22"\n\r#SET
	 // o/p e.g +CMGR: "REC UNREAD","+91xxxxxxxxxx",,"12/09/02,14:14:21+22"\n\r#REPO

	Index = ParseBuff((char *)&RxBuffer[0], '+');				// Parse up buffer for + symbol
	if(*Index == 'C') {																	// We have message read OK +CMGR, next we check for it's validity
		Index = ParseBuff(Index, '+');										// look again for +91 etc.
		if(*Index == '9') {																// if so get the number
			--Index;																				// go back 2 characters, we need to get from +9
			for(c = 0; c < PHONE_NO_LEN; c++)								// copies the 14 length cell phone number, Including '+'
				SenderNo[c] = *Index++;
			SenderNo[PHONE_NO_LEN] = '\0';									// append NULL
		}
		Index = ParseBuff(Index, '#');										// Now parse up the buffer for # symbol
		if(*Index != 0x0D)
			for(c = 0; c < 4; c++)
				*CmdPtr++ = *Index++;													// Copies 6 character length <OTA command>
		else {
			Index = ParseBuff((char *)&RxBuffer[0], '*');
			if(Index) {
				Index--;
				for(c = 0; c < 10; c++) {
					UssCode[c] = *Index;
					if(*Index++ == '#') {
						UssCode[c + 1] = '\0';										// Append NULL termination
						break;
					}
				}
				if(c > 8)
					return 2;
				Uart2Transfer("AT+CUSD=1\r", 0xC8, 2);
				Uart2TxString("AT+CUSD=1,\"");
				Uart2TxString(&UssCode[0]);
				Uart2Transfer("\"\r", 0x1388, 5);
				Index = ParseBuff((char *)&RxBuffer[0], '+');				// Parse up buffer for + symbol
				if(StrCmp((char *)Index, "CUSD: ", 4)) {
					char UssdResult[150];
					Index += 9;
					for(c = 0; c < 150; c++) {
						UssdResult[c] = *Index;
						if(*Index++ == '\r')
							break;
					}
					StatusReport(&SenderNo[0], &UssdResult[0], DEBUG_TXT_OFF);
				}
				Uart2Transfer("AT+CUSD=0\r", 0xC8, 2);
			}
		}
	}			// if(+CMR) conditions

	// The Kill Command
	if( StrCmp(&Command[0], "KILL", 4) ) {
		Uart2TxString("AT+CMGD=");
		Uart2TxChar(MsgIndx | 0x30);																		// Convert to ASCII
		Uart2Transfer("\r\n", 0x1388, 3);
		if( StrCmp(&SenderNo[0], &CSTR_NUM[0], PHONE_NO_LEN	) ) {
			StatusReport(&CSTR_NUM[0], "Kill Initiated", DEBUG_TXT_ON);
			msDelay(2000);
			PowerSIM908(GSM_TURN_OFF);
			msDelay(2000);
			NVIC_SystemReset();
		}
	}
	// Report Command
	else if( StrCmp(&Command[0], "REPO", 4) )
		StatusReport(&SenderNo[0], "DEBUG", DEBUG_TXT_ON);
	// Set Master Number Command
	else if( StrCmp(&Command[0], "SMN", 3) ) {
		Index = ParseBuff((char *)&RxBuffer[0], '$');										// Parse up buffer for $ symbol
		if(Index){
			for(c = 0; c < PHONE_NO_LEN; c++)
				NewCstrNo[c] = *Index++;
			NewCstrNo[c] = '\0';
		}
		if(VerifyPhoneNo(&NewCstrNo[0])) {															// Check the validity of number
			SystemStatus.RoninMode = 0;
			StrCpy(&CSTR_NUM[0], &NewCstrNo[0], '\0', PHONE_NO_LEN);			// Copy new number present variable
			CSTR_NUM[PHONE_NO_LEN] = '\0';
			SystemConfigUpdateFlag = 1;																		// Update new Config to FLASH
			// Write location 1 thru 15, 0th location is reserved for CheckSum
			StatusReport(&CSTR_NUM[0], "Master Cell Phone# updated", DEBUG_TXT_OFF);
		}
		else
			StatusReport(&CSTR_NUM[0], "Master Cell Phone# Update failed", DEBUG_TXT_OFF);
	}
	// Set Remote Device Number -1
	else if( StrCmp(&Command[0], "SR1", 3) ) {
		Index = ParseBuff((char *)&RxBuffer[0], '$');										// Parse up buffer for $ symbol
		if(Index){
			for(c = 0; c < PHONE_NO_LEN; c++)
				NewRemDevNo_1[c] = *Index++;
			NewRemDevNo_1[c] = '\0';
			if(VerifyPhoneNo(&NewRemDevNo_1[0])) {												// Check the validity of number
				StrCpy(&REM_DEV_NUM_1[0], &NewRemDevNo_1[0], '\0', PHONE_NO_LEN);
				SystemConfigUpdateFlag = 1;																			// Update new Config to FLASH
				StatusReport(&CSTR_NUM[0], "Remote Phone#1 updated", DEBUG_TXT_OFF);
			}
			else
				StatusReport(&CSTR_NUM[0], "Remote Phone#1 Update failed", DEBUG_TXT_OFF);
		}
	}
	// Set Remote Device Number -2
	else if( StrCmp(&Command[0], "SR2", 3) ) {
		Index = ParseBuff((char *)&RxBuffer[0], '$');								// Parse up buffer for $ symbol
		if(Index){
			for(c = 0; c < PHONE_NO_LEN; c++)
				NewRemDevNo_2[c] = *Index++;
			NewRemDevNo_2[c] = '\0';
			if(VerifyPhoneNo(&NewRemDevNo_2[0])) {												// Check the validity of number
				StrCpy(&REM_DEV_NUM_2[0], &NewRemDevNo_2[0], '\0', PHONE_NO_LEN);
				SystemConfigUpdateFlag = 1;																				// Update new Config to FLASH
				StatusReport(&CSTR_NUM[0], "Remote Phone#2 updated", DEBUG_TXT_OFF);
			}
			else
				StatusReport(&CSTR_NUM[0], "Remote Phone#2 Update failed", DEBUG_TXT_OFF);
		}
	}
	// Mode Change request
	else if( StrCmp(&Command[0], "MOD", 3) ) {
		c = Command[3] - 0x30;								// Convert ASCII to Num
		if( c < 4 ) {
			SystemStatus.SystemMode = c;
			SystemConfigUpdateFlag = 1;																			// Update new Config to FLASH
			StatusReport(&CSTR_NUM[0], "Mod Change Success", DEBUG_TXT_OFF);
		}
		else
			StatusReport(&CSTR_NUM[0], "Mod Change Failed!", DEBUG_TXT_OFF);
	}
	// Access Point change request 
	else if( StrCmp(&Command[0], "APN", 3) ) {
		Index = ParseBuff((char *)&RxBuffer[0], '$');										// Parse up buffer for $ symbol
		if(Index){
			for(c = 0; (c < APN_LEN - 1) && (*Index != '$')&& (*Index != '\r'); c++)
				APN[c] = *Index++;
			APN[c] = '\0';
			if(*Index == '$') {
				SystemConfigUpdateFlag = 1;
				StatusReport(&CSTR_NUM[0], "APN updated", DEBUG_TXT_OFF);
			}
			else
				StatusReport(&CSTR_NUM[0], "APN update failed", DEBUG_TXT_OFF);
		}
	}	
	// New FTP Credentials
	else if(StrCmp(&Command[0], "FTP", 3)) {
	Index = ParseBuff((char *)&RxBuffer[0], '$');										// Parse up buffer for $ symbol
		if (Index) {
			for (c = 0; (c < FTP_SRVR_NAME_LEN - 1) && (*Index != '$') && (*Index != '\r'); c++)
				NewFtpServerName[c] = *Index++;
			NewFtpServerName[c] = '\0';
			if (*Index++ == '$') {
				for (c = 0; (c < FTP_USER_NAME_LEN - 1) && (*Index != '$') && (*Index != '\r'); c++)
					NewFtpUserName[c] = *Index++;
				NewFtpUserName[c] = '\0';
				if (*Index++ == '$') {
					for (c = 0; (c < FTP_PSWD_LEN -1) && (*Index != '$') && (*Index != '\r'); c++)
						NewFtpUserPassword[c] = *Index++;
					NewFtpUserPassword[c] = '\0';
					if (*Index == '$') {
						SystemConfigUpdateFlag = 1;
						strcpy(FtpServerName, NewFtpServerName);
						strcpy(FtpUserName, NewFtpUserName);
						strcpy(FtpUserPassword, NewFtpUserPassword);
						StatusReport(&CSTR_NUM[0], "FTP Credentials updated", DEBUG_TXT_OFF);
					}
					else
						StatusReport(&CSTR_NUM[0], "FTP Credentials failed", DEBUG_TXT_OFF);
				}
			}
		}
	}
	else if(StrCmp(&Command[0], "CFG", 3) )
		StatusReport(&CSTR_NUM[0], "Current Config", DUMP_CONFIG);
	// Image Upload Trigger
	else if( StrCmp(&Command[0], "IMG", 3) )  {
		AlertStatus |= 8;
		Index = ParseBuff((char *)&RxBuffer[0], '$');										// Parse up buffer for $ symbol
		if(Index){
			for(c = 0; (c < IMAGE_NAME_LEN) && (*Index != '$')&& (*Index != '\r'); c++)
				UniqRespName[c] = *Index++;
			UniqRespName[c] = '\0';	
		}
	}
	// System SMS distribution list update
	else if( StrCmp(&Command[0], "SYS", 3) )
		WebRequest |= 1;
	// System Status update request
	else if( StrCmp(&Command[0], "STA", 3) )
		WebRequest |= 2;
	
	if(SystemConfigUpdateFlag)
		UpdateSystemConfig();
		
	msDelay(2000);
	// Now delete that SMS, since we have a valid read +CMR (processed it)
	Uart2TxString("AT+CMGD=");
	Uart2TxChar(MsgIndx | 0x30);																		// Convert to ASCII
	Uart2Transfer("\r\n", 0x1388, 3);

	return (ProbModemRspns());
}

uint32_t ImageFIFO2Flash(uint32_t FlashAddress, uint32_t ImgLength)
{
	uint32_t FlashAddrOffSet = 0;
	uint32_t LoopCount, ReadCount;
	
	uint8_t FlashBuffer[64];
	uint32_t FlashBufferSize = sizeof(FlashBuffer);
	
	FlashSectorErase(1);  // 0x10000 and higher
	
	// ArduCAM setup
	ArdCAMChipSelect(true);
	// CS_ is now asserted
	ArduCAMBurstRead (ARDC_REG_BURST_FIFO);																				// Burst FIFO read operation
	
	for(LoopCount = 0; LoopCount < ImgLength; LoopCount += FlashBufferSize) {
		if(FlashAddrOffSet  + FlashBufferSize < ImgLength) {
			for(ReadCount = 0; ReadCount < FlashBufferSize; ReadCount++)
				FlashBuffer[ReadCount] = ArduCAMBurstRead(0x00);												// Dummy write
			// Now write the acquired 64 bytes to Flash memory
			FlashDataProgram(FlashAddrOffSet + FlashAddress, &FlashBuffer[0], FlashBufferSize, SPI_FLASH_INS_PP);
			FlashAddrOffSet  += FlashBufferSize;
		}
		else {
			for(ReadCount = 0; ReadCount < (ImgLength - FlashAddrOffSet); ReadCount++)
				FlashBuffer[ReadCount] = ArduCAMBurstRead(0x00);
			// Now write the remaining bytes to Flash memory
			FlashDataProgram(FlashAddrOffSet + FlashAddress, &FlashBuffer[0], (ImgLength - FlashAddrOffSet), SPI_FLASH_INS_PP);
		}
	}
	// SS line: INACTIVE = HIGH
	ArdCAMChipSelect(false);
	
	return 0;		
}
/**
 *  \brief Check if a given a string is composed of printable characters with no white spaces.
 *  
 *  \param [in] StrParam Input String 
 *  \param [in] ParamSize Input String size
 *  \return Is StrParam passes the check, returns true, else false
 *  
 *  \details More details
 */
uint32_t isPrintableASCIInoSpace(char *StrParam, uint32_t ParamSize) 
{
	uint32_t LoopCount, Flag = 1;

	// Loops until max string size or  until NULL character termination
	for (LoopCount = 0; (LoopCount < ParamSize) && (*StrParam != '\0'); LoopCount++) {
		if (*StrParam < 0x21 || *StrParam > 0x7E) {
			Flag = 0;
			break;
		}
		StrParam++;
	}
	return Flag;
}
