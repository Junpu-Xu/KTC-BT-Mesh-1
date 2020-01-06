/*---------------------------------------------------------------------------------------------------------
IDE information

IDE-Version:
uVision V5.23.0.0
Copyright (C) 2017 ARM Ltd and ARM Germany GmbH. All rights reserved.

License Information:
Junpu Xu
EasyRD
LIC=D0MII-T15AM-B0TNC-7EV6U-BMK3S-W19YC

Tool Version Numbers:
Toolchain:        PK51 Prof. Developers Kit  Version: 9.00
Toolchain Path:    D:\program files\Keil\C51\BIN
C Compiler:         C51.exe    V9.00
Assembler:          A51.exe    V8.01
Linker/Locator:     BL51.exe    V6.22
Librarian:             LIB51.exe    V4.24
Hex Converter:      OH51.exe    V2.6
CPU DLL:               S8051.DLL            V3.72
Dialog DLL:         DP51.DLL             V2.59
Target DLL:             Nuvoton_8051_Keil_uVision_Driver.dll      V1.09
Dialog DLL:         TP51.DLL             V2.58
--------------------------------------------------------------------------------*/


/*----------------------------------Software version record -------------------------------------
v0.0.0	2019_05_20
--initial version. Timmer configured,ADC configured
v0.0.1	2019_05_24
--UART functionality added
--ADC added,band-gap calibration added

v0.0.2 2019_05_27
--under testing version only

v0.0.3 2019_05_30
-- data flash operation added

v0.0.5 2019_07_08
-- ADC sensing keys and switches implemented

v0.0.6	2019_07_10

v0.0.7	2019_07_13

v0.0.8	2019_07_15

v0.1.1	2019_07_23
-- first functional version
-- Battery & Lamp sensing calibrated...

v0.1.2	2019_08_02
-- Batter OVP added. (UVP=3.5V;OVP=4V)

v0.1.3 2019_09_18
--GAP voltage calibrated

v0.1.4	2019_09_20
-- Load/lamp check calibrated
-- Battery check (NC,UVP,OVP) calibrated

v0.1.5 2020_01_06		(to worked with TELINK BT chip)
-- UART configuration according to "MCU-BT UART communication,v0.2,2020_01_06"
-- UART receive/send frame tested
-- "Query_Firmware_Version" implemented
------------------------------------------------------------------------------------------------------------*/
//***********************************************************************************************************

//***********************************************************************************************************
// 
//***********************************************************************************************************
#include "N76E003.h"
#include "SFR_Macro.h"
#include "Common.h"
#include "Delay.h"
#include "string.h"
#include "stdlib.h"
#include "Function_define.h"


//!!! IMPORTANT
//choose which project is to be worked with...
//#define BT_MESH_FEATURE_IN
#define EMERGENCY_LAMP_SINGLE_BATTERY
//#define DEBUG_PRINTF_LOG_EN

//v0.1.5
#define Firmware_INFO_1	0x00
#define Firmware_INFO_2	0x01
#define Firmware_INFO_3	0x05
//#define SOFTWARE_INFO "software version = 0.1.5,2020_01_06"\r\n"

//Data flash
#define ADDR_BASE 0x4700
#define ADDR_SYSTEM_DATA	0x00		//relative address for system data
#define LENGTH_SYSTEM_DATA	0x10	//16 bytes for system data

#define FLASH_L_LOW			ADDR_BASE+1
#define FLASH_L_HIGH		ADDR_BASE+2
#define FLASH_H_LOW			ADDR_BASE+3
#define FLASH_H_HIGH		ADDR_BASE+4

volatile unsigned char xdata page_buffer[128];
void Read_APROM_PAGE();
void Write_DATAFLASH_BYTE_BUFFER(UINT8 u8_address,UINT8 u8_Data);
UINT8 Read_APROM_BYTE(UINT16 code *u16_addr);
void Write_DATAFLASH_BYTE(unsigned int u16EPAddr,unsigned char u8EPData);
void Write_DATAFLASH_PAGE(void);
void clear_flash(void);
void system_save_data(void);
void system_read_data(void);

//timmer
	bit F_1ms = 0;
	bit F_500ms = 0;
	UINT16 xdata ui_Day_counter = 0;
	UINT8 xdata uc_Hour_counter = 0;
	UINT8 xdata uc_Minute_counter = 0;
	UINT8 xdata uc_Second_counter = 0;
	UINT8 xdata uc_500ms_counter = 0;
	
	UINT16 xdata ui_Duration_Test_Cycle_Day = 0;
	UINT8 xdata uc_Duration_Test_Cycle_Hour = 0,uc_Duration_Test_Cycle_Minute = 0,uc_Duration_Test_Cycle_Second = 0;

	UINT8 xdata uc_AC_Charging_Time_Second = 0,uc_AC_Charging_Time_Minute = 0,uc_AC_Charging_Time_Hour = 0;
	UINT8 xdata uc_AC_Charging_Target_Hour = 0,uc_AC_Charging_Target_Minute = 0,uc_AC_Charging_Target_Second = 0;
	UINT8 xdata uc_Duration_Test_Time_Second = 0,uc_Duration_Test_Time_Minute = 0,uc_Duration_Test_Time_Hour = 0,uc_Duration_Test_Done = 0;
	UINT8 xdata uc_Emergency_Time_Second = 0,uc_Emergency_Time_Minute = 0,uc_Emergency_Time_Hour = 0;
	UINT8 xdata uc_Functional_Test_Time_Second = 0,uc_Functional_Test_Done = 0;

	UINT16 xdata ui_ms_counter = 0;	

//UART 
//#define UART_ECHO_DEBUG
#define BT_UART_ON

	//FRAME HEADER 0x5AA5
	#define BT_UART_FRAME_HEAD1	0x5A
	#define BT_UART_FRAME_HEAD2	0xA5

	#define BUFFER_SIZE		16
	char xdata *UART_BUFFER;
	char xdata *UART_BUFFER_TEMP;
	UINT16 xdata u16_CNT=0,u16_CNT1=0;
	bit riflag;
	volatile UINT8 xdata U8_UART_Receive_Pakage_Status;
	#define UART_RECEIVE_PACKAGE_START	1
	#define UART_RECEIVE_PACKAGE_COMPLETE	2
	#define UART_RECEIVE_PACKAGE_PAUSE	0

	#define TxBufferSize	150
	#define RxBufferSize	80
	
	volatile UINT8 xdata U8_UART_BUS_STATUS = 0;
	#define UART_BUS_STATUS_STANDBY	0	//ready for communication
	#define UART_BUS_STATUS_BUSY	1	//UART is busy now
	
	volatile UINT8 U8_UART_TxD_Handle_STATUS = 0;
	#define UART_HANDLE_STATUS_STANDBY	0
	#define UART_HANDLE_STATUS_BUSY	1
	#define UART_HANDLE_STATUS_WAITING_ACK	2	//MCU waits for ACK from BT
	#define UART_HANDLE_STATUS_WAITING_ANSWER	3	//MCU waits for answer from BT
	#define UART_HANDLE_STATUS_TIME_OUT	4
	#define UART_HANDLE_STATUS_TO_REPORT	5	//MCU is going to report to BT
	#define UART_HANDLE_STATUS_TO_ACK	6	//MCU is going to ack BT\
	
	UINT8 xdata U8_UART_TxD_Handle_ticks_ms = 0;
	#define UART_TxD_HANDLE_CYCLE_MS	5
	
	#define UART_MAX_TRY_TIMEOUT	4
	
	UINT8 U8_UART_TxD_Handle(void);
	UINT8 U8_BT_MESH_UART_RxD_Handle(void);

	#define MESH_DATA_MAX_LEN	10
	struct Mesh_Data_package
	{
		UINT8	DATA_Package_Header1;
		UINT8	DATA_Package_Header2;
		UINT8	DATA_Length;		
		UINT8	Function_Byte;
		UINT8	DATA[MESH_DATA_MAX_LEN];
		UINT8	DATA_CHECK_SUM;
		UINT8	STATUS;
		UINT8 	DATA_Pointer;
	} xdata Mesh_Data_RxD,Mesh_Data_TxD;
	#define MESH_Data_RxD_STANDBY	0
	#define MESH_Data_RxD_FRAME_HEAD_READING	1
	#define MESH_Data_RxD_FRAME_HEAD_RECEIVED	2
	#define MESH_Data_RxD_DATA_LEN_RECEIVED		3
	#define MESH_Data_RxD_DATA_READING			4
	#define MESH_Data_RxD_DATA_RECEIVED			5
	#define MESH_Data_RxD_DATA_SUM_RECEIVED		6
	#define MESH_Data_RxD_FRAME_COMPLETE		7
	#define MESH_Data_RxD_BUSY					8
	#define MESH_Data_RxD_LEN_ERROR				100
	#define MESH_Data_RxD_SUM_ERROR				101
	#define MESH_Data_RxD_TO_be_Processed		201
	

//BT_UART Query instruction set
	#define Query_Firmware_Version	0x01


//BT_UART Command instruction set
	//#define Query_Firmware_Version	0x81

	#define Instruction_Send	0xAAFB
	#define Address_Broadcast	0xFFFF

	UINT8 xdata TxBuffer[TxBufferSize]; 
	UINT8 xdata RxBuffer[RxBufferSize];
	volatile UINT8 RxBuffer_read_counter = 0x00;
	volatile UINT8 RxBuffer_write_counter = 0x00;
	volatile UINT8 TxBuffer_read_counter = 0x00;
	volatile UINT8 TxBuffer_write_counter = 0x00;

void v_BT_MESH_UART_Answer_processing();

//ADC
//Load-Lamp voltage sensing: 100K + 4.7K
#define LOAD_OVP_1V		1
#define LOAD_OVP_mV	1794	//1.79V @AIN, 40V lamp voltage when open (35V requested)
#define LOAD_FALLING_1V	0
#define LOAD_FALLING_0_01V 48	//0.48V @AIN
#define LOAD_OPEN_1V	0
#define LOAD_OPEN_mV	50	//0.05V @AIN

//battery voltage sensing: 3.3K + 33K
#define BATTERY_UVP_mV	3180	//3.18V @AIN,battery voltage 3.5V
#define BATTERY_DIS_INTERNAL_LOSS_mV	200	//Batter internal voltage loss compensation. 0.2V@MCU = 0.22V battery voltage drop during discharging
#define BATTERY_NC_1V	0
#define BATTERY_NC_mV	450	//0.45V @AIN,0.5V gap between connect & disconnect battery
#define BATTERY_OVP_mV	3640	//3.64V @AIN,battery voltage 4V
#define BATTERY_CHAG_INTERNAL_LOSS_mV	90	//Batter internal voltage loss compensation. 0.09V@MCU = 0.1V battery voltage drop during charging


#ifdef BT_MESH_FEATURE_IN
	//BT related definition
	#define PWRC_AT_INSTRUCTION_MODE	clr_P14
	#define PWRC_TRANS_MODE		set_P14
	#define BT_STAT_PIN		P15
	#define BT_RESET_PIN	P05
	UINT8 U8_BT_Connect_Status(void); 
	
	UINT8 xdata U8_BT_Short_Address = 0;

	#define KEY_AUTO_MANUAL_REF1_1V	1
	#define KEY_AUTO_MANUAL_REF1_0_01V	34	//1.34V
	#define KEY_AUTO_MANUAL_REF2_1V	1
	#define KEY_AUTO_MANUAL_REF2_0_01V	65	//1.65V
	#define KEY_AUTO_MANUAL_REF3_1V	2
	#define KEY_AUTO_MANUAL_REF3_0_01V	24	//2.24V
	
	UINT8 xdata KEY_AUTO_TEST_STATUS_TMP,KEY_MANUAL_TEST_STATUS_TMP;
	UINT8 xdata KEY_AUTO_TEST_STATUS,KEY_MANUAL_TEST_STATUS;
	UINT8 xdata KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER;
	
	#define SW_3_BITS_REF1_1V	1
	#define SW_3_BITS_REF1_0_01V	23	//1.23V
	#define SW_3_BITS_REF2_1V	1
	#define SW_3_BITS_REF2_0_01V	34	//1.34V
	#define SW_3_BITS_REF3_1V	1
	#define SW_3_BITS_REF3_0_01V	49	//1.49V
	#define SW_3_BITS_REF4_1V	1
	#define SW_3_BITS_REF4_0_01V	65	//1.65V
	#define SW_3_BITS_REF5_1V	1
	#define SW_3_BITS_REF5_0_01V	96	//1.96V
	#define SW_3_BITS_REF6_1V	2
	#define SW_3_BITS_REF6_0_01V	24	//2.24V
	#define SW_3_BITS_REF7_1V	2
	#define SW_3_BITS_REF7_0_01V	72	//2.72V	
	
	UINT8 xdata SW_S1_STATUS_TMP,SW_S2_STATUS_TMP,SW_S3_STATUS_TMP;
	UINT8 xdata SW_S1_STATUS,SW_S2_STATUS,SW_S3_STATUS;
	UINT8 xdata SW_3_BITS_SENSING_FILTER_CUNTER;
	
	#define KEY_SENSE_FILTER	10	
#endif
	volatile bit F_ADC_Complete = 0;
	bit F_ADC_All_Channels_Complete = 0;
	UINT16 xdata ADC_Value_Array[10] = {0x03ff};
	UINT8 xdata ADC_index = 0;	
	
	UINT16 xdata ADC_Value = 0;
	UINT16 xdata Bandgap_Value = 0;
	UINT16 xdata Load_Value = 0;
	UINT16 xdata Load_Value_mV = 0;
	UINT16 xdata Battery_Value = 0;
	UINT16 xdata Battery_Value_mV = 0;
	UINT16 xdata Battery_Value_mV_bkup = 0;
	UINT16 xdata KEY_Auto_Manual_Value = 0;
	UINT16 xdata SW_3_bits_Value = 0;	


	UINT16 xdata Battery_Value_array[10] = {0x03ff};
	UINT8 xdata Battery_index = 0;
	UINT8 xdata lamp_bad = 0;
	UINT8 xdata Battery_bad = 0;
	UINT8 xdata Battery_Fully_Charged = 0;
	UINT8 xdata uc_Lamp_bad_counter = 0;
	
	UINT16 xdata ADC_Value_Max = 0;
	UINT16 xdata ADC_Value_Min = 0x3FFF;
	UINT16 xdata battery_max = 0;
	UINT16 xdata battery_min = 0x3FFF;
	UINT16 xdata Load_max = 0;
	UINT16 xdata Load_min = 0x3FFF;
	
	bit F_OVP_Rising = 0;
	bit F_OVP_Falling = 0;
	
// system state machines
	UINT8 uc_System_State = 0;
	#define System_Reset 	0
	#define AC_Charging		1
	#define Battery_Fault	2
	#define Functional_Test 3
	#define Duration_Test	4
	#define Battery_Life_Fault	5
	#define Lamp_Fault		6
	#define Emergency_Operation 7
	#define Reset_Charging_Time 8	

	bit F_Duration_Test = 0;
	bit F_Duration_Test_Pending = 1;
	bit F_Mannual_Duration_Test_Pending;
	bit F_Functional_Test;
	bit F_Charging_Full;

//LED operation
	#define OFF		0
	#define Flashing_Slow	4
	#define Flashing_Medium 2
	#define Flashing_Fast	3
	#define ON				1
	UINT8 xdata led_green_state = OFF;
	UINT8 xdata led_red_state = OFF;
	UINT8 xdata led_green_state_before_test,led_red_state_before_test,led_red_state_before_Emergency;

	#define GREEN_LED_PIN_LOW	clr_P00
	#define GREEN_LED_PIN_HIGH	set_P00
	#define GREEN_LED_PIN_TOGGLE	P00 = ~P00
	#define RED_LED_PIN_LOW	clr_P01
	#define RED_LED_PIN_HIGH	set_P01	
	#define RED_LED_PIN_TOGGLE	P01 = ~P01	
//testing mode
	UINT8 xdata auto_test_flag = 0;  // 1: Auto-self, 0:Normal


//battery charging detection
	UINT8 xdata U8_Battery_Discharge_ticks_ms = 0;
	#define Battery_NC_Check_Discharge_Ticks_ms		10	//10ms every 5 Second
	UINT8 xdata uc_Battery_Charging_Flag_Pre =0;
	UINT8 xdata uc_Battery_Fault_Flag = 0;

//key detection	
	#define KEY_AUTO_TEST	P13
	#define KEY_MANUAL_TEST	P05

	UINT8 xdata key_state = 0;
	UINT8 xdata key_state_prev = 0;
	UINT16 xdata ui_Key_Pressed_ms_counter = 0;


//AC mains detection
	//sbit AC_MAIN_DETECT	 =P1^1;	//CHRG_DET, check if AC mains available
	#define AC_MAIN_DETECT P11	//CHRG_DET, check if AC mains available

	UINT8 xdata AC_Mains_state = 0;
	UINT8 xdata uc_AC_Mains_State_ms_counter = 0;

//LAMP CTL,Battery Ctl
#ifdef EMERGENCY_LAMP_SINGLE_BATTERY
	#define LAMP_EN		P14
	#define LAMP_CTL	P15		//LAMP CTL pin
	#define CHARGE_ENABLE	P10
#endif

#ifdef BT_MESH_FEATURE_IN
	#define LAMP_EN		P10		//to be checked???
	#define LAMP_CTL	P10		//LAMP CTL pin
	#define CHARGE_ENABLE	P17
#endif	

UINT8 xdata uc_Charging_Status;
	
static void Var_Init(void);
static void v_Timming_Processing(void);
static void v_LED_Operation(void);
static void v_Battery_Charging_Operation(void);
static void v_Key_Detection(void);
static void v_KEY_AUTO_MANUAL_SENSE();
static UINT8 U8_SW_3_BITS_SENSE();
static void v_AC_Mains_Detection(void);
static void v_State_Machine_Processing(void);
static void v_Auto_Test_Detect(void);
static void Battery_Test_Proc(void);
static void Load_Test_Proc(void);
static void v_ADC_process(void);
UINT16 Bandgap_Calibrate(UINT8 Value_1V,UINT8 Value_0_01V);
UINT16 ADC_to_AIN_mV(UINT16 Value_ADC);
UINT16 U16_Read_Bandgap();
UINT16 xdata U16_Bandgap_Value;

char asciitohex(char ascii_byte);
char hextoascii(char hex_byte);

void v_init();

void v_Load_Mesh_Send_Data(UINT16 Instruction,UINT16 Address,UINT8 Data[]);

//every 1 ms
void Timer1_Overflow_ISR(void) interrupt 3
{
	clr_TF1;	//clear overflow flag
	clr_TR1;
	TH1 = 0xFA;
	TL1 = 0xCB;
	set_TR1;
	
	F_1ms = 1;

	
}

/**
 * FUNCTION_PURPOSE: serial interrupt, echo received data.
 * FUNCTION_INPUTS: P0.7(RXD) serial input
 * FUNCTION_OUTPUTS: P0.6(TXD) serial output
 */
void SerialPort0_ISR(void) interrupt 4 
{
	if (RI==1)
	{                                       /* if reception occur */
		clr_RI;                             /* clear reception flag for next reception */
		RxBuffer[RxBuffer_write_counter] = SBUF;
		
#ifdef UART_ECHO_DEBUG
	putchar(RxBuffer[RxBuffer_write_counter]);
#endif		
		
		//SBUF = RxBuffer[RxBuffer_write_counter];
		
		RxBuffer_write_counter ++;
		if(RxBuffer_write_counter >= RxBufferSize)
		{
			RxBuffer_write_counter = 0;
		}

		riflag =1;
		
		
    }
    if(TI==1)
	{
		clr_TI;                             /* if emission occur */
 		if(TxBuffer_read_counter != TxBuffer_write_counter)
		{
			SBUF = TxBuffer[TxBuffer_read_counter];
			TxBuffer_read_counter ++;
			if(TxBuffer_read_counter >= TxBufferSize)
			{
				TxBuffer_read_counter = 0;
			}
			
			U8_UART_BUS_STATUS = UART_BUS_STATUS_BUSY;
		} 
		else
		{
			U8_UART_BUS_STATUS = UART_BUS_STATUS_STANDBY;
		}
	}
}

void Timer2_Event_ISR(void) interrupt 5
{
	clr_TF2;	//clear overflow flag
	F_500ms = 1;
	//P00 = ~P00;
	//P01 = ~P01;
	
}

void ADC_Complete_ISR(void) interrupt 11
{
	clr_ADCF;	//clear overflow flag

	F_ADC_Complete = 1;	//Set flag of ADC complete
	//set_ADCS;
	
	
	
	
}

char putchar (char c)
{
//		while (!TI_1);  /* wait until transmitter ready */
	TxBuffer[TxBuffer_write_counter] = c;
	TxBuffer_write_counter ++;
	if(TxBuffer_write_counter >= TxBufferSize)
	{
		TxBuffer_write_counter = 0;
	}
		
		//SBUF = c;      /* output character */
		return (c);
}

void v_Load_Mesh_Send_Data(UINT16 Instruction,UINT16 Address,UINT8 Data[])
{
	UINT8 temp_Mesh_Send_Data,i_Mesh_Send_Data;

	//load instruction
	temp_Mesh_Send_Data = (Instruction >> 8) & 0xFF;
	putchar(temp_Mesh_Send_Data);	
	temp_Mesh_Send_Data = Instruction & 0xFF;
	putchar(temp_Mesh_Send_Data);
	
	//load address
	temp_Mesh_Send_Data = (Address >> 8) & 0xFF;
	putchar(temp_Mesh_Send_Data);	
	temp_Mesh_Send_Data = Address & 0xFF;
	putchar(temp_Mesh_Send_Data);	
	
	//load data
	//10 bytes data
	for(i_Mesh_Send_Data = 0;i_Mesh_Send_Data < 10; i_Mesh_Send_Data++)
	{
		putchar(Data[i_Mesh_Send_Data]);
	}
	
	
}

//v_init() function
void v_init()
{	
	//Clock source: HIRC=16MHz
	set_HIRCST;	
	//CKDIV = 1;	//clock divider = 1. F_sys = 16MHz/2 = 8MHz

	set_CLOEN;	//Clock Output Enable
	
	/*
	Timer2 setting, system real time clock
	*/
	//T2MOD setting
	{
		set_LDEN;	//Enable Auto-reload
		TIMER2_DIV_256;	//T2DIV = 101, divider = 128
		clr_CAPCR;	//
		clr_LDTS1;
		clr_LDTS0;	//Auto reload when timer2 overflow
		//Auto reload value
		RCMP2H = 0x85;
		RCMP2L = 0xEE;	//To generate 2Hz real time clock. auto reload valaue = 65536 - 8MHz/128 = 34286;

		clr_CMRL2;	//Timer2 in auto-reload mode
		
		set_ET2;	//Enable Timer2 interrup

		set_TR2;	//Start Timer2
	}
	
	/*
	Timer1 setting, system tick clock
	*/
	{
		TIMER1_MODE1_ENABLE;	//Timer1,16bits counter mode
		
		TIMER0_MODE1_ENABLE;	//Timer0,16bits counter mode
		clr_GATE_T1;
		clr_CT_T1;
		
		clr_T1M;	//Timer0 clock is 1/12 of system clock
		P2S = 0;
		
		set_ET1;	//Enable Timer1 interrup
		
		set_TR1;	//Start Timer1
	}
	
	/*
	ADC configuration
	*/	
	{
		Enable_ADC_AIN5;
		Enable_ADC_AIN6;
		Enable_ADC_BandGap;
		
		clr_ADCEX;	//ADC start via setting ADCS bit
		
		clr_ADFBEN;		//ADC asserting Fault Brake Disabled
		clr_ADCMPEN;	//ADC result comparator Disabled
		
		set_EADC;	//Enable ADC interrupt
		
		set_ADCEN;	//ADC enable
		
	}
	
	

	/*
	Pin configuration
	*/
#ifdef EMERGENCY_LAMP_SINGLE_BATTERY
	{
		P05_Quasi_Mode;		//Pin1,Key_Manual_Test
		P06_PushPull_Mode;	//Pin2, TxD
		P07_Quasi_Mode;		//Pin3,RxD
		
		P15_PushPull_Mode;	//Pin10,LAMP_CTL
		P14_PushPull_Mode;	//Pin11,LAMP_EN
		P13_Quasi_Mode;		//Pin12,Key_Auto_Test
		
		P11_Input_Mode;		//Pin14,CHG_DET
		P10_PushPull_Mode;	//Pin15,CHRG_EN
		P00_PushPull_Mode;	//Pin16,LED_GREEN
		P01_PushPull_Mode;	//Pin17,LED_RED
		
		P03_Input_Mode;		//Pin19,VBAT
		#define VBAT_ADC_CHANNEL_NR	6	//channel 6
		P04_Input_Mode;		//Pin20,VLAMP
		#define VLAMP_ADC_CHANNEL_NR	5	//Channel 5
	}
#endif
#ifdef BT_MESH_FEATURE_IN
	{
		P05_PushPull_Mode;		//Pin1,Reset_BT
		P06_PushPull_Mode;	//Pin2, TxD
		P07_Quasi_Mode;		//Pin3,RxD
		
		P30_Input_Mode;		//Pin5,SW_detect	AIN1 for detecting 3bits switch
		#define SW_DET_ADC_CHANNEL_NR	1	//channel 1
		
		P17_PushPull_Mode;	//Pin6,CHRG_EN
		
		
		P15_Quasi_Mode;	//Pin10,BT_STAT
		P14_PushPull_Mode;	//Pin11,BT_PWRC
		P13_Quasi_Mode;		//Pin12,CHRG_Status
		P12_Quasi_Mode;		//Pin13,POWER_DET
		
		P11_Input_Mode;		//Pin14,KEY_AUTO_Manual AIN7 for detecting Auto/Manual test button
		#define KEY_AUTO_MANUAL_ADC_CHANNEL_NR	7	//channel 7
		
		P10_PushPull_Mode;	//Pin15,LAMP_CTL
		P00_PushPull_Mode;	//Pin16,LED_GREEN
		P01_PushPull_Mode;	//Pin17,LED_RED
		
		P03_Input_Mode;		//Pin19,VLAMP
		#define VLAMP_ADC_CHANNEL_NR	6	//Channel 6
		P04_Input_Mode;		//Pin20,VBAT
		#define VBAT_ADC_CHANNEL_NR	5	//channel 5
	}
#endif

	AC_MAIN_DETECT = ON;	

	InitialUART0_Timer3(115200);


	set_RB8;					//This bit is for setting the stop bit 2 high/low status,  */
	clr_TI; 
    set_ES;           //enable UART interrupt
    set_EA;           //enable global interrupt
	
	Var_Init();
}



static void Var_Init(void)
{
	//init variables
	ui_Day_counter = 0;
	uc_Hour_counter = 0;
	uc_Minute_counter = 0;
	uc_Second_counter = 0;
	uc_500ms_counter = 0;
	F_500ms = 0;
	
	auto_test_flag = 0;	//default for normal test
	
	uc_AC_Charging_Time_Second = 0;
	uc_AC_Charging_Time_Minute = 0;
	uc_AC_Charging_Time_Hour = 0;


#ifdef BT_MESH_FEATURE_IN
	KEY_AUTO_TEST_STATUS = OFF;
	KEY_MANUAL_TEST_STATUS = OFF;
	KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER = 0;
	
	SW_S1_STATUS = OFF;
	SW_S1_STATUS = OFF;
	SW_S1_STATUS = OFF;
	SW_3_BITS_SENSING_FILTER_CUNTER = 0;
#endif	
				
	//Get a seed for random function from battery voltage
	Enable_ADC_BandGap;	//bandgap selected
	clr_ADCF;	//clear overflow flag
	set_ADCS;	//Start ADC
	while (ADCS == 1)
	{
		nop;
	}
	Timer0_Delay100us(1);
	srand(((ADCRH<<4) + ADCRL));
	
	uc_AC_Charging_Target_Hour = (24 + rand()%24);
	uc_AC_Charging_Target_Minute = rand()%60;
	uc_AC_Charging_Target_Second = rand()%60;
	
	v_Auto_Test_Detect();
	
	if(auto_test_flag == 1)
	{
		uc_AC_Charging_Target_Hour = 0;
		uc_AC_Charging_Target_Minute = 0;
		uc_AC_Charging_Target_Second = (24 + rand()%24);		
	}
	
	F_Charging_Full = 0;
	
	ui_Duration_Test_Cycle_Day = (358 + rand()%6);
	uc_Duration_Test_Cycle_Hour = rand()%24;
	uc_Duration_Test_Cycle_Minute = rand()%60;
	uc_Duration_Test_Cycle_Second = rand()%60;
	if(auto_test_flag == 1)
	{
		ui_Duration_Test_Cycle_Day = 0;
		uc_Duration_Test_Cycle_Hour = 0;
		uc_Duration_Test_Cycle_Minute = 4;
		uc_Duration_Test_Cycle_Second = rand()%60;		
	}
	
	
	uc_Duration_Test_Time_Second = 0;
	uc_Duration_Test_Time_Minute = 0;
	uc_Duration_Test_Time_Hour = 0;
	uc_Duration_Test_Done = 0;
	
	uc_Emergency_Time_Second = 0;
	uc_Emergency_Time_Minute = 0;
	uc_Emergency_Time_Hour = 0;
	
	uc_Functional_Test_Time_Second = 0;
	uc_Functional_Test_Done = 0;
	
	ui_ms_counter = 0;
	
	led_green_state = OFF;
	led_red_state = OFF;
	
	
	uc_System_State = System_Reset;
	
	AC_Mains_state = OFF;
	
	
	F_Duration_Test_Pending = 1;	//Set Duration_Test_Pending Flag
	F_Mannual_Duration_Test_Pending = 0;
	F_Duration_Test = 0;
	F_Functional_Test = 0;
	
	uc_Battery_Fault_Flag = 0;
	
	Enable_ADC_BandGap;	
	set_ADCS;	//Start ADC conversion
	
	F_ADC_Complete = 0;
	F_ADC_All_Channels_Complete = 0;	
	
	U16_Bandgap_Value = U16_Read_Bandgap();
	
	
	
	//UART related variables init
	
	U8_UART_BUS_STATUS = UART_BUS_STATUS_STANDBY;
	U8_UART_TxD_Handle_STATUS = UART_HANDLE_STATUS_STANDBY;
	RxBuffer_read_counter = 0;
	RxBuffer_write_counter = 0;
	TxBuffer_read_counter = 0;
	TxBuffer_write_counter = 0;
	U8_UART_TxD_Handle_ticks_ms = 0;
}

/************************************************************************************************************
*    Main function 
************************************************************************************************************/
void main (void)
{
	UINT8 i_temp;
	UINT8 Data_Test[16] = {"FUCK56789A"};
	char* p_temp;
//	UINT8 datatemp;
	
	
	v_init();

#ifdef DEBUG_PRINTF_LOG_EN	
	//software version info
	//printf(SOFTWARE_INFO,"\r\n");
	//printf("software version = ",Firmware_INFO,"\r\n");
	TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
	Timer0_Delay1ms(10);	
#endif




	LAMP_CTL = OFF;	//Turn Off Q6
	LAMP_EN = OFF;		//Disable LAMP */	

    while(1)
		{
			if (riflag)
			{
				//P12 = ~P12;		//In debug mode check UART_BUFFER[u16_CNT] to check receive data
				riflag = 0;
				
				
				if(U8_BT_MESH_UART_RxD_Handle() == UART_HANDLE_STATUS_TIME_OUT)
				{
					#ifdef DEBUG_PRINTF_LOG_EN
						printf("UART RxD timeout \r\n");
					#endif
				}
				
				if(Mesh_Data_RxD.STATUS == MESH_Data_RxD_TO_be_Processed)
				{
					Mesh_Data_RxD.STATUS = MESH_Data_RxD_STANDBY;
					v_BT_MESH_UART_Answer_processing();
				}	
			}
			
			if(U8_UART_Receive_Pakage_Status == UART_RECEIVE_PACKAGE_COMPLETE)
			{
				U8_UART_Receive_Pakage_Status = UART_RECEIVE_PACKAGE_PAUSE;
				#ifdef DEBUG_PRINTF_LOG_EN
					printf("%s\n",UART_BUFFER);
					TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
					Timer0_Delay1ms(10);
				#endif
			}
			
			//Timming processing,every 500ms system tick
			if(F_500ms)
			{
				F_500ms = 0;	
				uc_500ms_counter++;
				if((uc_500ms_counter % 2) == 0)
				{					
					v_Timming_Processing();
					//printf("second = %c\r\n", hextoascii(uc_Second_counter));
				}

				//when battery charging status == ON, disconnect charging for 20ms every 3 second,to do battery NC check				
				if((uc_500ms_counter % 6) == 0)
				{
					if(uc_Charging_Status == ON)
					{
						U8_Battery_Discharge_ticks_ms = 0;	//reset pulse counter,for No_Battery check 
					}										
					//printf("500ms_counter = %c\r\n", hextoascii(uc_500ms_counter));
				}
				
				#ifdef DEBUG_PRINTF_LOG_EN
					printf("System state=0x%c\r\n", hextoascii(uc_System_State));
					printf("VCC @MCU = %d\r\n",ADC_to_AIN_mV(0xFFF));
				#endif
#ifdef BT_MESH_FEATURE_IN								
				//if both auto-test & manual test button is pressed
				if((KEY_AUTO_TEST_STATUS == ON) && (KEY_MANUAL_TEST_STATUS == ON))
				{
					//function to be defined...


				}
#endif				
			}		
			
			//every 1ms
			if(F_1ms)
			{
				F_1ms = 0;

			
				v_LED_Operation();				
				v_Battery_Charging_Operation();
				v_AC_Mains_Detection();			
				v_State_Machine_Processing();
				
				U8_UART_TxD_Handle_ticks_ms++;
				U8_Battery_Discharge_ticks_ms++;

			}
			
			//around every 26.6K sampling rate
			//run once every 38us
			if(F_ADC_Complete)
			{
				F_ADC_Complete = 0;
				v_ADC_process();
			}
			
			//All ADC channels sampling complete, around every ... ms
			if(F_ADC_All_Channels_Complete)
			{
				F_ADC_All_Channels_Complete = 0;
				
				Load_Test_Proc();
				Battery_Test_Proc();
				
				v_KEY_AUTO_MANUAL_SENSE();
				i_temp = U8_SW_3_BITS_SENSE();

	
			}
			
			
			//UART Handle
			if(U8_UART_TxD_Handle_ticks_ms == UART_TxD_HANDLE_CYCLE_MS)
			{
				U8_UART_TxD_Handle_ticks_ms = 0;	//reset
				

				
				if(U8_UART_TxD_Handle() == UART_HANDLE_STATUS_TIME_OUT)
				{
					TI = 1;
				}
			}
				
		}
	
}
static void v_Timming_Processing(void)
{
	//every second,system time update
	
	//system_save_data();
	
	uc_Second_counter++;
	if(uc_Second_counter >= 60)
	{
		uc_Second_counter = 0;
		
		//Every Minute
		uc_Minute_counter++;
		if(uc_Minute_counter >= 60)
		{
			uc_Minute_counter = 0;
			
			//Every Hour
			uc_Hour_counter++;
			if(uc_Hour_counter >= 24)
			{
				uc_Hour_counter = 0;
				
				//Every Day
				//Reset every 364days = 52weeks
				ui_Day_counter++;
				if(ui_Day_counter >= 364)
				{
					ui_Day_counter = 0;
					
						//re-configure a random time for next duration test
						ui_Duration_Test_Cycle_Day = (358 + rand()%6);
						uc_Duration_Test_Cycle_Hour = rand()%24;
						uc_Duration_Test_Cycle_Minute = rand()%60;
						uc_Duration_Test_Cycle_Second = rand()%60;
				}
			}
		}
		if(auto_test_flag == 1)	//Auto_testing mode,every 5 minutes reset system time
		{
			if(uc_Minute_counter >= 5)
			{
				uc_Minute_counter = 0;
				//re-configure a random time for next duration test
				ui_Duration_Test_Cycle_Day = 0;
				uc_Duration_Test_Cycle_Hour = 0;
				uc_Duration_Test_Cycle_Minute = 4;
				uc_Duration_Test_Cycle_Second = rand()%60;				
			}
		}
	}	
	//Random time for Duration test
	if((ui_Day_counter == ui_Duration_Test_Cycle_Day)
		&& (uc_Hour_counter == uc_Duration_Test_Cycle_Hour)
			&& (uc_Minute_counter == uc_Duration_Test_Cycle_Minute)
				&&(uc_Second_counter == uc_Duration_Test_Cycle_Second))
				{
					F_Duration_Test = 1;
				}
	
	//to Functional_Test, every week
	if(((ui_Day_counter % 7) == 6) 
		&& (uc_Hour_counter == 23)
			&&(uc_Minute_counter == 59)
				&&(uc_Second_counter == 59))
				{
					F_Functional_Test = 1;
				}
	//auto test mode,every 1 minute
	if((auto_test_flag == 1) && (uc_Second_counter == 59))
	{
		F_Functional_Test = 1;
	}
	
	
	//AC_Charging time update
	if(uc_System_State == AC_Charging)
	{
		//every second,AC_Charging time update
		uc_AC_Charging_Time_Second++;
		if(uc_AC_Charging_Time_Second >= 60)
		{
			uc_AC_Charging_Time_Second = 0;
			
			//Every Minute
			uc_AC_Charging_Time_Minute++;
			if(uc_AC_Charging_Time_Minute >= 60)
			{
				uc_AC_Charging_Time_Minute = 0;
				
				//Every Hour
				uc_AC_Charging_Time_Hour++;
				if(uc_AC_Charging_Time_Hour >= 48)
				{
					uc_AC_Charging_Time_Hour = 48;		//48 hours max
				}
			}
		}

		if((uc_AC_Charging_Time_Hour == uc_AC_Charging_Target_Hour)
			&& (uc_AC_Charging_Time_Minute == uc_AC_Charging_Target_Minute)
				&& (uc_AC_Charging_Time_Second == uc_AC_Charging_Target_Second))
		{
			F_Charging_Full = 1;				
		}				
	}
	
	//Duration_Test time update
	if((uc_System_State == Duration_Test) || (uc_System_State == Emergency_Operation))
	{
		//every second,Duration_Test time update
		uc_Duration_Test_Time_Second++;
		if(uc_Duration_Test_Time_Second >= 60)
		{
			uc_Duration_Test_Time_Second = 0;
			
			//Every Minute
			uc_Duration_Test_Time_Minute++;
			if(uc_Duration_Test_Time_Minute >= 60)
			{
				uc_Duration_Test_Time_Minute = 0;
				
				//Every Hour
				uc_Duration_Test_Time_Hour++;
				if(uc_Duration_Test_Time_Hour >= 3)
				{
					uc_Duration_Test_Time_Hour = 3;		//Max 3 hours
					uc_Duration_Test_Done = 1;
				}
			}
		}
		//Auto-testing mode,30 seconds instead of 3 hours
		if((auto_test_flag == 1) && (uc_Duration_Test_Time_Second >= 30))
		{
			uc_Duration_Test_Done = 1;
		}			
	}		
	
	//Emergency operation time update
	if(uc_System_State == Emergency_Operation)
	{
		//every second,Emergency_Operation time update
		uc_Emergency_Time_Second++;
		if(uc_Emergency_Time_Second >= 60)
		{
			uc_Emergency_Time_Second = 0;
			
			//Every Minute
			uc_Emergency_Time_Minute++;
			if(uc_Emergency_Time_Minute >= 60)
			{
				uc_Emergency_Time_Minute = 0;
				
				//Every Hour
				uc_Emergency_Time_Hour++;
				if(uc_Emergency_Time_Hour >= 250)
				{
					uc_Emergency_Time_Hour = 250;		//Max 250 hours
				}
			}
		}			
	}

	//Functional_Test time update
	if(uc_System_State == Functional_Test)
	{
		//functional test period 108 seconds
		uc_Functional_Test_Time_Second++;
		if(uc_Functional_Test_Time_Second >= 108)
		{
			uc_Functional_Test_Done = 1;
		}
		//Auto-testing mode,10 seconds instead of 108 seconds
		if((auto_test_flag == 1) && (uc_Functional_Test_Time_Second >= 10))
		{
			uc_Functional_Test_Done = 1;
		}				
	}			
}

static void v_Auto_Test_Detect(void)
{   
	if(KEY_AUTO_TEST == 0)
	{
		clear_flash();		//to be added,2019_05_27
		auto_test_flag = 1;	//set auto test flag, for quick testing
	}
	
	//Allow system to be in quick test mode by pressing the manual testing button during power on
	if(KEY_MANUAL_TEST == 0)
	{
		clear_flash();	//to be added,2019_05_27
		auto_test_flag = 1;	//set auto test flag, for quick testing
	}	
	
	//to avoid possible mixup of manual testing function, button has to be released once
	ui_Key_Pressed_ms_counter = 0;	
	while(ui_Key_Pressed_ms_counter <= 100)
	{
		if(KEY_MANUAL_TEST == 0)
		{
			ui_Key_Pressed_ms_counter = 0;
		}
		else
		{
			ui_Key_Pressed_ms_counter++;
		}
	}
}

//function not yet tested
static void v_Key_Detection(void)
{
#ifdef EMERGENCY_LAMP_SINGLE_BATTERY	
	{
		//Key pressed
		if(KEY_MANUAL_TEST == 0)
		{
			ui_Key_Pressed_ms_counter++;
			if(ui_Key_Pressed_ms_counter >= 10000)
			{
				ui_Key_Pressed_ms_counter = 10000;		//Max 10 seconds count
			}
		}
		else
		{
			if ((ui_Key_Pressed_ms_counter >= 5000) && (ui_Key_Pressed_ms_counter <= 10000))	////longer than 5 seconds but less than 10 seconds pressed, set Manual_Duration_Test
			//if(ui_Key_Pressed_ms_counter >= 3000)	//>3 seconds pressed, set Manual_Duration_Test
			{
				F_Mannual_Duration_Test_Pending = 1;

			}
			else if ((ui_Key_Pressed_ms_counter >= 1000) && (ui_Key_Pressed_ms_counter <= 2000))	//pressed longer than 1 second but less than 2 seconds
			{
				//Get into functional test
				F_Functional_Test = 1;
			}
			
			ui_Key_Pressed_ms_counter = 0;
		}
	}
#endif

#ifdef BT_MESH_FEATURE_IN
	{
		UINT16 temph = 0;
//		temph = Bandgap_Calibrate(KEY_AUTO_MANUAL_REF1_0_01V,KEY_AUTO_MANUAL_REF1_0_01V);
//		temph = temph + Bandgap_Calibrate(0,TOLERANCE_SENSING_0_01V);	//0.05V tolerance added
	}
#endif
	
}


static void v_KEY_AUTO_MANUAL_SENSE(void)
{
#ifdef BT_MESH_FEATURE_IN

	UINT16 temph = 0;
	
	//compare with average value of ref1 & ref2
	temph = (Bandgap_Calibrate(KEY_AUTO_MANUAL_REF1_1V,KEY_AUTO_MANUAL_REF1_0_01V) 
			+ Bandgap_Calibrate(KEY_AUTO_MANUAL_REF2_1V,KEY_AUTO_MANUAL_REF2_0_01V));
	temph = (temph >> 1);	
	if(KEY_Auto_Manual_Value <= temph)
	{
		if((KEY_AUTO_TEST_STATUS == ON) && (KEY_MANUAL_TEST_STATUS == ON))
		{
			
		}
		else
		{
			if((KEY_AUTO_TEST_STATUS_TMP == ON) && (KEY_MANUAL_TEST_STATUS_TMP == ON))
			{
				KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER ++;
				if(KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER >= KEY_SENSE_FILTER)
				{
					KEY_AUTO_TEST_STATUS = ON;
					KEY_MANUAL_TEST_STATUS = ON;
					KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER = 0;
				}
			}
			else
			{
				KEY_AUTO_TEST_STATUS_TMP = ON;
				KEY_MANUAL_TEST_STATUS_TMP = ON;
				KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER = 0;
			}
		}
		
		return;
	}

	//compare with average value of ref2 & ref3
	temph = (Bandgap_Calibrate(KEY_AUTO_MANUAL_REF2_1V,KEY_AUTO_MANUAL_REF2_0_01V) 
			+ Bandgap_Calibrate(KEY_AUTO_MANUAL_REF3_1V,KEY_AUTO_MANUAL_REF3_0_01V));
	temph = (temph >> 1);	
	if(KEY_Auto_Manual_Value <= temph)
	{
		if((KEY_AUTO_TEST_STATUS == ON) && (KEY_MANUAL_TEST_STATUS == OFF))
		{
			
		}
		else
		{
			if((KEY_AUTO_TEST_STATUS_TMP == ON) && (KEY_MANUAL_TEST_STATUS_TMP == OFF))
			{
				KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER ++;
				if(KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER >= KEY_SENSE_FILTER)
				{
					KEY_AUTO_TEST_STATUS = ON;
					KEY_MANUAL_TEST_STATUS = OFF;
					KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER = 0;
				}
			}
			else
			{
				KEY_AUTO_TEST_STATUS_TMP = ON;
				KEY_MANUAL_TEST_STATUS_TMP = OFF;
				KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER = 0;
			}
		}
		
		return;
	}


	//compare with average value of ref3 & 3.3 (full scale)
	temph = (Bandgap_Calibrate(KEY_AUTO_MANUAL_REF3_1V,KEY_AUTO_MANUAL_REF3_0_01V) 
			+ Bandgap_Calibrate(3,3));
	temph = (temph >> 1);	
	if(KEY_Auto_Manual_Value <= temph)
	{
		if((KEY_AUTO_TEST_STATUS == OFF) && (KEY_MANUAL_TEST_STATUS == ON))
		{
			
		}
		else
		{
			if((KEY_AUTO_TEST_STATUS_TMP == OFF) && (KEY_MANUAL_TEST_STATUS_TMP == ON))
			{
				KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER ++;
				if(KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER >= KEY_SENSE_FILTER)
				{
					KEY_AUTO_TEST_STATUS = OFF;
					KEY_MANUAL_TEST_STATUS = ON;
					KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER = 0;
				}
			}
			else
			{
				KEY_AUTO_TEST_STATUS_TMP = OFF;
				KEY_MANUAL_TEST_STATUS_TMP = ON;
				KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER = 0;
			}
		}
		
		return;
	}
	//check if both OFF	
	else
	{
		if((KEY_AUTO_TEST_STATUS == OFF) && (KEY_MANUAL_TEST_STATUS == OFF))
		{
			
		}
		else
		{
			if((KEY_AUTO_TEST_STATUS_TMP == OFF) && (KEY_MANUAL_TEST_STATUS_TMP == OFF))
			{
				KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER ++;
				if(KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER >= KEY_SENSE_FILTER)
				{
					KEY_AUTO_TEST_STATUS = OFF;
					KEY_MANUAL_TEST_STATUS = OFF;
					KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER = 0;
				}
			}
			else
			{
				KEY_AUTO_TEST_STATUS_TMP = OFF;
				KEY_MANUAL_TEST_STATUS_TMP = OFF;
				KEY_AUTO_MANUAL_SENSING_FILTER_CUNTER = 0;
			}
		}
		
		return;		
	}

#endif
}


static UINT8 U8_SW_3_BITS_SENSE()
{
#ifdef BT_MESH_FEATURE_IN
	UINT16 temph = 0;
	
	//compare with average value of ref1 & ref2
	temph = (Bandgap_Calibrate(SW_3_BITS_REF1_1V,SW_3_BITS_REF1_0_01V) 
			+ Bandgap_Calibrate(SW_3_BITS_REF2_1V,SW_3_BITS_REF2_0_01V));
	temph = (temph >> 1);	
	if(SW_3_bits_Value <= temph)
	{
		if((SW_S1_STATUS == ON) && (SW_S2_STATUS == ON) && (SW_S3_STATUS == ON))
		{
			
		}
		else
		{
			if((SW_S1_STATUS_TMP == ON) && (SW_S2_STATUS_TMP == ON) && (SW_S3_STATUS_TMP == ON))
			{
				SW_3_BITS_SENSING_FILTER_CUNTER ++;
				if(SW_3_BITS_SENSING_FILTER_CUNTER >= KEY_SENSE_FILTER)
				{
					SW_S1_STATUS = ON;
					SW_S2_STATUS = ON;
					SW_S3_STATUS = ON;
					SW_3_BITS_SENSING_FILTER_CUNTER = 0;
				}
			}
			else
			{
				SW_S1_STATUS_TMP = ON;
				SW_S2_STATUS_TMP = ON;
				SW_S3_STATUS_TMP = ON;
				SW_3_BITS_SENSING_FILTER_CUNTER = 0;
			}
		}
		
		return	0x07;	//BT Group -7
	}

	//compare with average value of ref2 & ref3
	temph = (Bandgap_Calibrate(SW_3_BITS_REF2_1V,SW_3_BITS_REF2_0_01V) 
			+ Bandgap_Calibrate(SW_3_BITS_REF3_1V,SW_3_BITS_REF3_0_01V));
	temph = (temph >> 1);	
	if(SW_3_bits_Value <= temph)
	{
		if((SW_S1_STATUS == ON) && (SW_S2_STATUS == ON) && (SW_S3_STATUS == OFF))
		{
			
		}
		else
		{
			if((SW_S1_STATUS_TMP == ON) && (SW_S2_STATUS_TMP == ON) && (SW_S3_STATUS_TMP == OFF))
			{
				SW_3_BITS_SENSING_FILTER_CUNTER ++;
				if(SW_3_BITS_SENSING_FILTER_CUNTER >= KEY_SENSE_FILTER)
				{
					SW_S1_STATUS = ON;
					SW_S2_STATUS = ON;
					SW_S3_STATUS = OFF;
					SW_3_BITS_SENSING_FILTER_CUNTER = 0;
				}
			}
			else
			{
				SW_S1_STATUS_TMP = ON;
				SW_S2_STATUS_TMP = ON;
				SW_S3_STATUS_TMP = OFF;
				SW_3_BITS_SENSING_FILTER_CUNTER = 0;
			}
		}
		
		return	0x06;	//BT Group -6
	}		
	
	//compare with average value of ref3 & ref4
	temph = (Bandgap_Calibrate(SW_3_BITS_REF3_1V,SW_3_BITS_REF3_0_01V) 
			+ Bandgap_Calibrate(SW_3_BITS_REF4_1V,SW_3_BITS_REF4_0_01V));
	temph = (temph >> 1);	
	if(SW_3_bits_Value <= temph)
	{
		if((SW_S1_STATUS == ON) && (SW_S2_STATUS == OFF) && (SW_S3_STATUS == ON))
		{
			
		}
		else
		{
			if((SW_S1_STATUS_TMP == ON) && (SW_S2_STATUS_TMP == OFF) && (SW_S3_STATUS_TMP == ON))
			{
				SW_3_BITS_SENSING_FILTER_CUNTER ++;
				if(SW_3_BITS_SENSING_FILTER_CUNTER >= KEY_SENSE_FILTER)
				{
					SW_S1_STATUS = ON;
					SW_S2_STATUS = OFF;
					SW_S3_STATUS = ON;
					SW_3_BITS_SENSING_FILTER_CUNTER = 0;
				}
			}
			else
			{
				SW_S1_STATUS_TMP = ON;
				SW_S2_STATUS_TMP = OFF;
				SW_S3_STATUS_TMP = ON;
				SW_3_BITS_SENSING_FILTER_CUNTER = 0;
			}
		}
		
		return	0x05;	//BT Group -5
	}	
	
	//compare with average value of ref4 & ref5
	temph = (Bandgap_Calibrate(SW_3_BITS_REF4_1V,SW_3_BITS_REF4_0_01V) 
			+ Bandgap_Calibrate(SW_3_BITS_REF5_1V,SW_3_BITS_REF5_0_01V));
	temph = (temph >> 1);	
	if(SW_3_bits_Value <= temph)
	{
		if((SW_S1_STATUS == ON) && (SW_S2_STATUS == OFF) && (SW_S3_STATUS == OFF))
		{
			
		}
		else
		{
			if((SW_S1_STATUS_TMP == ON) && (SW_S2_STATUS_TMP == OFF) && (SW_S3_STATUS_TMP == OFF))
			{
				SW_3_BITS_SENSING_FILTER_CUNTER ++;
				if(SW_3_BITS_SENSING_FILTER_CUNTER >= KEY_SENSE_FILTER)
				{
					SW_S1_STATUS = ON;
					SW_S2_STATUS = OFF;
					SW_S3_STATUS = OFF;
					SW_3_BITS_SENSING_FILTER_CUNTER = 0;
				}
			}
			else
			{
				SW_S1_STATUS_TMP = ON;
				SW_S2_STATUS_TMP = OFF;
				SW_S3_STATUS_TMP = OFF;
				SW_3_BITS_SENSING_FILTER_CUNTER = 0;
			}
		}
		
		return	0x04;	//BT Group -4
	}	
	
	//compare with average value of ref5 & ref6
	temph = (Bandgap_Calibrate(SW_3_BITS_REF5_1V,SW_3_BITS_REF5_0_01V) 
			+ Bandgap_Calibrate(SW_3_BITS_REF6_1V,SW_3_BITS_REF6_0_01V));
	temph = (temph >> 1);	
	if(SW_3_bits_Value <= temph)
	{
		if((SW_S1_STATUS == OFF) && (SW_S2_STATUS == ON) && (SW_S3_STATUS == ON))
		{
			
		}
		else
		{
			if((SW_S1_STATUS_TMP == OFF) && (SW_S2_STATUS_TMP == ON) && (SW_S3_STATUS_TMP == ON))
			{
				SW_3_BITS_SENSING_FILTER_CUNTER ++;
				if(SW_3_BITS_SENSING_FILTER_CUNTER >= KEY_SENSE_FILTER)
				{
					SW_S1_STATUS = OFF;
					SW_S2_STATUS = ON;
					SW_S3_STATUS = ON;
					SW_3_BITS_SENSING_FILTER_CUNTER = 0;
				}
			}
			else
			{
				SW_S1_STATUS_TMP = OFF;
				SW_S2_STATUS_TMP = ON;
				SW_S3_STATUS_TMP = ON;
				SW_3_BITS_SENSING_FILTER_CUNTER = 0;
			}
		}
		
		return	0x03;	//BT Group -3
	}		
	
	//compare with average value of ref6 & ref7
	temph = (Bandgap_Calibrate(SW_3_BITS_REF6_1V,SW_3_BITS_REF6_0_01V) 
			+ Bandgap_Calibrate(SW_3_BITS_REF7_1V,SW_3_BITS_REF7_0_01V));
	temph = (temph >> 1);	
	if(SW_3_bits_Value <= temph)
	{
		if((SW_S1_STATUS == OFF) && (SW_S2_STATUS == ON) && (SW_S3_STATUS == OFF))
		{
			
		}
		else
		{
			if((SW_S1_STATUS_TMP == OFF) && (SW_S2_STATUS_TMP == ON) && (SW_S3_STATUS_TMP == OFF))
			{
				SW_3_BITS_SENSING_FILTER_CUNTER ++;
				if(SW_3_BITS_SENSING_FILTER_CUNTER >= KEY_SENSE_FILTER)
				{
					SW_S1_STATUS = OFF;
					SW_S2_STATUS = ON;
					SW_S3_STATUS = OFF;
					SW_3_BITS_SENSING_FILTER_CUNTER = 0;
				}
			}
			else
			{
				SW_S1_STATUS_TMP = OFF;
				SW_S2_STATUS_TMP = ON;
				SW_S3_STATUS_TMP = OFF;
				SW_3_BITS_SENSING_FILTER_CUNTER = 0;
			}
		}
		
		return	0x02;	//BT Group -2
	}		
	
	//compare with average value of ref7 & 3.3V
	temph = (Bandgap_Calibrate(SW_3_BITS_REF7_1V,SW_3_BITS_REF7_0_01V) 
			+ Bandgap_Calibrate(3,3));
	temph = (temph >> 1);	
	if(SW_3_bits_Value <= temph)
	{
		if((SW_S1_STATUS == OFF) && (SW_S2_STATUS == OFF) && (SW_S3_STATUS == ON))
		{
			
		}
		else
		{
			if((SW_S1_STATUS_TMP == OFF) && (SW_S2_STATUS_TMP == OFF) && (SW_S3_STATUS_TMP == ON))
			{
				SW_3_BITS_SENSING_FILTER_CUNTER ++;
				if(SW_3_BITS_SENSING_FILTER_CUNTER >= KEY_SENSE_FILTER)
				{
					SW_S1_STATUS = OFF;
					SW_S2_STATUS = OFF;
					SW_S3_STATUS = ON;
					SW_3_BITS_SENSING_FILTER_CUNTER = 0;
				}
			}
			else
			{
				SW_S1_STATUS_TMP = OFF;
				SW_S2_STATUS_TMP = OFF;
				SW_S3_STATUS_TMP = ON;
				SW_3_BITS_SENSING_FILTER_CUNTER = 0;
			}
		}
		
		return	0x01;	//BT Group -1
	}
	//check if all OFF
	else
	{
		if((SW_S1_STATUS == OFF) && (SW_S2_STATUS == OFF) && (SW_S3_STATUS == OFF))
		{
			
		}
		else
		{
			if((SW_S1_STATUS_TMP == OFF) && (SW_S2_STATUS_TMP == OFF) && (SW_S3_STATUS_TMP == OFF))
			{
				SW_3_BITS_SENSING_FILTER_CUNTER ++;
				if(SW_3_BITS_SENSING_FILTER_CUNTER >= KEY_SENSE_FILTER)
				{
					SW_S1_STATUS = OFF;
					SW_S2_STATUS = OFF;
					SW_S3_STATUS = OFF;
					SW_3_BITS_SENSING_FILTER_CUNTER = 0;
				}
			}
			else
			{
				SW_S1_STATUS_TMP = OFF;
				SW_S2_STATUS_TMP = OFF;
				SW_S3_STATUS_TMP = OFF;
				SW_3_BITS_SENSING_FILTER_CUNTER = 0;
			}
		}
		
		return	0x00;	//BT Group -0	
	}
#endif	
	return 0x00;
}

//This function block is not yet tested @2019_05_27, due to error on GPIO (P11)
static void v_AC_Mains_Detection(void)
{
	if(AC_Mains_state == OFF)
	{
		if(AC_MAIN_DETECT == OFF)	//AC OFF
		{
			uc_AC_Mains_State_ms_counter = 0;
		}
		else
		{
			uc_AC_Mains_State_ms_counter++;
			if(uc_AC_Mains_State_ms_counter >= 10)	//10ms filter
			{
				uc_AC_Mains_State_ms_counter = 10;
				AC_Mains_state = ON;
				
				LAMP_CTL = OFF;	//Turn off Q6				
			}
		}
	}
	
	if(AC_Mains_state == ON)
	{
		if(AC_MAIN_DETECT == OFF)	//AC OFF
		{
			uc_AC_Mains_State_ms_counter++;
			if(uc_AC_Mains_State_ms_counter >= 10)	//10ms filter
			{
				uc_AC_Mains_State_ms_counter = 10;
				AC_Mains_state = OFF;

				LAMP_CTL = ON;	//Turn ON Q6				
			}			
		}
		else
		{
			uc_AC_Mains_State_ms_counter = 0;
		}
	}	
}

static void v_Battery_Charging_Operation(void)
{
	switch (uc_Charging_Status)
	{
		case ON:
			if(U8_Battery_Discharge_ticks_ms <= Battery_NC_Check_Discharge_Ticks_ms)
			{
				CHARGE_ENABLE = OFF;
			}
			else
			{
				if(Battery_Fully_Charged == 0)
				{
					CHARGE_ENABLE = ON;
				}
				else
				{
					CHARGE_ENABLE = OFF;
				}
			}
		break;
		
		case OFF:
			CHARGE_ENABLE = OFF;
		break;
		
		default:
			CHARGE_ENABLE = ON;
		break;
	}
}

static void v_LED_Operation(void)
{
	//GPIO_WriteReverse(LED_RED_PORT, LED_RED_PIN);
	#if 1
	ui_ms_counter++;
	if(ui_ms_counter>=1000)
	{
		ui_ms_counter = 0;
	}	
	
	//Green LED operation
	switch(led_green_state)
	{
		case OFF:
			GREEN_LED_PIN_LOW;
		break;
		case ON:
			GREEN_LED_PIN_HIGH;
		break;		
		case Flashing_Slow:
			if((ui_ms_counter % 500) == 0)
			{
				GREEN_LED_PIN_TOGGLE;
			}
		break;
		case Flashing_Medium:
			if((ui_ms_counter % 250) == 0)
			{
				GREEN_LED_PIN_TOGGLE;
			}
		break;
		case Flashing_Fast:
			if((ui_ms_counter % 125) == 0)
			{
				GREEN_LED_PIN_TOGGLE;
			}
		break;
		default:
			GREEN_LED_PIN_LOW;
		break;				
	}

	//Red LED operation
	switch(led_red_state)
	{
		case OFF:
			RED_LED_PIN_LOW;
		break;
		case ON:
			RED_LED_PIN_HIGH;
		break;		
		case Flashing_Slow:
			if((ui_ms_counter % 500) == 0)
			{
				RED_LED_PIN_TOGGLE;
			}
		break;
		case Flashing_Medium:
			if((ui_ms_counter % 250) == 0)
			{
				RED_LED_PIN_TOGGLE;
			}
		break;
		case Flashing_Fast:
			if((ui_ms_counter % 125) == 0)
			{
				RED_LED_PIN_TOGGLE;
			}
		break;
		default:
			RED_LED_PIN_LOW;
		break;				
	}	
#endif
}

static void v_State_Machine_Processing(void)
{
	switch(uc_System_State)
	{
		case System_Reset:
		{
			F_Duration_Test_Pending = 1;
			
			//LED states
			led_green_state = OFF;
			led_red_state = OFF;
			
			uc_Charging_Status = ON;		//Charging allowed
			LAMP_CTL = OFF;	//Turn Off Q6
			LAMP_EN = OFF;		//Disable LAMP */			
			
			//State transition
			if(AC_Mains_state == ON)
			{
				uc_System_State = Reset_Charging_Time;		
			}
		}	
		break;

		case AC_Charging:
		{
			//LED states
			if(F_Mannual_Duration_Test_Pending == 1)
			{
				led_green_state = Flashing_Slow;
				led_red_state = OFF;	//Overwrite RED LED
			}
			else if(led_red_state == OFF)	//fault indication overwrites others...
			{
				if(F_Duration_Test_Pending == 1)
				{
					led_green_state = Flashing_Slow;
				}
			}
			else
			{
				led_green_state = OFF;
			} 


			
/* 			GPIO_WriteHigh(CHG_ENABLE_PORT, CHG_ENABLE_PIN);	//Charging allowed
			GPIO_WriteLow(LAMP_CONTROL_PORT, LAMP_CONTROL_PIN);	//Turn Off Q6
			GPIO_WriteLow(LAMP_EN_PORT, LAMP_EN_PIN);		//Disable LAMP */
			uc_Charging_Status = ON;		//Charging allowed
			LAMP_CTL = OFF;	//Turn Off Q6
			LAMP_EN = OFF;		//Disable LAMP */
			F_OVP_Rising = 0;
			F_OVP_Falling = 0;
			
			v_Key_Detection();	//Manual testing check up

			//to Emergency operation
			if(AC_Mains_state == OFF)
			{
				uc_System_State = Emergency_Operation;
				
				uc_Emergency_Time_Hour = 0;
				uc_Emergency_Time_Minute = 0;
				uc_Emergency_Time_Second = 0;	

				uc_Duration_Test_Time_Second = 0;
				uc_Duration_Test_Time_Minute = 0;
				uc_Duration_Test_Time_Hour = 0;
				uc_Duration_Test_Done = 0;		
				Battery_bad	= 0;
				lamp_bad = 0;	
				F_OVP_Rising = 0;
				F_OVP_Falling = 0;
				
				led_red_state_before_Emergency = led_red_state;
			}	
			
			//Normal operation
			//to Duration_Test
			if((F_Charging_Full == 1) && ((F_Duration_Test_Pending == 1)||(F_Mannual_Duration_Test_Pending == 1)))
			{
				uc_System_State = Duration_Test;
				F_Duration_Test_Pending = 0;
				F_Mannual_Duration_Test_Pending = 0;
				
				uc_Duration_Test_Time_Second = 0;
				uc_Duration_Test_Time_Minute = 0;
				uc_Duration_Test_Time_Hour = 0;
				uc_Duration_Test_Done = 0;		
				Battery_bad	= 0;
				lamp_bad = 0;	
				F_OVP_Rising = 0;
				F_OVP_Falling = 0;				
			}
			if((F_Charging_Full == 1) && (F_Duration_Test == 1))
			{
				uc_System_State = Duration_Test;
				F_Duration_Test = 0;
				
				uc_Duration_Test_Time_Second = 0;
				uc_Duration_Test_Time_Minute = 0;
				uc_Duration_Test_Time_Hour = 0;
				uc_Duration_Test_Done = 0;		
				Battery_bad	= 0;
				lamp_bad = 0;	
				F_OVP_Rising = 0;
				F_OVP_Falling = 0;				
			}			
			
			//to Functional_Test
			if(F_Functional_Test == 1)
			{
				F_Functional_Test = 0;
				uc_System_State = Functional_Test;
	
				uc_Functional_Test_Time_Second = 0;
				uc_Functional_Test_Done = 0;
				Battery_bad	= 0;
				lamp_bad = 0;
				F_OVP_Rising = 0;
				F_OVP_Falling = 0;				
				
				led_green_state_before_test = led_green_state;
				led_red_state_before_test = led_red_state;
				//led_red_state_before_test = OFF;
			}
			
			
			//to Battery_Fault
			if(uc_Battery_Fault_Flag == 1)
			{
				uc_System_State = Battery_Fault;
			}
		}
		break;		
		
		case Battery_Fault:
		{
			led_green_state = OFF;
			led_red_state = ON;
			
/* 			GPIO_WriteHigh(CHG_ENABLE_PORT, CHG_ENABLE_PIN);	//Charging allowed
			GPIO_WriteLow(LAMP_EN_PORT, LAMP_EN_PIN);		//Disable LAMP */
			uc_Charging_Status = ON;	//Charging allowed
			LAMP_EN = OFF;		//Disable LAMP
			F_OVP_Rising = 0;
			F_OVP_Falling = 0;			
			if(AC_Mains_state == ON)
			{
				LAMP_CTL = OFF;	//Turn Off Q6
			}
			else
			{
				LAMP_CTL = ON;	//Turn ON Q6
			}
		}
		break;	

		case Functional_Test:
		{
			led_green_state = Flashing_Fast;
			led_red_state = OFF;

/* 			GPIO_WriteLow(CHG_ENABLE_PORT, CHG_ENABLE_PIN);	//charging OFF
			GPIO_WriteHigh(LAMP_CONTROL_PORT, LAMP_CONTROL_PIN);	//Turn ON Q6
			GPIO_WriteHigh(LAMP_EN_PORT, LAMP_EN_PIN);		//Enable LAMP  */		
			uc_Charging_Status = OFF;	//charging OFF
			LAMP_CTL = ON;	//Turn ON Q6
			LAMP_EN = ON;		//Enable LAMP 
			//to Emergency operation
			if(AC_Mains_state == OFF)
			{
				uc_System_State = Emergency_Operation;
				
				uc_Emergency_Time_Hour = 0;
				uc_Emergency_Time_Minute = 0;
				uc_Emergency_Time_Second = 0;		

				uc_Duration_Test_Time_Second = 0;
				uc_Duration_Test_Time_Minute = 0;
				uc_Duration_Test_Time_Hour = 0;
				uc_Duration_Test_Done = 0;		
				Battery_bad	= 0;
				lamp_bad = 0;	
				F_OVP_Rising = 0;
				F_OVP_Falling = 0;				
			}				
			
			if(uc_Functional_Test_Done == 1)
			{
				uc_System_State = AC_Charging;	
				led_green_state = led_green_state_before_test;
				led_red_state = led_red_state_before_test;
			}
			
			//to Battery_Life_Fault
			if(Battery_bad == 1)
			{
				uc_System_State = Battery_Life_Fault;
			}
			
			//to Lamp_Fault
			if(lamp_bad == 1)
			{
				uc_System_State = Lamp_Fault;
			}
		}
		break;	

		case Duration_Test:
		{
			led_green_state = Flashing_Medium;
			led_red_state = OFF;

/* 			GPIO_WriteLow(CHG_ENABLE_PORT, CHG_ENABLE_PIN);	//charging OFF
			GPIO_WriteHigh(LAMP_CONTROL_PORT, LAMP_CONTROL_PIN);	//Turn ON Q6
			GPIO_WriteHigh(LAMP_EN_PORT, LAMP_EN_PIN);		//Enable LAMP 	 */	
			uc_Charging_Status = OFF;	//charging OFF
			LAMP_CTL = ON;	//Turn ON Q6
			LAMP_EN = ON;		//Enable LAMP 
			
			if(uc_Duration_Test_Done == 1)
			{
				uc_System_State = Reset_Charging_Time;		
				led_green_state = ON;
				led_red_state = OFF;					
			}

			//to Emergency operation
			if(AC_Mains_state == OFF)
			{
				uc_System_State = Emergency_Operation;
				F_Duration_Test_Pending = 1;
				
				uc_Emergency_Time_Hour = 0;
				uc_Emergency_Time_Minute = 0;
				uc_Emergency_Time_Second = 0;		

				uc_Duration_Test_Time_Second = 0;
				uc_Duration_Test_Time_Minute = 0;
				uc_Duration_Test_Time_Hour = 0;
				uc_Duration_Test_Done = 0;		
				Battery_bad	= 0;
				lamp_bad = 0;	
				F_OVP_Rising = 0;
				F_OVP_Falling = 0;				
			}			
			
			//to Battery_Life_Fault
			if(Battery_bad == 1)
			{
				uc_System_State = Battery_Life_Fault;
			}
			
			//to Lamp_Fault
			if(lamp_bad == 1)
			{
				uc_System_State = Lamp_Fault;
			}			
		}
		break;	

		case Battery_Life_Fault:
		{
			led_green_state = OFF;
			led_red_state = Flashing_Medium;
			
/* 			GPIO_WriteHigh(CHG_ENABLE_PORT, CHG_ENABLE_PIN);	//Charging allowed
			GPIO_WriteLow(LAMP_EN_PORT, LAMP_EN_PIN);		//Disable LAMP	
			GPIO_WriteHigh(LAMP_CONTROL_PORT, LAMP_CONTROL_PIN);	//Turn ON Q6 */
			uc_Charging_Status = ON;		//Charging allowed
			LAMP_EN = OFF;		//Disable LAMP	
			LAMP_CTL = ON;	//Turn ON Q6

			F_OVP_Rising = 0;
			F_OVP_Falling = 0;			
			
			//to AC_Charging,reset AC_Charging time
			if(AC_Mains_state == ON)
			{
				uc_System_State = Reset_Charging_Time;			
			}			
		}
		break;	

		case Lamp_Fault:
		{
			led_green_state = OFF;
			led_red_state = Flashing_Fast;	

/* 			GPIO_WriteHigh(CHG_ENABLE_PORT, CHG_ENABLE_PIN);	//Charging allowed
			GPIO_WriteLow(LAMP_EN_PORT, LAMP_EN_PIN);		//Disable LAMP	
			GPIO_WriteHigh(LAMP_CONTROL_PORT, LAMP_CONTROL_PIN);	//Turn ON Q6 */
			uc_Charging_Status = ON;	//Charging allowed
			LAMP_EN = OFF;		//Disable LAMP	
			LAMP_CTL = ON;	//Turn ON Q6
			
			F_OVP_Rising = 0;
			F_OVP_Falling = 0;			

			//to AC_Charging,reset AC_Charging time
			if(AC_Mains_state == ON)
			{
				uc_System_State = Reset_Charging_Time;				
			}
		}
		break;
		
		case Emergency_Operation:
		{
			led_green_state = OFF;
			led_red_state = OFF;
	

/* 			GPIO_WriteLow(CHG_ENABLE_PORT, CHG_ENABLE_PIN);	//Charging OFF
			GPIO_WriteHigh(LAMP_CONTROL_PORT, LAMP_CONTROL_PIN);	//Turn ON Q6
			GPIO_WriteHigh(LAMP_EN_PORT, LAMP_EN_PIN);		//Enable LAMP	 */
			uc_Charging_Status = OFF;	//Charging OFF
			LAMP_CTL = ON;	//Turn ON Q6
			LAMP_EN = ON;		//Enable LAMP	

			//to Battery_Life_Fault
			if((Battery_bad == 1) && (uc_Duration_Test_Done==0))
			{
				uc_System_State = Battery_Life_Fault;
			}
			
			//to Lamp_Fault
			if(lamp_bad == 1)
			{
				uc_System_State = Lamp_Fault;
			}	

			//to AC_Charging,reset AC_Charging time
			if(AC_Mains_state == ON)
			{
				uc_System_State = Reset_Charging_Time;		
				
				
				if(led_red_state_before_Emergency == OFF)
				{
					led_green_state = ON;
					led_red_state = OFF;	
				}	
				else
				{
					led_red_state = led_red_state_before_Emergency;
				}
			}			
		}
		break;	

		case Reset_Charging_Time:
		{
				uc_System_State = AC_Charging;
				
				uc_AC_Charging_Time_Second = 0;
				uc_AC_Charging_Time_Minute = 0;
				uc_AC_Charging_Time_Hour = 0;	

				if(F_Duration_Test_Pending == 1)
				{
					uc_AC_Charging_Target_Hour = (24 + rand()%24);
					uc_AC_Charging_Target_Minute = rand()%60;
					uc_AC_Charging_Target_Second = rand()%60;
					
					if(auto_test_flag == 1)
					{
					uc_AC_Charging_Target_Hour = 0;
					uc_AC_Charging_Target_Minute = 0;
					uc_AC_Charging_Target_Second = (24 + rand()%24);		
					}
				}
				else
				{
					uc_AC_Charging_Target_Hour = 24;
					uc_AC_Charging_Target_Minute = 0;
					uc_AC_Charging_Target_Second = 0;
					
					if(auto_test_flag == 1)
					{
					uc_AC_Charging_Target_Hour = 0;
					uc_AC_Charging_Target_Minute = 0;
					uc_AC_Charging_Target_Second = 24;		
					}					
				}
				
				F_Charging_Full = 0;
				
				F_Functional_Test = 0;	//over-write functional test
		}
		break;
		default:
		break;	
	}	
}

/********************************************************************
*Prototype   : void Load_Test_Proc(void)
*Arguments   : void
*Return      : void
*Description : Load_Test_Proc 
			check load OK: 2.4V, bad:3.4V
********************************************************************/
static void Load_Test_Proc(void)
{
	UINT16 temph = 0;

//	temph = Bandgap_Calibrate(LOAD_OVP_1V,LOAD_OVP_0_01V);	
	//if(Load_Value > temph)
	if(Load_Value_mV > LOAD_OVP_mV)	
	{
		lamp_bad = 1;				
	}
	
/* 	if(Load_Value > temph) // 27:34
	{
		F_OVP_Rising = 1;				
	}
	
	temph = Bandgap_Calibrate(LOAD_FALLING_1V,LOAD_FALLING_0_01V);
	if((Load_Value < temph) && (F_OVP_Rising == 1))
	{
		lamp_bad = 1;//lamp bad	
	} */
	
	
	//Two LED branches
	//With updated circuitry,Load_Value = 3~6 when the main led branch is open
	//detect this only when lamp = ON
	if((uc_System_State == Functional_Test)
		|| (uc_System_State == Duration_Test)
			|| (uc_System_State == Emergency_Operation))
	{
		if(Load_Value_mV < LOAD_OPEN_mV)		
		{
			uc_Lamp_bad_counter ++;
			if(uc_Lamp_bad_counter >= 25)
			{
				lamp_bad = 1;//lamp bad	
			}
		}
		else
		{
			uc_Lamp_bad_counter = 0;
		}
	}

}
/********************************************************************
*Prototype   : void Battery_Test_Proc(void)
*Arguments   : void
*Return      : void
*Description : Battery_Test_Proc
			check battery OK:3.6V,bad:3.1V
********************************************************************/
static void Battery_Test_Proc(void)
{
	UINT16 temph = 0;
	
	//do battery NC check
	if((U8_Battery_Discharge_ticks_ms >= 2) && (U8_Battery_Discharge_ticks_ms <= Battery_NC_Check_Discharge_Ticks_ms))
	{
		//battery nc check
		if((Battery_Value_mV_bkup > Battery_Value_mV) && ((Battery_Value_mV_bkup - Battery_Value_mV) >= BATTERY_NC_mV))
		{
			uc_Battery_Fault_Flag = 1;			
		}	
		#ifdef DEBUG_PRINTF_LOG_EN
			printf("Bat vol bkup @MCU = %d\r\n",Battery_Value_mV_bkup);
			printf("Bat vol @MCU = %d\r\n",Battery_Value_mV);
		#endif
	}
	//do battery voltag too low check	
	else if (U8_Battery_Discharge_ticks_ms >= (Battery_NC_Check_Discharge_Ticks_ms + 2))
	{
		//UVP check
		if(Battery_Value_mV < (BATTERY_UVP_mV - BATTERY_DIS_INTERNAL_LOSS_mV))
		{  
			Battery_bad = 1;//battery bad
		}
		
		//OVP check
		if((Battery_Value_mV >= (BATTERY_OVP_mV + BATTERY_CHAG_INTERNAL_LOSS_mV)) && (Battery_Fully_Charged == 0))
		{  
			Battery_Fully_Charged = 1;//battery full,no charge needed
		}
		if((Battery_Value_mV < BATTERY_OVP_mV) && (Battery_Fully_Charged == 1))
		{  
			Battery_Fully_Charged = 0;//battery not full,charge needed
		}		
		
		//update backup value only during charging state
		if(Battery_Fully_Charged == 0)
		{
			Battery_Value_mV_bkup = Battery_Value_mV;
		}
		else
		{
			Battery_Value_mV_bkup = BATTERY_OVP_mV;
		}
		
		U8_Battery_Discharge_ticks_ms = Battery_NC_Check_Discharge_Ticks_ms + 2;
	}
}




static void v_ADC_process(void)
{
	//Bandgap channel processing
	if((ADCCON0 & 0x0F) == 0x08)
	{
		if(ADC_index >= 10)	//skip the first 10 sampling result, for accuracy
		{
			ADC_Value_Array[ADC_index - 10] = ((ADCRH<<4) + ADCRL);
			//ADC_Value_Array[ADC_index - 200] = 0x01;
			
			if(ADC_Value_Array[ADC_index-10]>ADC_Value_Max)
			{
				ADC_Value_Max = ADC_Value_Array[ADC_index-10];
			}
			if(ADC_Value_Array[ADC_index-10]<ADC_Value_Min)
			{
				ADC_Value_Min = ADC_Value_Array[ADC_index-10];
			}
		}

		if(ADC_index >= 19)	//take the sampling value Nr10-19
		{			
			ADC_Value = 0;
			ADC_Value = (ADC_Value_Array[0]+ADC_Value_Array[1]+ADC_Value_Array[2]+ADC_Value_Array[3]+ADC_Value_Array[4]
				   +ADC_Value_Array[5]+ADC_Value_Array[6]+ADC_Value_Array[7]+ADC_Value_Array[8]+ADC_Value_Array[9]);
			ADC_Value = (ADC_Value - ADC_Value_Max);
			ADC_Value = (ADC_Value - ADC_Value_Min);
			Bandgap_Value = (ADC_Value>>3);
			ADC_Value_Max = 0;
			ADC_Value_Min = 0x3FFF;
			
			//enable channel VLAMP_ADC_CHANNEL_NR
			{
				#if (VLAMP_ADC_CHANNEL_NR == 1)
					Enable_ADC_AIN1;	//Channel 1 selected
				#elif (VLAMP_ADC_CHANNEL_NR == 2)
					Enable_ADC_AIN2;	//Channel 2 selected
				#elif (VLAMP_ADC_CHANNEL_NR == 3)
					Enable_ADC_AIN3;	//Channel 3 selected
				#elif (VLAMP_ADC_CHANNEL_NR == 4)
					Enable_ADC_AIN4;	//Channel 4 selected
				#elif (VLAMP_ADC_CHANNEL_NR == 5)
					Enable_ADC_AIN5;	//Channel 5 selected
				#elif (VLAMP_ADC_CHANNEL_NR == 6)
					Enable_ADC_AIN6;	//Channel 6 selected
				#elif (VLAMP_ADC_CHANNEL_NR == 7)
					Enable_ADC_AIN7;	//Channel 7 selected
				#endif
			}
			
			ADC_index = 0;
		}
		else
		{		
			ADC_index++;
		}
		
		set_ADCS;	//Start ADC
	}
	
	//channel VLAMP_ADC_CHANNEL_NR processing;Load voltage
	else if((ADCCON0 & 0x0F) == VLAMP_ADC_CHANNEL_NR)
	{
		if(ADC_index >= 10)
		{			
			ADC_Value = 0;
			ADC_Value = (ADC_Value_Array[0]+ADC_Value_Array[1]+ADC_Value_Array[2]+ADC_Value_Array[3]+ADC_Value_Array[4]
				   +ADC_Value_Array[5]+ADC_Value_Array[6]+ADC_Value_Array[7]+ADC_Value_Array[8]+ADC_Value_Array[9]);
			ADC_Value = (ADC_Value - ADC_Value_Max);
			ADC_Value = (ADC_Value - ADC_Value_Min);
			Load_Value = (ADC_Value>>3);
			ADC_Value_Max = 0;
			ADC_Value_Min = 0x3FFF;
			
			Load_Value_mV = ADC_to_AIN_mV(Load_Value); 
			//enable channel VBAT_ADC_CHANNEL_NR
			{
				#if (VBAT_ADC_CHANNEL_NR == 1)
					Enable_ADC_AIN1;	//Channel 1 selected
				#elif (VBAT_ADC_CHANNEL_NR == 2)
					Enable_ADC_AIN2;	//Channel 2 selected
				#elif (VBAT_ADC_CHANNEL_NR == 3)
					Enable_ADC_AIN3;	//Channel 3 selected
				#elif (VBAT_ADC_CHANNEL_NR == 4)
					Enable_ADC_AIN4;	//Channel 4 selected
				#elif (VBAT_ADC_CHANNEL_NR == 5)
					Enable_ADC_AIN5;	//Channel 5 selected
				#elif (VBAT_ADC_CHANNEL_NR == 6)
					Enable_ADC_AIN6;	//Channel 6 selected
				#elif (VBAT_ADC_CHANNEL_NR == 7)
					Enable_ADC_AIN7;	//Channel 7 selected
				#endif
			}
			
			ADC_index = 0;
			
		}
		else
		{		
			ADC_Value_Array[ADC_index] = ((ADCRH<<4) + ADCRL);
			
			if(ADC_Value_Array[ADC_index]>ADC_Value_Max)
			ADC_Value_Max = ADC_Value_Array[ADC_index];
			if(ADC_Value_Array[ADC_index]<ADC_Value_Min)
			ADC_Value_Min = ADC_Value_Array[ADC_index];	
	
	
			ADC_index++;
		}
		
		set_ADCS;	//Start ADC
		

	}	
	
	//channel VBAT_ADC_CHANNEL_NR processing;Vbattery voltage
	else if((ADCCON0 & 0x0F) == VBAT_ADC_CHANNEL_NR)
	{
		if(ADC_index >= 10)
		{			
			ADC_Value = 0;
			ADC_Value = (ADC_Value_Array[0]+ADC_Value_Array[1]+ADC_Value_Array[2]+ADC_Value_Array[3]+ADC_Value_Array[4]
				   +ADC_Value_Array[5]+ADC_Value_Array[6]+ADC_Value_Array[7]+ADC_Value_Array[8]+ADC_Value_Array[9]);
			ADC_Value = (ADC_Value - ADC_Value_Max);
			ADC_Value = (ADC_Value - ADC_Value_Min);
			Battery_Value = (ADC_Value>>3);
			ADC_Value_Max = 0;
			ADC_Value_Min = 0x3FFF;
			
			
			ADC_index = 0;
			
			
			Battery_Value_mV = ADC_to_AIN_mV(Battery_Value);
			
			
			#ifdef BT_MESH_FEATURE_IN
				//enable channel SW_DET_ADC_CHANNEL_NR
				{
					#if (SW_DET_ADC_CHANNEL_NR == 1)
						Enable_ADC_AIN1;	//Channel 1 selected
					#elif (SW_DET_ADC_CHANNEL_NR == 2)
						Enable_ADC_AIN2;	//Channel 2 selected
					#elif (SW_DET_ADC_CHANNEL_NR == 3)
						Enable_ADC_AIN3;	//Channel 3 selected
					#elif (SW_DET_ADC_CHANNEL_NR == 4)
						Enable_ADC_AIN4;	//Channel 4 selected
					#elif (SW_DET_ADC_CHANNEL_NR == 5)
						Enable_ADC_AIN5;	//Channel 5 selected
					#elif (SW_DET_ADC_CHANNEL_NR == 6)
						Enable_ADC_AIN6;	//Channel 6 selected
					#elif (SW_DET_ADC_CHANNEL_NR == 7)
						Enable_ADC_AIN7;	//Channel 7 selected
					#endif
				}			
			#else	
				Enable_ADC_BandGap;	//Channel 8 (band-gap) selected	
				F_ADC_All_Channels_Complete = 1;
			#endif
		}
		else
		{		
			ADC_Value_Array[ADC_index] = ((ADCRH<<4) + ADCRL);
			
			if(ADC_Value_Array[ADC_index]>ADC_Value_Max)
			ADC_Value_Max = ADC_Value_Array[ADC_index];
			if(ADC_Value_Array[ADC_index]<ADC_Value_Min)
			ADC_Value_Min = ADC_Value_Array[ADC_index];	
	
	
			ADC_index++;
		}
		
		set_ADCS;	//Start ADC
	}
	
	//channel SW_DET_ADC_CHANNEL_NR processing;3 bits switch buttons
#ifdef 	BT_MESH_FEATURE_IN
	else if((ADCCON0 & 0x0F) == SW_DET_ADC_CHANNEL_NR)
#else
	else if(0)
#endif		
	{
		if(ADC_index >= 10)
		{			
			ADC_Value = 0;
			ADC_Value = (ADC_Value_Array[0]+ADC_Value_Array[1]+ADC_Value_Array[2]+ADC_Value_Array[3]+ADC_Value_Array[4]
				   +ADC_Value_Array[5]+ADC_Value_Array[6]+ADC_Value_Array[7]+ADC_Value_Array[8]+ADC_Value_Array[9]);
			ADC_Value = (ADC_Value - ADC_Value_Max);
			ADC_Value = (ADC_Value - ADC_Value_Min);
			SW_3_bits_Value = (ADC_Value>>3);
			ADC_Value_Max = 0;
			ADC_Value_Min = 0x3FFF;
			
			ADC_index = 0;
			
			#ifdef BT_MESH_FEATURE_IN
				//enable channel KEY_AUTO_MANUAL_ADC_CHANNEL_NR
				{
					#if (KEY_AUTO_MANUAL_ADC_CHANNEL_NR == 1)
						Enable_ADC_AIN1;	//Channel 1 selected
					#elif (KEY_AUTO_MANUAL_ADC_CHANNEL_NR == 2)
						Enable_ADC_AIN2;	//Channel 2 selected
					#elif (KEY_AUTO_MANUAL_ADC_CHANNEL_NR == 3)
						Enable_ADC_AIN3;	//Channel 3 selected
					#elif (KEY_AUTO_MANUAL_ADC_CHANNEL_NR == 4)
						Enable_ADC_AIN4;	//Channel 4 selected
					#elif (KEY_AUTO_MANUAL_ADC_CHANNEL_NR == 5)
						Enable_ADC_AIN5;	//Channel 5 selected
					#elif (KEY_AUTO_MANUAL_ADC_CHANNEL_NR == 6)
						Enable_ADC_AIN6;	//Channel 6 selected
					#elif (KEY_AUTO_MANUAL_ADC_CHANNEL_NR == 7)
						Enable_ADC_AIN7;	//Channel 7 selected
					#endif
				}			
			#else	
				Enable_ADC_BandGap;	//Channel 8 (band-gap) selected	
				F_ADC_All_Channels_Complete = 1;
			#endif
		}
		else
		{		
			ADC_Value_Array[ADC_index] = ((ADCRH<<4) + ADCRL);
			
			if(ADC_Value_Array[ADC_index]>ADC_Value_Max)
			ADC_Value_Max = ADC_Value_Array[ADC_index];
			if(ADC_Value_Array[ADC_index]<ADC_Value_Min)
			ADC_Value_Min = ADC_Value_Array[ADC_index];	
	
	
			ADC_index++;
		}
		
		set_ADCS;	//Start ADC
	}	

	//channel KEY_AUTO_MANUAL_ADC_CHANNEL_NR processing;Auto/Manual button switch detection
#ifdef 	BT_MESH_FEATURE_IN
	else if((ADCCON0 & 0x0F) == KEY_AUTO_MANUAL_ADC_CHANNEL_NR)
#else
	else if(0)
#endif			
	{
		if(ADC_index >= 10)
		{			
			ADC_Value = 0;
			ADC_Value = (ADC_Value_Array[0]+ADC_Value_Array[1]+ADC_Value_Array[2]+ADC_Value_Array[3]+ADC_Value_Array[4]
				   +ADC_Value_Array[5]+ADC_Value_Array[6]+ADC_Value_Array[7]+ADC_Value_Array[8]+ADC_Value_Array[9]);
			ADC_Value = (ADC_Value - ADC_Value_Max);
			ADC_Value = (ADC_Value - ADC_Value_Min);
			KEY_Auto_Manual_Value = (ADC_Value>>3);
			ADC_Value_Max = 0;
			ADC_Value_Min = 0x3FFF;
			
			ADC_index = 0;

			Enable_ADC_BandGap;	//Channel 8 (band-gap) selected	
			F_ADC_All_Channels_Complete = 1;

		}
		else
		{		
			ADC_Value_Array[ADC_index] = ((ADCRH<<4) + ADCRL);
			
			if(ADC_Value_Array[ADC_index]>ADC_Value_Max)
			ADC_Value_Max = ADC_Value_Array[ADC_index];
			if(ADC_Value_Array[ADC_index]<ADC_Value_Min)
			ADC_Value_Min = ADC_Value_Array[ADC_index];	
	
	
			ADC_index++;
		}
		
		set_ADCS;	//Start ADC
	}	
	
	else
	{
		Enable_ADC_BandGap;	
		set_ADCS;	//Start ADC conversion		
	}
}

// calibrate reference voltage with Bandgap result.
// Value_1V: reference voltage in 1V
// Value_0_01V: reference voltage in 0.01V
UINT16 Bandgap_Calibrate(UINT8 Value_1V,UINT8 Value_0_01V)
{
	UINT16 U16_temp_calibrate;
	
	U16_temp_calibrate = (Bandgap_Value/10);
	U16_temp_calibrate = U16_temp_calibrate * Value_0_01V;
	U16_temp_calibrate = (U16_temp_calibrate/10);
	
	U16_temp_calibrate = (U16_temp_calibrate + (Value_1V * Bandgap_Value));	//Bandgap_Value
	
	return U16_temp_calibrate;

}

//convert ADC value back to AIN, each bit stands for 0.001V
UINT16 ADC_to_AIN_mV(UINT16 Value_ADC)
{
	UINT16 xdata U16_TEMP_ADC_H,U16_TEMP_ADC_L,U16_TEMP_ADC1,U16_TEMP_ADC2,U16_TEMP_ADC3,U16_TEMP_ADC4;
	UINT16 xdata U16_MUL_AH,U16_MUL_AL,U16_MUL_BH,U16_MUL_BL;
	UINT8 xdata U8_i_ADC_AIN;
	
	
	//(Value_ADC*U16_Bandgap_Value);
	U16_MUL_AH = (Value_ADC >> 8);
	U16_MUL_AL = (Value_ADC & 0xFF);
	U16_MUL_BH = (U16_Bandgap_Value >> 8);
	U16_MUL_BL = (U16_Bandgap_Value & 0xFF);
	
	U16_TEMP_ADC1 = U16_MUL_AL * U16_MUL_BL;	//AL*BL
	U16_TEMP_ADC2 = U16_MUL_AH * U16_MUL_BL;	//AH*BL
	U16_TEMP_ADC3 = U16_MUL_AL * U16_MUL_BH;	//AL*BH
	U16_TEMP_ADC4 = U16_MUL_AH * U16_MUL_BH;	//AH*BH
	
	U16_TEMP_ADC_L = U16_TEMP_ADC1;
	U16_TEMP_ADC_H = U16_TEMP_ADC4;
	if(U16_TEMP_ADC_L > (0xFFFF - (U16_TEMP_ADC2 << 8)))	//overflow
	{
		U16_TEMP_ADC_H++;
	}
	U16_TEMP_ADC_L = U16_TEMP_ADC_L + (U16_TEMP_ADC2 << 8);
	U16_TEMP_ADC_H = U16_TEMP_ADC_H + (U16_TEMP_ADC2 >> 8);
	
	if(U16_TEMP_ADC_L > (0xFFFF - (U16_TEMP_ADC3 << 8)))	//overflow
	{
		U16_TEMP_ADC_H++;
	}
	U16_TEMP_ADC_L = U16_TEMP_ADC_L + (U16_TEMP_ADC3 << 8);
	U16_TEMP_ADC_H = U16_TEMP_ADC_H + (U16_TEMP_ADC3 >> 8);	
	
	//(Value_ADC*U16_Bandgap_Value)/Bandgap_Value
	U8_i_ADC_AIN = 0;
	U16_TEMP_ADC1 = 0;
	U16_TEMP_ADC2 = 0;
	U16_TEMP_ADC3 = 0;
	while (U8_i_ADC_AIN <= 15)
	{
		U16_TEMP_ADC1 = U16_TEMP_ADC_H/Bandgap_Value;
		U16_TEMP_ADC2 = (U16_TEMP_ADC1 << (16-U8_i_ADC_AIN));
		U16_TEMP_ADC3 = U16_TEMP_ADC2 + U16_TEMP_ADC3;
		
		U16_TEMP_ADC_H = (U16_TEMP_ADC_H % Bandgap_Value);
		U16_TEMP_ADC_H = (U16_TEMP_ADC_H << 1) + (U16_TEMP_ADC_L >> 15);
		U16_TEMP_ADC_L = (U16_TEMP_ADC_L << 1);
				
		U8_i_ADC_AIN++;
	}
	
	U16_TEMP_ADC3 = U16_TEMP_ADC3*3;
	U16_TEMP_ADC3 = U16_TEMP_ADC3/4;
	
	
	return U16_TEMP_ADC3;	//in mV
	
/* 	Temp_AIN_0_1V = (Value_ADC*10)/Bandgap_Value;
	Temp_AIN_0_01V = (((Value_ADC*10) % Bandgap_Value) * 10)/Bandgap_Value + 1; 	//round up
	Temp_AIN_0_01V = Temp_AIN_0_01V + (Temp_AIN_0_1V * 10);
	
	return Temp_AIN_0_01V; */
	
}

UINT16 U16_Read_Bandgap()
{
	UINT16 U16_temp;
	UINT8 U8_Bandgap_high,U8_Bandgap_low;
	
	set_IAPEN;
	IAPAL = 0x0C;
	IAPAH = 0x00;
	IAPCN = 0x04;
	set_IAPGO;
	U8_Bandgap_high = IAPFD;
	
	IAPAL = 0x0D;
	IAPAH = 0x00;
	IAPCN = 0x04;
	set_IAPGO;
	U8_Bandgap_low = IAPFD;	
	U8_Bandgap_low = U8_Bandgap_low & 0x0F;
	clr_IAPEN;
	
	U16_temp = U8_Bandgap_high;
	U16_temp = (U16_temp <<4 ) + U8_Bandgap_low;
	
	return U16_temp;
}

/* UINT16 U16_Read_Bandgap_Value()
{
	if(ADC_index >= 10)	//skip the first 10 sampling result, for accuracy
	{
		ADC_Value_Array[ADC_index - 10] = ((ADCRH<<4) + ADCRL);
		//ADC_Value_Array[ADC_index - 200] = 0x01;
		
		if(ADC_Value_Array[ADC_index-10]>ADC_Value_Max)
		{
			ADC_Value_Max = ADC_Value_Array[ADC_index-10];
		}
		if(ADC_Value_Array[ADC_index-10]<ADC_Value_Min)
		{
			ADC_Value_Min = ADC_Value_Array[ADC_index-10];
		}
	}

	if(ADC_index >= 19)	//take the sampling value Nr10-19
	{			
		ADC_Value = 0;
		ADC_Value = (ADC_Value_Array[0]+ADC_Value_Array[1]+ADC_Value_Array[2]+ADC_Value_Array[3]+ADC_Value_Array[4]
			   +ADC_Value_Array[5]+ADC_Value_Array[6]+ADC_Value_Array[7]+ADC_Value_Array[8]+ADC_Value_Array[9]);
		ADC_Value = (ADC_Value - ADC_Value_Max);
		ADC_Value = (ADC_Value - ADC_Value_Min);
		Bandgap_Value = (ADC_Value>>3);
		ADC_Value_Max = 0;
		ADC_Value_Min = 0x3FFF;
		
		//enable channel VLAMP_ADC_CHANNEL_NR
		{
			#if (VLAMP_ADC_CHANNEL_NR == 1)
				Enable_ADC_AIN1;	//Channel 1 selected
			#elif (VLAMP_ADC_CHANNEL_NR == 2)
				Enable_ADC_AIN2;	//Channel 2 selected
			#elif (VLAMP_ADC_CHANNEL_NR == 3)
				Enable_ADC_AIN3;	//Channel 3 selected
			#elif (VLAMP_ADC_CHANNEL_NR == 4)
				Enable_ADC_AIN4;	//Channel 4 selected
			#elif (VLAMP_ADC_CHANNEL_NR == 5)
				Enable_ADC_AIN5;	//Channel 5 selected
			#elif (VLAMP_ADC_CHANNEL_NR == 6)
				Enable_ADC_AIN6;	//Channel 6 selected
			#elif (VLAMP_ADC_CHANNEL_NR == 7)
				Enable_ADC_AIN7;	//Channel 7 selected
			#endif
		}
		
		ADC_index = 0;
	}
	else
	{		
		ADC_index++;
	}
	
	set_ADCS;	//Start ADC
} */

//***********************************************************************************************************
//Function: N76E003 APROM program DATAFLASH as EEPROM way 
//***********************************************************************************************************
//reading one byte takes ~4us with 8M Fcpu
UINT8 Read_APROM_BYTE(UINT16 code *u16_addr)
{
	UINT8 rdata;
	rdata = *u16_addr>>8;
	return rdata;
}

void Read_APROM_PAGE(void)
{
	UINT8 looptmp=0;
	
//Save APROM data to XRAM0
	for(looptmp=0;looptmp<0x80;looptmp++)
	{
		page_buffer[looptmp] = Read_APROM_BYTE((unsigned int code *)(ADDR_BASE+looptmp));
	}	
}

		
void Write_DATAFLASH_BYTE_BUFFER(UINT8 u8_address,UINT8 u8_Data)
{
// Modify customer data in XRAM
		page_buffer[u8_address&0x7f] = u8_Data;
}	

//page write takes around 10ms with 8M Fcpu
void Write_DATAFLASH_PAGE(void)
{
	UINT8 looptmp=0;
	
	//Erase APROM DATAFLASH page
		IAPAL = ADDR_BASE&0xff;
		IAPAH = (ADDR_BASE>>8)&0xff;
		IAPFD = 0xFF;
	  set_IAPEN; 
		set_APUEN;
    IAPCN = 0x22; 		
 		set_IAPGO; 
		
//Save changed RAM data to APROM DATAFLASH
		set_IAPEN; 
		set_APUEN;
	  IAPCN = 0x21;
		for(looptmp=0;looptmp<0x80;looptmp++)
		{
			IAPAL = (ADDR_BASE&0xff)+looptmp;
      IAPAH = (ADDR_BASE>>8)&0xff;
			IAPFD = page_buffer[looptmp];
			set_IAPGO;			
		}
		clr_APUEN;
		clr_IAPEN;
}

void system_save_data(void)
{
	Write_DATAFLASH_BYTE_BUFFER(ADDR_SYSTEM_DATA,0x55);
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 1),0xAA);
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 2),((ui_Day_counter&0xFF00)>>8));
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 3),(ui_Day_counter&0x00FF));
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 4),uc_Hour_counter);
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 5),uc_Minute_counter);
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 6),uc_Second_counter);
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 7),uc_System_State);
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 8),uc_AC_Charging_Time_Hour);
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 9),uc_AC_Charging_Time_Minute);
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 10),uc_AC_Charging_Time_Second);
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 11),0x55);
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 12),0xAA);
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 13),0x55);
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 14),0xAA);
	Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + 15),0x55);

	Write_DATAFLASH_PAGE();

}

void clear_flash(void)
{
	UINT8 u8_temp_i = 0;
	for(u8_temp_i = 0;u8_temp_i < LENGTH_SYSTEM_DATA;u8_temp_i++)
	{
		Write_DATAFLASH_BYTE_BUFFER((ADDR_SYSTEM_DATA + u8_temp_i),0);
	}
	
 	Write_DATAFLASH_PAGE();
}

char hextoascii(char hex_byte)
{
    char result;
    if((hex_byte>=0)&&(hex_byte<=9))            //变成ascii数字
        result = hex_byte + 0x30;
    else if((hex_byte >= 10)&&(hex_byte <= 15)) //变成ascii大写字母
        result = hex_byte + 0x37;
    else
        result = 0xff;
    return result;
}

char asciitohex(char ascii_byte)
{
    char result;
    if((ascii_byte>=0x30)&&(ascii_byte<=0x39))            //变成ascii数字
        result = ascii_byte - 0x30;
    else if((ascii_byte >= 0x41)&&(ascii_byte <= 0x46)) //变成ascii大写字母
        result = ascii_byte - 0x37;
    else
        result = 0xff;
    return result;
}

UINT8 U8_UART_TxD_Handle(void)
{
	switch (U8_UART_TxD_Handle_STATUS)
	{
		case UART_HANDLE_STATUS_BUSY:
			return 0;
		break;
		
		case UART_HANDLE_STATUS_STANDBY:
		{
			if(U8_UART_BUS_STATUS  == UART_BUS_STATUS_STANDBY)
			{
				TI = 1;	//trigger TI, get into UART ISR
			}
			return 0;
		}
		break;
		
		case UART_HANDLE_STATUS_WAITING_ACK:	//MCU waits for ACK from BT
		{
			return 0;
		}
		break;
		
		case UART_HANDLE_STATUS_WAITING_ANSWER:	//MCU waits for answer from BT
		{
			return 0;
		}
		break;

		case UART_HANDLE_STATUS_TIME_OUT:
		{
			return 0;
		}
		break;

		case UART_HANDLE_STATUS_TO_REPORT:	//MCU is going to report to BT
		{
			return 0;
		}
		break;

		case UART_HANDLE_STATUS_TO_ACK:		//MCU is going to ack BT
		{
			return 0;
		}
		break;		
	}
	return 0;
	//return UART_BUS_STATUS_STANDBY;
}

UINT8 U8_BT_MESH_UART_RxD_Handle(void)
{
	while(RxBuffer_read_counter != RxBuffer_write_counter)
	{	
		switch (Mesh_Data_RxD.STATUS)
		{
			case MESH_Data_RxD_STANDBY:
				if(RxBuffer[RxBuffer_read_counter] == BT_UART_FRAME_HEAD1)
				{
					Mesh_Data_RxD.DATA_Package_Header1 = BT_UART_FRAME_HEAD1;
					Mesh_Data_RxD.STATUS = MESH_Data_RxD_FRAME_HEAD_READING;
				}
			break;

			case MESH_Data_RxD_FRAME_HEAD_READING:
				if(RxBuffer[RxBuffer_read_counter] == BT_UART_FRAME_HEAD2)
				{
					Mesh_Data_RxD.DATA_Package_Header2 = BT_UART_FRAME_HEAD2;
					Mesh_Data_RxD.STATUS = MESH_Data_RxD_FRAME_HEAD_RECEIVED;
				}
			break;
			
			case MESH_Data_RxD_FRAME_HEAD_RECEIVED:
				{
					Mesh_Data_RxD.DATA_Length = RxBuffer[RxBuffer_read_counter];
					Mesh_Data_RxD.STATUS = MESH_Data_RxD_DATA_LEN_RECEIVED;
					Mesh_Data_RxD.DATA_Pointer = 0;
					
					//data length check
					if((Mesh_Data_RxD.DATA_Length > MESH_DATA_MAX_LEN)	//length out of limit
						|| (Mesh_Data_RxD.DATA_Length < 1))	//length out of limit
					{
						Mesh_Data_RxD.STATUS = MESH_Data_RxD_LEN_ERROR;
						//to be defined
					}
				}
			break;			
			
			case MESH_Data_RxD_DATA_LEN_RECEIVED:
			{
				Mesh_Data_RxD.DATA[Mesh_Data_RxD.DATA_Pointer] = RxBuffer[RxBuffer_read_counter];
				Mesh_Data_RxD.DATA_Pointer ++;
				if (Mesh_Data_RxD.DATA_Pointer == Mesh_Data_RxD.DATA_Length)	//data complete
				{
					Mesh_Data_RxD.STATUS = MESH_Data_RxD_DATA_RECEIVED;
				}
			}
			break;

			case MESH_Data_RxD_DATA_RECEIVED:
			{
				UINT8	U8_temp_Checksum,U8_temp_Checksum_i;
				Mesh_Data_RxD.DATA_CHECK_SUM = RxBuffer[RxBuffer_read_counter];
				
				//check if checksum correct
				U8_temp_Checksum = 0;
				U8_temp_Checksum_i = 0;
				while(U8_temp_Checksum_i < Mesh_Data_RxD.DATA_Length)
				{
					U8_temp_Checksum = (UINT8)(U8_temp_Checksum + Mesh_Data_RxD.DATA[U8_temp_Checksum_i]);
					U8_temp_Checksum_i++;
				}
				U8_temp_Checksum = U8_temp_Checksum + Mesh_Data_RxD.DATA_Length;
				if(Mesh_Data_RxD.DATA_CHECK_SUM == U8_temp_Checksum)
				{
					Mesh_Data_RxD.Function_Byte = Mesh_Data_RxD.DATA[0];
					Mesh_Data_RxD.STATUS = MESH_Data_RxD_TO_be_Processed;	//complete frame received without problem
				}
				else
				{
					Mesh_Data_RxD.STATUS = MESH_Data_RxD_SUM_ERROR;	//checksum error
				}
			}
			break;
			
			case MESH_Data_RxD_FRAME_COMPLETE:
					Mesh_Data_RxD.STATUS = MESH_Data_RxD_TO_be_Processed;
				
			break;
			
 			case MESH_Data_RxD_TO_be_Processed:
					//Mesh_Data_RxD.STATUS = MESH_Data_RxD_STANDBY;
				
			break;
			
			case MESH_Data_RxD_SUM_ERROR:
					Mesh_Data_RxD.STATUS = MESH_Data_RxD_STANDBY;
			
			break;
			case MESH_Data_RxD_LEN_ERROR:
					Mesh_Data_RxD.STATUS = MESH_Data_RxD_STANDBY;
				
			break;			
		}

		//read pointer update
		if(RxBuffer_read_counter >= RxBufferSize)
		{
			RxBuffer_read_counter=0;
		}
		else
		{
			RxBuffer_read_counter++;
		}		
		
		
		//process received data package
		if(Mesh_Data_RxD.STATUS == MESH_Data_RxD_TO_be_Processed)
		{

			
			//function to be defined...	
		}
		
	}	
	
	//Modify BT short address in mesh network
	
	
	//report BT short address in mesh network
	
	
	//report network ID
	
	//Modify network ID
	
	
	//report BT MAC address
	
	//report BT name
	
	//modify BT name
	
	//report system info
	
	return 0;
}

void v_BT_MESH_UART_Answer_processing()
{
	UINT8 U8_temp_UART_TxD_i;
	UINT8 U8_temp_UART_TxD_Checksum;
	
	switch(Mesh_Data_RxD.Function_Byte)
	{
		case Query_Firmware_Version:
			{
				Mesh_Data_TxD.DATA_Length = 0x03;
				Mesh_Data_TxD.DATA[0] = Firmware_INFO_1;
				Mesh_Data_TxD.DATA[1] = Firmware_INFO_2;
				Mesh_Data_TxD.DATA[2] = Firmware_INFO_3;
			}
		break;
	}
	
	

	
	
	Mesh_Data_RxD.STATUS = MESH_Data_RxD_STANDBY;

	putchar(BT_UART_FRAME_HEAD1);
	putchar(BT_UART_FRAME_HEAD2);
	putchar(Mesh_Data_TxD.DATA_Length);
	
	U8_temp_UART_TxD_i = 0;
	U8_temp_UART_TxD_Checksum = 0;
	while(U8_temp_UART_TxD_i < Mesh_Data_TxD.DATA_Length)
	{
		putchar(Mesh_Data_TxD.DATA[U8_temp_UART_TxD_i]);
		U8_temp_UART_TxD_Checksum = (UINT8)(U8_temp_UART_TxD_Checksum + Mesh_Data_TxD.DATA[U8_temp_UART_TxD_i]);
		U8_temp_UART_TxD_i++;
	}
	U8_temp_UART_TxD_Checksum = (UINT8)(U8_temp_UART_TxD_Checksum + Mesh_Data_TxD.DATA_Length);
	putchar(U8_temp_UART_TxD_Checksum);	
	
/* 	//
	while(RxBuffer_read_counter != RxBuffer_write_counter)
	{		
		//new data package received,start from "+"
		//if(RxBuffer[RxBuffer_read_counter] == 0x2B)	//"+"
		if(RxBuffer[RxBuffer_read_counter] == '+')	//"+"
		{
			U8_UART_Receive_Pakage_Status = UART_RECEIVE_PACKAGE_START;
			
			UART_BUFFER_TEMP = NULL;
			UART_BUFFER = UART_BUFFER_TEMP;
			*UART_BUFFER_TEMP = '+';	//"ASCII "+"
		}
		else if (U8_UART_Receive_Pakage_Status == UART_RECEIVE_PACKAGE_START)
		{
			//ends up with "/r/n", a complete package received
			//if((*UART_BUFFER_TEMP == 0x0D) && (RxBuffer[RxBuffer_read_counter] == 0x0A))
			if((*UART_BUFFER_TEMP == '\r') && (RxBuffer[RxBuffer_read_counter] == '\n'))	
			{
				U8_UART_Receive_Pakage_Status = UART_RECEIVE_PACKAGE_COMPLETE;
			}
			UART_BUFFER_TEMP++;
			*UART_BUFFER_TEMP = RxBuffer[RxBuffer_read_counter];			
		}
				
		//read pointer update
		if(RxBuffer_read_counter >= RxBufferSize)
		{
			RxBuffer_read_counter=0;
		}
		else
		{
			RxBuffer_read_counter++;
		}		
	} */
}