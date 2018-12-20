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
v0.0.0	2018_03_21
v0.0.1,2018_12_14
	-- MCU can talk with BT module (Mesh function of BT module not yet tested)

v0.0.2,2018_12_17
	-- Two nodes mesh network built-up

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
#include "Function_define.h"

#define BUFFER_SIZE		16
UINT8  UART_BUFFER[BUFFER_SIZE],temp;
UINT16 u16_CNT=0,u16_CNT1=0;
bit riflag;

#define TxBufferSize	50
#define RxBufferSize	50

struct Mesh_Data_Send
{
	UINT16	Instruction;
	UINT16	Target_Short_Address;
	UINT8	DATA[10];
	//UINT8	DATA[10] = {1,2,3,4,5,6,7,8,9,0x0A};
} Mesh_Data_Send_1;

#define Instruction_Send	0xAAFB
#define Address_Broadcast	0xFFFF

UINT8 xdata TxBuffer[TxBufferSize]; 
UINT8 xdata RxBuffer[RxBufferSize];
volatile UINT8 RxBuffer_read_counter = 0x00;
volatile UINT8 RxBuffer_write_counter = 0x00;
volatile UINT8 TxBuffer_read_counter = 0x00;
volatile UINT8 TxBuffer_write_counter = 0x00;

void v_init();
void v_Load_Mesh_Send_Data(UINT16 Instruction,UINT16 Address,UINT8 Data[]);

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
		} 
	}
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
	set_CLOEN;	//Clock Output Enable
	
	/*BT module PWRC pin configuration
		Connect state:
				--PWRC = 1:Transparent transmission
				--PWRC = 0:AT instruction/Querry. 
									In this sate, BT device (MP for instance) can still send data via BT module to MCU, 
									but BT does not forward data from MCU to MP(BT device)
		Unconnect state:
				--PWRC = 1:AT instruction/Querry
				--PWRC = 0:AT instruction/Querry	
	*/
	P14_PushPull_Mode;		//P14 for BT's PWRC pin
	
		P12_PushPull_Mode;

		

	InitialUART0_Timer3(115200);

	



		
		set_RB8;					//This bit is for setting the stop bit 2 high/low status,  */
		clr_TI; 
    set_ES;           //enable UART interrupt
    set_EA;           //enable global interrupt
}

/************************************************************************************************************
*    Main function 
************************************************************************************************************/
void main (void)
{
	UINT8 i;
	UINT8 Data_Test[16] = {"FUCK56789A"};
	//char *Data_Test1[16];
	
	
	v_init();

    while(1)
		{
			if (riflag)
			{
				P12 = ~P12;		//In debug mode check UART_BUFFER[u16_CNT] to check receive data
				riflag = 0;
			}
			
			//Send_Data_To_UART0(0x55);
			//putchar(0x55);
			
			
			//printf("\n temp= 0x%bX",0x55);
			
			//Querry Version number
			//Response:JDY-10M-Y2.2-MESH
			//printf("AT+VER\r\n");
			
			//Querry MAC address
			//Response:+MAC:<mac address>
			//printf("AT+MAC\r\n");
		

			//SET BT address
			//Response:+OK
/* 			printf("AT+NAMEKTC-BT-MESH-1\r\n");	
			TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
			Timer0_Delay1ms(200);	
			Timer0_Delay1ms(2);	 */			

			

			//Querry BT address
			//Response:+NAME=<BT name>
/* 			printf("AT+NAME\r\n");
			TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
			Timer0_Delay1ms(200);	
			Timer0_Delay1ms(2);		 */	
			
			//Querry BAUD rate
			//Response:+BAUD=<0--115200>
			//printf("AT+BAUD\r\n");	

			//Set device type
			//Response:+ok
			/*
					A0:Transparent transmission mode
					B1:LED light mode
					C0:Low power consumption telecontroller			
			*/
			//printf("AT+CLSSA0\r\n");
				

			//clr_P14;
			set_P14;

			//Querry device type
			//Response:+CLSS=<A0--Transparent transmission mode>
			//printf("AT+CLSS\r\n");
			
			//SET mesh network ID 
			//(ID has to be exactly 12 characters, if not enough, fill in with space)
			//(ID has to be data within 0-9,A,B,C,D,E,F)
			//Response:+OK
			//printf("AT+NETID0123456BCDEF\r\n");	
			//TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
			//Timer0_Delay1ms(200);	
			//Timer0_Delay1ms(2);				
			
		

			//Querry mesh network ID
			//Response:+NETID=<netID>
			//printf("AT+NETID\r\n");		
			//TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
			//Timer0_Delay1ms(200);	
			//Timer0_Delay1ms(2);			

			//SET mesh network short address
			//(short address has to be exactly 2 characters, if not enough, fill in with space)
			//(short address has to be data within 0-9,A,B,C,D,E,F)
			//Response:+OK
			//printf("AT+MADDR55\r\n");	
			//TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
			//Timer0_Delay1ms(200);	
			//Timer0_Delay1ms(2);				
			

			//Querry mesh network short address
			//Response:+MADDR=<short address>
			//printf("AT+MADDR\r\n");	
			//TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
			//Timer0_Delay1ms(200);	
			//Timer0_Delay1ms(2);			
			
			//SET APP connection passcode
			//(APP connection passcode has to be exactly 4 characters, if not enough, fill in with space)
			//(APP connection passcode can be any asCII code)
			//Response:+OK
/* 			printf("AT+PSS8HAB\r\n");					
			TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
			Timer0_Delay1ms(200);	
			Timer0_Delay1ms(2);		 */	

			//Querry APP connection passcode
			//Response:+PSS=<passcode>
/* 			printf("AT+PSS\r\n");
			TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
			Timer0_Delay1ms(200);	
			Timer0_Delay1ms(2); */				

			//SET APP passcode connection switch
			//ISCEN0 -- APP dosn't require a passcode
			//ISCEN1 -- open APP passcode switch
			//Response:+OK
/* 			printf("AT+ISCEN0\r\n");	
			TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
			Timer0_Delay1ms(200);	
			Timer0_Delay1ms(2);				
			 */
		

			//Querry APP passcode connection switch
			//Response:+ISCEN=<ISCEN>
			//						0 -- APP dosn't require a passcode
			//						1 -- open APP passcode switch
/* 			printf("AT+ISCEN\r\n");	
			TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
			Timer0_Delay1ms(200);			
			Timer0_Delay1ms(2);	 */
			
			//Broadcast send data
/* 			putchar(0xAA);
			putchar(0xFB);
			putchar(0xFF);
			putchar(0xFF);
			printf("123456789A"); */
			strcpy(Data_Test,"IAMNOTFINEOK");			
			v_Load_Mesh_Send_Data(Instruction_Send,Address_Broadcast,Data_Test);
			
			//*Data_Test1 = "IAMNOTFINEOK";
			//v_Load_Mesh_Send_Data(Instruction_Send,Address_Broadcast,Data_Test1);

			
			TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
			Timer0_Delay1ms(200);			
			Timer0_Delay1ms(2); 
			
			//printf(0x55);
			
			TI = 1;	//Triggering UART buffer sending process ... for putchar() & Printf()
			//P12 = ~P12;		//In debug mode check UART_BUFFER[u16_CNT] to check receive data
			
			Timer0_Delay1ms(200);		
			Timer0_Delay1ms(2);		
		}
	
}

