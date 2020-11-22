/**************************************************************/
/*					Power Rider Unit						  */
/*		PoweRider dll allow you to use the Power Rider Unit	  */
/*															  */
/*					   .dll Version 1.5.b				      */
/*			   Developed by Redler Technologies.			  */
/*				      All Rights Reserved.				      */
/*				   https://www.redler.co.il/				  */
/*															  */
/**************************************************************/
/* Some comments are From: "USB-CAN Bus Interface Adapter Interface Function Library User Instruction.pdf"*/


/*************************************************************************************************/
/*															 								     */
/* 1- Functions that return "char*" type, return the result value in case the request was valid  */
/*	  and return error type otherwise.	   													     */
/* 2- Function that takes as arguments (parameters) pointer (*) type, return one or more value(s)*/
/*	  																							 */
/*    e.g.  extern "C" __declspec(dllexport) char* __cdecl GetVIn_OutCommand(					 */
/*			int Type, uint16_t ID , sNETWORK* PsNetwork, 									     */
/*			int ChannelNo, 														  				 */
/*			float *VIn, 														  				 */
/*			float *VOut, 														  				 */
/*			int *ChannelNum);														  			 */
/*	 a) The function return "success" in case the request was valid and no error was detected,	 */
/*		otherwise return "Error: 0x...." error code of 2 bytes length.							 */
/*	 b) The function takes as arguments the PowerRider Unit CanBus ID, the channel number,       */
/*		float *VIn will take the channel input voltage, float *VIn will take the channel output  */
/*		voltage, int *ChannelNum will take the channel number you entered returned by the Unit   */
/*															 								     */
/*															 								     */
/*************************************************************************************************/
#ifndef DLLMAIN_H
#define DLLMAIN_H
#include "stdafx.h"
#include "ControlCAN.h" // CanUSB function declaration file.
#include "UDPCom.h"

/***** Communication accessories type *****/

#define CANALYST 1
#define UDP 2
#define CANKVASER 3

/***** Priority argument *****/

#define PRIORITY 0x00000006

/***** Divers *****/

#define MSG_OK 1
#define MAX_RETRY_COUNT 10			// Number of time to listen to the bus before timeout.
#define TIME_BEFORE_RETRY 70		// Time to wait before each retry

/***** Error code (int) *****/

#define SEND_ERROR_CODE -1
#define TYPE_NOT_DETECTED_CODE -2
#define RECEIVE_ERROR_CODE -3
#define TIMEOUT_CODE -4
#define SYSTEM_ERROR_CODE -5
#define MSG_LENGHT_ERROR_CODE -6
#define UNKNOWN_ERROR_CODE -7
#define SIZE_EXCEED -8

/***** Error code (string - char*) *****/

#define TYPE_NOT_DETECTED "Type not detected"
#define SEND_ERROR "Send error"
#define RECEIVE_ERROR "Receive error"
#define TIMEOUT "Error: 0xF0FF"
#define SYSTEM_ERROR "System error"
#define MSG_LENGHT_ERROR "Msg length error"
#define UNKNOWN_ERROR "Unknown error"
#define SIZE_EXCEED_ERROR "Sise exceed error"
#define SYSTEM_ERROR_PREFIX 0xE0
#define _MAKE_HEX(value) #value
#define MAKE_HEX(value) _MAKE_HEX(value)

/***** Communication delay definition ******/

#define WAIT_TX_RX	10	// Wait between tx and rx operation (in usec)
#define WAIT_RX_TX	20	// Wait between rx and tx operation (in usec), in case of reiterative operation like "Generale string"
#define DELTA_T		100	// Wait between rx and tx operation (in msec), in case of reiterative operation like "Generale string" ONLY FOR UDP

/*** Channel Parameter No 25 defines list */
#define SYSTEM_PARAM	1
#define CHANNEL_PARAM	2

#define CHANNEL_NUMBER_UNIT_DIGIT	'6'
#define ACII_0_CONST				49

#define NOT			100
#define ADN_OPER	220 // old 120
#define OR_OPER		221 // old 121
#define XOR_OPER	222 // old 122
#define NONE		0
#define IN1			1
#define IN2			2
#define IN3			3
#define IN4			4
#define IN5			5
#define IN6			6
#define IN7			7
#define IN8			8
#define IN9			9
#define IN10		10
#define IN11		11
#define IN12		12
#define IN13		13
#define IN14		14
#define IN15		15
#define IN16		16
#define LAST_CH		16
#define SPARE5		17
#define LTCM		18
#define EMERGENCY	19
#define OUT1		20
#define OUT2		21
#define OUT3		22
#define OUT4		23
#define SPARE9		24
#define SPARE10		25
#define SPARE11		26
#define SPARE12		27
#define SPARE13		28
#define SPARE14		29
#define PARAMERR	30
#define TEMPERR		31
#define VOLTERR		32 // Total Voltage input  error Vin1+Vin2
#define BOM			33 // Black Out mode
#define BITdef		34 // General BIT error (any BIT error)
#define ANY_TRIP	35 // At least one of Channel Trip                      TANCH
#define VIN1_LOW	36 // Voltage input 1 Low error (VIN1 LOW )             EV1L
#define VIN1_HIGH	37 // Voltage input 1 High error (VIN1 HIGH )           EV1H
#define VIN2_LOW	38 // Voltage input 2 Low error (VIN2 LOW )             EV2L
#define VIN2_HIGH	39 // Voltage input 2 High error (VIN2 HIGH )           EV2H
#define VIN1		40 // Voltage input 1 error (VIN1 LOW + VIN1 HIGH  )    EVI1
#define VIN2		41 // Voltage input 2 error (VIN2 LOW + VIN2 HIGH  )    EVI2
#define SPARE17		42
#define SPARE18		43
#define SPARE19		44
#define CH1_STS		45
#define CH2_STS		46
#define CH3_STS		47
#define CH4_STS		48
#define CH5_STS		49
#define CH6_STS		50
#define CH7_STS		51
#define CH8_STS		52
#define CH9_STS		53
#define CH10_STS	54
#define CH11_STS	55
#define CH12_STS	56
#define CH13_STS	57
#define CH14_STS	58
#define CH15_STS	59
#define CH16_STS	60
#define SPARE24		61
#define SPARE25		62
#define CH1_TRIP	63
#define CH2_TRIP	64
#define CH3_TRIP	65
#define CH4_TRIP	66
#define CH5_TRIP	67
#define CH6_TRIP	68
#define CH7_TRIP	69
#define CH8_TRIP	70
#define CH9_TRIP	71
#define CH10_TRIP	72
#define CH11_TRIP	73
#define CH12_TRIP	74
#define CH13_TRIP	75
#define CH14_TRIP	76
#define CH15_TRIP	77
#define CH16_TRIP	78
#define SPARE30		79
#define SPARE31		80
#define CH1_ERR		81
#define CH2_ERR		82
#define CH3_ERR		83
#define CH4_ERR		84
#define CH5_ERR		85
#define CH6_ERR		86
#define CH7_ERR		87
#define CH8_ERR		88
#define CH9_ERR		89
#define CH10_ERR	90
#define CH11_ERR	91
#define CH12_ERR	92
#define CH13_ERR	93
#define CH14_ERR	94
#define CH15_ERR	95
#define CH16_ERR	96
#define SPARE36		97
#define SPARE37		98
#define SPARE		99


// return the dll version
extern "C" __declspec(dllexport) char* __cdecl dllVersion();

// Open, Initialize and Start Can Bus connection.
extern "C" __declspec(dllexport) char* __cdecl OpenCan(
	int Type, 
	unsigned long BaudRate);

// Open, Initialize and Start UDP connection.
#if defined _M_IX86
typedef char* T;
#elif defined _M_X64
typedef const wchar_t* T;
#endif
extern "C" __declspec(dllexport) int __cdecl OpenUDP(
	T IP,
	int Port,
	sNETWORK* PsNetwork);

// Read any received message.
extern "C" __declspec(dllexport) char* __cdecl ReadAddon(
	int Type,
	sNETWORK* PsNetwork);

// Get Channel Status function.
/*************************************************************************************************/
/*	3.1.1 Channel Status 1[Identifier:0x15500000] Channels 1 - 10.								 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*		• The target will send back the status of each channel and if a trip					 */
/*		  event has happened.																	 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ChannelStatus(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	int *StatusChannel,
	int *TripCHannel, 
	sNETWORK* PsNetwork);

// Get Channel Combined Status 1 function.
/*************************************************************************************************/
/*	3.1.2 Channel combined Status 1 [Identifier: 0x15500028] Channels 1-8.						 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*		• The target will send back the status of each channel and if a trip					 */
/*		  event has happened.																	 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ChannelCombinedStatus1(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	uint16_t *BIT,
	uint8_t *InStatus,
	uint8_t *OutStatus,
	uint32_t *ChStatus, 
	sNETWORK* PsNetwork);

// Get Channel Combined Status 2 function.
/*************************************************************************************************/
/*	3.1.3 Channel combined Status 2 [Identifier: 0x15500028] Channels 9-16.						 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*		• The target will send back the status of each channel and if a trip					 */
/*		  event has happened.																	 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ChannelCombinedStatus2(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	uint16_t *BIT,
	uint8_t *InStatus,
	uint8_t *OutStatus,
	uint32_t *ChStatus, 
	sNETWORK* PsNetwork);

// Set Channel Control function.
/*************************************************************************************************/
/*	3.1.4 Channel control [Identifier: 0x15500002] Channels 1-16.								 */
/*		• Packet structure sent from the source to the target.									 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ChannelControl(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	unsigned __int16 Control,
	unsigned __int16 Mask,
	sNETWORK* PsNetwork);

// Get Curent Command function.
/*************************************************************************************************/
/*	3.1.5 Current Command [Identifier: 0x15500003] Channels 1-16.								 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl CurentCommand(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	int ChannelNo,
	float *ADCunits,
	sNETWORK* PsNetwork);

//	Get Vin/Vout Command function.
/*************************************************************************************************/
/*	3.1.8 Get Vin/Vout Command [Identifier 0x15500008].											 */
/*		• VIn, VOut and ChannelNum pointers values are empty									 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl GetVIn_OutCommand(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	int ChannelNo,
	float *VIn,
	float *VOut,
	sNETWORK* PsNetwork);

// Get Version/Board Type function.
/*************************************************************************************************/
/*	3.1.9 Version/ Board Type [Identifier: 0x15500009]											 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*                                                                                               */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl GetSWVersion(
	int Type, 
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNETWORK
	/*sNETWORK* PsNetwork*/);

// Get System Temperature function.
/*************************************************************************************************/
/*	3.1.10 System Temperature [Identifier: 0x1550000A].											 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*		• CPU and Temp(s) pointers values are empty												 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl SystemTemperature(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	float *Temp1, // Return Channel 1-5 Temperatures
	float *Temp2, // Return Channel 6-10 Temperatures
	float *CPU, 
	sNETWORK* PsNetwork);

// Get System Parameters function.
/*************************************************************************************************/
/*	3.1.11 System Parameter [Identifier: 0x1550000B].											 */
/*		• The source is sampling the target with the parameter number and index if needed.		 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl GetSystemParam(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	int ParamNo,
	int Index, 
	sNETWORK* PsNetwork);

// Set System Parameters function.
/*************************************************************************************************/
/*	3.1.11 System Parameter [Identifier: 0x1550000B].											 */
/*		• The source packet contains the parameter number and ValIn or *StrIn or Index if needed.*/
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl SetSystemParam(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	int ParamNo,
	float ValIn,
	char *StrIn,
	int Index, 
	sNETWORK* PsNetwork);

// Get Channel Parameters function.
/*************************************************************************************************/
/*	3.1.12 Channel Parameter [Identifier: 0x1550000C].											 */
/*		• The source is sampling the target with the parameter number and index if needed.		 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl GetChannelParam(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ChannelNo,
	int ParamNo, 
	sNETWORK* PsNetwork);

// Set Channel Parameters function.
/*************************************************************************************************/
/*	3.1.12 Channel Parameter [Identifier: 0x1550000C].											 */
/*		• The source packet contains the channel and parameter number and *StrIn or Index.		 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl SetChannelParam(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	int ChannelNo,
	int ParamNo,
	float ValIn,
	char *StrIn,
	sNETWORK* PsNetwork);

// Save configuration command function.
/*************************************************************************************************/
/*	3.1.13 Channel Parameter [Identifier: 0x15500013].											 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ConfigSaveCommand(
	int Type,
	uint8_t ID, 
	uint8_t dstAdd,
	sNETWORK* PsNetwork);

// Go to bootloader configuration command function.
/*************************************************************************************************/
/*	3.1.13 Channel Parameter [Identifier: 0x15500013].											 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ConfigGotoBootloaderCommand(
	int Type,
	uint8_t ID, 
	uint8_t dstAdd,
	sNETWORK* PsNetwork);

// SW reset configuration command function.
/*************************************************************************************************/
/*	3.1.13 Channel Parameter [Identifier: 0x15500013].											 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ConfigSWResetCommand(
	int Type,
	uint8_t ID, 
	uint8_t dstAdd,
	sNETWORK* PsNetwork);

// Configuration Mode command function.
/*************************************************************************************************/
/*	3.1.13 Channel Parameter [Identifier: 0x15500013].											 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ConfigConfigModeCommand(
	int Type,
	uint8_t ID, 
	uint8_t dstAdd,
	sNETWORK* PsNetwork);

// Sleep mode configuration mode command function
/*************************************************************************************************/
/*	3.1.13 Channel Parameter [Identifier: 0x15500013].											 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*		1 Enter the unit to sleep mode.															 */
/*		0 Exit from sleep mode (Active Mode).													 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ConfigSleepModeCommand(
	int Type,
	uint8_t ID, 
	uint8_t dstAdd,
	uint8_t sts,
	sNETWORK* PsNetwork);

// Standbye configuration mode command function.
/*************************************************************************************************/
/*	3.1.13 Channel Parameter [Identifier: 0x15500013].											 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ConfigStdbyModeCommand(
	int Type,
	uint8_t ID, 
	uint8_t dstAdd,
	sNETWORK* PsNetwork);

// CAN Bus configuration mode command function.
/*************************************************************************************************/
/*	3.1.13 Channel Parameter [Identifier: 0x15500013].											 */
/*		• Enter the CAN Bus configuration mode.													 */
/*		1  Enter to CANBUS silent mode                                                           */
/*		2  Exit from CANBUS silent mode                                                          */
/*		3  CANBUS disable mode                                                                   */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ConfigCANModeCommand(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd, 
	uint16_t sts,
	sNETWORK* PsNetwork);

// Tripped Event Channel Reset configuration command function.
/*************************************************************************************************/
/*	3.1.13 Channel Parameter [Identifier: 0x15500013].											 */
/*		• Enter the channel number to reset the channel tripped event.							 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ConfigChannelResetCommand(
	int Type,
	uint8_t ID, 
	uint8_t dstAdd, 
	uint16_t ChannelNum,
	sNETWORK* PsNetwork);

// Set default parameters function.
/*************************************************************************************************/
/*	 Set default parameters [Identifier: 0x155000013].											 */
/*		• Return to default parameters unit.													 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl SetDefaultParams(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork);

// Event status command function.
/*************************************************************************************************/
/*	3.1.14 Channel Parameter [Identifier: 0x1550000F].											 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl EventStatusCommand(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	int ChannelNo,
	int *HS_ResetCounter,
	int *SW_ResetCounter,
	int *ResetFail,
	int *TripType, 
	sNETWORK* PsNetwork);

// Get log function.
/*************************************************************************************************/
/*	3.1.15 Log File command [Identifier: 0x15500012].											 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl GetLog(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	char *EventLog, 
	sNETWORK* PsNetwork);

// Clear log function.
/*************************************************************************************************/
/*	3.1.15 Log File command [Identifier: 0x15500012].											 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ClearLog(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork);

// Clear time log function.
/*************************************************************************************************/
/*	3.1.15 Log File command [Identifier: 0x15500012].											 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl ClearTimeLog(
	int Type,
	uint8_t ID, 
	uint8_t dstAdd,
	sNETWORK* PsNetwork);

// Bit Command function.
/*************************************************************************************************/
/*	3.1.16 BIT command [Identifier: 0x15500021].												 */
/*		• The source is sampling the target with the ID and parameter number.					 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl BitCommand(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	int ParamNo,
	int *Bit,
	int *Test, 
	sNETWORK* PsNetwork);

// Get Discrete Ports function.
/*************************************************************************************************/
/*	3.1.17 Discrete ports [Identifier: 0x15500022].												 */
/*		• The source is sampling the target with the ID and parameter number.					 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl GetDiscretePorts(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	int ParamNo,
	int* Result, 
	sNETWORK* PsNetwork);

// Set Discrete Ports function.
/*************************************************************************************************/
/*	3.1.17 Discrete ports [Identifier: 0x15500022].												 */
/*		• The source is sampling the target with the ID and parameter number.					 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl SetDiscretePorts(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ParamNo, 
	sNETWORK* PsNetwork);

// Get Serial Number function.
/*************************************************************************************************/
/*	3.1.18 Serial number [Identifier: 0x15500024].												 */
/*		• The source is sampling the target with the ID.										 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl GetS_N(
	int Type,
	uint8_t ID, 
	uint8_t dstAdd,
	sNETWORK* PsNetwork);

// Set Serial Number function.
/*************************************************************************************************/
/*	3.1.18 Serial number [Identifier: 0x15500024].												 */
/*		• The source is sampling the target with the ID.										 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl SetS_N(
	int Type,
	uint8_t ID, 
	uint8_t dstAdd,
	char *SN, 
	sNETWORK* PsNetwork);

// Get Part number command function.
/*************************************************************************************************/
/*	3.1.19 P/N Command [Identifier: 0x15500027].												 */
/*		• The source is sampling the target with an empty payload packet.						 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl P_NCommand(
	int Type,
	uint8_t ID, 
	uint8_t dstAdd,
	sNETWORK* PsNetwork);

// Get offset Data function.
/*************************************************************************************************/
/*	3.1.20 Offset measurement command [Identifier: 0x15500025].									 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl GetOffsetData(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	int ParamNo,
	int ChannelNo,
	int OffsetType,
	float *Result,
	sNETWORK* PsNetwork);

// Set offset Data function.
/*************************************************************************************************/
/*	3.1.20 Offset measurement command [Identifier: 0x15500025].									 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl SetOffsetData(
	int Type, 
	uint8_t ID, 
	uint8_t dstAdd,
	int ParamNo,
	int ChannelNo,
	int OffsetType,
	float OffsetData,
	sNETWORK* PsNetwork);

// Get NetRider Information function.
/*************************************************************************************************/
/*	x NetRider Information command [Identifier: 0x1550002F].									 */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl InternalDataInfo(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ParamNo,
	sNETWORK* PsNetwork);

// Receive periodically Status Response.
/*************************************************************************************************/
/*	 Receive periodically (each 200/250 ms) Status Response command [Identifier: 0x1550002C].    */
/*   return 1 if the received msg is correct, otherwise - 1.                                     */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl StatusResponse(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	float *response,
	sNETWORK* PsNetwork);

// Receive periodically Plot Command.
/*************************************************************************************************/
/*	 Receive periodically (each 200/250 ms) Plot Command command [Identifier: 0x1550002E].       */
/*   return 1 if the received msg is correct, otherwise - 1.                                     */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl PlotCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint8_t *input,
	uint16_t time,
	uint8_t interval,
	int length,
	uint8_t *offsetResponse,
	uint16_t *scaleResponse,
	sNETWORK* PsNetwork);

// Receive periodically Plot Status.
/*************************************************************************************************/
/*	 Receive periodically (each 200/250 ms) Plot Status command [Identifier: 0x1550002D].        */
/*   return 1 if the received msg is correct, otherwise - 1.                                     */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl PlotStatus(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	double *response,
	sNETWORK* PsNetwork);

// Get Status Response.
/*************************************************************************************************/
/*	 Get Status Response command [Identifier: 0x1550002C].										 */
/*   return 1 if the received msg is correct, otherwise - 1.                                     */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl GetStatusResponse(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	float *response,
	sNETWORK* PsNetwork);

// Log Sync function
/*************************************************************************************************/
/*	 Set Default Parameters command [Identifier: 0x15500013].									 */
/*   return success or error msg						.                                        */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl LogSync(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint8_t logSync,
	sNETWORK* PsNetwork);

// Set Black-Out Mode function.
/*************************************************************************************************/
/*	 Set Black-Out Mode Enable or Disable Parameters command [Identifier: 0x15500013].			 */
/*   return success or error msg.																 */
/*   blackOut param => each bit determine if channel black-out mode is enable or not,            */
/*					   0 restaure to original paramaeters							             */
/*************************************************************************************************/
extern "C" __declspec(dllexport) char* __cdecl SetBlackOutMode(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint16_t blackOut,
	uint16_t* blackOutRes,
	sNETWORK* PsNetwork);

// Get Black-Out Mode function.
// Set Black-Out Mode function.
/*************************************************************************************************/
/*	 Get Black-Out Mode Enable or Disable Parameters command [Identifier: 0x15500013].			 */
/*   return success or error msg.																 */
/*   blackOutRes param => each bit determine if channel black-out mode is enable or not,         */
/*					   0 restaure to original paramaeters							             */
/*************************************************************************************************/
extern __declspec(dllexport) char* __cdecl GetBlackOutMode(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint16_t* blackOutRes,
	sNETWORK* PsNetwork);

// Set Channel Priority Scenario function.
/*************************************************************************************************/
/*	 Set Channel Priority Scenario Number Parameters command [Identifier: 0x15500013].			 */
/*   return success or error msg.																 */
/*   scenarioNum is the scenario number to set.													 */
/*   scenarioNumRes param => return the scenario number,								         */
/*					   0 restaure to original paramaeters							             */
/*************************************************************************************************/
extern __declspec(dllexport) char* __cdecl SetChannelPriorityScenario(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint8_t scenarioNum,
	uint8_t* scenarioNumRes,
	sNETWORK* PsNetwork);

// Get Channel Priority Scenario function.
/*************************************************************************************************/
/*	 Get Channel Priority Scenario Number Parameters command [Identifier: 0x15500013].			 */
/*   return success or error msg.																 */
/*   scenarioNumRes param => return the scenario number,								         */
/*					   0 restaure to original paramaeters							             */
/*************************************************************************************************/
extern __declspec(dllexport) char* __cdecl GetChannelPriorityScenario(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint16_t* scenarioNumRes,
	sNETWORK* PsNetwork);

// This function is used to close the connection.	
// Return value = 1, which means that the operation is successful; \
// Return 0 indicates that the operation failed; \
// Return -1 indicates that the device does not exist.
extern "C" __declspec(dllexport) char* __cdecl Close(int Type);

// This function is used to close the connection.	
// Return value = 1, which means that the operation is successful; \
// Return 0 indicates that the operation failed; \
// Return -1 indicates that the device does not exist.
extern "C" __declspec(dllexport) int __cdecl CloseUDP(sNETWORK* PsNETWORK);

// This function is used to reset the CAN controller.	
// Return value = 1, which means that the operation is successful; \
// Return 0 indicates that the operation failed; \
// Return -1 indicates that the device does not exist.
extern "C" __declspec(dllexport) int __cdecl Reset(int Type);

#endif