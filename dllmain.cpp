// dllmain.cpp : Defines the entry point for the DLL application.

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

#define DLL_VERSION "1.5.b"
#define MAX_CHANNEL_NUMBER 16
//#define OLD_CHANNEL_PARAM_25

#include "stdafx.h"

#include "dllmain.h"	// dll vfunction declaration file.

#include "ControlCAN.h"	// Canalyst-II CanUSB function declaration file.
#include "canlib.h"		// Kvaser CanUSB function declaration file.

#include <string>		// std::string, std::to_string
#include <thread>		// std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
#include <stdio.h>		// C program for implementation of ftoa()
#include <math.h>		// C program for implementation of ftoa()
#include <stdlib.h>
#include <ctype.h>		// toupper function
#include <iomanip>		// setprecision
#include <sstream>		// stringstream
#include <cstdio> 
#include <algorithm>	// transform function of string input
#include <Objbase.h>

#pragma region UDP_Include

#include "UDPCom.h"

#include <sdkddkver.h>
#include <WinSock2.h>
#include <Windows.h>
#include <iostream>
#include <Ws2tcpip.h>

#pragma endregion UDP_Include

#include <fstream>

#pragma region Canalyst-II

int DeviceType = 4;					 // for USBCAN-2A/USBCAN-2C/CANalyst-II the type Value is 4.
int DeviceInd = 0;					 // Device Index, for example, when there is only one USB-CAN adapter, the index number is 0, when there are multiple USB- CAN adapters, the index numbers in an ascending order starting from 0.
unsigned long Reserved = 0;			 // Retention parameters, fill in 0.
int CanIndex = 0;				     // CAN channel index, such as when there is only one CAN channel, the index number is 0, if there are two, the index number can be 0 or 1.
unsigned int Filter = 0;			 // 0,1 receives all frames. 2 standard frame filtering, 3 is an extended frame filtering.
unsigned long AccCode = 0;			 // Receive filtered acceptance code.
unsigned long AccMask = 0xFFFFFFFF;	 // Receive filter mask.
unsigned int Mode = 0;				 // mode: 0 represents normal mode, 1 represents listening-only mode, 2 represents self-test mode.

#pragma endregion Canalyst-II

#pragma region Kvaser

uint32_t ChannelNumber = 0;
uint32_t flags = 16; // Extended ID
long *CanBusFreq = new long(-3);
unsigned int *tseg1 = new unsigned int(0);
unsigned int *tseg2 = new unsigned int(0);
unsigned int *sjw = new unsigned int(0);
unsigned int *noSamp = new unsigned int(0);
unsigned int *syncmode = new unsigned int(0);
int32_t ChannelHandler = 0;
unsigned int dlc = 0;
unsigned int flag = 4;
unsigned int *kvaserFlag = new unsigned int();

#pragma endregion Kvaser

typedef struct MsgType {
	PVCI_CAN_OBJ sendbuf = new VCI_CAN_OBJ();	// Send data object using Canalyst-II
	PVCI_CAN_OBJ pCanObj = new VCI_CAN_OBJ();	// Receive data object using Canalyst-II
	CanEventMessage KvaserMsg;					// Send-Receive data object using Kvaser
	UDPObj udp_connection;						// Send-Receive data object using UDP
} _MsgType;

using namespace std;

/* General: In the functions VCI_Transmit and VCI_Receive they are used in each of the functions belows, VCI_CAN_OBJ structure is used to transmit CAN message frame.*/

#pragma region Internal_Function

UINT RevertID(UINT IDReceived) {

	return (IDReceived >> 16 << 16) | (IDReceived << 24) >> 16 | (IDReceived << 16) >> 24;
}
bool verifySWReset_ID(unsigned int ID_Tx, unsigned int ID_Rx) {
	uint16_t SW_Rst = (ID_Rx >> 16) & 0xFFFF;
	uint16_t tx = ID_Tx & 0xFFFF;
	uint16_t rx = ID_Rx & 0xFFFF;

	if ((SW_Rst == 0x18FA) && ((tx >> 8) == (rx & 0xFF)) && ((rx >> 8) == (tx & 0xFF))) return true;
	else return false;

}
bool verifySWReset_Data(unsigned char* data) {
	unsigned char temp[5] = { 0 };
	for (int i = 0; i < 5; i++)
		temp[i] = data[i];
	unsigned char data_tx_arr[][5]{
		{ 133, 162, 52, 25, 1 },
		{ 4, 139, 52, 170, 8 }
	};
	int index = 0;
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 5; j++) {
			if (temp[j] == data_tx_arr[i][j])
				index++;
		}
		if (index == 5)
			return true;
		index = 0;
	}
	return false;
}

// function used to get numeric data from the PowerRider Unit. \
//return 1 if the PowerRider Unit transmit a valid packet otherwise -1.
int rx(int Type, MsgType* message, unsigned char* fillData, long* MsgIDin) {
	uint8_t DataOut[8] = { 0 };
	int Timeout = 1;
	unsigned int *dlc = new unsigned int();
	int response = 0;
	switch (Type) {
	case CANALYST:
		response = VCI_Receive((DWORD)DeviceType, (DWORD)DeviceInd, (DWORD)CanIndex, message->pCanObj, 1, (ULONG)Timeout);
		*dlc = message->pCanObj->DataLen;
		for (int i = 0; i < (int)(*dlc); i++)
			fillData[i] = message->pCanObj->Data[i];
		*MsgIDin = message->pCanObj->ID;
		break;
	case CANKVASER:
		response = canRead(ChannelHandler, MsgIDin, DataOut, dlc, kvaserFlag, 0);
		for (int i = 0; i < (int)(*dlc); i++)
			fillData[i] = DataOut[i];
		break;
	default:
		return TYPE_NOT_DETECTED_CODE;
	}
	if (((fillData[0] & 0xF0) == SYSTEM_ERROR_PREFIX) && *dlc == 2)
		return SYSTEM_ERROR_CODE;
	return response;
}

// function used to get numeric data from the PowerRider Unit. \
//return 1 if the PowerRider Unit transmit a valid packet otherwise -1.
int GeneralCAN_Receive(int Type, MsgType* message, unsigned char* DataRx) {
	int sendNum = 0, recNum = 0, dataLenTx = 0, count = 0, ans = 0;
	long *MsgIDRx = new long(0), MsgIDTx = 0;
	uint8_t DataTx[8] = { 0 };// , DataRx[8] = { 0 };
	unsigned int *dlc = new unsigned int();

	switch (Type) {
	case CANALYST:
		sendNum = ((message->sendbuf->Data[0] << 16) | (((message->sendbuf->Data[1]) << 26) & 0xFF000000) | ((message->sendbuf->Data[3] << 21) & 0x03F00000));
		MsgIDTx = message->sendbuf->ID;
		dataLenTx = message->sendbuf->DataLen;
		for (int i = 0; i < dataLenTx; i++)
			DataTx[i] = message->sendbuf->Data[i];
		break;
	case CANKVASER:
		sendNum = ((message->KvaserMsg.data[0] << 16) | (((message->KvaserMsg.data[1]) << 26) & 0xFF000000) | ((message->KvaserMsg.data[3] << 21) & 0x03F00000));
		MsgIDTx = message->KvaserMsg.id;
		dataLenTx = message->KvaserMsg.dlc;
		for (int i = 0; i < dataLenTx; i++)
			DataTx[i] = message->KvaserMsg.data[i];
		break;
	default:
		return TYPE_NOT_DETECTED_CODE;
	}

	if ((((MsgIDTx >> 16) & 0xFF) == 0xB) || (((MsgIDTx >> 16) & 0xFF) == 0xC)) {
		if (DataTx[2] != 0x53) { // Verify if is a "set" operation
			if (DataTx[0] == 9 && (((MsgIDTx >> 16) & 0xFF) == 0xB)) {
				do {
					ans = rx(Type, message, DataRx, MsgIDRx);
					if (RevertID(*MsgIDRx) == MsgIDTx && DataTx[0] == DataRx[6] && DataTx[3] == DataRx[7])
						break;
					else {
						if (ans == SYSTEM_ERROR_CODE)
							break;
						count++;
						std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
					}
				} while (count <= MAX_RETRY_COUNT);
			}
			else if (DataTx[0] == 5 && (((MsgIDTx >> 16) & 0xFF) == 0xC)) {
				do {
					ans = rx(Type, message, DataRx, MsgIDRx);
					recNum = ((DataRx[7] << 8) | (DataRx[6])) << 16;
					if (RevertID(*MsgIDRx) == MsgIDTx && sendNum == recNum)
						break;
					else {
						if (ans == SYSTEM_ERROR_CODE)
							break;
						count++;
						std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
					}
				} while (count <= MAX_RETRY_COUNT);
			}
			else {
				do {
					ans = rx(Type, message, DataRx, MsgIDRx);
					if (RevertID(*MsgIDRx) == MsgIDTx && DataRx[6] == DataTx[0])
						break;
					else if (ans == -1)
						return RECEIVE_ERROR_CODE;
					else {
						if (ans == SYSTEM_ERROR_CODE)
							break;
						count++;
						std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
					}
				} while (count <= MAX_RETRY_COUNT);
			}
		}
		else {
			do {
				ans = rx(Type, message, DataRx, MsgIDRx);
				if (ans == -1)
					return RECEIVE_ERROR_CODE;
				else if (RevertID(*MsgIDRx) == MsgIDTx && DataRx[0] == DataTx[0])
					break;
				else {
					if (ans == SYSTEM_ERROR_CODE)
						break;
					count++;
					std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
				}
			} while (count <= MAX_RETRY_COUNT);
		}
	}
	else if ((MsgIDTx >> 16) == 0x1813 && verifySWReset_Data(DataTx)) {
		do {
			ans = rx(Type, message, DataRx, MsgIDRx);
			if (ans == 1 && verifySWReset_ID(MsgIDTx, *MsgIDRx))
				return MSG_OK;
			else if (ans == -1)
				return RECEIVE_ERROR_CODE;
			else {
				std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
				count++;
			}
		} while (!verifySWReset_ID(MsgIDTx, *MsgIDRx) && count <= MAX_RETRY_COUNT);
	}
	else {
		do {
			ans = rx(Type, message, DataRx, MsgIDRx);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			if (ans == SYSTEM_ERROR_CODE)
				break;
			count++;
		} while (RevertID(*MsgIDRx) != MsgIDTx && count <= MAX_RETRY_COUNT);
	}

	if (count >= MAX_RETRY_COUNT)
		return TIMEOUT_CODE;
	else if (ans == SYSTEM_ERROR_CODE)
		return SYSTEM_ERROR_CODE;
	return MSG_OK;
}

// helper fonction to concatenate string
string strcatCust(char *StringChannelOut, std::string String2Add, std::string StringIn) {

	return StringIn + String2Add;
}

// fill the message to be send to the PowerRider Unit
void FillsendBuff(unsigned char* dataIn, float ValIn) {
	dataIn[4] = (unsigned __int8)ValIn;
	dataIn[5] = (unsigned __int8)(static_cast<int>(ValIn) >> 8);
	dataIn[6] = (unsigned __int8)(static_cast<int>(ValIn) >> 16);
	dataIn[7] = (unsigned __int8)(static_cast<int>(ValIn) >> 24);
}

// fill the message to be send to the PowerRider Unit from string value only used for channel param no 19
bool FillsendBuffStr(unsigned char* dataIn, std::string ValIn) {
	try {
		dataIn[4] = (unsigned __int8)(std::stoi)(ValIn, nullptr, 10);
		dataIn[5] = (unsigned __int8)(static_cast<int>((std::stoi)(ValIn, nullptr, 10)) >> 8);
		dataIn[6] = (unsigned __int8)(static_cast<int>((std::stoi)(ValIn, nullptr, 10)) >> 16);
		dataIn[7] = (unsigned __int8)(static_cast<int>((std::stoi)(ValIn, nullptr, 10)) >> 24);

		return true;
	}
	catch (const std::exception&) { return false; };
}

// In case an entry value is invalid, the dll return Error code and no message will be send to the unit 
char* InternalError(std::string SystParamError, int ChannelNo, std::string ChParamError) {
	std::string temp;
	char *StringSystemOut = new char();

	if (ChannelNo != -1) {
		temp = "Error: 0xF" + std::to_string(ChannelNo) + ChParamError;
	}
	else {
		temp = "Error: ";
		temp += MAKE_HEX(SYSTEM_ERROR_PREFIX);
		temp += SystParamError;
	}
	StringSystemOut = new char[temp.length() + 5];
	strcpy_s(StringSystemOut, temp.length() + 1, temp.c_str());
	return StringSystemOut;
}

// Build the error code received from the PowerRider Unit and return it
char* Error(int Type = CANALYST, unsigned char* Data = NULL, char* erroReturn = NULL) {

	switch (Type) {
	case CANALYST:
		if (Data[0] != NULL) {
			std::string result = "Error: 0x";
			char *temp = (char*)malloc(1);
			temp = new char[5]; // buffer
			char *StringSystemOut = (char*)malloc(result.length() + 5 + 1);
			sprintf_s(temp, 5, "%X", ((Data[0] << 8) | (Data[1])));
			result += temp;
			strcpy_s(StringSystemOut, result.length() + 1, result.c_str());
			if (erroReturn != NULL)
				strcpy_s(erroReturn, result.length() + 1, result.c_str());
			return StringSystemOut;
		}
		else
			return UNKNOWN_ERROR;
	case UDP:
		if (Data[0] != NULL) {
			std::string result;
			std::stringstream stream;
			stream << std::hex << ((unsigned char)(Data[0]) << 8 | (unsigned char)(Data[1]));
			result = stream.str();
			std::transform(result.begin(), result.end(), result.begin(), ::toupper);
			result = "Error: 0x" + result;
			if (erroReturn != NULL)
				strcpy_s(erroReturn, result.length() + 1, result.c_str());
			return UNKNOWN_ERROR;
		}
		else
			return UNKNOWN_ERROR;
	case CANKVASER:
		if (Data[0] > 0 && Data[1] >= 0) {
			std::string result = "Error: 0x";
			char *temp = (char*)malloc(1);
			temp = new char[5]; // buffer
			char *StringSystemOut = (char*)malloc(result.length() + 5 + 1);
			sprintf_s(temp, 5, "%X", (Data[0] << 8 | Data[1]));
			result += temp;
			strcpy_s(StringSystemOut, result.length() + 1, result.c_str());
			if (erroReturn != NULL)
				strcpy_s(erroReturn, result.length() + 1, result.c_str());
			return StringSystemOut;
		}
		else if (Data[0] > 0 && Data[1] == 0) {
			static char errArr[1024] = {};
			std::string temp = "";
			canGetErrorText((canStatus)Data[0], errArr, 1024);
			for (int k = 0; k < 1024; k++) {
				if (errArr[k] != 0)
					temp += static_cast<char>(errArr[k]);
			}
			if (erroReturn != NULL)
				strcpy_s(erroReturn, temp.length() + 1, temp.c_str());
			memcpy(errArr, temp.c_str(), temp.size() + 1);
			return errArr;
		}
		else
			return TIMEOUT;
	}
	return TYPE_NOT_DETECTED;
}

void fillMsg(int Type, int opID, uint8_t ID, uint8_t dstAdd, MsgType* message, unsigned char* dataIn = NULL, int dataLen = NULL, sNETWORK* PsNetwork = NULL) {
	switch (Type) {
	case CANALYST:
		message->sendbuf->ID = PRIORITY << 26 | opID << 16 | ID << 8 | dstAdd;	// Identifier.
		message->sendbuf->RemoteFlag = 0;										// Whether it is a remote flag. = 1 indicates remote flag, = 0 indicates data flag.
		message->sendbuf->ExternFlag = 1; 										// Whether it is a extern flag. = 1 indicates extern flag, = 0 indicates standard flag.
		if (dataLen == 0) {
			message->sendbuf->DataLen = 0;										// Data length(<=8)? that is, the length of data.
			message->sendbuf->Data[8] = { 0 };
		}
		else {
			message->sendbuf->DataLen = dataLen;
			for (int i = 0; i < dataLen; i++)
				message->sendbuf->Data[i] = dataIn[i];
		}
		break;
	case CANKVASER:
		message->KvaserMsg.id = PRIORITY << 26 | opID << 16 | ID << 8 | dstAdd;
		message->KvaserMsg.flags = flag;
		if (dataLen == 0) {
			message->KvaserMsg.dlc = 0;
			message->KvaserMsg.data[8] = { 0 };
		}
		else {
			message->KvaserMsg.dlc = dataLen;
			for (int i = 0; i < dataLen; i++)
				message->KvaserMsg.data[i] = dataIn[i];
		}
		break;
	case UDP:
		message->udp_connection.Identifier = (PRIORITY << 26 | opID << 16) >> 16;
		message->udp_connection.tscvMessage = (char*)malloc(message->udp_connection.DataLen + 10 + 1);
		message->udp_connection.ID = ID << 8 | dstAdd;
		message->udp_connection.PSockaddr = PsNetwork->_udp.PSockaddr;
		message->udp_connection.sock = PsNetwork->_udp.sock;
		if (dataLen == 0) {
			message->udp_connection.DataLen = 0;
			message->udp_connection.tscvMessage = {};
		}
		else {
			message->udp_connection.DataLen = dataLen;
			for (int i = 0; i < dataLen; i++)
				message->udp_connection.tscvMessage[i] = dataIn[i];
		}
		//if (opID != 0x2E && opID != 0x2D) {
		//	(message->udp_connection.sock) = socket(AF_INET, SOCK_DGRAM, 0);
		//}
		break;
	}
}

int send(int Type, MsgType* message) {
	switch (Type) {
	case CANALYST:
		return VCI_Transmit((DWORD)DeviceType, (DWORD)DeviceInd, (DWORD)CanIndex, message->sendbuf, 1);
	case UDP:
		return UDP_Transmit(&message->udp_connection);
	case CANKVASER:
		return canWrite(ChannelHandler, message->KvaserMsg.id, message->KvaserMsg.data, message->KvaserMsg.dlc, flag);
	default:
		return TYPE_NOT_DETECTED_CODE;
	};
}

int receive(int Type, MsgType* message, unsigned char* fillData, int deltaT = 0) { // deltaT (ms) define the time delay between each send when its successivly (string operation, SN ...)
	int resp = 0;

	switch (Type) {
	case CANKVASER:
	case CANALYST:
		return GeneralCAN_Receive(Type, message, fillData);
	case UDP:
		resp = UDP_Receive(fillData, &message->udp_connection);
		std::this_thread::sleep_for(std::chrono::milliseconds(deltaT));
		return resp;
	default:
		return TYPE_NOT_DETECTED_CODE;
	};
}

char* ErrorDefinition(int code) {
	switch (code) {
	case SEND_ERROR_CODE:
		return SEND_ERROR;
	case TYPE_NOT_DETECTED_CODE:
		return TYPE_NOT_DETECTED;
	case RECEIVE_ERROR_CODE:
		return RECEIVE_ERROR;
	case TIMEOUT_CODE:
		return TIMEOUT;
	case SYSTEM_ERROR_CODE:
		return SYSTEM_ERROR;
	case MSG_LENGHT_ERROR_CODE:
		return MSG_LENGHT_ERROR;
	case SIZE_EXCEED:
		return SIZE_EXCEED_ERROR;
	default:
		return UNKNOWN_ERROR;
	}
}

/*
char* return_char_ans(char* buf, int buf_size) {

	size_t stSize = buf_size + sizeof(char);
	char* pszReturn = NULL;

	pszReturn = (char*)::CoTaskMemAlloc(stSize);
	// Copy the contents of test1
	// to the memory pointed to by pszReturn.
	strcpy_s(pszReturn, stSize, buf);
	// Return pszReturn.
	return pszReturn;
}
*/

/* get */
char* _unbuild(int Type, uint8_t ID, uint8_t dstAdd, int ChannelNo, int ParamNo, sNETWORK* PsNetwork, unsigned char* fillData, MsgType* message, int seq) {
	std::string temp = "", data = "";
	char *StringChannelOut = new char();
	int combined = 0;
	unsigned __int8 Data8[4];
	int _msg_id;
	int j_max, j_min;

	switch (seq) {
	case SYSTEM_PARAM:
		_msg_id = 0xB;
		j_min = 1;
		j_max = 2;
		break;
	case CHANNEL_PARAM:
		_msg_id = 0xC;
		j_min = 2;
		j_max = 4;
		break;
	}

	for (int j = 0; j < j_max; j++) {

		for (int i = 0; i < 4; i++)
			fillData[i] = 0;

		switch (seq) {
		case SYSTEM_PARAM:
			if (j >= j_min)
				fillData[0] = 23;
			else
				fillData[0] = ParamNo;
			fillData[3] = ChannelNo; // Index;
			break;
		case CHANNEL_PARAM:
			if (j >= j_min)
				fillData[0] = ParamNo + j + 1;
			else
				fillData[0] = ParamNo + j;
			fillData[1] = ChannelNo;
			break;
		}

		fillMsg(Type, _msg_id, ID, dstAdd, message, fillData, 4, PsNetwork);

		int txrx = send(Type, message);
		if (txrx < 0)
			return ErrorDefinition(txrx);
		std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TX_RX));

		txrx = receive(Type, message, fillData, DELTA_T);

		if (txrx < 0) {
			if (txrx == SYSTEM_ERROR_CODE) {
				static char errorReturn[14];
				Error(Type, fillData, errorReturn);
				delete[] message;
				return errorReturn;
			}
			delete[] message;
			return ErrorDefinition(txrx);
		}
		combined = (fillData[3] << 24) | (fillData[2] << 16) | (fillData[1] << 8) | (fillData[0]);
		Data8[0] = fillData[0];
		Data8[1] = fillData[1];
		Data8[2] = fillData[2];
		Data8[3] = fillData[3];

		for (int i = 0; i < 4; i++)
		{
			if (Data8[i] > 0 && Data8[i] <= LAST_CH)
			{
				temp += "In";
				if ((char)(Data8[i] + 48) == ':')
					temp += "10";
				else if ((char)(Data8[i] + 48) == ';')
					temp += "11";
				else if ((char)(Data8[i] + 48) == '<')
					temp += "12";
				else if ((char)(Data8[i] + 48) == '=')
					temp += "13";
				else if ((char)(Data8[i] + 48) == '>')
					temp += "14";
				else if ((char)(Data8[i] + 48) == '?')
					temp += "15";
				else if ((char)(Data8[i] + 48) == '@')
					temp += "16";
				else
					temp += (char)(Data8[i] + 48);
			}
			else if (Data8[i] == EMERGENCY)
				temp += "EMRG";
			else if (Data8[i] >= OUT1 && Data8[i] <= OUT4) {
				temp += "OUT";
				temp += (char)(Data8[i] - OUT1 + ACII_0_CONST);
			}
			else if (Data8[i] == LTCM)
				temp += "LTCM";
			else if (Data8[i] == PARAMERR)
				temp += "EPAR";
			else if (Data8[i] == TEMPERR)
				temp += "ETEM";
			else if (Data8[i] == VOLTERR)
				temp += "EVOL";
			else if (Data8[i] == BOM)
				temp += "EBOM";
			else if (Data8[i] == BITdef)
				temp += "EBIT";
			else if (Data8[i] == ANY_TRIP)
				temp += "TANCH";
			else if (Data8[i] == VIN1_LOW)
				temp += "EV1L";
			else if (Data8[i] == VIN1_HIGH)
				temp += "EV1H";
			else if (Data8[i] == VIN2_LOW)
				temp += "EV2L";
			else if (Data8[i] == VIN2_HIGH)
				temp += "EV2H";
			else if (Data8[i] == VIN1)
				temp += "EVI1";
			else if (Data8[i] == VIN2)
				temp += "EVI2";
			else if (Data8[i] >= CH1_STS && Data8[i] <= (CH1_STS + LAST_CH)) {
				temp += "SCH";
				temp += (char)(Data8[i] - CH1_STS + ACII_0_CONST);
			}
			else if (Data8[i] >= CH1_TRIP && Data8[i] <= (CH1_TRIP + LAST_CH)) {
				temp += "TCH";
				temp += (char)(Data8[i] - CH1_TRIP + ACII_0_CONST);
			}
			else if (Data8[i] >= CH1_ERR && Data8[i] <= (CH1_ERR + LAST_CH)) {
				temp += "ECH";
				temp += (char)(Data8[i] - CH1_ERR + ACII_0_CONST);
			}
			else if (Data8[i] >= NOT && Data8[i] <= (NOT + LAST_CH))
			{
				temp += "!In";
				if ((char)(Data8[i] - NOT + 48) == ':')
					temp += "10";
				else if ((char)(Data8[i] - NOT + 48) == ';')
					temp += "11";
				else if ((char)(Data8[i] - NOT + 48) == '<')
					temp += "12";
				else if ((char)(Data8[i] - NOT + 48) == '=')
					temp += "13";
				else if ((char)(Data8[i] - NOT + 48) == '>')
					temp += "14";
				else if ((char)(Data8[i] - NOT + 48) == '?')
					temp += "15";
				else if ((char)(Data8[i] - NOT + 48) == '@')
					temp += "16";
				else
					temp += (char)(Data8[i] - NOT + 48);
			}
			else if (Data8[i] == NOT + EMERGENCY)
				temp += "!EMRG";
			else if (Data8[i] == NOT + LTCM)
				temp += "!LTCM";
			else if (Data8[i] >= (NOT + OUT1) && Data8[i] <= (NOT + OUT4)) {
				temp += "!OUT";
				temp += (char)(Data8[i] - (NOT + OUT1) + ACII_0_CONST);
			}
			else if (Data8[i] == NOT + PARAMERR)
				temp += "!EPAR";
			else if (Data8[i] == NOT + TEMPERR)
				temp += "!ETEM";
			else if (Data8[i] == NOT + VOLTERR)
				temp += "!EVOL";
			else if (Data8[i] == NOT + BOM)
				temp += "!EBOM";
			else if (Data8[i] == NOT + BITdef)
				temp += "!EBIT";
			else if (Data8[i] == NOT + ANY_TRIP)
				temp += "!TANCH";
			else if (Data8[i] == NOT + VIN1_LOW)
				temp += "!EV1L";
			else if (Data8[i] == NOT + VIN1_HIGH)
				temp += "!EV1H";
			else if (Data8[i] == NOT + VIN2_LOW)
				temp += "!EV2L";
			else if (Data8[i] == NOT + VIN2_HIGH)
				temp += "!EV2H";
			else if (Data8[i] == NOT + VIN1)
				temp += "!EVI1";
			else if (Data8[i] == NOT + VIN2)
				temp += "!EVI2";
			else if (Data8[i] >= (NOT + CH1_STS) && Data8[i] <= (NOT + CH16_STS)) {
				temp += "!SCH";
				temp += (char)(Data8[i] - (NOT + CH1_STS) + ACII_0_CONST);
			}
			else if (Data8[i] >= (NOT + CH1_TRIP) && Data8[i] <= (NOT + CH16_TRIP)) {
				temp += "!TCH";
				temp += (char)(Data8[i] - (NOT + CH1_TRIP) + ACII_0_CONST);
			}
			else if (Data8[i] >= (NOT + CH1_ERR) && Data8[i] <= (NOT + CH16_ERR)) {
				temp += "!ECH";
				temp += (char)(Data8[i] - (NOT + CH1_ERR) + ACII_0_CONST);
			}
			else if (Data8[i] == ADN_OPER)
				temp += "&";
			else if (Data8[i] == OR_OPER)
				temp += "|";
			else if (Data8[i] == XOR_OPER)
				temp += "^";
			else
				temp += "";
		}
		std::this_thread::sleep_for(std::chrono::microseconds(WAIT_RX_TX));
		message = new MsgType();
	}
	StringChannelOut = new char[temp.size() + 1];
	strcpy_s(StringChannelOut, temp.length() + 1, temp.c_str());
	delete[] message;
	if (StringChannelOut[0] == '.')
		return UNKNOWN_ERROR;
	return StringChannelOut;
}

/* set */
char* _build_control(int Type, uint8_t ID, uint8_t dstAdd, MsgType* message, sNETWORK* PsNetwork, int k, unsigned char* fillData, int ParamNo, int ChannelNo, unsigned char* DataOut, unsigned char* DataIn, int Val, int seq) {
	flag = false;
	int index = 0, DataIndex = 0;
	if (k == 0)
		flag = true;
#ifdef OLD_CHANNEL_PARAM_25
	while (!flag) {
		if (DataIn[index] == 'I') {
			index++;
			if (DataIn[index] == 'N') {
				index++;
				if (DataIn[index] > '0' && DataIn[index] <= '9') {
					Val = DataIn[index] - 48;
					index++;
					if (DataIn[index] > '0' && DataIn[index] <= '2') {
						Val *= 10;
						Val += DataIn[index] - 48;
						DataOut[DataIndex] = Val;
						index++;
						DataIndex++;
					}
					else {
						DataOut[DataIndex] = Val;
						DataIndex++;
					}
				}
				else return InternalError("DD", -1, "");
			}
			else return InternalError("DD", -1, "");
		}
		else if (DataIn[index] == 'S') {
			index++;
			if (DataIn[index] == 'C') {
				index++;
				if (DataIn[index] == 'H') {
					index++;
					if (DataIn[index] > '0' && DataIn[index] <= '9') {
						Val = DataIn[index] - 48;
						index++;
						if (DataIn[index] >= '0' && DataIn[index] <= '2') {
							Val *= 10;
							Val += DataIn[index] - 48;
							DataOut[DataIndex] = Val + 23;
							index++;
							DataIndex++;
						}
						else {
							DataOut[DataIndex] = Val + 23;
							DataIndex++;
						}
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else return InternalError("DD", -1, "");
		}
		else if (DataIn[index] == 'T') {
			index++;
			if (DataIn[index] == 'C') {
				index++;
				if (DataIn[index] == 'H') {
					index++;
					if (DataIn[index] > '0' && DataIn[index] <= '9') {
						Val = DataIn[index] - 48;
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '2') {
							Val *= 10;
							Val += DataIn[index] - 48;
							DataOut[DataIndex] = Val + 35;
							index++;
							DataIndex++;
						}
						else {
							DataOut[DataIndex] = Val + 35;
							DataIndex++;
						}
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else return InternalError("DD", -1, "");
		}
		else if (DataIn[index] == 'O') {
			index++;
			if (DataIn[index] == 'U') {
				index++;
				if (DataIn[index] == 'T') {
					index++;
					if (DataIn[index] > '0' && DataIn[index] <= '4') {
						Val = DataIn[index] - 48;
						DataOut[DataIndex] = Val + 13;
						DataIndex++; index++;
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else return InternalError("DD", -1, "");
		}
		else if (DataIn[index] == 'E') {
			index++;
			if (DataIn[index] == 'C') {
				index++;
				if (DataIn[index] == 'H') {
					index++;
					if (DataIn[index] > '0' && DataIn[index] <= '9') {
						Val = DataIn[index] - 48;
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '2') {
							Val *= 10;
							Val += DataIn[index] - 48;
							DataOut[DataIndex] = Val + 47;
							index++;
							DataIndex++;
						}
						else {
							DataOut[DataIndex] = Val + 47;
							DataIndex++;
						}
					}
					else flag = true;
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'V') {
				index++;
				if (DataIn[index] == 'O') {
					index++;
					if (DataIn[index] == 'L') {
						DataOut[DataIndex] = 20;
						DataIndex++; index++;
					}
					else flag = true;
				}
				else flag = true;
			}
			else if (DataIn[index] == 'B') {
				index++;
				if (DataIn[index] == 'I') {
					index++;
					if (DataIn[index] == 'T') {
						DataOut[DataIndex] = 22;
						DataIndex++; index++;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'O') {
					index++;
					if (DataIn[index] == 'M') {
						DataOut[DataIndex] = 21;
						DataIndex++; index++;
					}
					else flag = true;
				}
				else flag = true;
			}
			else if (DataIn[index] == 'P') {
				index++;
				if (DataIn[index] == 'A') {
					index++;
					if (DataIn[index] == 'R') {
						DataOut[DataIndex] = 18;
						DataIndex++; index++;
					}
					else flag = true;
				}
				else flag = true;
			}
			else if (DataIn[index] == 'M') {
				index++;
				if (DataIn[index] == 'R') {
					index++;
					if (DataIn[index] == 'G') {
						DataOut[DataIndex] = 13;
						DataIndex++; index++;
					}
					else flag = true;
				}
				else flag = true;
			}
			else if (DataIn[index] == 'T') {
				index++;
				if (DataIn[index] == 'E') {
					index++;
					if (DataIn[index] == 'M') {
						DataOut[DataIndex] = 19;
						DataIndex++; index++;
					}
					else flag = true;
				}
				else flag = true;
			}
			else return InternalError("DD", -1, "");
		}
		else if (DataIn[index] == '!') {
			index++;
			if (DataIn[index] == 'I') {
				index++;
				if (DataIn[index] == 'N') {
					index++;
					if (DataIn[index] > '0' && DataIn[index] <= '9') {
						Val = DataIn[index] - 48;
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '2') {
							Val *= 10;
							Val += DataIn[index] - 48;
							DataOut[DataIndex] = Val + 60;
							index++;
							DataIndex++;
						}
						else {
							DataOut[DataIndex] = Val + 60;
							DataIndex++;
						}
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'S') {
				index++;
				if (DataIn[index] == 'C') {
					index++;
					if (DataIn[index] == 'H') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - 48;
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '2') {
								Val *= 10;
								Val += DataIn[index] - 48;
								DataOut[DataIndex] = Val + 23 + 60;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + 23 + 60;
								DataIndex++;
							}
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'T') {
				index++;
				if (DataIn[index] == 'C') {
					index++;
					if (DataIn[index] == 'H') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - 48;
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '2') {
								Val *= 10;
								Val += DataIn[index] - 48;
								DataOut[DataIndex] = Val + 35 + 60;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + 35 + 60;
								DataIndex++;
							}
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'O') {
				index++;
				if (DataIn[index] == 'U') {
					index++;
					if (DataIn[index] == 'T') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '4') {
							Val = DataIn[index] - 48;
							DataOut[DataIndex] = Val + 13 + 60;
							DataIndex++; index++;
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'E') {
				index++;
				if (DataIn[index] == 'C') {
					index++;
					if (DataIn[index] == 'H') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - 48;
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '2') {
								Val *= 10;
								Val += DataIn[index] - 48;
								DataOut[DataIndex] = Val + 47 + 60;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + 47 + 60;
								DataIndex++;
							}
						}
						else flag = true;
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'V') {
					index++;
					if (DataIn[index] == 'O') {
						index++;
						if (DataIn[index] == 'L') {
							DataOut[DataIndex] = 20 + 60;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'B') {
					index++;
					if (DataIn[index] == 'I') {
						index++;
						if (DataIn[index] == 'T') {
							DataOut[DataIndex] = 22 + 60;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'O') {
						index++;
						if (DataIn[index] == 'M') {
							DataOut[DataIndex] = 21 + 60;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'P') {
					index++;
					if (DataIn[index] == 'A') {
						index++;
						if (DataIn[index] == 'R') {
							DataOut[DataIndex] = 18 + 60;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'M') {
					index++;
					if (DataIn[index] == 'R') {
						index++;
						if (DataIn[index] == 'G') {
							DataOut[DataIndex] = 13 + 60;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'T') {
					index++;
					if (DataIn[index] == 'E') {
						index++;
						if (DataIn[index] == 'M') {
							DataOut[DataIndex] = 19 + 60;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == ' ')
				flag = false;
			else if (DataIn[index] == 200)
				flag = true;
			else flag = true;
		}
		else if (DataIn[index] == '&') {
			DataOut[DataIndex] = 120;
			DataIndex++; index++;
		}
		else if (DataIn[index] == '|') {
			DataOut[DataIndex] = 121;
			DataIndex++; index++;
		}
		else if (DataIn[index] == '^') {
			DataOut[DataIndex] = 122;
			DataIndex++; index++;
		}
		else if (DataIn[index] == ' ') {
			index++;
		}
		else {
			flag = true;
			return InternalError("DD", -1, "");
		}
		if (index == k)
			flag = true;
	}
#else //  OLD_CHANNEL_PARAM_25
	while (!flag) {
		if (DataIn[index] == 'I') {
			index++;
			if (DataIn[index] == 'N') {
				index++;
				if (DataIn[index] > '0' && DataIn[index] <= '9') {
					Val = DataIn[index] - 48;
					index++;
					if (DataIn[index] >= '0' && DataIn[index] <= CHANNEL_NUMBER_UNIT_DIGIT) { // 16 channels
						Val *= 10;
						Val += DataIn[index] - 48;
						DataOut[DataIndex] = Val;
						index++;
						DataIndex++;
					}
					else {
						DataOut[DataIndex] = Val;
						DataIndex++;
					}
				}
				else return InternalError("DD", -1, "");
			}
			else return InternalError("DD", -1, "");
		}
		else if (DataIn[index] == 'S') {
			index++;
			if (DataIn[index] == 'C') {
				index++;
				if (DataIn[index] == 'H') {
					index++;
					if (DataIn[index] > '0' && DataIn[index] <= '9') {
						Val = DataIn[index] - ACII_0_CONST;
						index++;
						if (DataIn[index] >= '0' && DataIn[index] <= CHANNEL_NUMBER_UNIT_DIGIT) {
							Val *= 10;
							Val += DataIn[index] - ACII_0_CONST;
							DataOut[DataIndex] = Val + CH1_STS;
							index++;
							DataIndex++;
						}
						else {
							DataOut[DataIndex] = Val + CH1_STS;
							DataIndex++;
						}
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else return InternalError("DD", -1, "");
		}
		else if (DataIn[index] == 'T') {
			index++;
			if (DataIn[index] == 'A') {
				index++;
				if (DataIn[index] == 'N') {
					index++;
					if (DataIn[index] == 'C') {
						index++;
						if (DataIn[index] == 'H') {
							DataOut[DataIndex] = ANY_TRIP;
							DataIndex++; index++;
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'C') {
				index++;
				if (DataIn[index] == 'H') {
					index++;
					if (DataIn[index] > '0' && DataIn[index] <= '9') {
						Val = DataIn[index] - ACII_0_CONST;
						index++;
						if (DataIn[index] >= '0' && DataIn[index] <= CHANNEL_NUMBER_UNIT_DIGIT) {
							Val *= 10;
							Val += DataIn[index] - ACII_0_CONST;
							DataOut[DataIndex] = Val + CH1_TRIP;
							index++;
							DataIndex++;
						}
						else {
							DataOut[DataIndex] = Val + CH1_TRIP;
							DataIndex++;
						}
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else return InternalError("DD", -1, "");
		}
		else if (DataIn[index] == 'O') {
			index++;
			if (DataIn[index] == 'U') {
				index++;
				if (DataIn[index] == 'T') {
					index++;
					if (DataIn[index] > '0' && DataIn[index] <= '4') {
						Val = DataIn[index] - ACII_0_CONST;
						DataOut[DataIndex] = Val + OUT1;
						DataIndex++; index++;
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else return InternalError("DD", -1, "");
		}
		else if (DataIn[index] == 'E') {
			index++;
			if (DataIn[index] == 'C') {
				index++;
				if (DataIn[index] == 'H') {
					index++;
					if (DataIn[index] > '0' && DataIn[index] <= '9') {
						Val = DataIn[index] - ACII_0_CONST;
						index++;
						if (DataIn[index] >= '0' && DataIn[index] <= CHANNEL_NUMBER_UNIT_DIGIT) {
							Val *= 10;
							Val += DataIn[index] - ACII_0_CONST;
							DataOut[DataIndex] = Val + CH1_ERR;
							index++;
							DataIndex++;
						}
						else {
							DataOut[DataIndex] = Val + CH1_ERR;
							DataIndex++;
						}
					}
					else flag = true;
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'V') {
				index++;
				if (DataIn[index] == '1') {
					index++;
					if (DataIn[index] == 'L') {
						DataOut[DataIndex] = VIN1_LOW;
						DataIndex++; index++;
					}
					else if (DataIn[index] == 'H') {
						DataOut[DataIndex] = VIN1_HIGH;
						DataIndex++; index++;
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == '2') {
					index++;
					if (DataIn[index] == 'L') {
						DataOut[DataIndex] = VIN2_LOW;
						DataIndex++; index++;
					}
					else if (DataIn[index] == 'H') {
						DataOut[DataIndex] = VIN2_HIGH;
						DataIndex++; index++;
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'I') {
					index++;
					if (DataIn[index] == '1') {
						DataOut[DataIndex] = VIN1;
						DataIndex++; index++;
					}
					else if (DataIn[index] == '2') {
						DataOut[DataIndex] = VIN2;
						DataIndex++; index++;
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'O') {
					index++;
					if (DataIn[index] == 'L') {
						DataOut[DataIndex] = VOLTERR;
						DataIndex++; index++;
					}
					else flag = true;
				}
				else flag = true;
			}
			else if (DataIn[index] == 'B') {
				index++;
				if (DataIn[index] == 'I') {
					index++;
					if (DataIn[index] == 'T') {
						DataOut[DataIndex] = BITdef;
						DataIndex++; index++;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'O') {
					index++;
					if (DataIn[index] == 'M') {
						DataOut[DataIndex] = BOM;
						DataIndex++; index++;
					}
					else flag = true;
				}
				else flag = true;
			}
			else if (DataIn[index] == 'P') {
				index++;
				if (DataIn[index] == 'A') {
					index++;
					if (DataIn[index] == 'R') {
						DataOut[DataIndex] = PARAMERR;
						DataIndex++; index++;
					}
					else flag = true;
				}
				else flag = true;
			}
			else if (DataIn[index] == 'M') {
				index++;
				if (DataIn[index] == 'R') {
					index++;
					if (DataIn[index] == 'G') {
						DataOut[DataIndex] = EMERGENCY;
						DataIndex++; index++;
					}
					else flag = true;
				}
				else flag = true;
			}
			else if (DataIn[index] == 'T') {
				index++;
				if (DataIn[index] == 'E') {
					index++;
					if (DataIn[index] == 'M') {
						DataOut[DataIndex] = TEMPERR;
						DataIndex++; index++;
					}
					else flag = true;
				}
				else flag = true;
			}
			else return InternalError("DD", -1, "");
		}
		else if (DataIn[index] == 'L') {
			index++;
			if (DataIn[index] == 'T') {
				index++;
				if (DataIn[index] == 'C') {
					index++;
					if (DataIn[index] == 'M') {
						DataOut[DataIndex] = LTCM;
						DataIndex++; index++;
					}
					else flag = true;
				}
				else return InternalError("DD", -1, "");
			}
			else return InternalError("DD", -1, "");
		}
		else if (DataIn[index] == '!') {
			index++;
			if (DataIn[index] == 'I') {
				index++;
				if (DataIn[index] == 'N') {
					index++;
					if (DataIn[index] > '0' && DataIn[index] <= '9') {
						Val = DataIn[index] - 48;
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '2') {
							Val *= 10;
							Val += DataIn[index] - 48;
							DataOut[DataIndex] = Val + NOT;
							index++;
							DataIndex++;
						}
						else {
							DataOut[DataIndex] = Val + NOT;
							DataIndex++;
						}
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'S') {
				index++;
				if (DataIn[index] == 'C') {
					index++;
					if (DataIn[index] == 'H') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - ACII_0_CONST;
							index++;
							if (DataIn[index] >= '0' && DataIn[index] <= CHANNEL_NUMBER_UNIT_DIGIT) {
								Val *= 10;
								Val += DataIn[index] - ACII_0_CONST;
								DataOut[DataIndex] = Val + CH1_STS + NOT;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + CH1_STS + NOT;
								DataIndex++;
							}
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'T') {
				index++;
				if (DataIn[index] == 'A') {
					index++;
					if (DataIn[index] == 'N') {
						index++;
						if (DataIn[index] == 'C') {
							index++;
							if (DataIn[index] == 'H') {
								DataOut[DataIndex] = ANY_TRIP + NOT;
								DataIndex++; index++;
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'C') {
					index++;
					if (DataIn[index] == 'H') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - ACII_0_CONST;
							index++;
							if (DataIn[index] >= '0' && DataIn[index] <= CHANNEL_NUMBER_UNIT_DIGIT) {
								Val *= 10;
								Val += DataIn[index] - ACII_0_CONST;
								DataOut[DataIndex] = Val + CH1_TRIP + NOT;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + CH1_TRIP + NOT;
								DataIndex++;
							}
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'O') {
				index++;
				if (DataIn[index] == 'U') {
					index++;
					if (DataIn[index] == 'T') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '4') {
							Val = DataIn[index] - ACII_0_CONST;
							DataOut[DataIndex] = Val + OUT1 + NOT;
							DataIndex++; index++;
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'E') {
				index++;
				if (DataIn[index] == 'C') {
					index++;
					if (DataIn[index] == 'H') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - ACII_0_CONST;
							index++;
							if (DataIn[index] >= '0' && DataIn[index] <= CHANNEL_NUMBER_UNIT_DIGIT) {
								Val *= 10;
								Val += DataIn[index] - ACII_0_CONST;
								DataOut[DataIndex] = Val + CH1_ERR + NOT;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + CH1_ERR + NOT;
								DataIndex++;
							}
						}
						else flag = true;
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'V') {
					index++;
					if (DataIn[index] == '1') {
						index++;
						if (DataIn[index] == 'L') {
							DataOut[DataIndex] = VIN1_LOW + NOT;
							DataIndex++; index++;
						}
						else if (DataIn[index] == 'H') {
							DataOut[DataIndex] = VIN1_HIGH + NOT;
							DataIndex++; index++;
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == '2') {
						index++;
						if (DataIn[index] == 'L') {
							DataOut[DataIndex] = VIN2_LOW + NOT;
							DataIndex++; index++;
						}
						else if (DataIn[index] == 'H') {
							DataOut[DataIndex] = VIN2_HIGH + NOT;
							DataIndex++; index++;
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == 'I') {
						index++;
						if (DataIn[index] == '1') {
							DataOut[DataIndex] = VIN1 + NOT;
							DataIndex++; index++;
						}
						else if (DataIn[index] == '2') {
							DataOut[DataIndex] = VIN2 + NOT;
							DataIndex++; index++;
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == 'O') {
						index++;
						if (DataIn[index] == 'L') {
							DataOut[DataIndex] = VOLTERR + NOT;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'B') {
					index++;
					if (DataIn[index] == 'I') {
						index++;
						if (DataIn[index] == 'T') {
							DataOut[DataIndex] = BITdef + NOT;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'O') {
						index++;
						if (DataIn[index] == 'M') {
							DataOut[DataIndex] = BOM + NOT;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'P') {
					index++;
					if (DataIn[index] == 'A') {
						index++;
						if (DataIn[index] == 'R') {
							DataOut[DataIndex] = PARAMERR + NOT;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'M') {
					index++;
					if (DataIn[index] == 'R') {
						index++;
						if (DataIn[index] == 'G') {
							DataOut[DataIndex] = EMERGENCY + NOT;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'T') {
					index++;
					if (DataIn[index] == 'E') {
						index++;
						if (DataIn[index] == 'M') {
							DataOut[DataIndex] = TEMPERR + NOT;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'L') {
				index++;
				if (DataIn[index] == 'T') {
					index++;
					if (DataIn[index] == 'C') {
						index++;
						if (DataIn[index] == 'M') {
							DataOut[DataIndex] = LTCM + NOT;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == ' ')
				flag = false;
			else if (DataIn[index] == 200)
				flag = true;
			else flag = true;
		}
		else if (DataIn[index] == '&') {
			DataOut[DataIndex] = ADN_OPER;
			DataIndex++; index++;
		}
		else if (DataIn[index] == '|') {
			DataOut[DataIndex] = OR_OPER;
			DataIndex++; index++;
		}
		else if (DataIn[index] == '^') {
			DataOut[DataIndex] = XOR_OPER;
			DataIndex++; index++;
		}
		else if (DataIn[index] == ' ') {
			index++;
		}
		else {
			flag = true;
			return InternalError("DD", -1, "");
		}
		if (index == k)
			flag = true;
	}
#endif //  OLD_CHANNEL_PARAM_25

	if (seq == CHANNEL_PARAM) {
		index = 0;
		{
			fillData[0] = ParamNo;
			fillData[1] = ChannelNo;
			fillData[2] = 83; //'S' Setv
			fillData[4] = DataOut[0];
			fillData[5] = DataOut[1];
			fillData[6] = DataOut[2];
			fillData[7] = DataOut[3];
			fillMsg(Type, 0xC, ID, dstAdd, message, fillData, 8, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			txrx = receive(Type, message, fillData, DELTA_T);

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					return errorReturn;
				}
				return ErrorDefinition(txrx);
			}
			index += txrx;
		}
		{
			fillData[0] = 26;
			fillData[1] = ChannelNo;
			fillData[2] = 83; //'S' Setv
			fillData[4] = DataOut[4];
			fillData[5] = DataOut[5];
			fillData[6] = DataOut[6];
			fillData[7] = DataOut[7];
			fillMsg(Type, 0xC, ID, dstAdd, message, fillData, 8, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			txrx = receive(Type, message, fillData, DELTA_T);

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					return errorReturn;
				}
				return ErrorDefinition(txrx);
			}
			index += txrx;
		}
		{
			fillData[0] = 28;
			fillData[1] = ChannelNo;
			fillData[2] = 83; //'S' Set
			fillData[4] = DataOut[8];
			fillData[5] = DataOut[9];
			fillData[6] = DataOut[10];
			fillData[7] = DataOut[11];
			fillMsg(Type, 0xC, ID, dstAdd, message, fillData, 8, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			txrx = receive(Type, message, fillData, DELTA_T);

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					return errorReturn;
				}
				return ErrorDefinition(txrx);
			}
			index += txrx;
		}
		{
			fillData[0] = 29;
			fillData[1] = ChannelNo;
			fillData[2] = 83; //'S' Set
			fillData[4] = DataOut[12];
			fillData[5] = DataOut[13];
			fillData[6] = DataOut[14];
			fillData[7] = DataOut[15];
			fillMsg(Type, 0xC, ID, dstAdd, message, fillData, 8, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			txrx = receive(Type, message, fillData);
			delete[] message;

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					return errorReturn;
				}
				return ErrorDefinition(txrx);
			}
			index += txrx;
		}
		if (index == 4)
			return "success";
		return UNKNOWN_ERROR;
	}
	else if (seq == SYSTEM_PARAM) {
		index = 0;
		{
			fillData[4] = DataOut[0];
			fillData[5] = DataOut[1];
			fillData[6] = DataOut[2];
			fillData[7] = DataOut[3];

			fillMsg(Type, 0xB, ID, dstAdd, message, fillData, 8, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			txrx = receive(Type, message, fillData, DELTA_T);

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					delete[] message;
					return errorReturn;
				}
				delete[] message;
				return ErrorDefinition(txrx);
			}
			index += txrx;
		}
		std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
		{
			fillData[0] = 23;
			fillData[4] = DataOut[4];
			fillData[5] = DataOut[5];
			fillData[6] = DataOut[6];
			fillData[7] = DataOut[7];

			fillMsg(Type, 0xB, ID, dstAdd, message, fillData, 8, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			txrx = receive(Type, message, fillData);

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					delete[] message;
					return errorReturn;
				}
				delete[] message;
				return ErrorDefinition(txrx);
			}
			index += txrx;
		}
		delete[] message;
		if (index == 2)
			return "success";
		return UNKNOWN_ERROR;
	}
}

#pragma endregion Internal_Function

// return the dll version
__declspec(dllexport) char* __cdecl dllVersion() {

	return DLL_VERSION;
}

// Open, Initialize and Start Can Bus connection.
__declspec(dllexport) char* __cdecl OpenCan(
	int Type,
	unsigned long BaudRate) {

	if (Type == CANALYST) {
		unsigned char Timing0, Timing1;
		switch (BaudRate)	// Baud rate parameter options.
		{
		case 5000:Timing0 = 0x9F; Timing1 = 0xFF; break;
		case 10000:Timing0 = 0x9F; Timing1 = 0xFF; break;
		case 20000:Timing0 = 0x18; Timing1 = 0x1c; break;
		case 40000:Timing0 = 0x87; Timing1 = 0xff; break;
		case 50000:Timing0 = 0x09; Timing1 = 0x1c; break;
		case 80000:Timing0 = 0x83; Timing1 = 0xff; break;
		case 100000:Timing0 = 0x04; Timing1 = 0x1c; break;
		case 125000:Timing0 = 0x03; Timing1 = 0x1c; break;
		case 200000:Timing0 = 0x81; Timing1 = 0xfa; break;
		case 250000:Timing0 = 0x01; Timing1 = 0x1c; break;
		case 400000:Timing0 = 0x80; Timing1 = 0xfa; break;
		case 500000:Timing0 = 0x00; Timing1 = 0x1c; break;
		case 666000:Timing0 = 0x80; Timing1 = 0xb6; break;
		case 800000:Timing0 = 0x00; Timing1 = 0x16; break;
		case 1000000:Timing0 = 0x00; Timing1 = 0x14; break;
		default:Timing0 = 0x01; Timing1 = 0x1C;
		}

		VCI_INIT_CONFIG CanInit[1];		// Structure defines the initialization configuration of the CAN. The structure will be filled in VCI_InitCan function.
		CanInit->Timing0 = Timing0;		// Baud rate parameter.
		CanInit->Timing1 = Timing1;		// Baud rate parameter.
		CanInit->Filter = Filter;
		CanInit->AccCode = AccCode;
		CanInit->AccMask = AccMask;
		CanInit->Mode = Mode;
		CanInit->Reserved = Reserved;

		int OpenDevice = 0, InitCAN = 0, StartCAN = 0;	// These variables are flags to indicate if the function below succeeded. 

														// Each functions Return value = 1, which means that the operation is successful; Return 0 indicates that the operation failed; Return -1 indicates that the device does not exist.

		OpenDevice = VCI_OpenDevice((DWORD)DeviceType, (DWORD)DeviceInd, (DWORD)Reserved);		// This function is used to connect devices.
		InitCAN = VCI_InitCAN((DWORD)DeviceType, (DWORD)DeviceInd, (DWORD)CanIndex, (PVCI_INIT_CONFIG)CanInit);
		StartCAN = VCI_StartCAN((DWORD)DeviceType, (DWORD)DeviceInd, (DWORD)CanIndex);

		int Status[3] = { OpenDevice, InitCAN, StartCAN };

		int sum = 0;
		for (int i = 0; i < 3; i++)
			sum += Status[i];		// The sum returned by the function must be 3.
		if (sum == 3)
			return "success";
		else
			return "failed openning CAN";
	}
	else if (Type == CANKVASER) {
		unsigned char errArr[1024] = { '\0' };
		switch (BaudRate)	// Baud rate parameter options.
		{
		case 50000:  *CanBusFreq = -7; break;
		case 62000:  *CanBusFreq = -6; break;
		case 100000: *CanBusFreq = -5; break;
		case 125000: *CanBusFreq = -4; break;
		case 250000: *CanBusFreq = -3; break;
		case 500000: *CanBusFreq = -2; break;
		case 1000000:*CanBusFreq = -1; break;
		default:	 *CanBusFreq = -3;
		}
		canInitializeLibrary();
		canOpenChannel(ChannelNumber, flags);
		canTranslateBaud(CanBusFreq, tseg1, tseg2, sjw, noSamp, syncmode);
		canSetBusParams(ChannelHandler, *CanBusFreq, *tseg1, *tseg2, *sjw, *noSamp, *syncmode);
		canAccept(ChannelHandler, 0, 6);
		errArr[0] = canBusOn(ChannelHandler);
		if (errArr[0] == 0)
			return "success";
		else
			return Error(Type, errArr);
	}
	else
		return "Type Not Found!";
}

// Establish UDP Communication.
#if defined _M_IX86
typedef char* T;
#elif defined _M_X64
typedef const wchar_t* T;
#endif
__declspec(dllexport) int __cdecl OpenUDP(
	T IP,
	int Port,
	/*SOCKADDR_IN* PsNetwork,
	SOCKET* PSock*/
	sNETWORK* PsNetwork) {
	PsNetwork->_udp.PSockaddr = new SOCKADDR_IN();
	PsNetwork->_udp.sock = new SOCKET();

	return (OpenUDPCom(IP, Port, PsNetwork->_udp.PSockaddr, PsNetwork->_udp.sock));
}

// Read any received message.
__declspec(dllexport) char* __cdecl ReadAddon(
	int Type,
	sNETWORK* PsNetwork = NULL) {

	uint8_t DataOut[8] = { 0 };
	int Timeout = 1;
	unsigned int *dlc = new unsigned int();
	int response = 0;
	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();
	long *MsgIDin = new long(0);
	std::stringstream stream;

#pragma region UDP_def
	char MESSAGE[18] = {};
	int len = sizeof(message->udp_connection.PSockaddr);

	if (Type == UDP) {
		int count = 0, resp = 0;

		/* Socket file descriptors we want to wake
		up for, using select() */
		fd_set stReadFDS;

		/* FD_ZERO() clears out the fd_set called socks, so that
		it doesn't contain any file descriptors. */
		FD_ZERO(&stReadFDS);

		// Timeout of one second
		struct timeval stTimeOut;
		stTimeOut.tv_sec = 0;
		stTimeOut.tv_usec = 70000; //100 ms

								   /*---- Create the socket. The three arguments are: ----*/
								   /* 1) Internet domain 2) Stream socket 3) Default protocol (TCP in this case) */
								   /* FD_SET() adds the file descriptor "sock" to the fd_set,
								   so that select() will return if a connection comes in
								   on that socket (which means you have to do accept(), etc. */
		FD_SET(*message->udp_connection.sock, &stReadFDS);
		int t = select((*message->udp_connection.sock) + 1, &stReadFDS, NULL, NULL, &stTimeOut);
	}
#pragma endregion UDP_def

	switch (Type) {
	case CANALYST:
		response = VCI_Receive((DWORD)DeviceType, (DWORD)DeviceInd, (DWORD)CanIndex, message->pCanObj, 1, (ULONG)Timeout);
		stream << std::setfill('0') << std::setw(8) << std::hex << (*(message->pCanObj)).ID << "-";
		for (int i = 0; i < (int)(message->pCanObj->DataLen); i++)
			stream << std::setfill('0') << std::setw(2) << std::hex << (int)(message->pCanObj->Data[i]);
		if (response <= 0)
			return "NoMsg";
		break;
	case CANKVASER:
		response = canRead(ChannelHandler, MsgIDin, DataOut, dlc, kvaserFlag, 0);
		stream << std::setfill('0') << std::setw(8) << std::hex << *MsgIDin << "-";
		for (int i = 0; i < (int)(*dlc); i++)
			stream << std::setfill('0') << std::setw(2) << std::hex << (int)(DataOut[i]);
		if (response < 0 || *kvaserFlag == 0x20)
			return "NoMsg";
		break;
	case UDP:
		return "Not Implemented";
		//response = recvfrom(message->udp_connection.sock, (char*)MESSAGE, (int)sizeof(MESSAGE), 0, (SOCKADDR*)&(message->udp_connection.PsNetwork), &len);
		//stream << std::setfill('0') << std::setw(8) << std::hex << *MESSAGE;
		break;
	default:
		return "Unknown selected type.";
	}

	std::string result(stream.str());
	char *StringSystemOut = (char*)malloc(result.length() + 1);
	strcpy_s(StringSystemOut, result.length() + 1, result.c_str());
	return StringSystemOut;
}

// Get Channel Status function.
__declspec(dllexport) char* __cdecl ChannelStatus(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int *StatusChannel = NULL,
	int *TripCHannel = NULL,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillMsg(Type, 0x0, ID, dstAdd, message, fillData, 0, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			*TripCHannel = -1;
			*StatusChannel = -1;
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	int ChannelTrip[MAX_CHANNEL_NUMBER];
	int ChannelStatus[MAX_CHANNEL_NUMBER];

	for (int i = 0, j = 0; i < (MAX_CHANNEL_NUMBER / 2); i++, j += 2)
	{
		ChannelTrip[j] = (int)(fillData[i] & 0x3);//bit 0 & bit 1
		ChannelTrip[j + 1] = (int)((fillData[i] >> 4) & 0x3);//bit 4 & bit 5
		ChannelStatus[j] = (int)((fillData[i] >> 2) & 0x3);//bit 2 & bit 3
		ChannelStatus[j + 1] = (int)((fillData[i] >> 6) & 0x3);//bit 6 & bit 7
	}

	int _tripChanel = 0;
	int _statusChanel = 0;
	for (int i = 0; i < MAX_CHANNEL_NUMBER; i++) {
		_tripChanel |= ChannelTrip[i] << (i * 2);
	}
	*TripCHannel = _tripChanel;
	for (int i = 0; i < MAX_CHANNEL_NUMBER; i++) {
		_statusChanel |= ChannelStatus[i] << (i * 2);
	}
	*StatusChannel = _statusChanel;

	/*
	*TripCHannel = ((ChannelTrip[15] << 30) | (ChannelTrip[14] << 28) | (ChannelTrip[13] << 26) | (ChannelTrip[12] << 24) | \
		(ChannelTrip[11] << 22) | (ChannelTrip[10] << 20) | (ChannelTrip[9] << 18) | (ChannelTrip[8] << 16) | \
		(ChannelTrip[7] << 14) | (ChannelTrip[6] << 12) | (ChannelTrip[5] << 10) | (ChannelTrip[4] << 8) | (ChannelTrip[3] << 6) | \
		(ChannelTrip[2] << 4) | (ChannelTrip[1] << 2) | ChannelTrip[0]);
	*StatusChannel = ((ChannelStatus[15] << 22) | (ChannelStatus[14] << 22) | (ChannelStatus[13] << 22) | (ChannelStatus[12] << 22) | \
		(ChannelStatus[11] << 22) | (ChannelStatus[10] << 20) | (ChannelStatus[9] << 18) | (ChannelStatus[8] << 16) | \
		(ChannelStatus[7] << 14) | (ChannelStatus[6] << 12) | (ChannelStatus[5] << 10) | (ChannelStatus[4] << 8) | (ChannelStatus[3] << 6) | \
		(ChannelStatus[2] << 4) | (ChannelStatus[1] << 2) | ChannelStatus[0]);
	*/
	return "success";
}

// Set Channel Control function.
__declspec(dllexport) char* __cdecl ChannelControl(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	unsigned __int16 Control, // set the channels according to the data received in byte 0.
	unsigned __int16 Mask,
	sNETWORK* PsNetwork = NULL) {  // Mask bit is active high – ‘1’ will mask the relevant channel.

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	int ControlArr[2];
	int MaskArr[2];
	ControlArr[0] = Control & 0xff;
	ControlArr[1] = (Control >> 8);
	if (ControlArr[1] < 0)
		ControlArr[1] += 256;

	MaskArr[0] = Mask & 0xff;
	MaskArr[1] = (Mask >> 8);
	if (MaskArr[1] < 0)
		MaskArr[1] += 256;

	fillData[0] = ControlArr[0];
	fillData[1] = MaskArr[0];
	fillData[2] = ControlArr[1];
	fillData[3] = MaskArr[1];

	fillMsg(Type, 0x2, ID, dstAdd, message, fillData, 4, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	/*txrx = receive(Type, message, fillData);

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}*/
	delete[] message;
	return "success";
}

// Get Curent Command function.
__declspec(dllexport) char* __cdecl CurentCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ChannelNo, // Select the channel 
	float *ADCunits,
	sNETWORK* PsNetwork = NULL) { // The returned value

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	if (ChannelNo > 0 && ChannelNo < 5)
		fillMsg(Type, 0x3, ID, dstAdd, message, fillData, 0, PsNetwork);
	else if (ChannelNo > 4 && ChannelNo < 9)
		fillMsg(Type, 0x4, ID, dstAdd, message, fillData, 0, PsNetwork);
	else if (ChannelNo > 8 && ChannelNo < 13)
		fillMsg(Type, 0x5, ID, dstAdd, message, fillData, 0, PsNetwork);
	else if (ChannelNo > 12 && ChannelNo < 17)
		fillMsg(Type, 0x6, ID, dstAdd, message, fillData, 0, PsNetwork);
	else
		fillMsg(Type, 0x3, ID, dstAdd, message, fillData, 0, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			*ADCunits = -1;
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	if (ChannelNo == 1 || ChannelNo == 5 || ChannelNo == 9 || ChannelNo == 13)
		*ADCunits = (float)0.1 * static_cast<float>(fillData[1] << 8 | fillData[0]);
	else if (ChannelNo == 2 || ChannelNo == 6 || ChannelNo == 10 || ChannelNo == 14)
		*ADCunits = (float)0.1 * static_cast<float>(fillData[3] << 8 | fillData[2]);
	else if (ChannelNo == 3 || ChannelNo == 7 || ChannelNo == 11 || ChannelNo == 15)
		*ADCunits = (float)0.1 * static_cast<float>(fillData[5] << 8 | fillData[4]);
	else if (ChannelNo == 4 || ChannelNo == 8 || ChannelNo == 12 || ChannelNo == 16)
		*ADCunits = (float)0.1 * static_cast<float>(fillData[7] << 8 | fillData[6]);
	else
		*ADCunits = -1;

	return "success";
}

//	Get Vin/Vout Command function.
__declspec(dllexport) char* __cdecl GetVIn_OutCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ChannelNo,
	float *VIn, // Return Vin
	float *VOut, // Return Vout
	sNETWORK* PsNetwork = NULL) { // Return the selected channel

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = ChannelNo;
	fillMsg(Type, 0x8, ID, dstAdd, message, fillData, 1, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			*VIn = -1;
			*VOut = -1;
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	if (fillData[4] >= 0 && fillData[4] <= MAX_CHANNEL_NUMBER && fillData[4] == ChannelNo)
	{
		*VIn = (float)0.1 * static_cast<float>(fillData[1] << 8 | fillData[0]);
		*VOut = (float)0.1 * static_cast<float>(fillData[3] << 8 | fillData[2]);
	}
	return "success";
}

// Get Version/Board Type function.
__declspec(dllexport) char* __cdecl GetSWVersion(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork = NULL) {

	char *SWVerionOut = new char[20];
	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillMsg(Type, 0x9, ID, dstAdd, message, fillData, 0, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	std::string temp;
	temp = std::to_string(fillData[2]) + '.';
	temp += std::to_string(fillData[1]) + '.';
	temp += std::to_string(fillData[0]) + '.';
	temp += (char)(fillData[3]);

	memcpy(SWVerionOut, temp.c_str(), temp.size() + 1);
	return SWVerionOut;
}

// Get System Temperature function.
__declspec(dllexport) char* __cdecl SystemTemperature(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	float *Temp1, // Return Channel 1-5 Temperatures
	float *Temp2, // Return Channel 6-10 Temperatures
	float *CPU,
	sNETWORK* PsNetwork = NULL) { // Return CPU Temperature

	unsigned char fillData[8] = { 0 };
	fillData[0] = 1;
	MsgType* message = new MsgType();

	fillMsg(Type, 0xA, ID, dstAdd, message, fillData, 1, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			*Temp1 = -1;
			*Temp2 = -1;
			*CPU = -1;
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	*Temp1 = static_cast<float>((static_cast<int16_t>(fillData[1] << 8 | fillData[0])));
	*Temp2 = static_cast<float>((static_cast<int16_t>(fillData[3] << 8 | fillData[2])));
	*CPU = static_cast<float>((static_cast<int16_t>(fillData[5] << 8 | fillData[4])));
	return "success";
}

// Get System Parameters function.
__declspec(dllexport) char* __cdecl GetSystemParam(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ParamNo,
	int Index,
	sNETWORK* PsNetwork = NULL) {
	int MaxParamNum = 41;
	if (ParamNo > MaxParamNum || ParamNo < 0)
		return "Parameter number doesn't exist.";
	int ParamExist[] = {
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
		1, 1, 0, 0, 0, 0, 0, 0
	};
	if (!ParamExist[ParamNo])
		return "Parameter number doesn't exist.";
	std::stringstream stream;
	char *StringSystemOut = new char();
	int combined = 0;
	std::string temp = "";
	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	int ParamIndexArr[] = { 0, 0 ,0 ,0 ,0 ,1 ,1 ,0 ,0 ,1, \
							0, 0, 0, 0, 1, 1, 1, 1, 1, 1,  \
							0, 0, 0, 0, 0, 0, 1, 1, 1, 1,  \
							1, 0, 0, 1, 1, 0, 0, 0, 0, 1,  \
							1, 0, 0, 0, 0, 0, 0, 0 };

	//if (ParamIndexArr[ParamNo])
	//	fillMsg(Type, 0xB, ID, dstAdd, message, fillData, 4, PsNetwork);
	//else
	fillMsg(Type, 0xB, ID, dstAdd, message, fillData, 4, PsNetwork);

	if (ParamNo == 9) {
		for (int i = 0; i < 6; i++)
		{
			for (int i = 0; i < 4; i++)
				fillData[i] = 0;
			fillData[0] = 9; // System General string. 
			fillData[3] = i * 6;

			fillMsg(Type, 0xB, ID, dstAdd, message, fillData, 4, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);

			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			txrx = receive(Type, message, fillData, DELTA_T);

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					delete[] message;
					return errorReturn;
				}
				delete[] message;
				return ErrorDefinition(txrx);
			}
			for (int k = 0; k < 6; k++)
				temp += static_cast<char>(fillData[k]);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_RX_TX));
		}
		StringSystemOut = new char[temp.length() + 1];
		strcpy_s(StringSystemOut, temp.length() + 1, temp.c_str());
		delete[] message;
		return StringSystemOut;
	}
	else if (ParamNo == 17 || ParamNo == 23) {
		return _unbuild(Type, ID, dstAdd, Index, ParamNo, PsNetwork, fillData, message, SYSTEM_PARAM);

#ifdef INSIDE
		unsigned __int8 Data8[4];

		for (int j = 0; j < 2; j++) {

			for (int i = 0; i < 4; i++)
				fillData[i] = 0;

			if (j >= 1)
				fillData[0] = 23;
			else
				fillData[0] = ParamNo;
			fillData[3] = Index;

			fillMsg(Type, 0xB, ID, dstAdd, message, fillData, 4, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);

			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			txrx = receive(Type, message, fillData, DELTA_T);

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					delete[] message;
					return errorReturn;
				}
				delete[] message;
				return ErrorDefinition(txrx);
			}
			combined = (fillData[3] << 24) | (fillData[2] << 16) | (fillData[1] << 8) | (fillData[0]);
			Data8[0] = fillData[0];
			Data8[1] = fillData[1];
			Data8[2] = fillData[2];
			Data8[3] = fillData[3];

			for (int i = 0; i < 4; i++)
			{
				if (Data8[i] > 0 && Data8[i] < 13)
				{
					temp += "In";
					if ((char)(Data8[i] + 48) == ':')
						temp += "10";
					else if ((char)(Data8[i] + 48) == ';')
						temp += "11";
					else if ((char)(Data8[i] + 48) == '<')
						temp += "12";
					else
						temp += (char)(Data8[i] + 48);
				}
				else if (Data8[i] == 13)
					temp += "EMRG";
				else if (Data8[i] > 13 && Data8[i] < 18) {
					temp += "OUT";
					temp += (char)(Data8[i] + 48 - 13);
				}
				else if (Data8[i] == 18)
					temp += "EPAR";
				else if (Data8[i] == 19)
					temp += "ETEM";
				else if (Data8[i] == 20)
					temp += "EVOL";
				else if (Data8[i] == 21)
					temp += "EBOM";
				else if (Data8[i] == 22)
					temp += "EBIT";
				else if (Data8[i] > 23 && Data8[i] < 36) {
					temp += "SCH";
					temp += (char)(Data8[i] - 23 + 48);
				}
				else if (Data8[i] > 35 && Data8[i] < 48) {
					temp += "TCH";
					temp += (char)(Data8[i] - 35 + 48);
				}
				else if (Data8[i] > 47 && Data8[i] < 60) {
					temp += "ECH";
					temp += (char)(Data8[i] - 47 + 48);
				}
				else if (Data8[i] > 60 && Data8[i] < 73)
				{
					temp += "!In";
					if ((char)(Data8[i] - 60 + 48) == ':')
						temp += "10";
					else if ((char)(Data8[i] - 60 + 48) == ';')
						temp += "11";
					else if ((char)(Data8[i] - 60 + 48) == '<')
						temp += "12";
					else
						temp += (char)(Data8[i] - 60 + 48);
				}
				else if (Data8[i] == 73)
					temp += "!EMRG";
				else if (Data8[i] > 73 && Data8[i] < 78) {
					temp += "!OUT";
					temp += (char)(Data8[i] - 73 + 48);
				}
				else if (Data8[i] == 78)
					temp += "!EPAR";
				else if (Data8[i] == 79)
					temp += "!ETEM";
				else if (Data8[i] == 80)
					temp += "!EVOL";
				else if (Data8[i] == 81)
					temp += "!EBOM";
				else if (Data8[i] == 82)
					temp += "!EBIT";
				else if (Data8[i] > 83 && Data8[i] < 96) {
					temp += "!SCH";
					temp += (char)(Data8[i] - 83 + 48);
				}
				else if (Data8[i] > 95 && Data8[i] < 108) {
					temp += "!TCH";
					temp += (char)(Data8[i] - 95 + 48);
				}
				else if (Data8[i] > 107 && Data8[i] < 120) {
					temp += "!ECH";
					temp += (char)(Data8[i] - 107 + 48);
				}
				else if (Data8[i] == 120)
					temp += "&";
				else if (Data8[i] == 121)
					temp += "|";
				else if (Data8[i] == 122)
					temp += "^";
				else
					temp += "";
			}
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_RX_TX));
		}
		StringSystemOut = new char[temp.size() + 1];
		strcpy_s(StringSystemOut, temp.length() + 1, temp.c_str());
		delete[] message;

		if (StringSystemOut[0] == '.')
			return UNKNOWN_ERROR;
		return StringSystemOut;
#endif // INSIDE
	}
	else {
		int iMax = 4;
		for (int i = 0; i < iMax; i++)
			fillData[i] = 0;

		fillData[0] = ParamNo;

		if (ParamIndexArr[ParamNo])
			fillData[3] = Index;
		fillMsg(Type, 0xB, ID, dstAdd, message, fillData, iMax, PsNetwork);

		int txrx = send(Type, message);
		if (txrx < 0)
			return ErrorDefinition(txrx);

		std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

		txrx = receive(Type, message, fillData);
		delete[] message;

		if (txrx < 0) {
			if (txrx == SYSTEM_ERROR_CODE) {
				static char errorReturn[14];
				Error(Type, fillData, errorReturn);
				return errorReturn;
			}
			return ErrorDefinition(txrx);
		}
		combined = (fillData[3] << 24) | (fillData[2] << 16) | (fillData[1] << 8) | (fillData[0]);
		switch (fillData[6])
		{
		case 0: // System up time
			temp = std::to_string(combined);
			break;
		case 1: // Unit ID address
			temp = std::to_string(combined);
			break;
		case 2: // R_sense
			stream << std::fixed << std::setprecision(4) << 0.010 * combined;
			temp = stream.str();
			break;
		case 3: // Opamp Gain
			temp = std::to_string(combined);
			break;
		case 4: // Ring Cancel delay 
			if (combined != 0)
				return "ON";
			else
				return "OFF";
			break;
		case 5: // Vin low TH.
			stream << std::fixed << std::setprecision(1) << 0.10 * combined;
			temp = stream.str();
			break;
		case 6: // Vin high TH. P1
			stream << std::fixed << std::setprecision(1) << 0.10 * combined;
			temp = stream.str();
			break;
		case 7: // Temp low TH. 
			temp = std::to_string(combined);
			break;
		case 8: // Temp high TH.
			temp = std::to_string(combined);
			break;
		case 10: // Sys.Tech.Data
			temp = std::to_string(combined);
			break;
		case 11: // Sys.Time
			temp = std::to_string(combined);
			break;
		case 12: // CAN_BAUD
			switch (combined) {
			case 1:
				return "1000";
			case 2:
				return "800";
			case 3:
				return "500";
			case 4:
				return "250";
			case 5:
				return "125";
			case 6:
				return "100";
			case 7:
				return "83.3";
			case 8:
				return "62.5";
			case 9:
				return "50";
			case 10:
				return "20";
			case 11:
				return "10";
			default:
				return "Error";
			}
			break;
		case 13: // RS_BAUD
			switch (combined) {
			case 1:
				return "921600";
			case 2:
				return "460800";
			case 3:
				return "256000";
			case 4:
				return "115200";
			case 5:
				return "57600";
			case 6:
				return "38400";
			case 7:
				return "19200";
			case 8:
				return "14400";
			case 9:
				return "9600";
			case 10:
				return "2400";
			case 11:
				return "1200";
			default:
				return "Error";
			}
			break;
		case 14: // Dig. Out n Delay before Active
			temp = std::to_string(combined);
			break;
		case 15: // Dig. Out n Delay before Deactivating
			temp = std::to_string(combined);
			break;
		case 16: // Dig. Out n OFF Timeout 
			temp = std::to_string(combined);
			break;
		case 17:
			break;
		case 18: // Dig. Out n logic inverse
			switch (combined) {
			case 0:
				return "None";
			case 1:
				return "Invers";
			case 2:
				return "BlinkF";
			case 3:
				return "BlinkF&Invers";
			case 4:
				return "BlinkM";
			case 5:
				return "BlinkM&Invers";
			case 8:
				return "BlinkS";
			case 9:
				return "BlinkS&Invers";
			default:
				return "Error";
			}
			break;
		case 19: // Dig. Input type
			switch (fillData[0]) {
			case 0:
				return "None";
			case 1:
				return "Toggle";
			case 2:
				return "Moment.Up";
			case 3:
				return "Moment.Down";
			default:
				return "Error";
			}
			break;
		case 20: // Output startup state
			temp = std::to_string(combined);
			break;
		case 21: // Max led current
			temp = std::to_string(combined);
			break;
		case 22: // Unit address
			temp = std::to_string(combined);
			break;
		case 24: // destinatino address
			temp = std::to_string(combined);
			break;
		case 25: // Log Update Time
			temp = std::to_string(combined);
			break;
		case 26: // Over Voltage Time P1
			temp = std::to_string(combined);
			break;
		case 27: // Over Voltage Time P2
			temp = std::to_string(combined);
			break;
		case 28: // Over Voltage Time P3
			temp = std::to_string(combined);
			break;
		case 29: // Vin high TH. P2
			stream << std::fixed << std::setprecision(1) << 0.10 * combined;
			temp = stream.str();
			break;
		case 30: // Vin high TH. P3
			stream << std::fixed << std::setprecision(1) << 0.10 * combined;
			temp = stream.str();
			break;
		case 31: // Total max current S1
			stream << std::fixed << std::setprecision(1) << 0.10 * combined;
			temp = stream.str();
			break;
		case 32: // Channel priority scenario
			stream << std::fixed << std::setprecision(1) << combined;
			temp = stream.str();
			break;
		case 33: // Input function mode 
			switch (combined) {
			case 0:
				return "GPIO";
			case 1:
				return "Sleep Mode";
			case 2:
				return "Standby Mode";
			case 3:
				return "CAN Silent Mode";
			case 4:
				return "Emergency Stop";
			case 5:
				return "Black Out Mode";
			case 6:
				return "SW Reset";
			case 7:
				return "CH Priority Mode 1";
			case 8:
				return "CH Priority Mode 2";
			case 9:
				return "CH Priority Mode 3";
			default:
				return "Error";
			}
			break;
		case 34: // Input logic inverse
			switch (combined) {
			case 0:
				return "None";
			case 1:
				return "Invers";
			default:
				return "Error";
			}
			break;
		case 35: // Total max current S2
			stream << std::fixed << std::setprecision(1) << 0.10 * combined;
			temp = stream.str();
			break;
		case 36: // Total max current S3
			stream << std::fixed << std::setprecision(1) << 0.10 * combined;
			temp = stream.str();
			break;
		case 37: // Over Voltage1 Retry delay 
			temp = std::to_string(combined);
			break;
		case 38: // Over Voltage2 Retry delay 
			temp = std::to_string(combined);
			break;
		case 39: // Vin1/2 Low HYSTERESIS
		case 40: // Vin1/2 High HYSTERESIS
			stream << std::fixed << std::setprecision(1) << 0.10 * combined;
			temp = stream.str();
			break;
		case 41: // Communication Timeout  
			switch (combined) {
			case 0:
				return "Disable";
			case 1:
				return "50 ms";
			case 2:
				return "100 ms";
			case 3:
				return "250 ms";
			case 4:
				return "500 ms";
			case 5:
				return "1 sec";
			case 6:
				return "1.5 sec";
			case 7:
				return "2 sec";
			case 8:
				return "5 sec";
			case 9:
				return "10 sec";
			case 10:
				return "30 sec";
			case 11:
				return "1 min";
			case 12:
				return "5 min";
			case 13:
				return "10 min";
			case 14:
				return "20 min";
			default:
				return "Param No Error";
			}
			break;
		default:
			return "Param No Error";
		}

		StringSystemOut = new char[temp.length() + 1];
		strcpy_s(StringSystemOut, temp.length() + 1, temp.c_str());

		if (StringSystemOut[0] == '.' && ParamNo != 2)
			return UNKNOWN_ERROR;

		return StringSystemOut;
	}
}

// Set System Parameters function.
__declspec(dllexport) char* __cdecl SetSystemParam(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ParamNo,
	float ValIn,
	char *StrIn,
	int Index,
	sNETWORK* PsNetwork = NULL) {
	int MaxParamNum = 41;
	if (ParamNo > MaxParamNum || ParamNo < 0)
		return "Parameter number doesn't exist.";
	int ParamExist[] = {
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
		1, 1, 0, 0, 0, 0, 0, 0
	};

	if (!ParamExist[ParamNo])
		return "Parameter number doesn't exist.";

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	int SubArr[32] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	std::string temp(StrIn);
	std::transform(temp.begin(), temp.end(), temp.begin(), ::tolower);
	bool flag = true;
	int index = 0, Val = 0, DataIndex = 0;
	unsigned __int8 DataOut[8] = { 0 };
	int k = 0;
	for (k = 0; k < 100; k++) {
		if (StrIn[k] == '\0')
			break;
	}
	char *cDataIn = new char[k + 1];
	unsigned __int8 *DataIn = new unsigned __int8[k + 1];
	*cDataIn = { 0 }, *DataIn = { 0 };

	// copying the contents of the 
	// string to char array 
	strcpy_s(cDataIn, temp.length() + 1, temp.c_str());
	for (int i = 0; i < k + 1; i++) {
		DataIn[i] = static_cast<int>(toupper(cDataIn[i]));
	}
	for (int i = 0; i < 8; i++)
		fillData[i] = 0;
	fillData[0] = ParamNo;
	fillData[2] = 83; //'S' Set

	int ParamIndexArr[] = { 0, 0 ,0 ,0 ,0 ,1 ,1 ,0 ,0 ,1, \
						   0, 0, 0, 0, 1, 1, 1, 1, 1, 1,  \
						   0, 0, 0, 0, 0, 0, 1, 1, 1, 1,  \
						   1, 0, 0, 1, 1, 0, 0, 0, 0, 1,  \
						   1, 0, 0, 0, 0, 0, 0, 0};
	if (ParamIndexArr[ParamNo])
		fillData[3] = Index;

	switch (ParamNo)
	{
	case 0: // System up time
		FillsendBuff(fillData, ValIn);
		break;
	case 1: // Unit ID address
		FillsendBuff(fillData, ValIn);
		break;
	case 2: // R_sense
		ValIn *= 100;
		FillsendBuff(fillData, ValIn);
		break;
	case 3: // Opamp Gain
		FillsendBuff(fillData, ValIn);
		break;
	case 4:
		if (temp.compare("on") == 0)
			ValIn = 1;
		else
			ValIn = 0;
		FillsendBuff(fillData, ValIn);
		break;
	case 5: // Vin low TH.
		ValIn *= 10;
		FillsendBuff(fillData, ValIn);
		break;
	case 6: // Vin high TH.
		ValIn *= 10;
		FillsendBuff(fillData, ValIn);
		break;
	case 7: // Temp low TH.
		FillsendBuff(fillData, ValIn);
		break;
	case 8: // Temp high TH.
		FillsendBuff(fillData, ValIn);
		break;
	case 9: // System General string. 
	{
		if (strlen(StrIn) < 32) {
			for (int i = 0; i < (int)strlen(StrIn); i++)
				SubArr[i] = static_cast<int>(StrIn[i]);
			index = 0;

			for (int i = 0; i < 8; i++)
			{
				for (int i = 0; i < 8; i++)
					fillData[i] = 0;
				fillData[0] = ParamNo;
				fillData[2] = 83; //'S' Set
				fillData[3] = i * 4;
				fillData[4] = SubArr[i * 4];
				fillData[5] = SubArr[(i * 4) + 1];
				fillData[6] = SubArr[(i * 4) + 2];
				fillData[7] = SubArr[(i * 4) + 3];

				fillMsg(Type, 0xB, ID, dstAdd, message, fillData, 8, PsNetwork);

				int txrx = send(Type, message);
				if (txrx < 0)
					return ErrorDefinition(txrx);

				std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
				txrx = receive(Type, message, fillData, DELTA_T);

				if (txrx < 0) {
					if (txrx == SYSTEM_ERROR_CODE) {
						static char errorReturn[14];
						Error(Type, fillData, errorReturn);
						delete[] message;
						return errorReturn;
					}
					delete[] message;
					return ErrorDefinition(txrx);
				}
				index += txrx;
			}

			delete[] message;
			if (index == 8)
				return "success";
			return UNKNOWN_ERROR;
		}
		else
			return InternalError("D9", -1, "");
	}
	break;
	case 10: // Sys.Tech.Data
		FillsendBuff(fillData, ValIn);
		break;
	case 11: // Sys.Time
		FillsendBuff(fillData, ValIn);
		break;
	case 12: // CAN_BAUD
		ValIn *= 10; // Multiply CAN BAUD by 10, to obtain 83,3 kbps as 833 and enable the case 833.
		switch (static_cast<int>(ValIn)) {
		case 10000:
			ValIn = 1;
			break;
		case 8000:
			ValIn = 2;
			break;
		case 5000:
			ValIn = 3;
			break;
		case 2500:
			ValIn = 4;
			break;
		case 1250:
			ValIn = 5;
			break;
		case 1000:
			ValIn = 6;
			break;
		case 833:
			ValIn = 7;
			break;
		case 625:
			ValIn = 8;
			break;
		case 500:
			ValIn = 9;
			break;
		case 200:
			ValIn = 10;
			break;
		case 100:
			ValIn = 11;
			break;
		default:
			return InternalError("D7", -1, ""); // error 215
			break;
		}
		FillsendBuff(fillData, ValIn);
		break;
	case 13: // RS_BAUD
		switch (static_cast<int>(ValIn)) {
		case 921600:
			ValIn = 1;
			break;
		case 460800:
			ValIn = 2;
			break;
		case 256000:
			ValIn = 3;
			break;
		case 115200:
			ValIn = 4;
			break;
		case 57600:
			ValIn = 5;
			break;
		case 38400:
			ValIn = 6;
			break;
		case 19200:
			ValIn = 7;
			break;
		case 14400:
			ValIn = 8;
			break;
		case 9600:
			ValIn = 9;
			break;
		case 2400:
			ValIn = 10;
			break;
		case 1200:
			ValIn = 11;
			break;
		default:
			return InternalError("D8", -1, ""); // error 216
			break;
		}
		FillsendBuff(fillData, ValIn);
		break;
	case 14: // Dig. Out n Delay before Active
		FillsendBuff(fillData, ValIn);
		break;
	case 15: // Dig. Out n Delay before Deactivating
		FillsendBuff(fillData, ValIn);
		break;
	case 16: // Dig. Out n OFF Timeout 
		FillsendBuff(fillData, ValIn);
		break;
	case 17: // Dig. Out n control

		return _build_control(Type, ID, dstAdd, message, PsNetwork, k, fillData, ParamNo, 0, DataOut, DataIn, Val, SYSTEM_PARAM);

#ifdef INSIDE
		{
			bool flag = false;
			int index = 0, DataIndex = 0;
			if (k == 0)
				flag = true;
			while (!flag) {
				if (DataIn[index] == 'I') {
					index++;
					if (DataIn[index] == 'N') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - 48;
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '2') {
								Val *= 10;
								Val += DataIn[index] - 48;
								DataOut[DataIndex] = Val;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val;
								DataIndex++;
							}
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'S') {
					index++;
					if (DataIn[index] == 'C') {
						index++;
						if (DataIn[index] == 'H') {
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '9') {
								Val = DataIn[index] - 48;
								index++;
								if (DataIn[index] >= '0' && DataIn[index] <= '2') {
									Val *= 10;
									Val += DataIn[index] - 48;
									DataOut[DataIndex] = Val + 23;
									index++;
									DataIndex++;
								}
								else {
									DataOut[DataIndex] = Val + 23;
									DataIndex++;
								}
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'T') {
					index++;
					if (DataIn[index] == 'C') {
						index++;
						if (DataIn[index] == 'H') {
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '9') {
								Val = DataIn[index] - 48;
								index++;
								if (DataIn[index] > '0' && DataIn[index] <= '2') {
									Val *= 10;
									Val += DataIn[index] - 48;
									DataOut[DataIndex] = Val + 35;
									index++;
									DataIndex++;
								}
								else {
									DataOut[DataIndex] = Val + 35;
									DataIndex++;
								}
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'O') {
					index++;
					if (DataIn[index] == 'U') {
						index++;
						if (DataIn[index] == 'T') {
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '4') {
								Val = DataIn[index] - 48;
								DataOut[DataIndex] = Val + 13;
								DataIndex++; index++;
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'E') {
					index++;
					if (DataIn[index] == 'C') {
						index++;
						if (DataIn[index] == 'H') {
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '9') {
								Val = DataIn[index] - 48;
								index++;
								if (DataIn[index] > '0' && DataIn[index] <= '2') {
									Val *= 10;
									Val += DataIn[index] - 48;
									DataOut[DataIndex] = Val + 47;
									index++;
									DataIndex++;
								}
								else {
									DataOut[DataIndex] = Val + 47;
									DataIndex++;
								}
							}
							else flag = true;
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == 'V') {
						index++;
						if (DataIn[index] == 'O') {
							index++;
							if (DataIn[index] == 'L') {
								DataOut[DataIndex] = 20;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'B') {
						index++;
						if (DataIn[index] == 'I') {
							index++;
							if (DataIn[index] == 'T') {
								DataOut[DataIndex] = 22;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else if (DataIn[index] == 'O') {
							index++;
							if (DataIn[index] == 'M') {
								DataOut[DataIndex] = 21;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'P') {
						index++;
						if (DataIn[index] == 'A') {
							index++;
							if (DataIn[index] == 'R') {
								DataOut[DataIndex] = 18;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'M') {
						index++;
						if (DataIn[index] == 'R') {
							index++;
							if (DataIn[index] == 'G') {
								DataOut[DataIndex] = 13;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'T') {
						index++;
						if (DataIn[index] == 'E') {
							index++;
							if (DataIn[index] == 'M') {
								DataOut[DataIndex] = 19;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == '!') {
					index++;
					if (DataIn[index] == 'I') {
						index++;
						if (DataIn[index] == 'N') {
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '9') {
								Val = DataIn[index] - 48;
								index++;
								if (DataIn[index] > '0' && DataIn[index] <= '2') {
									Val *= 10;
									Val += DataIn[index] - 48;
									DataOut[DataIndex] = Val + 60;
									index++;
									DataIndex++;
								}
								else {
									DataOut[DataIndex] = Val + 60;
									DataIndex++;
								}
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == 'S') {
						index++;
						if (DataIn[index] == 'C') {
							index++;
							if (DataIn[index] == 'H') {
								index++;
								if (DataIn[index] > '0' && DataIn[index] <= '9') {
									Val = DataIn[index] - 48;
									index++;
									if (DataIn[index] > '0' && DataIn[index] <= '2') {
										Val *= 10;
										Val += DataIn[index] - 48;
										DataOut[DataIndex] = Val + 23 + 60;
										index++;
										DataIndex++;
									}
									else {
										DataOut[DataIndex] = Val + 23 + 60;
										DataIndex++;
									}
								}
								else return InternalError("DD", -1, "");
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == 'T') {
						index++;
						if (DataIn[index] == 'C') {
							index++;
							if (DataIn[index] == 'H') {
								index++;
								if (DataIn[index] > '0' && DataIn[index] <= '9') {
									Val = DataIn[index] - 48;
									index++;
									if (DataIn[index] > '0' && DataIn[index] <= '2') {
										Val *= 10;
										Val += DataIn[index] - 48;
										DataOut[DataIndex] = Val + 35 + 60;
										index++;
										DataIndex++;
									}
									else {
										DataOut[DataIndex] = Val + 35 + 60;
										DataIndex++;
									}
								}
								else return InternalError("DD", -1, "");
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == 'O') {
						index++;
						if (DataIn[index] == 'U') {
							index++;
							if (DataIn[index] == 'T') {
								index++;
								if (DataIn[index] > '0' && DataIn[index] <= '4') {
									Val = DataIn[index] - 48;
									DataOut[DataIndex] = Val + 13 + 60;
									DataIndex++; index++;
								}
								else return InternalError("DD", -1, "");
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == 'E') {
						index++;
						if (DataIn[index] == 'C') {
							index++;
							if (DataIn[index] == 'H') {
								index++;
								if (DataIn[index] > '0' && DataIn[index] <= '9') {
									Val = DataIn[index] - 48;
									index++;
									if (DataIn[index] > '0' && DataIn[index] <= '2') {
										Val *= 10;
										Val += DataIn[index] - 48;
										DataOut[DataIndex] = Val + 47 + 60;
										index++;
										DataIndex++;
									}
									else {
										DataOut[DataIndex] = Val + 47 + 60;
										DataIndex++;
									}
								}
								else flag = true;
							}
							else return InternalError("DD", -1, "");
						}
						else if (DataIn[index] == 'V') {
							index++;
							if (DataIn[index] == 'O') {
								index++;
								if (DataIn[index] == 'L') {
									DataOut[DataIndex] = 20 + 60;
									DataIndex++; index++;
								}
								else flag = true;
							}
							else flag = true;
						}
						else if (DataIn[index] == 'B') {
							index++;
							if (DataIn[index] == 'I') {
								index++;
								if (DataIn[index] == 'T') {
									DataOut[DataIndex] = 22 + 60;
									DataIndex++; index++;
								}
								else flag = true;
							}
							else if (DataIn[index] == 'O') {
								index++;
								if (DataIn[index] == 'M') {
									DataOut[DataIndex] = 21 + 60;
									DataIndex++; index++;
								}
								else flag = true;
							}
							else flag = true;
						}
						else if (DataIn[index] == 'P') {
							index++;
							if (DataIn[index] == 'A') {
								index++;
								if (DataIn[index] == 'R') {
									DataOut[DataIndex] = 18 + 60;
									DataIndex++; index++;
								}
								else flag = true;
							}
							else flag = true;
						}
						else if (DataIn[index] == 'M') {
							index++;
							if (DataIn[index] == 'R') {
								index++;
								if (DataIn[index] == 'G') {
									DataOut[DataIndex] = 13 + 60;
									DataIndex++; index++;
								}
								else flag = true;
							}
							else flag = true;
						}
						else if (DataIn[index] == 'T') {
							index++;
							if (DataIn[index] == 'E') {
								index++;
								if (DataIn[index] == 'M') {
									DataOut[DataIndex] = 19 + 60;
									DataIndex++; index++;
								}
								else flag = true;
							}
							else flag = true;
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == ' ')
						flag = false;
					else if (DataIn[index] == 200)
						flag = true;
					else flag = true;
				}
				else if (DataIn[index] == '&') {
					DataOut[DataIndex] = 120;
					DataIndex++; index++;
				}
				else if (DataIn[index] == '|') {
					DataOut[DataIndex] = 121;
					DataIndex++; index++;
				}
				else if (DataIn[index] == '^') {
					DataOut[DataIndex] = 122;
					DataIndex++; index++;
				}
				else if (DataIn[index] == ' ') {
					index++;
				}
				else {
					flag = true;
					return InternalError("DD", -1, "");
				}
				if (index == k)
					flag = true;
			}

			index = 0;
			{
				fillData[4] = DataOut[0];
				fillData[5] = DataOut[1];
				fillData[6] = DataOut[2];
				fillData[7] = DataOut[3];

				fillMsg(Type, 0xB, ID, dstAdd, message, fillData, 8, PsNetwork);

				int txrx = send(Type, message);
				if (txrx < 0)
					return ErrorDefinition(txrx);
				std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
				txrx = receive(Type, message, fillData, DELTA_T);

				if (txrx < 0) {
					if (txrx == SYSTEM_ERROR_CODE) {
						static char errorReturn[14];
						Error(Type, fillData, errorReturn);
						delete[] message;
						return errorReturn;
					}
					delete[] message;
					return ErrorDefinition(txrx);
				}
				index += txrx;
			}
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			{
				fillData[0] = 23;
				fillData[4] = DataOut[4];
				fillData[5] = DataOut[5];
				fillData[6] = DataOut[6];
				fillData[7] = DataOut[7];

				fillMsg(Type, 0xB, ID, dstAdd, message, fillData, 8, PsNetwork);

				int txrx = send(Type, message);
				if (txrx < 0)
					return ErrorDefinition(txrx);
				std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
				txrx = receive(Type, message, fillData);

				if (txrx < 0) {
					if (txrx == SYSTEM_ERROR_CODE) {
						static char errorReturn[14];
						Error(Type, fillData, errorReturn);
						delete[] message;
						return errorReturn;
					}
					delete[] message;
					return ErrorDefinition(txrx);
				}
				index += txrx;
			}
			delete[] message;
			if (index == 2)
				return "success";
			return UNKNOWN_ERROR;
	}
#endif
		break;
	case 18: // Dig. Out n logic inverse
	{
		if (temp.compare("none") == 0)
			ValIn = 0;
		else if (temp.compare("invers") == 0)
			ValIn = 1;
		else if (temp.compare("blinkf") == 0)
			ValIn = 2;
		else if (temp.compare("blinkf&invers") == 0)
			ValIn = 3;
		else if (temp.compare("blinkm") == 0)
			ValIn = 4;
		else if (temp.compare("blinkm&invers") == 0)
			ValIn = 5;
		else if (temp.compare("blinks") == 0)
			ValIn = 8;
		else if (temp.compare("blinks&invers") == 0)
			ValIn = 9;
		else
			return InternalError("DB", -1, ""); // Error 219
	}
	FillsendBuff(fillData, ValIn);
	break;
	case 19: // Dig. Input type
	{
		if (temp.compare("none") == 0)
			ValIn = 0;
		else if (temp.compare("toggle") == 0)
			ValIn = 1;
		else if (temp.compare("moment.up") == 0)
			ValIn = 2;
		else if (temp.compare("moment.down") == 0)
			ValIn = 3;
		else
			return InternalError("DC", -1, ""); // Error 220
	}
	FillsendBuff(fillData, ValIn);
	break;
	case 20: // Output startup state
		FillsendBuff(fillData, ValIn);
		break;
	case 21: // Max led current
		FillsendBuff(fillData, ValIn);
		break;
	case 22: // unit address
		FillsendBuff(fillData, ValIn);
		break;
	case 24: // destination address
		FillsendBuff(fillData, ValIn);
		break;
	case 25: // Log Update Time
		FillsendBuff(fillData, ValIn);
		break;
	case 26: // Over Voltage Time P1
		FillsendBuff(fillData, ValIn);
		break;
	case 27: // Over Voltage Time P2
		FillsendBuff(fillData, ValIn);
		break;
	case 28: // Over Voltage Time P3
		FillsendBuff(fillData, ValIn);
		break;
	case 29: // Vin high TH. P2
		ValIn *= 10;
		FillsendBuff(fillData, ValIn);
		break;
	case 30: // Vin high TH. P3
		ValIn *= 10;
		FillsendBuff(fillData, ValIn);
		break;
	case 31: // Total max current S1
		ValIn *= 10;
		FillsendBuff(fillData, ValIn);
		break;
	case 32: // Channel priority scenario
		FillsendBuff(fillData, ValIn);
		break;
	case 33: // Input function mode 
	{
		if (temp.compare("gpio") == 0)
			ValIn = 0;
		else if (temp.compare("sleep mode") == 0)
			ValIn = 1;
		else if (temp.compare("standby mode") == 0)
			ValIn = 2;
		else if (temp.compare("can silent mode") == 0)
			ValIn = 3;
		else if (temp.compare("emergency stop") == 0)
			ValIn = 4;
		else if (temp.compare("black out mode") == 0)
			ValIn = 5;
		else if (temp.compare("sw reset") == 0)
			ValIn = 6;
		else if (temp.compare("ch priority mode 1") == 0)
			ValIn = 7;
		else if (temp.compare("ch priority mode 2") == 0)
			ValIn = 8;
		else if (temp.compare("ch priority mode 3") == 0)
			ValIn = 9;
		else
			return InternalError("DB", -1, ""); // Error 219
		FillsendBuff(fillData, ValIn);
		break;
	}
	case 34: // Input logic inverse
	{
		if (temp.compare("none") == 0)
			ValIn = 0;
		else if (temp.compare("invers") == 0)
			ValIn = 1;
		else
			return InternalError("DB", -1, ""); // Error 219
	}
	FillsendBuff(fillData, ValIn);
	break;
	case 35: // Total max current S2
		ValIn *= 10;
		FillsendBuff(fillData, ValIn);
		break;
	case 36: // Total max current S3
		ValIn *= 10;
		FillsendBuff(fillData, ValIn);
		break;
	case 37: // Over Voltage1 Retry delay 
		FillsendBuff(fillData, ValIn);
		break;
	case 38: // Over Voltage2 Retry delay 
		FillsendBuff(fillData, ValIn);
		break;
	case 39: // Vin 1/2 Low HYSTERESIS 
	case 40: // Vin 1/2 High HYSTERESIS
		ValIn *= 10;
		FillsendBuff(fillData, ValIn);
		break;
	case 41: // Communication Timeout 
		if (temp.compare("disable") == 0)
			ValIn = 0;
		else if (temp.compare("50 ms") == 0)
			ValIn = 1;
		else if (temp.compare("100 ms") == 0)
			ValIn = 2;
		else if (temp.compare("250 ms") == 0)
			ValIn = 3;
		else if (temp.compare("500 ms") == 0)
			ValIn = 4;
		else if (temp.compare("1 sec") == 0)
			ValIn = 5;
		else if (temp.compare("1.5 sec") == 0)
			ValIn = 6;
		else if (temp.compare("2 sec") == 0)
			ValIn = 7;
		else if (temp.compare("5 sec") == 0)
			ValIn = 8;
		else if (temp.compare("10 sec") == 0)
			ValIn = 9;
		else if (temp.compare("30 sec") == 0)
			ValIn = 10;
		else if (temp.compare("1 min") == 0)
			ValIn = 11;
		else if (temp.compare("5 min") == 0)
			ValIn = 12;
		else if (temp.compare("10 min") == 0)
			ValIn = 13;
		else if (temp.compare("20 min") == 0)
			ValIn = 14;
		else if (temp.compare("") == 0)
			ValIn = 15;
		else
			return InternalError("DB", -1, ""); // Error 219
		FillsendBuff(fillData, ValIn);
		break; 
	default:
		break;
}

	fillMsg(Type, 0xB, ID, dstAdd, message, fillData, 8, PsNetwork);
	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);
	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	return "success";
}

// Get Channel Parameters function.
__declspec(dllexport) char* __cdecl GetChannelParam(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ChannelNo,
	int ParamNo,
	sNETWORK* PsNetwork = NULL) {
	int MaxParamNum = 35;
	if (ParamNo > MaxParamNum || ParamNo < 0)
		return "Parameter number doesn't exist.";
	int ParamExist[] = {
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
		1, 1, 1, 1, 1, 1, 0, 1, 0, 0, \
		1, 0, 1, 1, 1, 1
	};
	if (!ParamExist[ParamNo])
		return "Parameter number doesn't exist.";

	std::stringstream stream;
	std::string s;

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();
	std::string temp = "", data = "";
	char *StringChannelOut = new char();
	int combined = 0;
	unsigned __int8 Data8[4];

	if (ParamNo == 5) {
		for (int i = 0; i < 4; i++)
		{
			for (int l = 0; l < 4; l++)
				fillData[l] = 0;
			fillData[0] = 5; //  Channel General string 
			fillData[1] = ChannelNo;
			fillData[3] = i * 6;

			fillMsg(Type, 0xC, ID, dstAdd, message, fillData, 4, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

			txrx = receive(Type, message, fillData, DELTA_T);

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					delete[] message;
					return errorReturn;
				}
				delete[] message;
				return ErrorDefinition(txrx);
			}

			for (int k = 0; k < 6; k++)
				temp += static_cast<char>(fillData[k]);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_RX_TX));
		}
		delete[] message;
		StringChannelOut = (char*)malloc(temp.length() + 1 + 1);
		strcpy_s(StringChannelOut, temp.length() + 1, temp.c_str());
		return StringChannelOut;
	}
	else if (ParamNo == 25) {
		return _unbuild(Type, ID, dstAdd, ChannelNo, ParamNo, PsNetwork, fillData, message, CHANNEL_PARAM);

#ifdef INSIDE
		for (int j = 0; j < 4; j++) {
			for (int i = 0; i < 4; i++)
				fillData[i] = 0;
			if (j >= 2)
				fillData[0] = ParamNo + j + 1;
			else
				fillData[0] = ParamNo + j;
			fillData[1] = ChannelNo;

			fillMsg(Type, 0xC, ID, dstAdd, message, fillData, 4, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);
			std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TX_RX));

			txrx = receive(Type, message, fillData, DELTA_T);

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					delete[] message;
					return errorReturn;
				}
				delete[] message;
				return ErrorDefinition(txrx);
			}
			combined = (fillData[3] << 24) | (fillData[2] << 16) | (fillData[1] << 8) | (fillData[0]);
			Data8[0] = fillData[0];
			Data8[1] = fillData[1];
			Data8[2] = fillData[2];
			Data8[3] = fillData[3];

			for (int i = 0; i < 4; i++)
			{
				if (Data8[i] > 0 && Data8[i] <= LAST_CH)
				{
					temp += "In";
					if ((char)(Data8[i] + 48) == ':')
						temp += "10";
					else if ((char)(Data8[i] + 48) == ';')
						temp += "11";
					else if ((char)(Data8[i] + 48) == '<')
						temp += "12";
					else if ((char)(Data8[i] + 48) == '=')
						temp += "13";
					else if ((char)(Data8[i] + 48) == '>')
						temp += "14";
					else if ((char)(Data8[i] + 48) == '?')
						temp += "15";
					else if ((char)(Data8[i] + 48) == '@')
						temp += "16";
					else
						temp += (char)(Data8[i] + 48);
				}
				else if (Data8[i] == EMERGENCY)
					temp += "EMRG";
				else if (Data8[i] >= OUT1 && Data8[i] <= OUT4) {
					temp += "OUT";
					temp += (char)(Data8[i] + 48 - OUT1 - 1);
				}
				else if (Data8[i] == PARAMERR)
					temp += "EPAR";
				else if (Data8[i] == TEMPERR)
					temp += "ETEM";
				else if (Data8[i] == VOLTERR)
					temp += "EVOL";
				else if (Data8[i] == BOM)
					temp += "EBOM";
				else if (Data8[i] == BITdef)
					temp += "EBIT";
				else if (Data8[i] >= CH1_STS && Data8[i] <= (CH1_STS + LAST_CH)) {
					temp += "SCH";
					temp += (char)(Data8[i] - CH1_STS - 1 + 48);
				}
				else if (Data8[i] >= CH1_TRIP && Data8[i] <= (CH1_TRIP + LAST_CH)) {
					temp += "TCH";
					temp += (char)(Data8[i] - CH1_TRIP - 1 + 48);
				}
				else if (Data8[i] >= CH1_ERR && Data8[i] <= (CH1_ERR + LAST_CH)) {
					temp += "ECH";
					temp += (char)(Data8[i] - CH1_ERR - 1 + 48);
				}
				else if (Data8[i] >= NOT && Data8[i] <= (NOT + LAST_CH))
				{
					temp += "!In";
					if ((char)(Data8[i] - NOT + 48) == ':')
						temp += "10";
					else if ((char)(Data8[i] - NOT + 48) == ';')
						temp += "11";
					else if ((char)(Data8[i] - NOT + 48) == '<')
						temp += "12";
					else if ((char)(Data8[i] - NOT + 48) == '=')
						temp += "13";
					else if ((char)(Data8[i] - NOT + 48) == '>')
						temp += "14";
					else if ((char)(Data8[i] - NOT + 48) == '?')
						temp += "15";
					else if ((char)(Data8[i] - NOT + 48) == '@')
						temp += "16";
					else
						temp += (char)(Data8[i] - NOT + 48);
				}
				else if (Data8[i] == NOT + EMERGENCY)
					temp += "!EMRG";
				else if (Data8[i] >= (NOT + OUT1) && Data8[i] <= (NOT + OUT4)) {
					temp += "!OUT";
					temp += (char)(Data8[i] - (NOT + OUT1 - 1) + 48);
				}
				else if (Data8[i] == NOT + PARAMERR)
					temp += "!EPAR";
				else if (Data8[i] == NOT + TEMPERR)
					temp += "!ETEM";
				else if (Data8[i] == NOT + VOLTERR)
					temp += "!EVOL";
				else if (Data8[i] == NOT + BOM)
					temp += "!EBOM";
				else if (Data8[i] == NOT + BITdef)
					temp += "!EBIT";
				else if (Data8[i] >= (NOT + CH1_STS) && Data8[i] <= (NOT + CH16_STS)) {
					temp += "!SCH";
					temp += (char)(Data8[i] - (NOT + CH1_STS - 1) + 48);
				}
				else if (Data8[i] >= (NOT + CH1_TRIP) && Data8[i] <= (NOT + CH16_TRIP)) {
					temp += "!TCH";
					temp += (char)(Data8[i] - (NOT + CH1_TRIP - 1) + 48);
				}
				else if (Data8[i] >= (NOT + CH1_ERR) && Data8[i] <= (NOT + CH16_ERR)) {
					temp += "!ECH";
					temp += (char)(Data8[i] - (NOT + CH1_ERR - 1) + 48);
				}
				else if (Data8[i] == ADN_OPER)
					temp += "&";
				else if (Data8[i] == OR_OPER)
					temp += "|";
				else if (Data8[i] == XOR_OPER)
					temp += "^";
				else
					temp += "";
			}
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_RX_TX));
			message = new MsgType();
		}
		StringChannelOut = new char[temp.size() + 1];
		strcpy_s(StringChannelOut, temp.length() + 1, temp.c_str());
		delete[] message;
		if (StringChannelOut[0] == '.')
			return UNKNOWN_ERROR;
		return StringChannelOut;
#endif
	}
	else {

		for (int i = 0; i < 4; i++)
			fillData[i] = 0;
		fillData[0] = ParamNo;
		fillData[1] = ChannelNo;

		fillMsg(Type, 0xC, ID, dstAdd, message, fillData, 4, PsNetwork);

		int txrx = send(Type, message);
		if (txrx < 0)
			return ErrorDefinition(txrx);
		std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));//

		txrx = receive(Type, message, fillData);
		delete[] message;

		if (txrx < 0) {
			if (txrx == SYSTEM_ERROR_CODE) {
				static char errorReturn[14];
				Error(Type, fillData, errorReturn);
				return errorReturn;
			}
			return ErrorDefinition(txrx);
		}
		combined = (fillData[3] << 24) | (fillData[2] << 16) | (fillData[1] << 8) | (fillData[0]);
		Data8[0] = fillData[0];
		Data8[1] = (fillData[1] << 8);
		Data8[2] = (fillData[2] << 16);
		Data8[3] = (fillData[3] << 24);

		switch (fillData[6])
		{
		case 0: // Current threshold  
			temp = std::to_string(combined);
			break;
		case 1: // On delay
			temp = std::to_string(combined);
			break;
		case 2: // Hard Short Retry delay 
			temp = std::to_string(combined);
			break;
		case 3: // Hard Short Number of retry
			temp = std::to_string(combined);
			break;
		case 4: // Active on startup   
		{
			if (combined == 1)
				return "Active";
			else
				return "Not Active";
		}
		case 6: // Ch.Source control
		{
			switch (combined) {
			case 1:
				return "Enable";
				break;
			case 2:
				return "Enable&Timeout";
				break;
			case 0:
			default:
				return "Disable";
				break;
			}
		}
		case 7: // SW Tripped Retry delay 
			temp = std::to_string(combined);
			break;
		case 8: // SW Tripped Number of retry
			temp = std::to_string(combined);
			break;
		case 9: // Ch.Status reset delay //0.001 * 
			temp = std::to_string(combined);
			break;
		case 10: // Current OFF Time 
			temp = std::to_string(combined);
			break;
		case 11: // Max Current OFF Time 
			stream << std::fixed << std::setprecision(0) << combined;
			temp = stream.str();
			break;
		case 12: // Max Current threshold 
			stream << std::fixed << std::setprecision(1) << 0.10 * combined;
			temp = stream.str();
			break;
		case 13: // Nominal current value
			stream << std::fixed << std::setprecision(1) << 0.10 * combined;
			temp = stream.str();
			break;
		case 14: // I ^ 2 t threshold   
			stream << std::fixed << std::setprecision(3) << 0.0010 * combined;
			temp = stream.str();
			break;
		case 15: // Group channel control 
			switch (combined) {
			case 0:
				return "None";
			case 1:
				return "Group 1 S";
			case 2:
				return "Group 1 M";
			case 3:
				return "Group 2 S";
			case 4:
				return "Group 2 M";
			case 5:
				return "Group 3 S";
			case 6:
				return "Group 3 M";
			case 7:
				return "Group 4 S";
			case 8:
				return "Group 4 M";
			case 9:
				return "Group 5 S";
			case 10:
				return "Group 5 M";
			case 11:
				return "Group 6 S";
			case 12:
				return "Group 6 M";
			default:
				return "None";
			}
		case 16: // Technician data
			stream << std::fixed << std::setprecision(0) << 1.00 *combined;
			temp = stream.str();
			break;
		case 17: // PWM frequency
			stream << std::fixed << std::setprecision(2) << 0.010 * combined;
			temp = stream.str();
			break;
		case 18: // PWM duty cycle
			stream << std::fixed << std::setprecision(0) << 1.00 *combined;
			temp = stream.str();
			break;
		case 19: // Power Protection
			stream << std::fixed << std::setprecision(0) << 1.00 *combined;
			temp = stream.str();
			break;
		case 20: // I MAX Threshold ON      
			stream << std::fixed << std::setprecision(1) << 0.10 * combined;
			temp = stream.str();
			break;
		case 21: // PWM Control
		{
			if (combined == 1)
				return "Enable";
			else
				return "Disable";
		}
		case 22: // Ch. Delay before Opening
			temp = std::to_string(combined);
			break;
		case 23: // Ch. Delay before Closing
			temp = std::to_string(combined);
			break;
		case 24: // Ch. OFF Timeout 
			temp = std::to_string(combined);
			break;
		case 27: // MAX ON Curr Time
			stream << std::fixed << std::setprecision(0) << combined;
			temp = stream.str();
			break;
		case 30: // Log Curr TH  
			stream << std::fixed << std::setprecision(0) << combined;
			temp = stream.str();
			break;
		case 32: // Over Voltage Number of retry
			temp = std::to_string(combined);
			break;
		case 33: // Channel priority scenario 1
			temp = std::to_string(combined);
			break;
		case 34: // Channel priority scenario 2
			temp = std::to_string(combined);
			break;
		case 35: // Channel priority scenario 3
			temp = std::to_string(combined);
			break;
		default:
			temp = "ParamError";
			break;
		}

		StringChannelOut = new char[temp.length() + 1];
		strcpy_s(StringChannelOut, temp.length() + 1, temp.c_str());
		return StringChannelOut;
	}
}

// Set Channel Parameters function.
__declspec(dllexport) char* __cdecl SetChannelParam(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ChannelNo,
	int ParamNo,
	float ValIn,
	char *StrIn,
	sNETWORK* PsNetwork = NULL) {
	int MaxParamNum = 35;
	if (ParamNo > MaxParamNum || ParamNo < 0)
		return "Parameter number doesn't exist.";
	int ParamExist[36] = {
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
		1, 1, 1, 1, 1, 1, 0, 1, 0, 0, \
		1, 0, 1, 1, 1, 1
	};

	if (!ParamExist[ParamNo])
		return "Parameter number doesn't exist.";

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	int SubArr[32] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

	std::string temp(StrIn);
	std::transform(temp.begin(), temp.end(), temp.begin(), ::tolower);
	bool flag = true, valid = false;
	int index = 0, Val = 0, DataIndex = 0;
	unsigned __int8 DataOut[16] = { 0 };
	int k = 0;
	for (k = 0; k < 100; k++) {
		if (StrIn[k] == '\0')
			break;
	}
	char *cDataIn = (char*)malloc(k + 1 + 1);
	unsigned __int8 *DataIn = (unsigned __int8*)malloc(k + 1 + 1);

	for (int i = 0; i < k; i++) {
		cDataIn[i] = StrIn[i];
		DataIn[i] = static_cast<int>(toupper(cDataIn[i]));
	}

	for (int i = 0; i < 8; i++)
		fillData[i] = 0;
	fillData[0] = ParamNo;
	fillData[1] = ChannelNo;
	fillData[2] = 83; //'S' Set

	switch (ParamNo)
	{
	case 0: // Current threshold 
		FillsendBuff(fillData, ValIn);
		break;
	case 1: // On delay
		FillsendBuff(fillData, ValIn);
		break;
	case 2: // Hard Short Retry delay 
		FillsendBuff(fillData, ValIn);
		break;
	case 3: // Hard Short Number of retry 
		FillsendBuff(fillData, ValIn);
		break;
	case 4: // Active on startup 
		if (temp.compare("active") == 0) // if equal
			FillsendBuff(fillData, 1);
		else
			FillsendBuff(fillData, 0);
		break;
	case 5: //  Channel General string 
	{
		if (strlen(StrIn) < 25) {
			for (int i = 0; i < (int)strlen(StrIn); i++)
				SubArr[i] = static_cast<int>(StrIn[i]);
			index = 0;

			for (int i = 0; i < 5; i++)
			{
				for (int i = 0; i < 8; i++)
					fillData[i] = 0;
				fillData[0] = ParamNo;
				fillData[1] = ChannelNo;
				fillData[2] = 83; //'S' Set
				fillData[3] = i * 4;
				fillData[4] = SubArr[i * 4];
				fillData[5] = SubArr[(i * 4) + 1];
				fillData[6] = SubArr[(i * 4) + 2];
				fillData[7] = SubArr[(i * 4) + 3];

				fillMsg(Type, 0xC, ID, dstAdd, message, fillData, 8, PsNetwork);

				int txrx = send(Type, message);
				if (txrx < 0)
					return ErrorDefinition(txrx);

				std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

				txrx = receive(Type, message, fillData, DELTA_T);

				if (txrx < 0) {
					if (txrx == SYSTEM_ERROR_CODE) {
						static char errorReturn[14];
						Error(Type, fillData, errorReturn);
						delete[] message;
						return errorReturn;
					}
					delete[] message;
					return ErrorDefinition(txrx);
				}
				index += txrx;
			}

			delete[] message;
			if (index == 5)
				return "success";
			return UNKNOWN_ERROR;
		}
		else return InternalError("", ChannelNo, "D8");
	}
	break;
	case 6: // Ch.Source control
		fillData[3] = 0;// IOControlIn;
		if (temp.compare("enable") == 0)
			ValIn = 1;
		else if (temp.compare("enable&timeout") == 0)
			ValIn = 2;
		else
			ValIn = 0;
		FillsendBuff(fillData, ValIn);
		break;
	case 7: // SW Tripped Retry delay
		FillsendBuff(fillData, ValIn);
		break;
	case 8: // SW Tripped Number of retry
		FillsendBuff(fillData, ValIn);
		break;
	case 9: // Ch.Status reset delay 
		FillsendBuff(fillData, ValIn);
		break;
	case 10: // Current OFF Time
		FillsendBuff(fillData, ValIn);
		break;
	case 11: // Max Current OFF Time 
		FillsendBuff(fillData, ValIn);
		break;
	case 12: // Max Current threshold 
		ValIn *= 10;
		FillsendBuff(fillData, ValIn);
		break;
	case 13: // Nominal current value
		ValIn *= 10;
		FillsendBuff(fillData, ValIn);
		break;
	case 14: // I ^ 2 t threshold 
		ValIn *= 1000;
		FillsendBuff(fillData, ValIn);
		break;
	case 15: // Group channel control 
	{
		if (temp.compare("none") == 0)
			ValIn = 0;
		else if (temp.compare("group 1 s") == 0)
			ValIn = 1;
		else if (temp.compare("group 1 m") == 0)
			ValIn = 2;
		else if (temp.compare("group 2 s") == 0)
			ValIn = 3;
		else if (temp.compare("group 2 m") == 0)
			ValIn = 4;
		else if (temp.compare("group 3 s") == 0)
			ValIn = 5;
		else if (temp.compare("group 3 m") == 0)
			ValIn = 6;
		else if (temp.compare("group 4 s") == 0)
			ValIn = 7;
		else if (temp.compare("group 4 m") == 0)
			ValIn = 8;
		else if (temp.compare("group 5 s") == 0)
			ValIn = 9;
		else if (temp.compare("group 5 m") == 0)
			ValIn = 10;
		else if (temp.compare("group 6 s") == 0)
			ValIn = 11;
		else if (temp.compare("group 6 m") == 0)
			ValIn = 12;
		else
			return InternalError("", ChannelNo, "D9"); // Error 217
	}
	FillsendBuff(fillData, ValIn);
	break;
	case 16: // Technician data
		FillsendBuff(fillData, ValIn);
		break;
	case 17: // PWM frequency
		ValIn *= 100;
		FillsendBuff(fillData, ValIn);
		break;
	case 18: // PWM duty cycle
		FillsendBuff(fillData, ValIn);
		break;
	case 19: // Power Protection
		if (!FillsendBuffStr(fillData, temp)) {
			return InternalError("", ChannelNo, "DB"); // Error 219
		}
		break;
	case 20: // I MAX Threshold ON      
		ValIn *= 10;
		FillsendBuff(fillData, ValIn);
		break;
	case 21: // PWM Control
		if (temp.compare("enable") == 0)
			FillsendBuff(fillData, 1);
		else
			FillsendBuff(fillData, 0);
		break;
	case 22: // Ch. Delay before Opening
		FillsendBuff(fillData, ValIn);
		break;
	case 23: // Ch. Delay before Closing
		FillsendBuff(fillData, ValIn);
		break;
	case 24: // Ch. OFF Timeout 
		FillsendBuff(fillData, ValIn);
		break;
	case 25: // Dig. Input Cnt L
		return _build_control(Type, ID, dstAdd, message, PsNetwork, k, fillData, ParamNo, ChannelNo, DataOut, DataIn, Val, CHANNEL_PARAM);
#ifdef INSIDE
		flag = false;
		index = 0, DataIndex = 0;
		if (k == 0)
			flag = true;
#ifdef OLD_CHANNEL_PARAM_25
		while (!flag) {
			if (DataIn[index] == 'I') {
				index++;
				if (DataIn[index] == 'N') {
					index++;
					if (DataIn[index] > '0' && DataIn[index] <= '9') {
						Val = DataIn[index] - 48;
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '2') {
							Val *= 10;
							Val += DataIn[index] - 48;
							DataOut[DataIndex] = Val;
							index++;
							DataIndex++;
						}
						else {
							DataOut[DataIndex] = Val;
							DataIndex++;
						}
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'S') {
				index++;
				if (DataIn[index] == 'C') {
					index++;
					if (DataIn[index] == 'H') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - 48;
							index++;
							if (DataIn[index] >= '0' && DataIn[index] <= '2') {
								Val *= 10;
								Val += DataIn[index] - 48;
								DataOut[DataIndex] = Val + 23;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + 23;
								DataIndex++;
							}
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'T') {
				index++;
				if (DataIn[index] == 'C') {
					index++;
					if (DataIn[index] == 'H') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - 48;
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '2') {
								Val *= 10;
								Val += DataIn[index] - 48;
								DataOut[DataIndex] = Val + 35;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + 35;
								DataIndex++;
							}
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'O') {
				index++;
				if (DataIn[index] == 'U') {
					index++;
					if (DataIn[index] == 'T') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '4') {
							Val = DataIn[index] - 48;
							DataOut[DataIndex] = Val + 13;
							DataIndex++; index++;
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'E') {
				index++;
				if (DataIn[index] == 'C') {
					index++;
					if (DataIn[index] == 'H') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - 48;
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '2') {
								Val *= 10;
								Val += DataIn[index] - 48;
								DataOut[DataIndex] = Val + 47;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + 47;
								DataIndex++;
							}
						}
						else flag = true;
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'V') {
					index++;
					if (DataIn[index] == 'O') {
						index++;
						if (DataIn[index] == 'L') {
							DataOut[DataIndex] = 20;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'B') {
					index++;
					if (DataIn[index] == 'I') {
						index++;
						if (DataIn[index] == 'T') {
							DataOut[DataIndex] = 22;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'O') {
						index++;
						if (DataIn[index] == 'M') {
							DataOut[DataIndex] = 21;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'P') {
					index++;
					if (DataIn[index] == 'A') {
						index++;
						if (DataIn[index] == 'R') {
							DataOut[DataIndex] = 18;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'M') {
					index++;
					if (DataIn[index] == 'R') {
						index++;
						if (DataIn[index] == 'G') {
							DataOut[DataIndex] = 13;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'T') {
					index++;
					if (DataIn[index] == 'E') {
						index++;
						if (DataIn[index] == 'M') {
							DataOut[DataIndex] = 19;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == '!') {
				index++;
				if (DataIn[index] == 'I') {
					index++;
					if (DataIn[index] == 'N') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - 48;
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '2') {
								Val *= 10;
								Val += DataIn[index] - 48;
								DataOut[DataIndex] = Val + 60;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + 60;
								DataIndex++;
							}
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'S') {
					index++;
					if (DataIn[index] == 'C') {
						index++;
						if (DataIn[index] == 'H') {
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '9') {
								Val = DataIn[index] - 48;
								index++;
								if (DataIn[index] > '0' && DataIn[index] <= '2') {
									Val *= 10;
									Val += DataIn[index] - 48;
									DataOut[DataIndex] = Val + 23 + 60;
									index++;
									DataIndex++;
								}
								else {
									DataOut[DataIndex] = Val + 23 + 60;
									DataIndex++;
								}
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'T') {
					index++;
					if (DataIn[index] == 'C') {
						index++;
						if (DataIn[index] == 'H') {
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '9') {
								Val = DataIn[index] - 48;
								index++;
								if (DataIn[index] > '0' && DataIn[index] <= '2') {
									Val *= 10;
									Val += DataIn[index] - 48;
									DataOut[DataIndex] = Val + 35 + 60;
									index++;
									DataIndex++;
								}
								else {
									DataOut[DataIndex] = Val + 35 + 60;
									DataIndex++;
								}
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'O') {
					index++;
					if (DataIn[index] == 'U') {
						index++;
						if (DataIn[index] == 'T') {
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '4') {
								Val = DataIn[index] - 48;
								DataOut[DataIndex] = Val + 13 + 60;
								DataIndex++; index++;
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'E') {
					index++;
					if (DataIn[index] == 'C') {
						index++;
						if (DataIn[index] == 'H') {
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '9') {
								Val = DataIn[index] - 48;
								index++;
								if (DataIn[index] > '0' && DataIn[index] <= '2') {
									Val *= 10;
									Val += DataIn[index] - 48;
									DataOut[DataIndex] = Val + 47 + 60;
									index++;
									DataIndex++;
								}
								else {
									DataOut[DataIndex] = Val + 47 + 60;
									DataIndex++;
								}
							}
							else flag = true;
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == 'V') {
						index++;
						if (DataIn[index] == 'O') {
							index++;
							if (DataIn[index] == 'L') {
								DataOut[DataIndex] = 20 + 60;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'B') {
						index++;
						if (DataIn[index] == 'I') {
							index++;
							if (DataIn[index] == 'T') {
								DataOut[DataIndex] = 22 + 60;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else if (DataIn[index] == 'O') {
							index++;
							if (DataIn[index] == 'M') {
								DataOut[DataIndex] = 21 + 60;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'P') {
						index++;
						if (DataIn[index] == 'A') {
							index++;
							if (DataIn[index] == 'R') {
								DataOut[DataIndex] = 18 + 60;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'M') {
						index++;
						if (DataIn[index] == 'R') {
							index++;
							if (DataIn[index] == 'G') {
								DataOut[DataIndex] = 13 + 60;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'T') {
						index++;
						if (DataIn[index] == 'E') {
							index++;
							if (DataIn[index] == 'M') {
								DataOut[DataIndex] = 19 + 60;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == ' ')
					flag = false;
				else if (DataIn[index] == 200)
					flag = true;
				else flag = true;
			}
			else if (DataIn[index] == '&') {
				DataOut[DataIndex] = 120;
				DataIndex++; index++;
			}
			else if (DataIn[index] == '|') {
				DataOut[DataIndex] = 121;
				DataIndex++; index++;
			}
			else if (DataIn[index] == '^') {
				DataOut[DataIndex] = 122;
				DataIndex++; index++;
			}
			else if (DataIn[index] == ' ') {
				index++;
			}
			else {
				flag = true;
				return InternalError("DD", -1, "");
			}
			if (index == k)
				flag = true;
		}
#else //  OLD_CHANNEL_PARAM_25
		while (!flag) {
			if (DataIn[index] == 'I') {
				index++;
				if (DataIn[index] == 'N') {
					index++;
					if (DataIn[index] > '0' && DataIn[index] <= '9') {
						Val = DataIn[index] - 48;
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= CHANNEL_NUMBER_UNIT_DIGIT) { // 16 channels
							Val *= 10;
							Val += DataIn[index] - 48;
							DataOut[DataIndex] = Val;
							index++;
							DataIndex++;
						}
						else {
							DataOut[DataIndex] = Val;
							DataIndex++;
						}
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'S') {
				index++;
				if (DataIn[index] == 'C') {
					index++;
					if (DataIn[index] == 'H') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - 48;
							index++;
							if (DataIn[index] >= '0' && DataIn[index] <= CHANNEL_NUMBER_UNIT_DIGIT) {
								Val *= 10;
								Val += DataIn[index] - 48;
								DataOut[DataIndex] = Val + CH1_STS - 1;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + CH1_STS - 1;
								DataIndex++;
							}
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'T') {
				index++;
				if (DataIn[index] == 'A') {
					index++;
					if (DataIn[index] == 'N') {
						index++;
						if (DataIn[index] == 'C') {
							index++;
							if (DataIn[index] == 'H') {
								DataOut[DataIndex] = ANY_TRIP;
								DataIndex++; index++;
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'C') {
					index++;
					if (DataIn[index] == 'H') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - 48;
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= CHANNEL_NUMBER_UNIT_DIGIT) {
								Val *= 10;
								Val += DataIn[index] - 48;
								DataOut[DataIndex] = Val + CH1_TRIP - 1;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + CH1_TRIP - 1;
								DataIndex++;
							}
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'O') {
				index++;
				if (DataIn[index] == 'U') {
					index++;
					if (DataIn[index] == 'T') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '4') {
							Val = DataIn[index] - 48;
							DataOut[DataIndex] = Val + OUT1 - 1;
							DataIndex++; index++;
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == 'E') {
				index++;
				if (DataIn[index] == 'C') {
					index++;
					if (DataIn[index] == 'H') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - 48;
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= CHANNEL_NUMBER_UNIT_DIGIT) {
								Val *= 10;
								Val += DataIn[index] - 48;
								DataOut[DataIndex] = Val + CH1_ERR - 1;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + CH1_ERR - 1;
								DataIndex++;
							}
						}
						else flag = true;
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'V') {
					index++;
					if (DataIn[index] == '1') {
						index++;
						if (DataIn[index] == 'L') {
							Val = DataIn[index] - 48;
							DataOut[DataIndex] = Val + VIN1_LOW - 1;
							DataIndex++; index++;
						}
						else if (DataIn[index] == 'H') {
							Val = DataIn[index] - 48;
							DataOut[DataIndex] = Val + VIN1_HIGH - 1;
							DataIndex++; index++;
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == '2') {
						index++;
						if (DataIn[index] == 'L') {
							Val = DataIn[index] - 48;
							DataOut[DataIndex] = Val + VIN2_LOW - 1;
							DataIndex++; index++;
						}
						else if (DataIn[index] == 'H') {
							Val = DataIn[index] - 48;
							DataOut[DataIndex] = Val + VIN2_HIGH - 1;
							DataIndex++; index++;
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == 'I') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '2') {
							Val = DataIn[index] - 48;
							DataOut[DataIndex] = Val + VIN1 - 1;
							DataIndex++; index++;
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == 'O') {
						index++;
						if (DataIn[index] == 'L') {
							DataOut[DataIndex] = VOLTERR;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'B') {
					index++;
					if (DataIn[index] == 'I') {
						index++;
						if (DataIn[index] == 'T') {
							DataOut[DataIndex] = BITdef;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'O') {
						index++;
						if (DataIn[index] == 'M') {
							DataOut[DataIndex] = BOM;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'P') {
					index++;
					if (DataIn[index] == 'A') {
						index++;
						if (DataIn[index] == 'R') {
							DataOut[DataIndex] = PARAMERR;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'M') {
					index++;
					if (DataIn[index] == 'R') {
						index++;
						if (DataIn[index] == 'G') {
							DataOut[DataIndex] = EMERGENCY;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else if (DataIn[index] == 'T') {
					index++;
					if (DataIn[index] == 'E') {
						index++;
						if (DataIn[index] == 'M') {
							DataOut[DataIndex] = TEMPERR;
							DataIndex++; index++;
						}
						else flag = true;
					}
					else flag = true;
				}
				else return InternalError("DD", -1, "");
			}
			else if (DataIn[index] == '!') {
				index++;
				if (DataIn[index] == 'I') {
					index++;
					if (DataIn[index] == 'N') {
						index++;
						if (DataIn[index] > '0' && DataIn[index] <= '9') {
							Val = DataIn[index] - 48;
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '2') {
								Val *= 10;
								Val += DataIn[index] - 48;
								DataOut[DataIndex] = Val + 60;
								index++;
								DataIndex++;
							}
							else {
								DataOut[DataIndex] = Val + 60;
								DataIndex++;
							}
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'S') {
					index++;
					if (DataIn[index] == 'C') {
						index++;
						if (DataIn[index] == 'H') {
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '9') {
								Val = DataIn[index] - 48;
								index++;
								if (DataIn[index] > '0' && DataIn[index] <= '2') {
									Val *= 10;
									Val += DataIn[index] - 48;
									DataOut[DataIndex] = Val + 23 + 60;
									index++;
									DataIndex++;
								}
								else {
									DataOut[DataIndex] = Val + 23 + 60;
									DataIndex++;
								}
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'T') {
					index++;
					if (DataIn[index] == 'C') {
						index++;
						if (DataIn[index] == 'H') {
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '9') {
								Val = DataIn[index] - 48;
								index++;
								if (DataIn[index] > '0' && DataIn[index] <= '2') {
									Val *= 10;
									Val += DataIn[index] - 48;
									DataOut[DataIndex] = Val + 35 + 60;
									index++;
									DataIndex++;
								}
								else {
									DataOut[DataIndex] = Val + 35 + 60;
									DataIndex++;
								}
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'O') {
					index++;
					if (DataIn[index] == 'U') {
						index++;
						if (DataIn[index] == 'T') {
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '4') {
								Val = DataIn[index] - 48;
								DataOut[DataIndex] = Val + 13 + 60;
								DataIndex++; index++;
							}
							else return InternalError("DD", -1, "");
						}
						else return InternalError("DD", -1, "");
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == 'E') {
					index++;
					if (DataIn[index] == 'C') {
						index++;
						if (DataIn[index] == 'H') {
							index++;
							if (DataIn[index] > '0' && DataIn[index] <= '9') {
								Val = DataIn[index] - 48;
								index++;
								if (DataIn[index] > '0' && DataIn[index] <= '2') {
									Val *= 10;
									Val += DataIn[index] - 48;
									DataOut[DataIndex] = Val + 47 + 60;
									index++;
									DataIndex++;
								}
								else {
									DataOut[DataIndex] = Val + 47 + 60;
									DataIndex++;
								}
							}
							else flag = true;
						}
						else return InternalError("DD", -1, "");
					}
					else if (DataIn[index] == 'V') {
						index++;
						if (DataIn[index] == 'O') {
							index++;
							if (DataIn[index] == 'L') {
								DataOut[DataIndex] = 20 + 60;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'B') {
						index++;
						if (DataIn[index] == 'I') {
							index++;
							if (DataIn[index] == 'T') {
								DataOut[DataIndex] = 22 + 60;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else if (DataIn[index] == 'O') {
							index++;
							if (DataIn[index] == 'M') {
								DataOut[DataIndex] = 21 + 60;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'P') {
						index++;
						if (DataIn[index] == 'A') {
							index++;
							if (DataIn[index] == 'R') {
								DataOut[DataIndex] = 18 + 60;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'M') {
						index++;
						if (DataIn[index] == 'R') {
							index++;
							if (DataIn[index] == 'G') {
								DataOut[DataIndex] = 13 + 60;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else if (DataIn[index] == 'T') {
						index++;
						if (DataIn[index] == 'E') {
							index++;
							if (DataIn[index] == 'M') {
								DataOut[DataIndex] = 19 + 60;
								DataIndex++; index++;
							}
							else flag = true;
						}
						else flag = true;
					}
					else return InternalError("DD", -1, "");
				}
				else if (DataIn[index] == ' ')
					flag = false;
				else if (DataIn[index] == 200)
					flag = true;
				else flag = true;
			}
			else if (DataIn[index] == '&') {
				DataOut[DataIndex] = ADN_OPER;
				DataIndex++; index++;
			}
			else if (DataIn[index] == '|') {
				DataOut[DataIndex] = OR_OPER;
				DataIndex++; index++;
			}
			else if (DataIn[index] == '^') {
				DataOut[DataIndex] = XOR_OPER;
				DataIndex++; index++;
			}
			else if (DataIn[index] == ' ') {
				index++;
			}
			else {
				flag = true;
				return InternalError("DD", -1, "");
			}
			if (index == k)
				flag = true;
		}
#endif //  OLD_CHANNEL_PARAM_25

		index = 0;
		{
			fillData[0] = ParamNo;
			fillData[1] = ChannelNo;
			fillData[2] = 83; //'S' Setv
			fillData[4] = DataOut[0];
			fillData[5] = DataOut[1];
			fillData[6] = DataOut[2];
			fillData[7] = DataOut[3];
			fillMsg(Type, 0xC, ID, dstAdd, message, fillData, 8, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			txrx = receive(Type, message, fillData, DELTA_T);

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					return errorReturn;
				}
				return ErrorDefinition(txrx);
			}
			index += txrx;
		}
		{
			fillData[0] = 26;
			fillData[1] = ChannelNo;
			fillData[2] = 83; //'S' Setv
			fillData[4] = DataOut[4];
			fillData[5] = DataOut[5];
			fillData[6] = DataOut[6];
			fillData[7] = DataOut[7];
			fillMsg(Type, 0xC, ID, dstAdd, message, fillData, 8, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			txrx = receive(Type, message, fillData, DELTA_T);

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					return errorReturn;
				}
				return ErrorDefinition(txrx);
			}
			index += txrx;
		}
		{
			fillData[0] = 28;
			fillData[1] = ChannelNo;
			fillData[2] = 83; //'S' Set
			fillData[4] = DataOut[8];
			fillData[5] = DataOut[9];
			fillData[6] = DataOut[10];
			fillData[7] = DataOut[11];
			fillMsg(Type, 0xC, ID, dstAdd, message, fillData, 8, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			txrx = receive(Type, message, fillData, DELTA_T);

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					return errorReturn;
				}
				return ErrorDefinition(txrx);
			}
			index += txrx;
		}
		{
			fillData[0] = 29;
			fillData[1] = ChannelNo;
			fillData[2] = 83; //'S' Set
			fillData[4] = DataOut[12];
			fillData[5] = DataOut[13];
			fillData[6] = DataOut[14];
			fillData[7] = DataOut[15];
			fillMsg(Type, 0xC, ID, dstAdd, message, fillData, 8, PsNetwork);

			int txrx = send(Type, message);
			if (txrx < 0)
				return ErrorDefinition(txrx);
			std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
			txrx = receive(Type, message, fillData);
			delete[] message;

			if (txrx < 0) {
				if (txrx == SYSTEM_ERROR_CODE) {
					static char errorReturn[14];
					Error(Type, fillData, errorReturn);
					return errorReturn;
				}
				return ErrorDefinition(txrx);
			}
			index += txrx;
	}
		if (index == 4)
			return "success";
		return UNKNOWN_ERROR;
#endif
		break;
	case 27: // MAX ON Curr Time
		FillsendBuff(fillData, ValIn);
		break;
	case 30: // Log Curr TH  
		FillsendBuff(fillData, ValIn);
		break;
	case 32: // Over Voltage Number of retry
		FillsendBuff(fillData, ValIn);
		break;
	case 33: // Channel priority scenario 1
		FillsendBuff(fillData, ValIn);
		break;
	case 34: // Channel priority scenario 2
		FillsendBuff(fillData, ValIn);
		break;
	case 35: // Channel priority scenario 3
		FillsendBuff(fillData, ValIn);
		break;
	default:
		FillsendBuff(fillData, 0);
		break;
	}

	fillMsg(Type, 0xC, ID, dstAdd, message, fillData, 8, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);
	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	return "success";
}

// Bit Command function.
__declspec(dllexport) char* __cdecl BitCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ParamNo,
	int *Bit,
	int *Test,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();
	fillData[0] = ParamNo;
	fillData[1] = 1;

	std::string temp = "", LastBit = "", Empty = "";

	fillMsg(Type, 0x21, ID, dstAdd, message, fillData, 2, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			*Bit = -1;
			*Test = -1;
			LastBit = "Error";
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	int combined = (fillData[3] << 24) | (fillData[2] << 16) | (fillData[1] << 8) | (fillData[0]);
	int _Test = (fillData[6] << 16) | (fillData[5] << 8) | (fillData[4]);

	Empty = "success";
	for (int i = 0; i < 7; i++)
		temp += static_cast<char>(fillData[i]);

	switch (fillData[7]) {
	case 0: // BIT description 
		*Bit = combined;
		*Test = _Test;
		LastBit = Empty;
		break;
	case 1: // Last error sting 
		*Bit = combined;
		*Test = _Test;
		LastBit = Empty;
		break;
	case 2: // Last error timestamp 
		*Bit = combined;
		*Test = _Test;
		LastBit = temp;
		break;
	default: // Error
		*Bit = -1;
		*Test = -1;
		LastBit = "Error";
		break;
	}

	char* Val = (char*)malloc(LastBit.length() + 1 + 1);
	strcpy_s(Val, LastBit.length() + 1, LastBit.c_str());
	return Val;
}

// Get Discrete Ports function.
__declspec(dllexport) char* __cdecl GetDiscretePorts(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ParamNo,
	int* Result,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	for (int i = 0; i < 4; i++)
		fillData[i] = 0;
	fillData[0] = ParamNo;

	fillMsg(Type, 0x22, ID, dstAdd, message, fillData, 4, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	if (fillData[7] != 0) {
		*Result = (fillData[3] << 24) | (fillData[2] << 16) | (fillData[1] << 8) | (fillData[0]);
		return "success";
	}
	else
		return UNKNOWN_ERROR;
}

// Set Discrete Ports function.
__declspec(dllexport) char* __cdecl SetDiscretePorts(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ParamNo,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	for (int i = 0; i < 8; i++)
		fillData[i] = 0;
	fillData[0] = 3;
	fillData[2] = 83; //'S' Set
	fillData[4] = ParamNo & 0xFF;
	fillData[5] = ParamNo >> 8;
	fillMsg(Type, 0x22, ID, dstAdd, message, fillData, 8, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	return "success";
}

// Get Serial Number function.
__declspec(dllexport) char* __cdecl GetS_N(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork = NULL) {

	std::string temp;
	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	for (int i = 0; i < 2; i++)
	{
		for (int i = 0; i < 8; i++)
			fillData[i] = 0;
		fillData[3] = i * 6;

		fillMsg(Type, 0x24, ID, dstAdd, message, fillData, 8, PsNetwork);
		//message->udp_connection.sock = *PSock;

		int txrx = send(Type, message);
		if (txrx < 0)
			return ErrorDefinition(txrx);

		std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

		txrx = receive(Type, message, fillData, DELTA_T);

		if (txrx < 0) {
			if (txrx == SYSTEM_ERROR_CODE) {
				static char errorReturn[14];
				Error(Type, fillData, errorReturn);
				delete[] message;
				return errorReturn;
			}
			delete[] message;
			return ErrorDefinition(txrx);
		}

		for (int j = fillData[7], k = 0; j < (6 * (i + 1)), k < 6; j++, k++)
			temp += static_cast<char>(fillData[k]);
		std::this_thread::sleep_for(std::chrono::microseconds(WAIT_RX_TX));
	}

	char *SNOut = new char[temp.length() + 1];
	strcpy_s(SNOut, temp.length() + 1, temp.c_str());

	return SNOut;
}

// Set Serial Number function.
__declspec(dllexport) char* __cdecl SetS_N(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	char *SN,
	sNETWORK* PsNetwork = NULL) {

	std::string temp;
	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	int *SubArr = (int*)malloc(strlen(SN) + 1);
	for (int i = 0; i < (int)strlen(SN); i++)
		SubArr[i] = static_cast<int>(SN[i]);

	for (int i = 0; i < 3; i++)
	{
		for (int i = 0; i < 8; i++)
			fillData[i] = 0;
		fillData[2] = 83; // 'S' for Set
		fillData[3] = i * 4;
		fillData[4] = SubArr[(i * 4)];
		fillData[5] = SubArr[(i * 4) + 1];
		fillData[6] = SubArr[(i * 4) + 2];
		fillData[7] = SubArr[(i * 4) + 3];

		fillMsg(Type, 0x24, ID, dstAdd, message, fillData, 8, PsNetwork);

		int txrx = send(Type, message);
		if (txrx < 0)
			return ErrorDefinition(txrx);

		std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
	}
	delete[] message;

	return "success";
}

// Get Production Data function.
__declspec(dllexport) char* __cdecl GetProductionData(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork = NULL) {

	std::string temp;
	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	for (int i = 0; i < 6; i++)
	{
		for (int i = 0; i < 8; i++)
			fillData[i] = 0;
		fillData[3] = i * 6;

		fillMsg(Type, 0x14, ID, dstAdd, message, fillData, 8, PsNetwork);
		//message->udp_connection.sock = *PSock;

		int txrx = send(Type, message);
		if (txrx < 0)
			return ErrorDefinition(txrx);

		std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

		txrx = receive(Type, message, fillData, DELTA_T);

		if (txrx < 0) {
			if (txrx == SYSTEM_ERROR_CODE) {
				static char errorReturn[14];
				Error(Type, fillData, errorReturn);
				delete[] message;
				return errorReturn;
			}
			delete[] message;
			return ErrorDefinition(txrx);
		}

		for (int j = fillData[7], k = 0; j < (6 * (i + 1)), k < 6; j++, k++)
			temp += static_cast<char>(fillData[k]);
		std::this_thread::sleep_for(std::chrono::microseconds(WAIT_RX_TX));
	}

	char *SNOut = new char[temp.length() + 1];
	strcpy_s(SNOut, temp.length() + 1, temp.c_str());

	return SNOut;
}

// Set  Production Data function.
__declspec(dllexport) char* __cdecl SetProductionData(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	char *production_data,
	sNETWORK* PsNetwork = NULL) {

	std::string temp;
	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	int *SubArr = (int*)malloc(strlen(production_data) + 1);
	for (int i = 0; i < (int)strlen(production_data); i++)
		SubArr[i] = static_cast<int>(production_data[i]);

	for (int i = 0; i < 8; i++)
	{
		for (int i = 0; i < 8; i++)
			fillData[i] = 0;
		fillData[2] = 83; // 'S' for Set
		fillData[3] = i * 4;
		fillData[4] = SubArr[(i * 4)];
		fillData[5] = SubArr[(i * 4) + 1];
		fillData[6] = SubArr[(i * 4) + 2];
		fillData[7] = SubArr[(i * 4) + 3];

		fillMsg(Type, 0x14, ID, dstAdd, message, fillData, 8, PsNetwork);

		int txrx = send(Type, message);
		if (txrx < 0)
			return ErrorDefinition(txrx);

		std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));
	}
	delete[] message;

	return "success";
}

// Save configuration command function.
__declspec(dllexport) char* __cdecl ConfigSaveCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 174; // 0xAE
	fillData[1] = 117; // 0x75
	fillData[2] = 129; // 0x81
	fillData[3] = 165; // 0xA5
	fillData[4] = 3;	// 0x03

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 5, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::milliseconds(300));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	if (fillData[0] == 121) // 0x79- Ok ; 0xF2-Error
		return "success";
	else if ((fillData[0] >= 240) || (fillData[0] <= 252))
	{
		static char errorReturn[14];
		Error(Type, fillData, errorReturn);
		return errorReturn;
	}
	else
		return UNKNOWN_ERROR;
}

// Go to bootloader configuration command function.
__declspec(dllexport) char* __cdecl ConfigGotoBootloaderCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 174; // 0xAE
	fillData[1] = 117; // 0x75
	fillData[2] = 129; // 0x81
	fillData[3] = 52; // 0x34
	fillData[4] = 0; // 0x0

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 5, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::milliseconds(300));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	if (fillData[0] == 121) // 0x79- Ok ; 0xF2-Error
		return "success";
	else if ((fillData[0] >= 240) || (fillData[0] <= 252))
	{
		static char errorReturn[14];
		Error(Type, fillData, errorReturn);
		return errorReturn;
	}
	else
		return UNKNOWN_ERROR;
}

// SW reset configuration command function.
__declspec(dllexport) char* __cdecl ConfigSWResetCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 133; // 0x85
	fillData[1] = 162; // 0xA2
	fillData[2] = 52; // 0x34
	fillData[3] = 25; // 0x19
	fillData[4] = 1; // 0x1

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 5, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	return "success";
}

// Configuration Mode command function.
__declspec(dllexport) char* __cdecl ConfigConfigModeCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 163; // 0xA3
	fillData[1] = 89; // 0x59
	fillData[2] = 48; // 0x30
	fillData[3] = 247; // 0xF7
	fillData[4] = 2; // 0x2

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 5, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TX_RX));

	for (int i = 0; i < 8; i++)
		fillData[i] = 0;

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	if (fillData[0] == 121) // 0x79- Ok ; 0xF2-Error
		return "success";
	else if ((fillData[0] >= 240) || (fillData[0] <= 252))
	{
		static char errorReturn[14];
		Error(Type, fillData, errorReturn);
		return errorReturn;
	}
	else
		return UNKNOWN_ERROR;
}

// Sleep mode configuration mode command function
__declspec(dllexport) char* __cdecl ConfigSleepModeCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint8_t sts,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	if (sts)  //Enter to sleep
	{
		fillData[0] = 21; // 0x15
		fillData[1] = 138; // 0x8A
		fillData[2] = 25; // 0x19
		fillData[3] = 188; // 0xBC
		fillData[4] = 4; // 0x4		
	}
	else //Active mode
	{
		fillData[0] = 21; // 0x15
		fillData[1] = 138; // 0x8A
		fillData[2] = 25; // 0x19
		fillData[3] = 174; // 0xAE
		fillData[4] = 4; // 0x4
	}

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 5, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	if (fillData[0] == 121) // 0x79- Ok ; 0xF2-Error
		return "success";
	else if ((fillData[0] >= 240) || (fillData[0] <= 252))
	{
		static char errorReturn[14];
		Error(Type, fillData, errorReturn);
		return errorReturn;
	}
	else
		return UNKNOWN_ERROR;
}

// Standbye configuration mode command function.
__declspec(dllexport) char* __cdecl ConfigStdbyModeCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 130; // 0x82
	fillData[1] = 59; // 0x3B
	fillData[2] = 113; // 0x71
	fillData[3] = 31; // 0x1F
	fillData[4] = 5; // 0x5

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 5, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	if (fillData[0] == 121) // 0x79- Ok ; 0xF2-Error
		return "success";
	else if ((fillData[0] >= 240) || (fillData[0] <= 252))
	{
		static char errorReturn[14];
		Error(Type, fillData, errorReturn);
		return errorReturn;
	}
	else
		return UNKNOWN_ERROR;
}

// CAN Bus configuration mode command function.
__declspec(dllexport) char* __cdecl ConfigCANModeCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint16_t sts,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	if (sts == 1)  // Enter to CANBUS silent mode (0xAC328C01)
	{
		fillData[0] = 0x1;
		fillData[1] = 0x8C;
		fillData[2] = 0x32;
		fillData[3] = 0xAC;
		fillData[4] = 0x7;
	}
	else if (sts == 2) // Exit from CANBUS silent mode (0xAC328C02)
	{
		fillData[0] = 0x2;
		fillData[1] = 0x8C;
		fillData[2] = 0x32;
		fillData[3] = 0xAC;
		fillData[4] = 0x7;
	}
	else if (sts == 3) // CANBUS disable mode (0xAC328C03)
	{
		fillData[0] = 0x3;
		fillData[1] = 0x8C;
		fillData[2] = 0x32;
		fillData[3] = 0xAC;
		fillData[4] = 0x7;
	}
	else {
		fillData[0] = 0;
		fillData[1] = 0;
		fillData[2] = 0;
		fillData[3] = 0;
		fillData[4] = 0;
	}

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 5, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	if (fillData[0] == 121) // 0x79- Ok ; 0xF2-Error
		return "success";
	else if ((fillData[0] >= 240) || (fillData[0] <= 252))
	{
		static char errorReturn[14];
		Error(Type, fillData, errorReturn);
		return errorReturn;
	}
	else
		return UNKNOWN_ERROR;
}

// Tripped Event Channel Reset configuration command function.
__declspec(dllexport) char* __cdecl ConfigChannelResetCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint16_t ChannelNum,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 0x71;
	fillData[1] = 0xB2;
	fillData[2] = 0x4A;
	fillData[3] = (unsigned char)ChannelNum;
	fillData[4] = 0x06;

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 5, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	if (fillData[0] == 121) // 0x79- Ok ; 0xF2-Error
		return "success";
	else if ((fillData[0] >= 240) || (fillData[0] <= 252))
	{
		static char errorReturn[14];
		Error(Type, fillData, errorReturn);
		return errorReturn;
	}
	else
		return UNKNOWN_ERROR;
}

// Event status command function.
__declspec(dllexport) char* __cdecl EventStatusCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ChannelNo,
	int *HS_ResetCounter, // HardShot event counter
	int *SW_ResetCounter, // SW event counter 
	int *ResetFail, // FF Reset
	int *TripType,
	sNETWORK* PsNetwork = NULL) { // Trip type

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	for (int i = 0; i < 1; i++)
		fillData[i] = 0;
	fillData[0] = ChannelNo;

	fillMsg(Type, 0xF, ID, dstAdd, message, fillData, 1, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	*HS_ResetCounter = (fillData[1] << 8) | (fillData[0]); // HardShot event counter
	*SW_ResetCounter = (fillData[3] << 8) | (fillData[2]); // SW event counter 
	*ResetFail = (fillData[5] << 8) | (fillData[4]); // FF Reset
	*TripType = (fillData[7] << 8) | (fillData[6]); // Trip type	

	return "success";
}

// Get Part number command function.
__declspec(dllexport) char* __cdecl P_NCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork = NULL) {

	char *SWVerionOut = new char[20];
	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillMsg(Type, 0x27, ID, dstAdd, message, fillData, 0, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	std::string temp;
	for (int i = 0; i < 4; i++)
		temp += static_cast<char>(fillData[i + 4]);
	for (int i = 0; i < 4; i++)
		temp += static_cast<char>(fillData[i]);

	char *PNOut = (char*)malloc(temp.length() + 1 + 1);
	strcpy_s(PNOut, temp.length() + 1, temp.c_str());
	return PNOut;

}

// Get offset Data function.
__declspec(dllexport) char* __cdecl GetOffsetData(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ParamNo,
	int ChannelNo,
	int OffsetType,
	float* Result,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	for (int i = 0; i < 4; i++)
		fillData[i] = 0;

	fillData[0] = ParamNo; // Offset Name: e.g. Current => 1
	fillData[1] = ChannelNo;
	fillData[3] = OffsetType; //  Offset Type: Offset=> 0; Scale => 1

	fillMsg(Type, 0x25, ID, dstAdd, message, fillData, 4, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	int combined = (fillData[3] << 24) | (fillData[2] << 16) | (fillData[1] << 8) | (fillData[0]);

	switch (fillData[6])
	{
	case 0: // Vin 
		if (fillData[7] == 0)
			*Result = (float)0.1 * static_cast<float>(combined);
		else
			*Result = (float)0.001 * static_cast<float>(combined);
		break;
	case 1: // Current
		if (fillData[7] == 0)
			*Result = (float)0.1 * static_cast<float>(combined);
		else
			*Result = (float)0.001 * static_cast<float>(combined);
		break;
	case 2: // Vout
		if (fillData[7] == 0)
			*Result = (float)0.1 * static_cast<float>(combined);
		else
			*Result = (float)0.001 * static_cast<float>(combined);
		break;
	case 3: // Temp
		if (fillData[7] == 0)
			*Result = (float)1.0 * static_cast<float>(combined);
		else
			*Result = (float)0.0001 * static_cast<float>(combined);
		break;
	case 4: // CPU Temp
		if (fillData[7] == 0)
			*Result = (float)0.01 * static_cast<float>(combined);
		else
			*Result = (float)0.0001 * static_cast<float>(combined);
		break;
	default:
		return UNKNOWN_ERROR;
		break;
	}
	return "success";
}

// Set offset Data function.
__declspec(dllexport) char* __cdecl SetOffsetData(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ParamNo,
	int ChannelNo,
	int OffsetType,
	float OffsetData,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	for (int i = 0; i < 8; i++)
		fillData[i] = 0;

	fillData[0] = ParamNo; // Offset Name
	fillData[1] = ChannelNo;
	fillData[2] = 83; // 'S' for Set
	fillData[3] = OffsetType; //  Offset Type

	float tempVal = OffsetData;

	switch (ParamNo) {
	case 4:
		if (OffsetType == 0)
			tempVal *= 100;
		else
			tempVal *= 10000;
		break;
	case 3:
		if (OffsetType == 0)
			tempVal *= 1;
		else
			tempVal *= 1000;
		break;
	case 0:
	case 1:
	case 2:
	default:
		if (OffsetType == 0)
			tempVal *= 10;
		else
			tempVal *= 1000;
		break;
	}

	FillsendBuff(fillData, tempVal);
	fillMsg(Type, 0x25, ID, dstAdd, message, fillData, 8, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	delete[] message;
	return "success";
}

// Get NetRider Information function.
__declspec(dllexport) char* __cdecl InternalDataInfo(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	int ParamNo,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();
	std::stringstream stream;

	for (int i = 0; i < 8; i++)
		fillData[i] = 0;

	fillData[0] = ParamNo; // Info ID

	fillMsg(Type, 0x2F, ID, dstAdd, message, fillData, 1, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	int combined = (fillData[3] << 24) | (fillData[2] << 16) | (fillData[1] << 8) | (fillData[0]);

	switch (ParamNo)
	{
	case 0: // NetRider Status 
		stream << std::setfill('0') << std::setw(2) << std::dec << (int)(fillData[0]);
		break;
	case 1: // NetRider Port
		stream << std::setfill('0') << std::setw(4) << std::dec << (int)((fillData[1] << 8) | (fillData[0]));
		break;
	case 2: // NetRider IP
	case 3: // NetRider Server IP;
		stream << std::setfill('0') << std::setw(2) << std::dec << (int)(fillData[0]);
		stream << '.';
		stream << std::setfill('0') << std::setw(2) << std::dec << (int)(fillData[1]);
		stream << '.';
		stream << std::setfill('0') << std::setw(2) << std::dec << (int)(fillData[2]);
		stream << '.';
		stream << std::setfill('0') << std::setw(2) << std::dec << (int)(fillData[3]);
		break;
	case 4: // NetRider P/N
	case 5: // NetRider SW Ver
		for (int i = 0; i < 8; i++)
			stream << (char)(fillData[i]);
		break;
	case 6: // NetRider Reset
		stream << std::setfill('0') << std::setw(2) << std::dec << (int)(fillData[0]);
		break;
	case 7: // Read Param. Checksum
		stream << "0x" << std::setfill('0') << std::setw(8) << std::hex << (unsigned int)((fillData[3] << 24) | (fillData[2] << 16) | (fillData[1] << 8) | (fillData[0]));
		break;
	default:
		return UNKNOWN_ERROR;
		break;
	}
	std::string result(stream.str());
	char *StringSystemOut = (char*)malloc(result.length() + 1);
	strcpy_s(StringSystemOut, result.length() + 1, result.c_str());
	return StringSystemOut;

	return "success";
}

// Clear log function.
__declspec(dllexport) char* __cdecl ClearLog(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 1; // ClearLog

	fillMsg(Type, 0x12, ID, dstAdd, message, fillData, 1, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	delete[] message;
	return "success";
}

// Clear time log function.
__declspec(dllexport) char* __cdecl ClearTimeLog(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 2; // ClearTimeLog

	fillMsg(Type, 0x12, ID, dstAdd, message, fillData, 1, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	delete[] message;
	return "success";
}

string GetTime(int time, int sec = 0) {
	std::stringstream stream;

	if (sec != 0)
		stream << time + floor(sec / 60) << ":" << sec - 60 * (floor(sec / 60)) << " [min:s], ";
	else
		stream << std::fixed << std::setprecision(0) << time + floor(sec / 60) << " [min], ";
	return stream.str();
}

string BlackOut(uint16_t _blackOut) {

	int bitArr[MAX_CHANNEL_NUMBER];
	string out = "";
	for (int i = 0; i < MAX_CHANNEL_NUMBER; i++)
	{
		bitArr[i] = _blackOut % 2;
		_blackOut = _blackOut / 2;
	}
	for (int i = 0; i < MAX_CHANNEL_NUMBER; i++)
	{
		if (bitArr[i] == 1) {
			out += "Channel " + std::to_string(i + 1) + " Black-Out Mode Enable";
		}
		else {
			out += "Channel " + std::to_string(i + 1) + " Black-Out Mode Disable";
		}
		if (i < 11)
			out += ", ";
	}
	out += ".";
	return out;
}
// Get log function.
__declspec(dllexport) char* __cdecl GetLog(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	char *EventLog,
	sNETWORK* PsNetwork = NULL) {

	std::string temp = "";// EventLog;
	int count = 0, MsgCnt = 0, resp = 0;
	bool exit = false;

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();
	char *LogOut;

	fillData[0] = 0; // GetLog 

	fillMsg(Type, 0x12, ID, dstAdd, message, fillData, 1, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	while (!exit) {
		txrx = receive(Type, message, fillData);
		if (txrx < 0) {

			//if (txrx == SYSTEM_ERROR_CODE) {
			//	delete[] message;
			//	static char errorReturn[14];
			//	temp += Error(Type, fillData, errorReturn);

			//	LogOut = new char[temp.length() + 1];
			//	strcpy_s(LogOut, temp.length() + 1, temp.c_str());
			//	return LogOut;
			//}
			if (txrx == SYSTEM_ERROR_CODE) {
				txrx = 1;
			}
			else
				count++;
		}
		if (txrx > 0) {
			std::stringstream stream;
			switch (fillData[0]) {
			case 240:
				switch (fillData[1]) {
				case 0: temp += "Command Error.";
					exit = true; break;
				case 1: temp += "Please enter to Configuration mode.";
					exit = true; break;
				case 3: temp += "Please close openned channel(s).";
					exit = true; break;
				case 60: temp += "Complete a erase operation.";
					exit = true; break;
				case 61: temp += "Clear Log. file.";
					exit = true; break;
				case 62: temp += "Reset System Time.";
					exit = true; break;
				case 63: temp += "Logger Error !!!";
					exit = true; break;
				case 64: temp += "Logger Read Data Format Error !!!";
					exit = true; break;
				case 65: temp.length() == 0 ? temp = "No Logger data found." : temp += "End of Logger File.";
					exit = true; break;
				default: temp += "??"; break;
				}
				temp += "\n";
				break;
			default:
				switch (fillData[0]) {
				case 0:
					temp += std::to_string(MsgCnt) + ", System Msg,  System Time: " + GetTime((fillData[7] << 8 | fillData[6]) << 16 | (fillData[5] << 8 | fillData[4]), fillData[2]) + "System Error : ";
					switch (fillData[1]) {
					case 0: temp += "ADC error"; break;
					case 1: temp += "I2C error"; break;
					case 2: temp += "CAN error"; break;
					case 3: temp += "CAN Init. error"; break;
					case 4: temp += "Parameters error"; break;
					case 5: temp += "WDG error"; break;
					case 6: temp += "Channel error"; break;
					case 7: temp += "Low Temperature error"; break;
					case 8: temp += "Low Voltage error"; break;
					case 9: temp += "VCC Protection error"; break;
					case 10: temp += "ADC Ref. error"; break;
					case 11: temp += "OP ref. error"; break;
					case 12: temp += "High Temperature error"; break;
					case 13: temp += "High Voltage error"; break;
					case 14: temp += "Channel error2"; break;
					case 15: temp += "Low Temperature 2 error"; break;
					case 16: temp += "Low Voltage 2 error"; break;
					case 17: temp += "High Temperature 2 error"; break;
					case 18: temp += "High Voltage 2 error"; break;
					default: temp += " Error No Defined"; break;
					}
					temp += "\n";
					break;
				case 1:
				case 2:
				case 3:
				case 4:
				case 5:
				case 6:
				case 7:
				case 8:
				case 9:
				case 10:
				case 11:
				case 12:
				case 13:
				case 14:
				case 15:
					temp += std::to_string(MsgCnt) + ", CH Event,  System Time: " + GetTime((fillData[7] << 8 | fillData[6]) << 16 | (fillData[5] << 8 | fillData[4]), fillData[2]) + "CH ";
					temp += std::to_string(fillData[0]) + " Event: ";
					switch (fillData[1]) {
					case 0: temp += "No Trip"; break;
					case 1: temp += "OverCurrent Trip"; break;
					case 2: temp += "Hard Short Trip (HW)"; break;
					case 3: temp += "Short circuit (SW)"; break;
					case 4: temp += "I^2T Trip"; break;
					case 5: temp += "Temperature Trip"; break;
					case 6: temp += "Voltage 1 Low Trip"; break;
					case 7: temp += "BIT Fail"; break;
					case 8: temp += "EMERGENCY"; break;
					case 9: temp += "Sleep Mode"; break;
					case 10: temp += "SW MAX Vol Trip"; break;
					case 11: temp += "MAX Total Current Trip"; break;
					case 12: temp += "Lost Communication"; break;
					case 13: temp += "Voltage 1 High Trip"; break;
					case 14: temp += "Voltage 2 Low Trip"; break;
					case 15: temp += "FF Reset Error"; break;
					case 16: temp += "Channel error"; break;
					default: temp += "Voltage 2 High Trip"; break;
					}
					temp += "\n";
					break;
				case 32:
					temp += std::to_string(MsgCnt) + " ,System Start, System Time: " + GetTime((fillData[7] << 8 | fillData[6]) << 16 | (fillData[5] << 8 | fillData[4]), fillData[2]) + "System Startup, ";
					stream << std::fixed << std::setprecision(2) << fillData[1] * 1;
					temp += stream.str();
					temp += "\n";
					break;
				case 37:
					temp += std::to_string(MsgCnt) + " ,Sync msg, System Time: " + GetTime((fillData[7] << 8 | fillData[6]) << 16 | (fillData[5] << 8 | fillData[4]), fillData[2]) + "Sync. msg, ";
					stream << std::fixed << std::setprecision(2) << fillData[1] * 1;
					temp += stream.str();
					temp += "\n";
					break;
				case 38:
					temp += std::to_string(MsgCnt) + " ,Black-Out Mode msg, System Time: " + GetTime((fillData[7] << 8 | fillData[6]) << 16 | (fillData[5] << 8 | fillData[4]), 0) + "Black-Out Mode msg, ";
					temp += BlackOut(fillData[2] << 8 | fillData[1]);
					temp += "\n";
					break;
				case 48:
				case 49:
				case 50:
				case 51:
				case 52:
				case 53:
				case 54:
				case 55:
				case 56:
				case 57:
				case 58:
				case 59:
				case 60:
					temp += std::to_string(MsgCnt) + " ,CH CMD, System Time: " + GetTime((fillData[7] << 8 | fillData[6]) << 16 | (fillData[5] << 8 | fillData[4]), fillData[2]) + "CH " + to_string(fillData[0] - 47) + " Command ";
					if (fillData[1] < 5)
						temp += "ON , ";
					else
						temp += "OFF , ";
					stream << std::fixed << std::setprecision(2) << fillData[1] * 1;
					temp += stream.str();
					temp += "\n";
					break;
				case 80:
				case 81:
				case 82:
				case 83:
				case 84:
				case 85:
				case 86:
				case 87:
				case 88:
				case 89:
				case 90:
				case 91:
				case 92:
				case 93:
				case 94:
				case 95:
					temp += std::to_string(MsgCnt) + " ,CH CNT2, System Time: " + GetTime((fillData[7] << 8 | fillData[6]) << 16 | (fillData[5] << 8 | fillData[4])) + "CH" + to_string(fillData[0] - 79) + "  OFF , Vin ";
					stream << std::fixed << std::setprecision(2) << fillData[1] * 0.2 << " [V], Temperature " << fillData[2] * 1 << " [C].";
					temp += stream.str();
					temp += "\n";
					break;
				case 112:
				case 113:
				case 114:
				case 115:
				case 116:
				case 117:
				case 118:
				case 119:
				case 120:
				case 121:
				case 122:
				case 123:
				case 124:
				case 125:
				case 126:
				case 127:
				case 128:
				case 129:
				case 130:
					temp += std::to_string(MsgCnt) + " ,CH CNT1, System Time: " + GetTime((fillData[7] << 8 | fillData[6]) << 16 | (fillData[5] << 8 | fillData[4])) + "CH" + to_string(fillData[0] - 111) + "  ON , Vout ";
					stream << std::fixed << std::setprecision(2) << fillData[2] * 0.2 << " [V], Current " << fillData[1] * 0.2 << " [A].";
					temp += stream.str();
					temp += "\n";
					break;
				default:
					temp += "\n";
					break;
				}
				break;
			}
			count = 0;
			resp = 0;
			message->pCanObj = new VCI_CAN_OBJ();

		}
		if (count >= 5) {
			exit = true;
			//return ErrorDefinition(txrx);
		}
		else
			MsgCnt++;
	}
	LogOut = new char[temp.length() + 1];
	strcpy_s(LogOut, temp.length() + 1, temp.c_str());

	delete[] message;
	return LogOut;
}

// Get Channel Combined Status 1 function.
__declspec(dllexport) char* __cdecl ChannelCombinedStatus1(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint16_t *BIT,
	uint8_t *InStatus,
	uint8_t *OutStatus,
	uint32_t *ChStatus,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillMsg(Type, 0x28, ID, dstAdd, message, fillData, 0, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			*BIT = -1;
			*InStatus = -1;
			*OutStatus = -1;
			*ChStatus = -1;
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	*BIT = (fillData[1] << 8) | (fillData[0]); // BIT L | BIT H
	*InStatus = fillData[2]; // DI
	*OutStatus = fillData[3]; // DO
	*ChStatus = (fillData[7] << 24) | (fillData[6] << 16) | (fillData[5] << 8) | (fillData[4]); // Channel Status
	return "success";
}

// Get Channel Combined Status 2 function.
__declspec(dllexport) char* __cdecl ChannelCombinedStatus2(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint16_t *BIT,
	uint8_t *InStatus,
	uint8_t *OutStatus,
	uint32_t *ChStatus,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillMsg(Type, 0x29, ID, dstAdd, message, fillData, 0, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			*BIT = -1;
			*InStatus = -1;
			*OutStatus = -1;
			*ChStatus = -1;
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	*BIT = (fillData[1] << 8) | (fillData[0]); // BIT L | BIT H
	*InStatus = fillData[2]; // DI
	*OutStatus = fillData[3]; // DO
	*ChStatus = (fillData[7] << 24) | (fillData[6] << 16) | (fillData[5] << 8) | (fillData[4]); // Channel Status
	return "success";
}

// Send Plot Command. return 1 if the received msg is correct, otherwise -1.
__declspec(dllexport) char* __cdecl PlotCommand(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint8_t *input,
	uint16_t time,
	uint8_t interval,
	int length,
	uint8_t *offsetResponse,
	uint16_t *scaleResponse,
	sNETWORK* PsNetwork = NULL) {

	uint8_t fillData[8] = {};

	MsgType* message = new MsgType();

	for (int i = 0; i < length; i++) {
		fillData[i] = (uint8_t)(input[i]);
	}

	fillData[length] = (uint8_t)(time & 0xFF);			// Time LSB
	fillData[length + 1] = (uint8_t)(time >> 8);		// Time MSB
	fillData[length + 2] = interval;					// Interval between each points

	fillMsg(Type, 0x2E, ID, dstAdd, message, fillData, length + 3, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0)
		return ErrorDefinition(txrx);

	offsetResponse[0] = (uint8_t)(fillData[0]);
	offsetResponse[1] = (uint8_t)(fillData[3]);
	offsetResponse[2] = (uint8_t)(fillData[6]);

	scaleResponse[0] = (uint16_t)(fillData[2] << 8 | fillData[1]);
	scaleResponse[1] = (uint16_t)(fillData[5] << 8 | fillData[4]);
	scaleResponse[2] = (uint16_t)(fillData[7]);

	return "success";
}

/**********************************************************/
/*  --  Functions with receipt messages periodically  --  */
/*  --                   BEGINNING                    --  */
/**********************************************************/

// Receive periodically (each 200/250 ms) Status Response. return 1 if the received msg is correct, otherwise -1.
__declspec(dllexport) char* __cdecl StatusResponse(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	float *response,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillMsg(Type, 0x2C, ID, dstAdd, message, fillData, 0, PsNetwork);

	int txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0)
		return ErrorDefinition(txrx);

	response[0] = static_cast<float>(fillData[0] >> 4 & 0xF);							// Ch Num
	response[1] = static_cast<float>(fillData[0] & 0x3);								// Trip ch
	response[2] = static_cast<float>(fillData[0] >> 2 & 0x3);							// Status ch
	response[3] = static_cast<float>(fillData[2] << 8 | fillData[1]);					// BIT
	response[4] = static_cast<float>(fillData[3]);										// I/O Array
	response[5] = static_cast<float>(fillData[4] * 0.8);								// Vin
	response[6] = static_cast<float>(fillData[5] * 0.8);								// Vout
	response[7] = static_cast<float>(fillData[6] * 0.8);								// Curr
	response[8] = static_cast<float>(fillData[7] - 80);									// Temp

	return "success";
}

// Receive periodically (each / ms) Plot Status. return 1 if the received msg is correct, otherwise -1.
__declspec(dllexport) char* __cdecl PlotStatus(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	double *response,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillMsg(Type, 0x2D, ID, dstAdd, message, fillData, 0, PsNetwork);

	/*if (Type == UDP) {
		(message->udp_connection.sock) = socket(AF_INET, SOCK_DGRAM, 0);
	}*/
	int txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0)
		return ErrorDefinition(txrx);

	response[0] = static_cast<double>(fillData[2] << 16 | fillData[1] << 8 | fillData[0]);	// Time ms	// Time sec
	response[1] = static_cast<double>(fillData[5] << 8 | fillData[4]);						// Plot Channel 1
	response[2] = static_cast<double>(fillData[7] << 8 | fillData[6]);						// Plot Channel 2
	response[3] = static_cast<double>(fillData[3]);											// Plot Channel 3

	return "success";
}

/**********************************************************/
/*  --  Functions with receipt messages periodically  --  */
/*  --                      END                       --  */
/**********************************************************/

// Get Status Response. return 1 if the received msg is correct, otherwise -1.
__declspec(dllexport) char* __cdecl GetStatusResponse(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	float *response,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();
	float responseOut[9] = { 0 };

	fillMsg(Type, 0x2C, ID, dstAdd, message, fillData, 0, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	response[0] = static_cast<float>(fillData[0] >> 6 & 0x2);							// Ch Num
	response[1] = static_cast<float>(fillData[0] & 0x2);								// Trip ch
	response[2] = static_cast<float>(fillData[0] >> 2 & 0x2);							// Status ch
	response[3] = static_cast<float>(fillData[2] << 8 | fillData[1]);					// BIT
	response[4] = static_cast<float>(fillData[3]);										// I/O Array
	response[5] = static_cast<float>(fillData[4] * 0.8);								// Vin
	response[6] = static_cast<float>(fillData[5] * 0.8);								// Vout
	response[7] = static_cast<float>(fillData[6] * 0.8);								// Curr
	response[8] = static_cast<float>(fillData[7] - 80);									// Temp																								
	return "success";
}

// Default Param function.
__declspec(dllexport) char* __cdecl SetDefaultParams(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 0x04; // DefaultParam
	fillData[1] = 0x8B;
	fillData[2] = 0x34;
	fillData[3] = 0xAA;
	fillData[4] = 0x08;

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 5, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	txrx = receive(Type, message, fillData);
	delete[] message;

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	return "success";
}

// Log Sync function.
__declspec(dllexport) char* __cdecl LogSync(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint8_t logSync,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 0x05;
	fillData[1] = 0xA4;
	fillData[2] = 0x21;
	fillData[3] = 0xAC;
	fillData[4] = 0x09;
	fillData[5] = logSync;

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 6, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}

	if (fillData[0] == 0x79 && fillData[1] == 0x09)
		return "success";

	else
		return UNKNOWN_ERROR;
}

// Set Black-Out Mode function.
__declspec(dllexport) char* __cdecl SetBlackOutMode(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint16_t blackOut,
	uint16_t* blackOutRes,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 0x06;
	fillData[1] = 0xC3;
	fillData[2] = 0x33;
	fillData[3] = 0xAC;
	fillData[4] = 0x0A;
	fillData[5] = blackOut & 0xFF;
	fillData[6] = blackOut >> 8;

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 7, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	if (fillData[0] == 0x79 && fillData[1] == 0x0A) {
		*blackOutRes = (fillData[3] << 8) | (fillData[2]);
		return "success";
	}
	else
		return UNKNOWN_ERROR;

}

// Get Black-Out Mode function.
__declspec(dllexport) char* __cdecl GetBlackOutMode(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint16_t* blackOutRes,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 0x06;
	fillData[1] = 0xB2;
	fillData[2] = 0x3C;
	fillData[3] = 0xAC;
	fillData[4] = 0x0A;

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 5, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	if (fillData[0] == 0x79 && fillData[1] == 0x0A) {
		*blackOutRes = (fillData[3] << 8) | (fillData[2]);
		return "success";
	}
	else
		return UNKNOWN_ERROR;
}

// Set Channel Priority Scenario function.
__declspec(dllexport) char* __cdecl SetChannelPriorityScenario(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint8_t scenarioNum,
	uint8_t* scenarioNumRes,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 0x07;
	fillData[1] = 0x21;
	fillData[2] = 0x26;
	fillData[3] = 0xAB;
	fillData[4] = 0x0B;
	fillData[5] = scenarioNum;

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 6, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	if (fillData[0] == 0x79 && fillData[1] == 0x0B) {
		*scenarioNumRes = (fillData[3] << 8) | (fillData[2]);
		return "success";
	}
	else
		return UNKNOWN_ERROR;
}

// Get Channel Priority Scenario function.
__declspec(dllexport) char* __cdecl GetChannelPriorityScenario(
	int Type,
	uint8_t ID,
	uint8_t dstAdd,
	uint8_t* scenarioNumRes,
	sNETWORK* PsNetwork = NULL) {

	unsigned char fillData[8] = {};
	MsgType* message = new MsgType();

	fillData[0] = 0x07;
	fillData[1] = 0x47;
	fillData[2] = 0x11;
	fillData[3] = 0xAB;
	fillData[4] = 0x0B;

	fillMsg(Type, 0x13, ID, dstAdd, message, fillData, 5, PsNetwork);

	int txrx = send(Type, message);
	if (txrx < 0)
		return ErrorDefinition(txrx);

	std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TX_RX));

	if (txrx < 0) {
		if (txrx == SYSTEM_ERROR_CODE) {
			static char errorReturn[14];
			Error(Type, fillData, errorReturn);
			return errorReturn;
		}
		return ErrorDefinition(txrx);
	}
	if (fillData[0] == 0x79 && fillData[1] == 0x0B) {
		*scenarioNumRes = (fillData[3] << 8) | (fillData[2]);
		return "success";
	}
	else
		return UNKNOWN_ERROR;
}

// This function is used to close the connection.	
// Return value = 1, which means that the operation is successful; \
			//Return 0 indicates that the operation failed; \
			//Return -1 indicates that the device does not exist.
__declspec(dllexport) char* __cdecl Close(int Type) {
	unsigned char errArr[1024] = { '\0' };
	switch (Type) {
	case CANALYST:
		if (VCI_CloseDevice((DWORD)DeviceType, (DWORD)DeviceInd) == 0)
			return "success";
		else
			return "failed closing CAN";
	case CANKVASER:
		canBusOff(ChannelHandler);
		errArr[0] = canClose(ChannelHandler);
		if (errArr[0] == 0) {
			return "success";
		}
		return Error(Type, errArr);
	}
	return TYPE_NOT_DETECTED;
}

// This function is used to close the connection.	
// Return value = 1, which means that the operation is successful; \
			//Return 0 indicates that the operation failed; \
			//Return -1 indicates that the device does not exist.
__declspec(dllexport) int __cdecl CloseUDP(sNETWORK* PsNetwork) {

	return CloseUDPCom(PsNetwork->_udp.sock);
}
// This function is used to reset the CAN controller.	
// Return value = 1, which means that the operation is successful; \
			//Return 0 indicates that the operation failed; \
			//Return -1 indicates that the device does not exist.
__declspec(dllexport) int __cdecl Reset(int Type) {

	switch (Type) {
	case CANALYST:
	{
		int Reset = VCI_ResetCAN((DWORD)DeviceType, (DWORD)DeviceInd, (DWORD)CanIndex);
		return Reset;
	}
	default:
		return 1;
	}
}
