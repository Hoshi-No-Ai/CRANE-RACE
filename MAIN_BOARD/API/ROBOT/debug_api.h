#ifndef _DEBUG_API_H
#define _DEBUG_API_H

#include "main.h"
#include "debug_api.h"
#include "basic_type.h"
#include "stm32f4xx.h"
#include "gimbal_control_types.h"

typedef union
{
	SINT32 siRead;
	FP32 fpRead;
} UN_READ;
typedef union
{
	SINT32 siSave[512];
	FP32 fpSave[512];
} UN_SAVE;
extern u16 g_ucSavePit; // FLASH存储位
extern u16 g_ucReadPit; // FLASH读取位
extern UN_SAVE unSave;	// 存储结构体
extern UN_READ unRead;	// 读取结构体

/*接收数据*/
#define Receive(Data1, Data2, Data3) \
	Data1 = g_fpUartData[0];         \
	Data2 = g_fpUartData[1];         \
	Data3 = g_fpUartData[2]
/*发送数据*/
#define Send(Data1, Data2, Data3) \
	g_fpUartSendData[0] = Data1;  \
	g_fpUartSendData[1] = Data2;  \
	g_fpUartSendData[2] = Data3

/*数据读取命令*/
#define ReadInt(Data)                         \
	unRead.siRead = GetOneInt32(g_ucReadPit); \
	Data = unRead.siRead;                     \
	g_ucReadPit++
#define ReadFp(Data)                          \
	unRead.siRead = GetOneInt32(g_ucReadPit); \
	Data = unRead.fpRead;                     \
	g_ucReadPit++

/*数据存储命令*/
#define SaveInt(Data)                          \
	unSave.siSave[g_ucSavePit] = (SINT32)Data; \
	g_ucSavePit++
#define SaveFp(Data)                         \
	unSave.fpSave[g_ucSavePit] = (FP32)Data; \
	g_ucSavePit++

#endif
