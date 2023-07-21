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
}UN_READ;

typedef union
{
	SINT32 siSave[512];
	FP32 fpSave[512];
}UN_SAVE;
extern u16 g_ucSavePit;//FLASH�洢λ
extern u16 g_ucReadPit;//FLASH��ȡλ
extern UN_SAVE unSave;//�洢�ṹ��
extern  UN_READ	unRead;//��ȡ�ṹ��

/*��������*/
#define Receive(Data1, Data2, Data3)  	Data1 = g_fpUartData[0];\
										Data2 = g_fpUartData[1];\
										Data3 = g_fpUartData[2]
/*��������*/
#define Send(Data1, Data2, Data3)       g_fpUartSendData[0] = Data1;\
										g_fpUartSendData[1] = Data2;\
										g_fpUartSendData[2] = Data3

/*���ݶ�ȡ����*/
#define ReadInt(Data) unRead.siRead = GetOneInt32(g_ucReadPit); Data = unRead.siRead; g_ucReadPit++
#define ReadFp(Data) unRead.siRead = GetOneInt32(g_ucReadPit); Data = unRead.fpRead; g_ucReadPit++

/*���ݴ洢����*/
#define SaveInt(Data) unSave.siSave[g_ucSavePit] = (SINT32)Data; g_ucSavePit++
#define SaveFp(Data) unSave.fpSave[g_ucSavePit] = (FP32)Data; g_ucSavePit++

#endif

