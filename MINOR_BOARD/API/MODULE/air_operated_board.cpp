#include "air_operated_board.h"

//气动板相关全局变量
uint32_t g_uiAirValve=0;	   //气缸有关全局变量
uint32_t g_uiAirValvePre=0;  //气缸上一次状态
uint16_t g_usSwitch=0;     //开关有关全局变量
uint16_t g_usSwitchPre = 0;//保存上一时刻开关值

uint16_t g_usSwitch1=0;     //开关有关全局变量1
uint16_t g_usSwitchPre1 = 0;//保存上一时刻开关值1

/********************************************************************************************************
气动板有关函数
*********************************************************************************************************/
/**************************************************************
函数名:SendAirMsgByCan1()
函数功能: 通过CAN2发送气缸的变量
输入: pAir 要控制的气缸ID
输出：无
备注: 小气动板0-15，大气动板16-23，对应位至1为打开气缸
***************************************************************/
void SendAirMsgByCan2(uint32_t* pAir)
{
	static CanTxMsg TxMessage = {CAN_AIR_ID, 0x00, CAN_ID_STD, CAN_RTR_DATA, 4, 0,0,0,0,0,0,0,0};
	static uint32_t s_ucLastAir = 0;	
	s_ucLastAir = *pAir;
	*((uint32_t*)TxMessage.Data) = s_ucLastAir;
	CAN_Transmit(CAN2, &TxMessage);
}

void SendAirMsgByCan1(uint32_t* pAir)
{
	static CanTxMsg TxMessage = {CAN_AIR_ID, 0x00, CAN_ID_STD, CAN_RTR_DATA, 4, 0,0,0,0,0,0,0,0};
	static uint32_t s_ucLastAir = 0;	
	s_ucLastAir = *pAir;
	*((uint32_t*)TxMessage.Data) = s_ucLastAir;
	CAN_Transmit(CAN1, &TxMessage);
}

/**************************************************************
函数名:SendAirMsgByCan2_Plus()
函数功能: 通过CAN2发送气缸的变量
输入: pAir 要控制的气缸ID
输出：无
备注: 小气动板0-15，大气动板16-23，对应位至1为打开气缸
***************************************************************/
void SendAirMsgByCan2_Plus(uint32_t* pAir)
{
	static CanTxMsg TxMessage = {CAN_AIR_ID_Plus, 0x00, CAN_ID_STD, CAN_RTR_DATA, 4, 0,0,0,0,0,0,0,0};
	static uint32_t s_ucLastAir = 0;	
	s_ucLastAir = *pAir;
	*((uint32_t*)TxMessage.Data) = s_ucLastAir;
	CAN_Transmit(CAN2, &TxMessage);
}

/**************************************************************
函数名:Send_LED_Mode(uint8_t mode)
函数功能: 全彩LED控制
输入: mode LED闪烁模式
输出：无
备注: mode :0-3    3:red  2:green 1:blue  0:默认呼吸灯
***************************************************************/
void Send_LED_Mode(uint8_t mode)
{
	static CanTxMsg TxMessage = {0x50, 0x00, CAN_ID_STD, CAN_RTR_DATA, 1, 0,0,0,0,0,0,0,0};
	static uint8_t s_ucLastAir = 0;	
	s_ucLastAir = mode;
	TxMessage.Data[0] = s_ucLastAir;
	CAN_Transmit(CAN2, &TxMessage);
}

/**************************************************************
函数名:UpdateSwitchValue()
函数功能: 更新的值
输入: pAir 要控制的气缸ID
输出：无
备注: 小气动板0-15，大气动板16-23，对应位至1为打开气缸
***************************************************************/
void UpdateSwitchValue(uint16_t*pusSwitch,CanRxMsg * pRxMsg)
{
	*pusSwitch =*((uint16_t*) &(pRxMsg->Data[0]));
}

/**************************************************************
函数名:SendServoMsgByCan2()
函数功能: 通过CAN2发送舵机的变量
输入: chan 要控制的舵机ID
	  value 舵机信号的数值
输出：无
备注: 
***************************************************************/
void SendServoMsgByCan2(uint8_t chan,uint8_t value)
{
	static CanTxMsg TxMessage = {CAN_SERVO_ID, 0x00, CAN_ID_STD, CAN_RTR_DATA, 3, 0,0,0,0,0,0,0,0};
	static uint8_t s_aucLastPwm[4] = {75,75,75,75};
	if(s_aucLastPwm[chan] != value)//值改变时才发送
	{
		s_aucLastPwm[chan] = value;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = chan;
		TxMessage.Data[2] = value;
		CAN_Transmit(CAN1, &TxMessage);
	}
}
/**************************************************************
函数名:SendServoMsgByCan2_Plus()
函数功能: 通过CAN2发送舵机的变量
输入: chan 要控制的舵机ID
	  value 舵机信号的数值
输出：无
备注: 
***************************************************************/
void SendServoMsgByCan2_Plus(uint8_t chan,uint8_t value)
{
	static CanTxMsg TxMessage = {CAN_SERVO_ID_Plus, 0x00, CAN_ID_STD, CAN_RTR_DATA, 3, 0,0,0,0,0,0,0,0};
	static uint8_t s_aucLastPwm[4] = {150,150,150,150};
	if(s_aucLastPwm[chan] != value)//值改变时才发送
	{
		s_aucLastPwm[chan] = value;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = chan;
		TxMessage.Data[2] = value;
		CAN_Transmit(CAN2, &TxMessage);
	}
}
