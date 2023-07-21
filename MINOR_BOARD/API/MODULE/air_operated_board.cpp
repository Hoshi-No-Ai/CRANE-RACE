#include "air_operated_board.h"

//���������ȫ�ֱ���
uint32_t g_uiAirValve=0;	   //�����й�ȫ�ֱ���
uint32_t g_uiAirValvePre=0;  //������һ��״̬
uint16_t g_usSwitch=0;     //�����й�ȫ�ֱ���
uint16_t g_usSwitchPre = 0;//������һʱ�̿���ֵ

uint16_t g_usSwitch1=0;     //�����й�ȫ�ֱ���1
uint16_t g_usSwitchPre1 = 0;//������һʱ�̿���ֵ1

/********************************************************************************************************
�������йغ���
*********************************************************************************************************/
/**************************************************************
������:SendAirMsgByCan1()
��������: ͨ��CAN2�������׵ı���
����: pAir Ҫ���Ƶ�����ID
�������
��ע: С������0-15����������16-23����Ӧλ��1Ϊ������
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
������:SendAirMsgByCan2_Plus()
��������: ͨ��CAN2�������׵ı���
����: pAir Ҫ���Ƶ�����ID
�������
��ע: С������0-15����������16-23����Ӧλ��1Ϊ������
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
������:Send_LED_Mode(uint8_t mode)
��������: ȫ��LED����
����: mode LED��˸ģʽ
�������
��ע: mode :0-3    3:red  2:green 1:blue  0:Ĭ�Ϻ�����
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
������:UpdateSwitchValue()
��������: ���µ�ֵ
����: pAir Ҫ���Ƶ�����ID
�������
��ע: С������0-15����������16-23����Ӧλ��1Ϊ������
***************************************************************/
void UpdateSwitchValue(uint16_t*pusSwitch,CanRxMsg * pRxMsg)
{
	*pusSwitch =*((uint16_t*) &(pRxMsg->Data[0]));
}

/**************************************************************
������:SendServoMsgByCan2()
��������: ͨ��CAN2���Ͷ���ı���
����: chan Ҫ���ƵĶ��ID
	  value ����źŵ���ֵ
�������
��ע: 
***************************************************************/
void SendServoMsgByCan2(uint8_t chan,uint8_t value)
{
	static CanTxMsg TxMessage = {CAN_SERVO_ID, 0x00, CAN_ID_STD, CAN_RTR_DATA, 3, 0,0,0,0,0,0,0,0};
	static uint8_t s_aucLastPwm[4] = {75,75,75,75};
	if(s_aucLastPwm[chan] != value)//ֵ�ı�ʱ�ŷ���
	{
		s_aucLastPwm[chan] = value;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = chan;
		TxMessage.Data[2] = value;
		CAN_Transmit(CAN1, &TxMessage);
	}
}
/**************************************************************
������:SendServoMsgByCan2_Plus()
��������: ͨ��CAN2���Ͷ���ı���
����: chan Ҫ���ƵĶ��ID
	  value ����źŵ���ֵ
�������
��ע: 
***************************************************************/
void SendServoMsgByCan2_Plus(uint8_t chan,uint8_t value)
{
	static CanTxMsg TxMessage = {CAN_SERVO_ID_Plus, 0x00, CAN_ID_STD, CAN_RTR_DATA, 3, 0,0,0,0,0,0,0,0};
	static uint8_t s_aucLastPwm[4] = {150,150,150,150};
	if(s_aucLastPwm[chan] != value)//ֵ�ı�ʱ�ŷ���
	{
		s_aucLastPwm[chan] = value;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = chan;
		TxMessage.Data[2] = value;
		CAN_Transmit(CAN2, &TxMessage);
	}
}
