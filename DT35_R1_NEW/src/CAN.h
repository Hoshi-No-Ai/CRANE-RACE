#ifndef CAN_H
#define CAN_H

#define CAN_RX_BUF_MAX 16/*������2����*/			 //���������г���
#define CAN_TX_BUF_MAX 16/*������2����*/
#define CAN_RX_BUF_MARK	(CAN_RX_BUF_MAX-1)
#define CAN_TX_BUF_MARK	(CAN_TX_BUF_MAX-1)

#include "STM32Lib\\stm32f10x.h"

//extern static CanRxMsg CAN_RX_Buf[CAN_RX_BUF_MAX];	//���Ͷ���
//extern static CanTxMsg CAN_TX_Buf[CAN_TX_BUF_MAX];	//���ն���
extern volatile u32 CAN_Rx_Head ;					//����ͷ-���յ�ʱ���ƶ�
extern volatile u32 CAN_Rx_Tail ;					//����β-��ȡ��ʱ���ƶ�
extern volatile u32 CAN_Tx_Head ;					//����ͷ-���յ�ʱ���ƶ�
extern volatile u32 CAN_Tx_Tail ;					//����β-��ȡ��ʱ���ƶ�

/*----------------��ʼ��------------------------------*/
void CAN_Configuration(void);	//CAN��ʼ�����á�
void CAN_Interrupt(void);		//CAN�ж����ã�������ƥ��
								//���ܵ����ݺ󣬽���USB_LP_CAN1_RX0_IRQHandler

/*----------------�������------------------------------*/
void CANPutDatatoRxBuf(CanRxMsg *pRxMessage);	//�����ݷŽ�������
bool CANIsDataInRxBuf( void );					//��֪�������Ƿ�������
u32 CANGetRxBufLen(void);						//��ȡ��������Ч���ݵĳ���
CanRxMsg *CANGetRxBufDat( void );				//�Ӷ����л�ȡ����

/*----------------�������------------------------------*/
bool CANPutDatatoTxBuf(CanTxMsg *pTxMessage);	//�����ݷŽ����Ͷ�����
bool CANIsDataInTxBuf( void );					//��֪�������Ƿ�������
u32 CANGetTxBufLen(void);						//��ȡ��������Ч���ݵĳ���
CanTxMsg *CANGetTxBufDat( void );				//�Ӷ����л�ȡ����
void CANBeginSend(void);		//��������
void CANStopSend(void);			//��������

void SendUCHARByCan(u8 id , u8 dat);			 //����һ��1�ֽڵ�����
void SendUSHORT16ByCan(u8 id , u16 dat);		 //����һ��2�ֽڵ�����
void SendSINT32ByCan(u8 id , s32 dat);			 //����һ��4�ֽڵ�����
void SendArrayUCHAR8ByCan(u8 id , u8 dat[], u8 dlc);	 //����һ��u8���͵�����
void SendArrayUSHORT16ByCan(u8 id , u16 dat[], u8 dlc);	 //����һ��u16���͵�����

void SendDT35(s16 id , u16 k1, u16 k2, u16 k3, u16 k4);


#endif
