#ifndef CAN_H
#define CAN_H

#define CAN_RX_BUF_MAX 16/*必须是2的幂*/			 //缓冲区队列长度
#define CAN_TX_BUF_MAX 16/*必须是2的幂*/
#define CAN_RX_BUF_MARK	(CAN_RX_BUF_MAX-1)
#define CAN_TX_BUF_MARK	(CAN_TX_BUF_MAX-1)

#include "STM32Lib\\stm32f10x.h"

//extern static CanRxMsg CAN_RX_Buf[CAN_RX_BUF_MAX];	//发送队列
//extern static CanTxMsg CAN_TX_Buf[CAN_TX_BUF_MAX];	//接收队列
extern volatile u32 CAN_Rx_Head ;					//队列头-接收的时候移动
extern volatile u32 CAN_Rx_Tail ;					//队列尾-读取的时候移动
extern volatile u32 CAN_Tx_Head ;					//队列头-接收的时候移动
extern volatile u32 CAN_Tx_Tail ;					//队列尾-读取的时候移动

/*----------------初始化------------------------------*/
void CAN_Configuration(void);	//CAN初始化设置。
void CAN_Interrupt(void);		//CAN中断设置，过滤器匹配
								//接受到数据后，进入USB_LP_CAN1_RX0_IRQHandler

/*----------------接收相关------------------------------*/
void CANPutDatatoRxBuf(CanRxMsg *pRxMessage);	//把数据放进队列中
bool CANIsDataInRxBuf( void );					//获知缓冲中是否有数据
u32 CANGetRxBufLen(void);						//获取缓冲中有效数据的长度
CanRxMsg *CANGetRxBufDat( void );				//从队列中获取数据

/*----------------发送相关------------------------------*/
bool CANPutDatatoTxBuf(CanTxMsg *pTxMessage);	//把数据放进发送队列中
bool CANIsDataInTxBuf( void );					//获知缓冲中是否有数据
u32 CANGetTxBufLen(void);						//获取缓冲中有效数据的长度
CanTxMsg *CANGetTxBufDat( void );				//从队列中获取数据
void CANBeginSend(void);		//启动发送
void CANStopSend(void);			//结束发送

void SendUCHARByCan(u8 id , u8 dat);			 //发送一个1字节的数据
void SendUSHORT16ByCan(u8 id , u16 dat);		 //发送一个2字节的数据
void SendSINT32ByCan(u8 id , s32 dat);			 //发送一个4字节的数据
void SendArrayUCHAR8ByCan(u8 id , u8 dat[], u8 dlc);	 //发送一个u8类型的数组
void SendArrayUSHORT16ByCan(u8 id , u16 dat[], u8 dlc);	 //发送一个u16类型的数组

void SendDT35(s16 id , u16 k1, u16 k2, u16 k3, u16 k4);


#endif
