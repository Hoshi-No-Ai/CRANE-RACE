#ifndef __UART_PROTOCOL_H__
#define __UART_PROTOCOL_H__

#include "stm32f4xx.h"
#include "basic_type.h"
#include "uart.h"
#include "string.h"


typedef enum
{
	RX_FREE,
	RX_START_1,
	RX_START_2,
	RX_START_3,
	RX_ID,
	RX_DATAS,
	RX_TAIL_1,
	RX_TAIL_2,
	RX_WORK_STATE,
	RX_DATA_LEN,
}rx_protocol_e;

#define UART1_RX_DATA_LEN 	38
#define UART6_RX_DATA_LEN 	20
#define UART1_TX_DATA_LEN   10
#define UART6_TX_DATA_LEN   16

union Float2uchar
{ 
  float fl[7];
	unsigned char ch[28];
};

union s16_2char
{ 
	s16 s[4];
	unsigned char ch[8];
};



typedef struct
{
	uint8_t usart1_tx_start1;  //帧头1
	uint8_t usart1_tx_start2;  //帧头2
	uint8_t usart1_tx_datanum;
	
	s8 num[UART1_TX_DATA_LEN];
	
	uint8_t usart1_tx_tail1;   //帧尾1
	uint8_t usart1_tx_tail2;   //帧尾2
	
}uart1_tx_protocol_t;//发送信息报文结构体

typedef struct
{
	uint8_t usart1_rx_start1;  //帧头1
	uint8_t usart1_rx_start2;  //帧头2
	uint8_t date_len_float;
	uint8_t usart1_rx_datanum;
	
	s8 num[UART1_RX_DATA_LEN];
	
	uint8_t usart1_rx_tail1;   //帧尾1
	uint8_t usart1_rx_tail2;   //帧尾2
	
}uart1_rx_protocol_r;//接收信息报文结构体

/*串口6用作和下板通信*/
 typedef struct
 {
	uint8_t usart6_tx_start1;  //帧头1
	uint8_t usart6_tx_start2;  //帧头2
	uint8_t usart6_tx_datanum;
	
	s8 num[UART6_TX_DATA_LEN];
	
	uint8_t usart6_tx_tail1;   //帧尾1
	uint8_t usart6_tx_tail2;   //帧尾2
	
 }uart6_tx_protocol_t;//发送信息报文结构体


typedef struct
{
	uint8_t usart6_rx_start1;  //帧头1
	uint8_t usart6_rx_start2;  //帧头2
	uint8_t usart6_rx_datanum;
	
	s8 num[UART6_RX_DATA_LEN];
	
	uint8_t usart6_rx_tail1;   //帧尾1
	uint8_t usart6_rx_tail2;   //帧尾2
}uart6_rx_protocol_r;//接收信息报文结构体

extern u8 pLen_write;
extern s8 bBuf[20];

extern void USART1_DMA_Tx(void);
extern void Comm1Rx_IRQ(void);
void Comm6Rx_IRQ(void);
void Comm3Rx_IRQ(void);



extern uart1_tx_protocol_t eft1;
extern uart1_rx_protocol_r efr1;

extern uart6_tx_protocol_t eft6;
extern uart6_rx_protocol_r efr6;

extern void USART6_DMA_Tx(void);
#endif

