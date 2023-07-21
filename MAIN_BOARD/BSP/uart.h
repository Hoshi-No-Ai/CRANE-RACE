#ifndef __UART_H__
#define __UART_H__

#include "stm32f4xx.h"

#define USART_REC_LEN 200  //定义最大接收字节数 200

//#define USART1_RXDMA_LEN 22
//#define USART1_RXMB_LEN 22
#define USART1_RXDMA_LEN 8
#define USART1_RXMB_LEN 8
#define USART1_TXDMA_LEN 48

#define USART1_RX_STREAM DMA2_Stream5
#define USART1_TX_STREAM DMA2_Stream7

#define USART3_RXDMA_LEN 40
#define USART3_RXMB_LEN 40
#define USART3_TXDMA_LEN 48

#define USART3_RX_STREAM DMA1_Stream1
#define USART3_TX_STREAM DMA1_Stream3

/*串口6通信缓冲长度*/
#define USART6_RXDMA_LEN 40
#define USART6_RXMB_LEN 40
#define USART6_TXDMA_LEN 48

#define USART6_RX_STREAM DMA2_Stream1
#define USART6_TX_STREAM DMA2_Stream6

#define USART2_RXDMA_LEN 100
#define USART2_RXMB_LEN 100
#define USART2_TXDMA_LEN 100

#define USART2_RX_STREAM DMA1_Stream5
#define USART2_TX_STREAM DMA1_Stream6
#define USART2_TX_CHANNEL DMA_Channel_4
#define USART2_RX_CHANNEL DMA_Channel_4

extern uint8_t UA1RxMailbox[USART1_RXMB_LEN];
extern uint8_t UA1RxDMAbuf[USART1_RXDMA_LEN];

extern uint8_t UA3RxMailbox[USART3_RXMB_LEN];
extern uint8_t UA3RxDMAbuf[USART3_RXDMA_LEN];

extern uint8_t UA2RxDMAbuf[USART2_RXDMA_LEN];
extern uint8_t UA2RxMailbox[USART2_RXMB_LEN];

extern uint8_t UA6RxDMAbuf[USART6_RXDMA_LEN];
extern uint8_t UA6RxMailbox[USART6_RXMB_LEN];

struct USART_RX_TypeDef
{
	USART_TypeDef *USARTx;
	DMA_Stream_TypeDef *DMAy_Streamx;
	unsigned char *pMailbox;
	__IO unsigned char *pDMAbuf;
	unsigned short MbLen;
	unsigned short DMALen;
	unsigned short rxConter;
	unsigned short rxBufferPtr;
	unsigned short rxSize;
};

// struct USART_TX_TypeDef
//{
//	USART_TypeDef* USARTx;
//  	DMA_Stream_TypeDef* DMAy_Streamx;
//	unsigned char* pMailbox;
//	__IO unsigned char* pDMAbuf;
//	unsigned short MbLen;
//	unsigned short DMALen;
// };

void USART1_Configuration(void);
// void UART4_Configuration(void);
void USART3_Configuration(void);
void USART6_Configuration(void);
void USART2_Configuration(void);

#endif
