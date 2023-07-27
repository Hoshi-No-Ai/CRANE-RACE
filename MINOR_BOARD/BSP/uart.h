#ifndef __UART_H__
#define __UART_H__

#include "stm32f4xx.h"

#define USART1_RX_STREAM DMA2_Stream5
#define USART1_TX_STREAM DMA2_Stream7
#define USART1_RXDMA_LEN 38
#define USART1_RXMB_LEN 38
#define USART1_TXDMA_LEN 48

#define USART6_RX_STREAM        DMA2_Stream1
#define USART6_TX_STREAM        DMA2_Stream6
#define USART6_RXDMA_LEN           60
#define USART6_RXMB_LEN            60
#define USART6_TXDMA_LEN           60



#define USART2_RXDMA_LEN 100
#define USART2_RXMB_LEN 100
#define USART2_TXDMA_LEN 100

#define USART2_RX_STREAM DMA1_Stream5
#define USART2_TX_STREAM DMA1_Stream6
#define USART2_TX_CHANNEL DMA_Channel_4
#define USART2_RX_CHANNEL DMA_Channel_4




#define USART3_RXDMA_LEN 38
#define USART3_RXMB_LEN 38
#define USART3_TXDMA_LEN 48

#define USART3_RX_STREAM DMA1_Stream1
#define USART3_TX_STREAM DMA1_Stream3

extern uint8_t UA3RxMailbox[USART3_RXMB_LEN];
extern uint8_t UA3RxDMAbuf[USART3_RXDMA_LEN];

extern unsigned char UA1RxMailbox[USART1_RXMB_LEN];
extern unsigned char UA6RxMailbox[USART6_RXMB_LEN];








typedef struct
{
	USART_TypeDef* USARTx;
	DMA_Stream_TypeDef* DMAy_Streamx;
	unsigned char* pMailbox;
    __IO unsigned char* pDMAbuf;
	unsigned short MbLen;
	unsigned short DMALen;
	unsigned short rxConter;
	unsigned short rxBufferPtr;
  unsigned short rxSize;
}USART_RX_TypeDef;


extern void USART3_Configuration(void);
void USART2_Configuration(void);
extern void USART1_Configuration(void);
extern void USART6_Configuration(void);
extern unsigned short USART_Receive(USART_RX_TypeDef* USARTx);

#endif
