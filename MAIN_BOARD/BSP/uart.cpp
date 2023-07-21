#include "uart.h"

//串口1底层配置
unsigned char UA1RxDMAbuf[USART1_RXDMA_LEN] = {0};
unsigned char UA1RxMailbox[USART1_RXMB_LEN] = {0};
USART_RX_TypeDef USART1_Rcr = {USART1, USART1_RX_STREAM, UA1RxMailbox,
							   UA1RxDMAbuf, USART1_RXMB_LEN, USART1_RXDMA_LEN, 0, 0, 0};

void USART1_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Enable UART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART configuration */
	USART_InitStructure.USART_BaudRate = 38400; // esp8266为921600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); //使能串口空闲中断
												   //	//串口DMA接收、发送使能
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
//	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART1, ENABLE);

	/*DMA RX configuration */
	DMA_DeInit(DMA2_Stream5);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;						  //外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR); //内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UA1RxDMAbuf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //设置数据传输方向

	DMA_InitStructure.DMA_BufferSize = USART1_RXDMA_LEN;					//设置DMA一次传输数据量的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//设置外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//设置内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //设置外设的数据长度为字节（8bits）
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//设置内存的数据长度为字节（8bits）
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							//设置DMA模式为循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;					// DMA_Priority_VeryHigh;//DMA_Priority_Medium;//设置DMA通道的优先级为最高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream5, ENABLE);
}

uint8_t UA3RxDMAbuf[USART3_RXDMA_LEN] = {0};
uint8_t UA3RxMailbox[USART3_RXMB_LEN] = {0};
USART_RX_TypeDef USART3_Rcr = {USART3, USART3_RX_STREAM, UA3RxMailbox,
							   UA3RxDMAbuf, USART3_RXMB_LEN, USART3_RXDMA_LEN, 0, 0, 0};

void USART3_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* USART configuration */
	USART_InitStructure.USART_BaudRate = 921600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE); //使能串口空闲中断
	//串口DMA接收、发送使能
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART3, ENABLE);

	/*DMA RX configuration */
	DMA_DeInit(DMA1_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;						  //外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR); //内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UA3RxDMAbuf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //设置数据传输方向

	DMA_InitStructure.DMA_BufferSize = USART3_RXDMA_LEN;					//设置DMA一次传输数据量的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//设置外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//设置内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //设置外设的数据长度为字节（8bits）
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//设置内存的数据长度为字节（8bits）
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							//设置DMA模式为循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;						// DMA_Priority_VeryHigh;//DMA_Priority_Medium;//设置DMA通道的优先级为最高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream1, ENABLE);

	DMA_DeInit(DMA1_Stream3);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;							//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);	//内存地址
																			// DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&eft;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					//设置数据传输方向
	DMA_InitStructure.DMA_BufferSize = USART3_TXDMA_LEN;					//设置DMA一次传输数据量的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//设置外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//设置内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //设置外设的数据长度为字节（8bits）
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//设置内存的数据长度为字节（8bits）
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							// DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;					// DMA_Priority_VeryHigh;//DMA_Priority_Medium;//设置DMA通道的优先级为最高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);

	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Stream3, ENABLE);
}

/*************************************************************************
函 数 名：USART6_Configuration
函数功能：串口6底层配置
备    注：
*************************************************************************/

uint8_t UA6RxDMAbuf[USART6_RXDMA_LEN] = {0};
uint8_t UA6RxMailbox[USART6_RXMB_LEN] = {0};
USART_RX_TypeDef USART6_Rcr = {USART6, USART6_RX_STREAM, UA6RxMailbox,
							   UA6RxDMAbuf, USART6_RXMB_LEN, USART6_RXDMA_LEN, 0, 0, 0};

uint8_t UA6TxDMAbuf[USART6_TXDMA_LEN] = {0};

void USART6_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* Enable UART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	/*使能DMA时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	/* USART configuration */
	USART_InitStructure.USART_BaudRate = 115200; // esp8266为921600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART6, &USART_InitStructure);

	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	// USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);//使能串口空闲中断
	//串口DMA接收、发送使能

	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART6, ENABLE);

	//	/*DMA RX configuration */
	//	DMA_DeInit(USART6_RX_STREAM);
	//  DMA_InitStructure.DMA_Channel= DMA_Channel_5;//外设地址
	//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);//内存地址
	//  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UA6RxDMAbuf;
	//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//设置数据传输方向

	//  DMA_InitStructure.DMA_BufferSize = USART6_RXDMA_LEN;//设置DMA一次传输数据量的大小
	//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//设置外设地址不变
	//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//设置内存地址递增
	//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//设置外设的数据长度为字节（8bits）
	//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//设置内存的数据长度为字节（8bits）
	//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal//设置DMA模式为循环模式
	//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority_VeryHigh;//DMA_Priority_Medium;//设置DMA通道的优先级为最高优先级
	//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	//	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//  DMA_Init(USART6_RX_STREAM,&DMA_InitStructure);
	//  DMA_Cmd(USART6_RX_STREAM,ENABLE);

	DMA_DeInit(USART6_TX_STREAM);
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;							//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);	//内存地址
																			//    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&eft;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					//设置数据传输方向
	DMA_InitStructure.DMA_BufferSize = 0;									//设置DMA一次传输数据量的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//设置外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//设置内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //设置外设的数据长度为字节（8bits）
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//设置内存的数据长度为字节（8bits）
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							// DMA_Mode_Normal;//设置DMA模式为循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;					// DMA_Priority_VeryHigh;//DMA_Priority_Medium;//设置DMA通道的优先级为最高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART6_TX_STREAM, &DMA_InitStructure);

	DMA_ITConfig(USART6_TX_STREAM, DMA_IT_TC, ENABLE);
	DMA_Cmd(USART6_TX_STREAM, DISABLE);
}

uint8_t UA2RxDMAbuf[USART2_RXDMA_LEN] = {0};
uint8_t UA2RxMailbox[USART2_RXMB_LEN] = {0};
USART_RX_TypeDef USART2_Rcr = {USART2, USART2_RX_STREAM, UA2RxMailbox, UA2RxDMAbuf, USART2_RXMB_LEN, USART2_RXDMA_LEN, 0, 0, 0};

void USART2_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	/* Configure USART Tx and Rx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // GPIO_OType_PP
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);

	USART_DeInit(USART2);
	/* USART configuration */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //使能串口接收中断
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE); //使能串口空闲中断
												   //	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//使能串口空闲中断
												   //	//串口DMA接收、发送使能
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART2, ENABLE);

	DMA_DeInit(USART2_RX_STREAM);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;							//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);	//内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UA2RxDMAbuf;			//将串口4接收到的数据ucRxData_DMA1_Stream2[]里
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					//设置数据传输方向
	DMA_InitStructure.DMA_BufferSize = USART2_RXDMA_LEN;					//设置DMA一次传输数据量的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//设置外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//设置内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //设置外设的数据长度为字节（8bits）
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//设置内存的数据长度为字节（8bits）
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							// DMA_Mode_Normal;//设置DMA模式为循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;						// DMA_Priority_VeryHigh;//DMA_Priority_Medium;//设置DMA通道的优先级为最高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART2_RX_STREAM, &DMA_InitStructure);

	DMA_Cmd(USART2_RX_STREAM, ENABLE);

	DMA_DeInit(USART2_TX_STREAM);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;							//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);	//内存地址
																			//    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&eft;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					//设置数据传输方向
	DMA_InitStructure.DMA_BufferSize = USART2_TXDMA_LEN;					//设置DMA一次传输数据量的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//设置外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//设置内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //设置外设的数据长度为字节（8bits）
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//设置内存的数据长度为字节（8bits）
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							// DMA_Mode_Normal;//设置DMA模式为循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;					// DMA_Priority_VeryHigh;//DMA_Priority_Medium;//设置DMA通道的优先级为最高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART2_TX_STREAM, &DMA_InitStructure);

	DMA_ITConfig(USART2_TX_STREAM, DMA_IT_TC, ENABLE);
	DMA_Cmd(USART2_TX_STREAM, DISABLE);
}
