
 #include "uart.h"

uint8_t UA1RxDMAbuf[USART1_RXDMA_LEN] = {0};

uint8_t UA1RxMailbox[USART1_RXMB_LEN] = {0};

USART_RX_TypeDef USART1_Rcr = {USART1,USART1_RX_STREAM,UA1RxMailbox,UA1RxDMAbuf,USART1_RXMB_LEN,USART1_RXDMA_LEN,0,0,0};

uint8_t UA1TxDMAbuf[USART1_TXDMA_LEN] = {0};


void USART1_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* Enable UART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	/*ʹ��DMAʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	
	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	
	/* USART configuration */
	USART_InitStructure.USART_BaudRate = 115200;//921600;//esp8266Ϊ921600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//ʹ�ܴ��ڿ����ж�
	//����DMA���ա�����ʹ��
	
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART1, ENABLE);
	
	/*DMA RX configuration */
	DMA_DeInit(USART1_RX_STREAM);
  DMA_InitStructure.DMA_Channel= DMA_Channel_4;//�����ַ
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);//�ڴ��ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UA1RxDMAbuf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//�������ݴ��䷽��

  DMA_InitStructure.DMA_BufferSize = USART1_RXDMA_LEN;//����DMAһ�δ����������Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//�����ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//������������ݳ���Ϊ�ֽڣ�8bits��
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�����ڴ�����ݳ���Ϊ�ֽڣ�8bits��
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal//����DMAģʽΪѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority_VeryHigh;//DMA_Priority_Medium;//����DMAͨ�������ȼ�Ϊ������ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(USART1_RX_STREAM,&DMA_InitStructure);
  DMA_Cmd(USART1_RX_STREAM,ENABLE);
	
	DMA_DeInit(USART1_TX_STREAM);
  DMA_InitStructure.DMA_Channel= DMA_Channel_4;//�����ַ
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);//�ڴ��ַ
//    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&eft;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�������ݴ��䷽��
	DMA_InitStructure.DMA_BufferSize = 0;//����DMAһ�δ����������Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//�����ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//������������ݳ���Ϊ�ֽڣ�8bits��
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�����ڴ�����ݳ���Ϊ�ֽڣ�8bits��
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Normal;//����DMAģʽΪѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//DMA_Priority_VeryHigh;//DMA_Priority_Medium;//����DMAͨ�������ȼ�Ϊ������ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(USART1_TX_STREAM,&DMA_InitStructure);

  DMA_ITConfig(USART1_TX_STREAM,DMA_IT_TC,ENABLE);
	DMA_Cmd(USART1_TX_STREAM,DISABLE);
}



uint8_t UA6RxDMAbuf[USART6_RXDMA_LEN] = {0};
uint8_t UA6RxMailbox[USART6_RXMB_LEN] = {0};
USART_RX_TypeDef USART6_Rcr = {USART6,USART6_RX_STREAM,UA6RxMailbox,UA6RxDMAbuf,USART6_RXMB_LEN,USART6_RXDMA_LEN,0,0,0};

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
	
	/*ʹ��DMAʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	
	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
	
	
	/* USART configuration */
	USART_InitStructure.USART_BaudRate = 921600;//esp8266Ϊ921600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART6, &USART_InitStructure);
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);//ʹ�ܴ��ڿ����ж�
	//����DMA���ա�����ʹ��
	
	
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART6, ENABLE);
	
	/*DMA RX configuration */
	DMA_DeInit(USART6_RX_STREAM);
  DMA_InitStructure.DMA_Channel= DMA_Channel_5;//�����ַ
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);//�ڴ��ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UA6RxDMAbuf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//�������ݴ��䷽��

  DMA_InitStructure.DMA_BufferSize = USART6_RXDMA_LEN;//����DMAһ�δ����������Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//�����ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//������������ݳ���Ϊ�ֽڣ�8bits��
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�����ڴ�����ݳ���Ϊ�ֽڣ�8bits��
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal//����DMAģʽΪѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority_VeryHigh;//DMA_Priority_Medium;//����DMAͨ�������ȼ�Ϊ������ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(USART6_RX_STREAM,&DMA_InitStructure);
  DMA_Cmd(USART6_RX_STREAM,ENABLE);
	
	DMA_DeInit(USART6_TX_STREAM);
  DMA_InitStructure.DMA_Channel= DMA_Channel_5;//�����ַ
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);//�ڴ��ַ
//    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&eft;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�������ݴ��䷽��
	DMA_InitStructure.DMA_BufferSize = 0;//����DMAһ�δ����������Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//�����ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//������������ݳ���Ϊ�ֽڣ�8bits��
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�����ڴ�����ݳ���Ϊ�ֽڣ�8bits��
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Normal;//����DMAģʽΪѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//DMA_Priority_VeryHigh;//DMA_Priority_Medium;//����DMAͨ�������ȼ�Ϊ������ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(USART6_TX_STREAM,&DMA_InitStructure);

  DMA_ITConfig(USART6_TX_STREAM,DMA_IT_TC,ENABLE);
	DMA_Cmd(USART6_TX_STREAM,DISABLE);
}
