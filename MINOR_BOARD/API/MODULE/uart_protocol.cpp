#include "uart_protocol.h"

uart1_tx_protocol_t eft1 =
	{
		0x55,
		0x22,
		UART1_TX_DATA_LEN,
		{0},
		0x01,
		0xAA};

uart1_rx_protocol_r efr1 =
	{
		0x55,
		0x00,
		0x03,
		UART1_RX_DATA_LEN,

		{0},
		0x01,
		0xAA};

uart6_rx_protocol_r efr6 =
	{
		0x55,
		0x22,
		UART6_RX_DATA_LEN,
		{0},
		0x01,
		0xAA};

void USART1_DMA_Tx(void) // ����1 DMA���ͺ���
{
	DMA_ClearITPendingBit(USART1_TX_STREAM, DMA_IT_TCIF7);
	DMA_Cmd(USART1_TX_STREAM, DISABLE);
	DMA2_Stream7->M0AR = (uint32_t)&eft1;		 // ���õ�ǰ�������ݻ���ַ:Memory0 tARget
	DMA2_Stream7->NDTR = (uint32_t)sizeof(eft1); // ���õ�ǰ���������ݵ�����:Number of Data units to be TRansferred
	DMA_Cmd(USART1_TX_STREAM, ENABLE);
	while (DMA_GetCurrDataCounter(USART1_TX_STREAM))
		;
}

uint8_t ucData1;
/*����1��صĻ�����*/
uint8_t uart1_data_buf[UART1_RX_DATA_LEN];

void Comm1Rx_IRQ(void) // ����1����DMA���պ���
{
	u8 i = 0;
	static unsigned char Comm1_Rx_Status = RX_FREE; // ��ʼ״̬
	static unsigned char ucPit = 0;					// �����ֽڼ���
	//	unsigned char i = 0;			                                //������������������DMA����

	for (i = 0; i < UART1_RX_DATA_LEN + 5; i = i + 1) // �������飬�����Сi�ı����USART1_RXMB_LEN��С�仯
	{
		ucData1 = UA1RxMailbox[i]; // ȡ��һ���ֽ�
		/*********************************״̬���������ݰ�************************************/
		/*���ν����ж�֡ͷ֡β*/
		switch (Comm1_Rx_Status)
		{
		case RX_FREE:
			if (ucData1 == efr1.usart1_rx_start1)
			{
				Comm1_Rx_Status = RX_START_1; // ����״̬�½ӵ�0x55��Ϊ��ʼ
			}
			break;

		case RX_START_1:
			if (ucData1 == efr1.usart1_rx_start2)
			{
				Comm1_Rx_Status = RX_START_2;
			}
			else
			{
				Comm1_Rx_Status = RX_FREE;
			}
			break;

		case RX_START_2:
			if (ucData1 == efr1.date_len_float)
			{
				Comm1_Rx_Status = RX_START_3;
			}

		case RX_START_3:
			if (ucData1 == efr1.usart1_rx_datanum)
			{
				Comm1_Rx_Status = RX_DATAS;
			}
			else
			{
				Comm1_Rx_Status = RX_FREE;
			}
			break;

		case RX_DATAS:
			if (ucPit < efr1.usart1_rx_datanum) // ���û��������
			{
				*(uart1_data_buf + ucPit) = ucData1;
				ucPit++;
			}
			else // �������ж�0x00
			{
				ucPit = 0;
				if (ucData1 == efr1.usart1_rx_tail1)
				{
					Comm1_Rx_Status = RX_TAIL_1;
				}
				else
				{
					Comm1_Rx_Status = RX_FREE;
				}
			}
			break;

		case RX_TAIL_1:
		{
			if (ucData1 == efr1.usart1_rx_tail2) // ����ӵ���0xAA��������Ч
			{
				memcpy(efr1.num, uart1_data_buf, UART1_RX_DATA_LEN);
			}
			Comm1_Rx_Status = RX_FREE;
		}
		break;
		default:
			break;
		}
	}
}

void USART6_DMA_Tx(void) // ����6 DMA���ͺ���
{
	DMA_ClearITPendingBit(USART6_TX_STREAM, DMA_IT_TCIF6);
	DMA_Cmd(USART6_TX_STREAM, DISABLE);
	DMA2_Stream6->M0AR = (uint32_t)&eft6;		 // ���õ�ǰ�������ݻ���ַ:Memory0 tARget
	DMA2_Stream6->NDTR = (uint32_t)sizeof(eft6); // ���õ�ǰ���������ݵ�����:Number of Data units to be TRansferred
	DMA_Cmd(USART6_TX_STREAM, ENABLE);
	while (DMA_GetCurrDataCounter(USART6_TX_STREAM))
		;
}

uint8_t ucData6;
/*����1��صĻ�����*/
uint8_t uart6_data_buf[UART6_RX_DATA_LEN];

void Comm6Rx_IRQ(void) // ����6����DMA���պ���
{
	u8 i = 0;
	static unsigned char Comm6_Rx_Status = RX_FREE; // ��ʼ״̬
	static unsigned char ucPit = 0;					// �����ֽڼ���
	//	unsigned char i = 0;			                                //������������������DMA����

	for (i = 0; i < UART6_RX_DATA_LEN + 5; i = i + 1) // �������飬�����Сi�ı����USART1_RXMB_LEN��С�仯
	{
		ucData6 = UA6RxMailbox[i]; // ȡ��һ���ֽ�

		/*********************************״̬���������ݰ�************************************/
		/*���ν����ж�֡ͷ֡β*/
		switch (Comm6_Rx_Status)
		{
		case RX_FREE:
			if (ucData6 == efr6.usart6_rx_start1)
			{
				Comm6_Rx_Status = RX_START_1; // ����״̬�½ӵ�0x55��Ϊ��ʼ
			}
			break;

		case RX_START_1:
			if (ucData6 == efr6.usart6_rx_start2)
			{
				Comm6_Rx_Status = RX_START_2;
			}
			else
			{
				Comm6_Rx_Status = RX_FREE;
			}
			break;

		case RX_START_2:
			if (ucData6 == efr6.usart6_rx_datanum)
			{
				Comm6_Rx_Status = RX_DATAS;
			}
			else
			{
				Comm6_Rx_Status = RX_FREE;
			}
			break;

		case RX_DATAS:
			if (ucPit < efr6.usart6_rx_datanum) // ���û��������
			{
				*(uart6_data_buf + ucPit) = ucData6;
				ucPit++;
			}
			else // �������ж�0x00
			{
				ucPit = 0;
				if (ucData6 == efr6.usart6_rx_tail1)
				{
					Comm6_Rx_Status = RX_TAIL_1;
				}
				else
				{
					Comm6_Rx_Status = RX_FREE;
				}
			}
			break;

		case RX_TAIL_1:
		{
			if (ucData6 == efr6.usart6_rx_tail2) // ����ӵ���0xAA��������Ч
			{
				memcpy(efr6.num, uart6_data_buf, UART6_RX_DATA_LEN);
			}
			Comm6_Rx_Status = RX_FREE;
		}
		break;
		default:
			break;
		}
	}
}

u8 pLen_write = 0;
s8 bBuf[20];

uart6_tx_protocol_t eft6 =
	{
		0x55,
		0x22,
		UART6_TX_DATA_LEN,
		{0},
		0x01,
		0xAA

};

unsigned short USART_Receive(USART_RX_TypeDef *USARTx)
{
	USARTx->rxConter = USARTx->DMALen - DMA_GetCurrDataCounter(USARTx->DMAy_Streamx);

	USARTx->rxBufferPtr += USARTx->rxSize;

	if (USARTx->rxBufferPtr >= USARTx->DMALen)
	{
		USARTx->rxBufferPtr %= USARTx->DMALen;
	}

	if (USARTx->rxBufferPtr < USARTx->rxConter)
	{
		USARTx->rxSize = USARTx->rxConter - USARTx->rxBufferPtr;
		if (USARTx->rxSize <= USARTx->MbLen)
		{
			for (uint16_t i = 0; i < USARTx->rxSize; ++i)
			{
				*(USARTx->pMailbox + i) = *(USARTx->pDMAbuf + USARTx->rxBufferPtr + i);
			}
		}
	}
	else
	{
		USARTx->rxSize = USARTx->rxConter + USARTx->DMALen - USARTx->rxBufferPtr;
		if (USARTx->rxSize <= USARTx->MbLen)
		{
			for (uint16_t i = 0; i < USARTx->rxSize - USARTx->rxConter; ++i)
			{
				*(USARTx->pMailbox + i) = *(USARTx->pDMAbuf + USARTx->rxBufferPtr + i);
			}
			for (uint16_t i = 0; i < USARTx->rxConter; ++i)
			{
				*(USARTx->pMailbox + USARTx->rxSize - USARTx->rxConter + i) = *(USARTx->pDMAbuf + i);
			}
		}
	}
	return USARTx->rxSize;
}
