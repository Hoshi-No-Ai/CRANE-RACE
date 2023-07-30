#include "uart_protocol.h"
#include "string.h"
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
char string_cmpare_vallid[8] = "State;0";
char string_cmpare_signal_err[8] = "State;2";


char string_cmpare_outrange[8] = "State;4";

uint8_t uart1_data_buf[UART1_RX_DATA_LEN];
int32_t distance_uart1;
int else_err;
char right_ans[8];
char outrange_ans[8];
	char ifright[8];
		
int compare_string(char * a,char* b,int len)
{
	for(int i=0;i<len;i++)
	{
		if(a[i]==b[i])
		{
		}
		else
		{
			return 0;
		}
		
		
	}
	return 1;
	
}
//float sb_weite;
void Comm1Rx_IRQ(void) // ����1����DMA���պ���
{
	u8 i = 0;
	static unsigned char Comm1_Rx_Status = RX_FREE; // ��ʼ״̬
	static unsigned char ucPit = 0;					// �����ֽڼ���
	//	unsigned char i = 0;			                                //������������������DMA����

	for (i = 0; i < UART1_RX_DATA_LEN ; i = i + 1) // �������飬�����Сi�ı����USART1_RXMB_LEN��С�仯
	{
		ucData1 = UA1RxMailbox[i]; // ȡ��һ���ֽ�
		/*********************************״̬���������ݰ�************************************/
		/*���ν����ж�֡ͷ֡β*/
		switch (Comm1_Rx_Status)
		{
		case RX_FREE:
			if (ucData1 == 0x53)
			{
				Comm1_Rx_Status = RX_START_1; // ����״̬�½ӵ�0x55��Ϊ��ʼ
			}
			break;

		case RX_START_1:
			if (ucData1 == 0x74)
			{
				Comm1_Rx_Status = RX_START_2;
			}
			else
			{
				Comm1_Rx_Status = RX_FREE;
			}
			break;

		case RX_START_2:
			if (ucData1 == 0x61)
			{
				Comm1_Rx_Status = RX_START_3;
			}
			break;

		case RX_START_3:
			if (ucData1 ==0x74)
			{
				Comm1_Rx_Status = RX_DATAS;
			}
			else
			{
				Comm1_Rx_Status = RX_FREE;
			}
			break;

		case RX_DATAS:
		//	memcpy(string_distance1, UA1RxMailbox, 38);
	
		memcpy(ifright, &UA1RxMailbox[0], 8);		
			memcpy(right_ans,string_cmpare_vallid, 8);
		memcpy(outrange_ans, string_cmpare_outrange, 8);
		
					distance_uart1 = 0;

		if(compare_string(ifright,right_ans,7)  == 1)
		{
			if((int)UA1RxMailbox[26]>0x30)	distance_uart1 +=	((int)UA1RxMailbox[26]-0x30)*1000;
			if((int)UA1RxMailbox[27]>0x30)distance_uart1 +=	((int)UA1RxMailbox[27]-0x30)*100;
			if((int)UA1RxMailbox[28]>0x30)distance_uart1 +=	((int)UA1RxMailbox[28]-0x30)*10;
			if((int)UA1RxMailbox[29]>0x30)distance_uart1 +=	(int)UA1RxMailbox[29]-0x30;
			

		}
		else if(compare_string(ifright,string_cmpare_signal_err,7)  == 1)
		{
			if((int)UA1RxMailbox[26]>0x30)	distance_uart1 +=	((int)UA1RxMailbox[26]-0x30)*1000;
			if((int)UA1RxMailbox[27]>0x30)distance_uart1 +=	((int)UA1RxMailbox[27]-0x30)*100;
			if((int)UA1RxMailbox[28]>0x30)distance_uart1 +=	((int)UA1RxMailbox[28]-0x30)*10;
			if((int)UA1RxMailbox[29]>0x30)distance_uart1 +=	(int)UA1RxMailbox[29]-0x30;
			

		}
		else if(compare_string(ifright,outrange_ans,7)  == 1)
		{
				distance_uart1 = (UA1RxMailbox[26]-0x30)*1000+ ((int)UA1RxMailbox[27]-0x30)*100+ ( (int)UA1RxMailbox[28]-0x30)*10+  ((int)UA1RxMailbox[29]-0x30)*1;

		}
		
		if(!distance_uart1)
		{
		//	sb_weite =1;
			if((int)UA1RxMailbox[24]>0x30)	distance_uart1 +=	((int)UA1RxMailbox[24]-0x30)*1000;
			if((int)UA1RxMailbox[25]>0x30)distance_uart1 +=	((int)UA1RxMailbox[25]-0x30)*100;
			if((int)UA1RxMailbox[26]>0x30)distance_uart1 +=	((int)UA1RxMailbox[26]-0x30)*10;
			if((int)UA1RxMailbox[27]>0x30)distance_uart1 +=	(int)UA1RxMailbox[27]-0x30;
		}
		distance_uart1 -=120;
	//	distance_uart1 = atol(string_dis1);
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
				
			}
			Comm1_Rx_Status = RX_FREE;
		}
		break;
		default:
			break;
		}
	}
}




//uint8_t uart1_data_buf[UART1_RX_DATA_LEN];
int distance_uart3;


void Comm3Rx_IRQ(void) // ����1����DMA���պ���
{
	u8 i = 0;
	static unsigned char Comm1_Rx_Status = RX_FREE; // ��ʼ״̬
	static unsigned char ucPit = 0;					// �����ֽڼ���
	//	unsigned char i = 0;			                                //������������������DMA����

	for (i = 0; i < UART1_RX_DATA_LEN ; i = i + 1) // �������飬�����Сi�ı����USART1_RXMB_LEN��С�仯
	{
		ucData1 = UA3RxMailbox[i]; // ȡ��һ���ֽ�
		/*********************************״̬���������ݰ�************************************/
		/*���ν����ж�֡ͷ֡β*/
		switch (Comm1_Rx_Status)
		{
		case RX_FREE:
			if (ucData1 == 0x53)
			{
				Comm1_Rx_Status = RX_START_1; // ����״̬�½ӵ�0x55��Ϊ��ʼ
			}
			break;

		case RX_START_1:
			if (ucData1 == 0x74)
			{
				Comm1_Rx_Status = RX_START_2;
			}
			else
			{
				Comm1_Rx_Status = RX_FREE;
			}
			break;

		case RX_START_2:
			if (ucData1 == 0x61)
			{
				Comm1_Rx_Status = RX_START_3;
			}
			break;

		case RX_START_3:
			if (ucData1 ==0x74)
			{
				Comm1_Rx_Status = RX_DATAS;
			}
			else
			{
				Comm1_Rx_Status = RX_FREE;
			}
			break;

		case RX_DATAS:
		//	memcpy(string_distance1, UA1RxMailbox, 38);
	
		memcpy(ifright, &UA3RxMailbox[0], 8);		
			memcpy(right_ans,string_cmpare_vallid, 8);
		memcpy(outrange_ans, string_cmpare_outrange, 8);
		
					distance_uart3 = 0;

		if(compare_string(ifright,right_ans,7)  == 1)
		{
			if((int)UA3RxMailbox[29]>0x30)	distance_uart3 +=	((int)UA3RxMailbox[29]-0x30)*1000;
			if((int)UA3RxMailbox[30]>0x30)distance_uart3 +=	((int)UA3RxMailbox[30]-0x30)*100;
			if((int)UA3RxMailbox[31]>0x30)distance_uart3 +=	((int)UA3RxMailbox[31]-0x30)*10;
			if((int)UA3RxMailbox[32]>0x30)distance_uart3 +=	(int)UA3RxMailbox[32]-0x30;
			

		}
		else if(compare_string(ifright,outrange_ans,7)  == 1)
		{
				distance_uart3 = (UA3RxMailbox[29]-0x30)*1000+ ((int)UA3RxMailbox[30]-0x30)*100+ ( (int)UA3RxMailbox[31]-0x30)*10+  ((int)UA3RxMailbox[32]-0x30)*1;

		}
	//	distance_uart1 = atol(string_dis1);
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
