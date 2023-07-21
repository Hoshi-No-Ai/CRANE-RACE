#include "E103_W06.h"

union float2char uart6_tx_buf[40];
union float2char uart3_tx_buf[40];
uint8_t wifi_tx_buf[100];

void wifi_printf(char* fmt,...)  
{  
	uint16_t i,j;
	va_list ap;
	va_start(ap, fmt);
	vsprintf((char*)wifi_tx_buf, fmt, ap);
	va_end(ap);
	i = strlen((const char*)wifi_tx_buf);//�˴η������ݵĳ���
	for(j = 0; j < i; ++j)//ѭ����������
	{
		while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET); //�ȴ��ϴδ������ 
		USART_SendData(USART6, (uint8_t)wifi_tx_buf[j]); 	 //�������ݵ�����4 
	}
}

void wifi_init(void)
{
	
}

