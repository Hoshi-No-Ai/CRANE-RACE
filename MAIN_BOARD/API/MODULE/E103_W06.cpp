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
	i = strlen((const char*)wifi_tx_buf);//此次发送数据的长度
	for(j = 0; j < i; ++j)//循环发送数据
	{
		while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET); //等待上次传输完成 
		USART_SendData(USART6, (uint8_t)wifi_tx_buf[j]); 	 //发送数据到串口4 
	}
}

void wifi_init(void)
{
	
}

