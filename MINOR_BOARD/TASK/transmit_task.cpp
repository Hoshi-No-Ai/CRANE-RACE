#include "transmit_task.h"

uint16_t aa = 0x1243;

void Communication_with_chassis(void)
{ 
	deal_with_message();
	
	USART6_DMA_Tx();

}

void Communication_with_vision(void)
{
}

