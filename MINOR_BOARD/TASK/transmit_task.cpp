#include "transmit_task.h"

uint16_t aa = 0x1243;
extern cSucker sucker;

void Communication_with_chassis(void)
{ 
	deal_with_message();
//	sucker.slide_motor.pos_pid.fpFB = 1032;
//	sucker.lift_motor.pos_pid.fpFB = 7.86;
	memcpy(eft6.num, &sucker.slide_motor.pos_pid.fpFB, 4);
	memcpy(&eft6.num[4], &sucker.lift_motor.pos_pid.fpFB, 4);

	USART6_DMA_Tx();

}

void Communication_with_vision(void)
{
}

