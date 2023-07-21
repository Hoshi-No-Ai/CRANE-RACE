#include "system_monitor_task.h"

error_type_e system_error = NONE;
rate_monitor_t rate_monitor;
system_state_e system_state = SYS_INIT;

void motor_detection(void)
{
	for(int i = 0; i < 2; ++i)
	{
		if(rate_monitor.real_rate[0] < 950 || rate_monitor.real_rate[0] > 1050)
		{
			system_error = SEND_ERROR;
			system_state = SYS_ERROR;
		}
	}
	for(int i = 2; i < 6; ++i)
	{
		if(rate_monitor.real_rate[0] < 950 || rate_monitor.real_rate[0] > 1050)
		{
			system_error = GIMBAL_ERROR;
			system_state = SYS_ERROR;
		}
	}
}

uint8_t return_normal(void)
{
	for(int i = 0; i < 6; ++i)
	{
		if(rate_monitor.real_rate[i] < 995 || rate_monitor.real_rate[i] > 1050)
		{
			return 0;
		}
	}
	return 1;
}

void disable_all_motor(void)
{
	can_send_data(CAN1, 0x200, 0, 0, 0, 0);
	can_send_data(CAN1, 0x1FF, 0, 0, 0, 0);
	can_send_data(CAN2, 0x200, 0, 0, 0, 0);
	can_send_data(CAN2, 0x1FF, 0, 0, 0, 0);
}

void send_system_massage2main32(void)
{
	if(system_state == SYS_RUN)
	{
		eft1.num[0] = 1;
	}
	else
	{
		eft1.num[0] = 0;
		switch(system_error)
		{
		case GIMBAL_ERROR:
			eft1.num[1] = 1;
			break;
		case SEND_ERROR:
			eft1.num[1] = 2;
			break;
		case NONE://不会发生，为了消除keil警告
			break;
		}
	}
//	USART6_DMA_Tx();
}
