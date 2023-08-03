#include "hitcrt_os.h"
#include "stm32f4xx.h"
#include "bsp_init.h"
#include "uart_protocol.h"
#include "motor_control_task.h"
#include "gimbal_cal_task.h"
#include "system_monitor_task.h"
#include "action_task.h"
#include "air_operated_board.h"
#include "transmit_task.h"

// 声明任务
DECLARE_HITCRT_OS_TASK();

int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC(); // 临界区代码初始化

	bsp_init();

	OSInit(&err);
	OS_CRITICAL_ENTER();	   // 进入临界区，确保里面的语句不被打断
	CREATE_OS_TASK(init_task); // 创建初始化任务
	OS_CRITICAL_EXIT();		   // 退出临界区
	OSStart(&err);

	while (1);
}

// 初始化任务
void init_task(void *p)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p = p;
	CPU_Init();

	OSStatTaskCPUUsageInit(&err); // 开统计任务
	OS_CRITICAL_ENTER();		  // 进入临界区

	/*********************创建任务**********************/
	CREATE_OS_TASK(system_monitor_task);
	CREATE_OS_TASK(transmit_task);
	CREATE_OS_TASK(action_task);
	system_state = SYS_RUN;

	OS_CRITICAL_EXIT(); // 退出临界区
	OSTaskDel((OS_TCB *)0, &err);
}

// 系统监视器，发生严重掉帧时失能
void system_monitor_task(void *p)
{
	OS_ERR err;
	p = p;

	OSTimeDly_ms(2000);

	while (1)
	{
#ifdef ENABLE_MONITOR
		motor_detection();
#endif
		send_system_massage2main32();
		if (system_state == SYS_ERROR)
		{
			disable_all_motor();
			if (return_normal())
			{
				system_error = NONE;
				system_state = SYS_RUN;
			}
		}
		OSTimeDly_ms(10);
	}
}

void action_task(void *p)
{
	OS_ERR err;
	p = p;
	OSTimeDly_ms(2000);
	while (1)
	{

		init_all_motor();
		action_detection();
		motor_crl_task();
		rate_monitor.temp_rate[1]++;
		OSTimeDly_ms(1);
	} // 1
}

/*transmit_task用于串口接收与发送，并且处理数据*/

void transmit_task(void *p)
{
	OS_ERR err;
	p = p;
	OSTimeDly_ms(2000);
	while (1)
	{
		rate_monitor.temp_rate[6]++;
		Communication_with_chassis();
		// Communication_with_vision();
		OSTimeDly_ms(1); // 1
	}
}
