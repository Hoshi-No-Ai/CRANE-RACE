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

// ��������
DECLARE_HITCRT_OS_TASK();

int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC(); // �ٽ��������ʼ��

	bsp_init();

	OSInit(&err);
	OS_CRITICAL_ENTER();	   // �����ٽ�����ȷ���������䲻�����
	CREATE_OS_TASK(init_task); // ������ʼ������
	OS_CRITICAL_EXIT();		   // �˳��ٽ���
	OSStart(&err);

	while (1);
}

// ��ʼ������
void init_task(void *p)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p = p;
	CPU_Init();

	OSStatTaskCPUUsageInit(&err); // ��ͳ������
	OS_CRITICAL_ENTER();		  // �����ٽ���

	/*********************��������**********************/
	CREATE_OS_TASK(system_monitor_task);
	CREATE_OS_TASK(transmit_task);
	CREATE_OS_TASK(action_task);
	system_state = SYS_RUN;

	OS_CRITICAL_EXIT(); // �˳��ٽ���
	OSTaskDel((OS_TCB *)0, &err);
}

// ϵͳ���������������ص�֡ʱʧ��
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

/*transmit_task���ڴ��ڽ����뷢�ͣ����Ҵ�������*/

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
