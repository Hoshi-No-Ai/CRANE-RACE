
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "hitcrt_os.h"
#include "uart_protocol.h"
#include "system_monitor_task.h"
#include "motor_drive.h"
#include "global_declare.h"
#include "fetch_rings.h"
#include "gimbal_driver.h"
#include "open_close_motor.h"
#include "friction_belt_motor.h"
#include "action_task.h"
#include "sucker.h"
int GG_temptime = 0;
int GG_time = 0;

/*系统滴答时钟中断*/
void SysTick_Handler(void)
{
	CPU_SR cpu_sr;
	OS_CRITICAL_ENTER();
	OSIntEnter();
	OS_CRITICAL_EXIT();
	if (rate_monitor.time_base < 999)
	{
		rate_monitor.time_base++;
	}
	else
	{
		memcpy(rate_monitor.real_rate, rate_monitor.temp_rate, sizeof(rate_monitor.temp_rate));
		memset(rate_monitor.temp_rate, 0, sizeof(rate_monitor.temp_rate));
		rate_monitor.time_base = 0;

	}

	OSTimeTick();
	OSIntExit();
}

CanRxMsg RxMessage1;
float CRR1 = 0;
float CRR2 = 0;
float a, b, c;
extern cSucker sucker;
int num;
int lift_flag, slide_flag;
void CAN1_RX0_IRQHandler(void)
{
	CPU_SR cpu_sr;
	OS_CRITICAL_ENTER();
	OSIntEnter();
	OS_CRITICAL_EXIT();

	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage1);

	switch (RxMessage1.StdId)
	{
	case 0x201:
		sucker.lift_motor.encoder.siRawValue = Get_Encoder_Number(&RxMessage1);
		sucker.lift_motor.encoder.Encoder_Process(0);
		if (!lift_flag)
		{
			sucker.lift_motor.encoder.siDiff = 0;
			sucker.lift_motor.encoder.siSumValue = 0;
			lift_flag = 1;
		}
		else
		{
			sucker.lift_motor.pos_pid.fpFB = (fp32)sucker.lift_motor.encoder.siSumValue * 1.0f / sucker.lift_motor.encoder.siNumber / sucker.lift_motor.encoder.siGearRatio * 360.f;
			sucker.lift_motor.velt_pid.fpFB = (fp32)1.0f * Get_Speed(&RxMessage1) / sucker.lift_motor.encoder.siGearRatio;
		}
		break;

	case 0x202:
		sucker.slide_motor.encoder.siRawValue = Get_Encoder_Number(&RxMessage1);
		sucker.slide_motor.encoder.Encoder_Process(0);
		if (!slide_flag)
		{
			sucker.slide_motor.encoder.siDiff = 0;
			sucker.slide_motor.encoder.siSumValue = 0;
			slide_flag = 1;
		}
		else
		{
			sucker.slide_motor.pos_pid.fpFB = (fp32)sucker.slide_motor.encoder.siSumValue * 1.0f / sucker.slide_motor.encoder.siNumber / sucker.slide_motor.encoder.siGearRatio * 360.f;
			sucker.slide_motor.velt_pid.fpFB = (fp32)1.0f * Get_Speed(&RxMessage1) / sucker.slide_motor.encoder.siGearRatio;
		}
		break;

	default:
		break;
	}

	OSIntExit();
}

/*CAN1的发送中断处理*/
void CAN1_TX_IRQHandler(void)
{
	CPU_SR cpu_sr;
	OS_CRITICAL_ENTER();
	OSIntEnter();
	OS_CRITICAL_EXIT();

	OSIntExit();
}

/*CAN2的接收中断处理*/
CanRxMsg RxMessage2;

void CAN2_RX0_IRQHandler(void)

{

	CPU_SR cpu_sr;
	OS_CRITICAL_ENTER();
	OSIntEnter();
	OS_CRITICAL_EXIT();
	CAN_Receive(CAN2, CAN_FIFO0, &RxMessage2);

	switch (RxMessage2.StdId)
	{
	case 0x201:
		break;
	default:
		break;
	}
	OSIntExit();
}

/*CAN2的发送中断处理*/
void CAN2_TX_IRQHandler(void)
{
	CPU_SR cpu_sr;
	OS_CRITICAL_ENTER();
	OSIntEnter();
	OS_CRITICAL_EXIT();

	//	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage1);
	// switch (RxMessage1.StdId)
	//	{

	//	}
	OSIntExit();
}

/*串口1中断函数*/
extern USART_RX_TypeDef USART1_Rcr;

void USART1_IRQHandler(void)
{
	CPU_SR cpu_sr;
	OS_CRITICAL_ENTER();
	OSIntEnter();
	OS_CRITICAL_EXIT();

	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		// 数据处理
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	else if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		USART1->SR;
		USART1->DR; // 先读SR后读DR清楚中断标志位
		USART_Receive(&USART1_Rcr);
		Comm1Rx_IRQ();
	}
	OSIntExit();
}

/*串口6中断函数*/
extern USART_RX_TypeDef USART6_Rcr;

void USART6_IRQHandler(void)
{
	CPU_SR cpu_sr;
	OS_CRITICAL_ENTER();
	OSIntEnter();
	OS_CRITICAL_EXIT();

	if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{
		// 数据处理
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
	}
	else if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
	{
		USART6->SR;
		USART6->DR; // 先读SR后读DR清楚中断标志位
		USART_Receive(&USART6_Rcr);
		Comm6Rx_IRQ();
	}

	OSIntExit();
}



extern USART_RX_TypeDef USART2_Rcr;

void USART2_IRQHandler(void)
{
    CPU_SR cpu_sr;
    OS_CRITICAL_ENTER();
    OSIntEnter();
    OS_CRITICAL_EXIT();

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        // 数据处理
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
    else if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        USART2->SR;
        USART2->DR; // 先读SR后读DR清楚中断标志位
        USART_Receive(&USART2_Rcr);
       // Comm2Rx_IRQ();
    }

    OSIntExit();
}



extern USART_RX_TypeDef USART3_Rcr;

void USART3_IRQHandler(void)
{
    CPU_SR cpu_sr;
    OS_CRITICAL_ENTER();
    OSIntEnter();
    OS_CRITICAL_EXIT();

    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        // 数据处理
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
    else if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        USART3->SR;
        USART3->DR; // 先读SR后读DR清楚中断标志位
        USART_Receive(&USART3_Rcr);
        Comm3Rx_IRQ();
//Sys_Monitor.rate_monitor.temp_rate[MINOR_32]++;
    }
    OSIntExit();
}










