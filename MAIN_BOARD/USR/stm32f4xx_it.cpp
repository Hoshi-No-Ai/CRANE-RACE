#include "stm32f4xx_it.h"
#include "sucker.h"
#include "table.h"
#include "air_operated_board.h"
#include "basic_type.h"
#include "chassis.h"
#include "hitcrt_os.h"
#include "lan8720.h"
#include "navigation_task.h"
#include "stm32f4x7_eth.h"
#include "string.h"
#include "system_monitor_task.h"
#include "usart_protocol.h"
// #include "stm32f4x7_eth.c"

int dashabi, lift1_temp, sd_tb1_temp, sd_tb2_temp, mt3509_tmp, sucker_temp;
int shab_fps;
int lift1_fps, sd_tb1_fps, sd_tb2_fps, mt3509_fps, sucker_fps;

using _api_module_::WIFI_FLAG;
using _navigation_::calibration_current;
float task_time;
float handle_time;
/*系统滴答时钟中断*/
void SysTick_Handler(void)
{
    // static uint8_t div_cnt = 0;

    CPU_SR cpu_sr;
    OS_CRITICAL_ENTER();
    OSIntEnter();
    OS_CRITICAL_EXIT();

    if (Sys_Monitor.rate_monitor.time_base < 1000)
    {
        Sys_Monitor.rate_monitor.time_base++;
        task_time += 0.001;
        handle_time += 0.001;
    }
    else
    {
        memcpy(Sys_Monitor.rate_monitor.real_rate, Sys_Monitor.rate_monitor.temp_rate,
               sizeof(Sys_Monitor.rate_monitor.temp_rate));
        memset(Sys_Monitor.rate_monitor.temp_rate, 0, sizeof(Sys_Monitor.rate_monitor.temp_rate));
        Sys_Monitor.rate_monitor.time_base = 0;
        shab_fps = dashabi;
        sd_tb1_fps = sd_tb1_temp;
        sd_tb2_fps = sd_tb2_temp;
        mt3509_fps = mt3509_tmp;
        sucker_fps = sucker_temp;
        lift1_fps = lift1_temp;
        dashabi = 0;
        sd_tb2_temp = 0;
        mt3509_tmp = 0;
        sucker_temp = 0;
        sd_tb1_temp = 0;
        lift1_temp = 0;
    }

    if (nav.state == NAV_AUTO_PATH ||
        ((nav.state == NAV_CALIBRATION_1 || nav.state == NAV_CALIBRATION_2) && calibration_current != 0))
    {
        nav.auto_path.run_time++;
    }

    WIFI_FLAG++;
    OSTimeTick();
    OSIntExit();
}

float angle;

/*CAN的接收中断处理*/
static CanRxMsg RxMessage1;
static uint16_t dt35_distance[5];

static u8 absolute_U_flag, absolute_LD_flag, absolute_RD_flag;
int16_t dt35_1, dt35_2;
void CAN1_RX0_IRQHandler(void)
{
    int16_t temp_current;
    const fp32 rate_k = 150000.0f / 30000.0f;

    CPU_SR cpu_sr;
    OS_CRITICAL_ENTER();
    OSIntEnter();
    OS_CRITICAL_EXIT();

    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage1);
    switch (RxMessage1.StdId) // 底盘8个电机都放入其中，必不安全且掉帧
    {
    case 0x20: // 底盘光电开关

        break;

    case 0x201: // RIGHTUP
        if (Sys_Monitor.system_state == SYS_INIT)
        {
            Omni_chassis[RIGHTUP].m_run_motor.encoder.siDiff = 0;
            Omni_chassis[RIGHTUP].m_run_motor.encoder.siSumValue = 0;
        }
        else
        {
            Omni_chassis[RIGHTUP].m_run_motor.velt_pid.fpFB =
                (fp32)1.0f * C_Encoder::Get_Speed(&RxMessage1) /
                Omni_chassis[RIGHTUP].m_run_motor.encoder.siGearRatio / 60.0f * 2 * PI;
        }
        Sys_Monitor.rate_monitor.temp_rate[CHASSIS_RU]++;
        break;

    case 0x202: // LEFTUP
        if (Sys_Monitor.system_state == SYS_INIT)
        {
            Omni_chassis[LEFTUP].m_run_motor.encoder.siDiff = 0;
            Omni_chassis[LEFTUP].m_run_motor.encoder.siSumValue = 0;
        }
        else
        {
            Omni_chassis[LEFTUP].m_run_motor.velt_pid.fpFB =
                (fp32)1.0f * C_Encoder::Get_Speed(&RxMessage1) /
                Omni_chassis[LEFTUP].m_run_motor.encoder.siGearRatio / 60.0f * 2 * PI;
        }
        Sys_Monitor.rate_monitor.temp_rate[CHASSIS_LU]++;
        break;
    case 0x203: // 左下行进
        if (Sys_Monitor.system_state == SYS_INIT)
        {
            Omni_chassis[LEFTDOWN].m_run_motor.encoder.siDiff = 0;
            Omni_chassis[LEFTDOWN].m_run_motor.encoder.siSumValue = 0;
        }
        else
        {
            Omni_chassis[LEFTDOWN].m_run_motor.velt_pid.fpFB =
                (fp32)1.0f * C_Encoder::Get_Speed(&RxMessage1) /
                Omni_chassis[LEFTDOWN].m_run_motor.encoder.siGearRatio / 60.0f * 2 * PI;
        }
        Sys_Monitor.rate_monitor.temp_rate[CHASSIS_LD]++;
        break;

    case 0x204: // 右下转向
        if (Sys_Monitor.system_state == SYS_INIT)
        {
            Omni_chassis[RIGHTDOWN].m_run_motor.encoder.siDiff = 0;
            Omni_chassis[RIGHTDOWN].m_run_motor.encoder.siSumValue = 0;
        }
        else
        {
            Omni_chassis[RIGHTDOWN].m_run_motor.velt_pid.fpFB =
                (fp32)1.0f * C_Encoder::Get_Speed(&RxMessage1) /
                Omni_chassis[RIGHTDOWN].m_run_motor.encoder.siGearRatio / 60.0f * 2 * PI;
        }
        Sys_Monitor.rate_monitor.temp_rate[CHASSIS_RD]++;
        break;

    case 0x205:

        break;

    case 0x210:
        memcpy(&dt35_1, &RxMessage1.Data[4], 2);
        memcpy(&dt35_2, &RxMessage1.Data[6], 2);
        dt35_1 -= 297;
        dt35_2 -= 250;
        //        dt35_1 = 1.78 * dt35_1 - 76.564;
        //        dt35_2 = 1.81 * dt35_2 - 46.60;
        break;

    case 0x356:

        memcpy(&cRobot.cFollowoerWheel.degreeA, &RxMessage1.Data[0], 4);
        memcpy(&cRobot.cFollowoerWheel.degreeB, &RxMessage1.Data[4], 4);

        Sys_Monitor.rate_monitor.temp_rate[ENCODER_RUN]++;
        break;

    default:
        break;
    }
    OSIntExit();
}

/*CAN2的接收中断处理*/
static CanRxMsg RxMessage2;
void CAN2_RX0_IRQHandler(void)
{
    int16_t temp_current;
    const fp32 rate_k = 150000.0f / 30000.0f;

    CPU_SR cpu_sr;
    OS_CRITICAL_ENTER();
    OSIntEnter();
    OS_CRITICAL_EXIT();

    CAN_Receive(CAN2, CAN_FIFO0, &RxMessage2);
    switch (RxMessage2.StdId) // 底盘8个电机都放入其中，必不安全且掉帧
    {

        //       		 case 0x206:
        //					 sucker.lift_motor.encoder.Encoder_Process(sucker.lift_motor.encoder.Get_Encoder_Number(&RxMessage2), 0); //（此处也计算出了速度）
        //					  if (Sys_Monitor.system_state == SYS_INIT)
        //						{
        //								 sucker.lift_motor.encoder.siDiff =  0;
        //								  sucker.lift_motor.encoder.siSumValue =  0;
        //
        //            }
        //						else
        //						{
        //							mt3509_tmp ++;
        //								sucker.lift_motor.velt_pid.fpFB = (fp32)1.0f * sucker.lift_motor.encoder.Get_Speed(&RxMessage2) /
        //															                    sucker.lift_motor.encoder.siGearRatio;
        //								sucker.lift_motor.pos_pid.fpFB =
        //				        (fp32)sucker.lift_motor.encoder.siSumValue *1.0f /
        //				              sucker.lift_motor.encoder.siNumber /
        //				             sucker.lift_motor.encoder.siGearRatio * 360.0f;
        //
        //            }
        //            break;
        //
        //						  case 0x205:
        //					 sucker.slide_motor.encoder.Encoder_Process(sucker.slide_motor.encoder.Get_Encoder_Number(&RxMessage2), 0); //（此处也计算出了速度）
        //					     if (Sys_Monitor.system_state == SYS_INIT) {
        //								 sucker.slide_motor.encoder.siDiff =  0;
        //								  sucker.slide_motor.encoder.siSumValue =  0;
        //
        //            } else {
        //							sucker_temp++;
        //								sucker.slide_motor.velt_pid.fpFB = (fp32)1.0f * sucker.slide_motor.encoder.Get_Speed(&RxMessage2) /
        //															                    sucker.slide_motor.encoder.siGearRatio;
        //			sucker.slide_motor.pos_pid.fpFB =
        //				        (fp32)sucker.slide_motor.encoder.siSumValue *1.0f /
        //				              sucker.slide_motor.encoder.siNumber /
        //				             sucker.slide_motor.encoder.siGearRatio * 360.0f;
        //
        //            }
        //            break;

    case 0x201:
        table.slide_motor1.encoder.Encoder_Process(table.slide_motor1.encoder.Get_Encoder_Number(&RxMessage2), 0); // （此处也计算出了速度）
        if (Sys_Monitor.system_state == SYS_INIT)
        {
            table.slide_motor1.encoder.siDiff = 0;
            table.slide_motor1.encoder.siSumValue = 0;
        }
        else
        {
            sd_tb1_temp++;
            table.slide_motor1.velt_pid.fpFB = (fp32)1.0f * table.slide_motor1.encoder.Get_Speed(&RxMessage2) /
                                               table.slide_motor1.encoder.siGearRatio;
            table.slide_motor1.pos_pid.fpFB =
                (fp32)table.slide_motor1.encoder.siSumValue * 1.0f /
                table.slide_motor1.encoder.siNumber /
                table.slide_motor1.encoder.siGearRatio * 360.0f;
        }
        break;

    case 0x202:
        table.slide_motor2.encoder.Encoder_Process(table.slide_motor2.encoder.Get_Encoder_Number(&RxMessage2), 0); // （此处也计算出了速度）
        if (Sys_Monitor.system_state == SYS_INIT)
        {
            table.slide_motor2.encoder.siDiff = 0;
            table.slide_motor2.encoder.siSumValue = 0;
        }
        else
        {
            sd_tb2_temp++;
            table.slide_motor2.velt_pid.fpFB = (fp32)1.0f * table.slide_motor2.encoder.Get_Speed(&RxMessage2) /
                                               table.slide_motor2.encoder.siGearRatio;
            table.slide_motor2.pos_pid.fpFB =
                (fp32)table.slide_motor2.encoder.siSumValue * 1.0f /
                table.slide_motor2.encoder.siNumber /
                table.slide_motor2.encoder.siGearRatio * 360.0f;
        }
        break;

    case 0x203:
        table.lift_motor1.encoder.Encoder_Process(table.lift_motor1.encoder.Get_Encoder_Number(&RxMessage2), 0); // （此处也计算出了速度）
        if (Sys_Monitor.system_state == SYS_INIT)
        {
            table.lift_motor1.encoder.siDiff = 0;
            table.lift_motor1.encoder.siSumValue = 0;
        }
        else
        {
            lift1_temp++;

            table.lift_motor1.velt_pid.fpFB = (fp32)1.0f * table.lift_motor1.encoder.Get_Speed(&RxMessage2) /
                                              table.lift_motor1.encoder.siGearRatio;
            table.lift_motor1.pos_pid.fpFB =
                (fp32)table.lift_motor1.encoder.siSumValue * 1.0f /
                table.lift_motor1.encoder.siNumber /
                table.lift_motor1.encoder.siGearRatio * 360.0f;
        }

        break;

    case 0x204:
        table.lift_motor2.encoder.Encoder_Process(table.lift_motor2.encoder.Get_Encoder_Number(&RxMessage2), 0); // （此处也计算出了速度）
        if (Sys_Monitor.system_state == SYS_INIT)
        {
            table.lift_motor2.encoder.siDiff = 0;
            table.lift_motor2.encoder.siSumValue = 0;
        }
        else
        {
            table.lift_motor2.velt_pid.fpFB = (fp32)1.0f * table.lift_motor2.encoder.Get_Speed(&RxMessage2) /
                                              table.lift_motor2.encoder.siGearRatio;
            table.lift_motor2.pos_pid.fpFB =
                (fp32)table.lift_motor2.encoder.siSumValue * 1.0f /
                table.lift_motor2.encoder.siNumber /
                table.lift_motor2.encoder.siGearRatio * 360.0f;
            dashabi++;
        }
        break;

    case 0x210:
        memcpy(&dt35_1, &RxMessage1.Data[4], 2);
        memcpy(&dt35_2, &RxMessage1.Data[6], 2);
        //		dt35_1+=10;
        //		dt35_2+=30;
        dt35_1 = 1.78 * dt35_1 - 76.564;
        dt35_2 = 1.81 * dt35_2 - 46.60;
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

/*CAN2的发送中断处理*/
void CAN2_TX_IRQHandler(void)
{
    CPU_SR cpu_sr;
    OS_CRITICAL_ENTER();
    OSIntEnter();
    OS_CRITICAL_EXIT();

    OSIntExit();
}

extern USART_RX_TypeDef USART1_Rcr;

void USART1_IRQHandler(void)
{
    CPU_SR cpu_sr;
    OS_CRITICAL_ENTER();
    OSIntEnter();
    OS_CRITICAL_EXIT();

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        //        USART_Receive(&USART1_Rcr);
        // Comm1Rx_IRQ();
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
        Comm2Rx_IRQ();
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
        Sys_Monitor.rate_monitor.temp_rate[MINOR_32]++;
    }
    OSIntExit();
}

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
        //				USART6->SR;
        //		USART6->DR;//先读SR后读DR清楚中断标志位
        //		USART_Receive(&USART6_Rcr);
        Comm6Rx_IRQ();
    }
    //	else if(USART_GetITStatus(USART6, USART_IT_IDLE)!= RESET)
    //	{
    //		USART6->SR;
    //		USART6->DR;//先读SR后读DR清楚中断标志位
    //		USART_Receive(&USART6_Rcr);
    //		Comm6Rx_IRQ();
    //		test_usart6++;
    //	}
    OSIntExit();
}

void TIM7_IRQHandler(void)
{
    CPU_SR cpu_sr;
    OS_CRITICAL_ENTER();
    OSIntEnter();
    OS_CRITICAL_EXIT();
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) // 溢出中断
    {
    }
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update); // 清除中断标志位
    OSIntExit();
}

extern void lwip_pkt_handle(void); // 在lwip_comm.c里面定义
// 以太网中断服务函数
void ETH_IRQHandler(void)
{
    while (ETH_GetRxPktSize(DMARxDescToGet) != 0) // 检测是否收到数据包
    {
        lwip_pkt_handle();
    }
    ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
    ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
}
