#include "main.h"
#include "sucker.h"
#include "table.h"
#include "action_task.h"
using _api_module_::calculate_flag;
using _api_module_::f_g_error;
using _navigation_::vision_enable;
using _navigation_::vision_true;
using _remote_ctrl_::auto_enable;

#define servo_degree(x) ((2000) / 180.f * x + 500)

/*每编译前，需要在编译器的【宏定义选项】中管理下列宏！*/
// #define ENABLE_DEBUG
// #define ENABLE_MONITOR
// #define WIFI_REMOTE_CTRL
// #define DT35_PATH
#define AUTO_FETCH

DECLARE_HITCRT_OS_TASK(); // 声明任务

u8 udp_demo_sendbuf[ARM_DEBUG_SIZE * 4 + 4] = "Explorer STM32F407 NETCONN UDP demo send data\r\n";
u8 udp_flag; // UDP数据发送标

int main(void)
{
    OS_ERR err;
    CPU_SR_ALLOC(); // 临界区代码初始化

    bsp_init();

    delay_ms(100);
    delay_ms(100);

    //	陀螺仪零漂
    calculate_flag = 1;
    f_g_error = -3.587097922961;
    //	flag_tuoluo=1;
    Sys_Monitor.system_state = SYS_INIT;

    OSInit(&err);

#ifdef ENABLE_DEBUG
    lwip_comm_init(); // lwip初????
    udp_demo_test();

#endif

    Sys_Monitor.system_state = SYS_RUN;

    OS_CRITICAL_ENTER(); // 进入临界区，保里面的句不打断

    CREATE_OS_TASK(system_monitor_task);   // 创建初化任务
    CREATE_OS_TASK(remote_lcd_task);       // 创建遥控器显示屏任务
    CREATE_OS_TASK(read_remote_ctrl_task); // 创建手柄盘扫描任
    CREATE_OS_TASK(motor_control_task);
    CREATE_OS_TASK(navigation_task); // 创建定位导航任务

#ifdef ENABLE_DEBUG
    CREATE_OS_TASK(debug_task);
#endif

    CREATE_OS_TASK(action_task);
    CREATE_OS_TASK(transmit_task);

    OS_CRITICAL_EXIT(); // 退出临界区
    OSStart(&err);

    while (1)
        ;
}

void system_monitor_task(void *p)
{
    OS_ERR err;
    CPU_SR_ALLOC();
    p = p;
    CPU_Init();

    OSStatTaskCPUUsageInit(&err); // 开统任

    OSTimeDly_ms(2000); // 给必要的延时，等待接收到帧率

    while (1)
    {
#ifdef ENABLE_MONITOR
        Sys_Monitor.motor_detection();
        // Sys_Monitor.minor32_detection();
        // Sys_Monitor.air_operated_board_detection();
        // Sys_Monitor.remote_ctrl_detection();
#endif

        if (Sys_Monitor.system_state == SYS_ERROR && Sys_Monitor.system_error != CHASSIS_ENCODER_ERROR)
        {
            Sys_Monitor.error_alarm();
            disable_all_motor();
            if (Sys_Monitor.return_normal())
            {
                Sys_Monitor.system_error = NONE;
                Sys_Monitor.system_state = SYS_RUN;
            }
        }
        OSTimeDly_ms(10);
    }
}

s16 pwm1 = 0;
s16 pwm2 = 0;
s16 pwm3 = 0;
s16 pwm4 = 0;
u16 pwm_value_3_3 = 0; // S1 TIM
int _servo_degree = 180;
void motor_control_task(void *p)
{
    OS_ERR err;
    p = p;

    while (1)
    {
        TIM_SetCompare3(TIM3, servo_degree(_servo_degree));

        if (Sys_Monitor.system_state == SYS_RUN ||
            (Sys_Monitor.system_state == SYS_ERROR && Sys_Monitor.system_error == CHASSIS_ENCODER_ERROR))
        {
            if (/*run_flag ==*/1)
            {
                omni_chassis_control();
                AirOperation.SendServoMsgByCan_Plus(pwm1, pwm2, pwm3, pwm4);
                // chassis_control();
                sucker.slide_to_aim();
                sucker.lift_to_aim();
                table.slide_to_aim();
                table.lift_to_aim();

                C_Motor::can_send_data(CAN2, 0x200, (int16_t)table.slide_motor1.pid_current, (int16_t)table.slide_motor2.pid_current,
                                       table.lift_motor1.pid_current, table.lift_motor2.pid_current);

                //							C_Motor::can_send_data(CAN2, 0x1FF,
                //							table.lift_motor1.pid_current, sucker.lift_motor.pid_current,0,0);
            }
        }
        OSTimeDly_ms(1);
    }
}

void read_remote_ctrl_task(void *p) // 手柄盘扫描任
{
    OS_ERR err;
    p = p;
    OSTimeDly_ms(500);

    while (1)
    {
        //        Js_Deal();
        Key_Deal();

        OSTimeDly_ms(20); // 和nrf帧率匹配，wifi则为10ms
    }
}

static int dt35_time = 0;
static int vision_time = 0;
void navigation_task(void *p)
{
    OS_ERR err;
    p = p;

    OSTimeDly_ms(2000); // 等随动轮Init
    while (1)
    {
        cRobot.RobotLocation();
#ifdef DT35_PATH
        if (nav.state == NAV_AUTO_PATH && dt35_time > 4)
        {
            cRobot.DT35_relocation_new();
            dt35_time = 0;
        }
#endif
        cRobot.Cal_RobotVelt();

        // if (/*nav.state == NAV_STOPX&&*/ vision_enable /*&& vision_time > 100*/)
        // {
        //     delta_des_cola(target_num.cola);
        //     vision_true = des_base_aruco(aruco_fdb);
        //     //            vision_time = 0;
        //     vision_enable = 0;
        // }

        navigation();

//        vision_time++;
#ifdef DT35_PATH
        dt35_time++;
#endif
        OSTimeDly_ms(2);
    }
}

#ifdef ENABLE_DEBUG
int at;
extern char tcp_demo_sendbuf[ARM_DEBUG_SIZE * 4 + 4];
extern Frame debug_frame;
void debug_task(void *p)
{
    OS_ERR err;
    p = p;

    while (1)
    {
        debug_updata();

        at = sizeof(debug_frame);
        memcpy(tcp_demo_sendbuf, &debug_frame, at);
        udp_demo_senddata(udppcb);
        lwip_periodic_handle();
        OSTimeDly_ms(5);
    }
}
#endif

void remote_lcd_task(void *p) // esp8266发送任
{
    OS_ERR err;
    p = p;
    OSTimeDly_ms(100);
    while (1)
    {
			sucker.drive_sucker();
        lcd_display();
        OSTimeDly_ms(25);
    }
}
extern cSucker sucker;
void action_task(void *p)
{
    OS_ERR err;
    p = p;

    while (1)
    {
        robot_movement();
        movement_check(auto_enable);
        position_check();
        
        handle_box();
        OSTimeDly_ms(10);
    }
}
extern DesSet DES;

float sucker_slide_r = 200;
float sucker_lift_r = 3000;
extern uint8_t omtor_mode1;
void transmit_task(void *p)
{
    OS_ERR err;
    p = p;

    while (1)
    {
        memcpy(uart3_eft.num, &DES.sucker_slide, 4);
        memcpy(&uart3_eft.num[4], &DES.sucker_lift, 4);

        memcpy(&uart3_eft.num[8], &sucker_slide_r, 4);
        memcpy(&uart3_eft.num[12], &sucker_lift_r, 4);
        uart3_eft.num[13] = omtor_mode1;
        USART3_DMA_Tx();

        OSTimeDly_ms(1);
    }
}
