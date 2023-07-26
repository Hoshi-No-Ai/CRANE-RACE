#include "main.h"
#include "sucker.h"
#include "table.h"
#include "action_task.h"
using _api_module_::calculate_flag;
using _api_module_::f_g_error;
using _remote_ctrl_::auto_enable;
#define servo_degree(x) ((2000) / 180 * x + 500)

/*ÿ�α���ǰ����Ҫ�ڱ������ġ��궨��ѡ��й������к꣡*/
// #define ENABLE_DEBUG
// #define ENABLE_MONITOR
// #define WIFI_REMOTE_CTRL
// #define DT35_PATH
#define AUTO_FETCH

DECLARE_HITCRT_OS_TASK(); // ��������

u8 udp_demo_sendbuf[ARM_DEBUG_SIZE * 4 + 4] = "Explorer STM32F407 NETCONN UDP demo send data\r\n";
u8 udp_flag; // UDP���ݷ��ͱ�־

int main(void)
{
    OS_ERR err;
    CPU_SR_ALLOC(); // �ٽ��������ʼ��

    bsp_init();

    delay_ms(100);
    delay_ms(100);

    //	��������Ư
    calculate_flag = 1;
    f_g_error = -3.587097922961;
    //	flag_tuoluo=1;
    Sys_Monitor.system_state = SYS_INIT;

    OSInit(&err);

#ifdef ENABLE_DEBUG
    lwip_comm_init(); // lwip��ʼ?
    udp_demo_test();

#endif

    Sys_Monitor.system_state = SYS_RUN;

    OS_CRITICAL_ENTER(); // �����ٽ�����ȷ���������䲻�����

    CREATE_OS_TASK(system_monitor_task);   // ������ʼ������
    CREATE_OS_TASK(remote_lcd_task);       // ����ң������ʾ������
    CREATE_OS_TASK(read_remote_ctrl_task); // �����ֱ�����ɨ������
    CREATE_OS_TASK(motor_control_task);
    CREATE_OS_TASK(navigation_task); // ������λ��������

#ifdef ENABLE_DEBUG
    CREATE_OS_TASK(debug_task);
#endif

    CREATE_OS_TASK(action_task);
    CREATE_OS_TASK(transmit_task);

    OS_CRITICAL_EXIT(); // �˳��ٽ���
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

    OSStatTaskCPUUsageInit(&err); // ��ͳ������

    OSTimeDly_ms(2000); // ����Ҫ����ʱ���ȴ����յ�֡��

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

void read_remote_ctrl_task(void *p) // �ֱ�����ɨ������
{
    OS_ERR err;
    p = p;
    OSTimeDly_ms(500);

    while (1)
    {
        //        Js_Deal();
        Key_Deal();

        OSTimeDly_ms(20); // ��nrf֡��ƥ�䣬wifi��Ϊ10ms
    }
}

static int dt35_time = 0;
void navigation_task(void *p)
{
    OS_ERR err;
    p = p;

    OSTimeDly_ms(2000); // ���涯��Init��
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
        navigation();

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

void remote_lcd_task(void *p) // esp8266��������
{
    OS_ERR err;
    p = p;
    OSTimeDly_ms(100);
    while (1)
    {
        lcd_display();
        OSTimeDly_ms(25);
    }
}
extern cSucker sucker;
void action_task(void *p)
{
    OS_ERR err;
    p = p;

    while (1) {
        robot_movement();
        movement_check(auto_enable);
        position_check();
			sucker.drive_sucker();
			 handle_box();
        OSTimeDly_ms(10);
    }
}
extern DesSet DES;

float sucker_slide_r = 50;
	float sucker_lift_r = 1000;

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
        USART3_DMA_Tx(); // עµ𾍲»»Ᏺɏ°巢ˍ

        // ¸øɏ°巢µİ-3λʇsucker_slide£¬4-7λʇsucker_lift
        OSTimeDly_ms(1);
    }
}
