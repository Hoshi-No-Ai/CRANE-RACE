#include "motor_all.h"
#include "sucker.h"
des DES{
	0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f

};

s32 time;
MOTOR_MODE motor_mode;
int air_flag = 0;
int HT_init_flag = 0;
int GG_flag = 0;
int time_delay = 0;
extern cSucker sucker;
void motor_crl_task(void)
{
	sucker.lift_to_aim();
	sucker.slide_to_aim();
	can_send_data(CAN1, 0x200, sucker.lift_motor.pid_current, sucker.slide_motor.pid_current, 0, 0);
}

void init_all_motor(void)
{
	rate_monitor.temp_rate[3]++;
}
