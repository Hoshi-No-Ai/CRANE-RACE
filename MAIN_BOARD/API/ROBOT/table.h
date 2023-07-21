#ifndef __TABLE_H__
#define __TABLE_H__
#include "sucker.h"
#include "motor_drive.h"
#include "pid_algorithm.h"

#define ENCODER_NUMBER 8191
#define SLIDE_GEAR_RATIO (36.f * 24) // 3508减速比3591/187，小轮大轮齿比38:21

// slide_3508
#define SLIDE_VELT_KP 200.0f
#define SLIDE_VELT_KI 0.f
#define SLIDE_VELT_KD 0.f
#define SLIDE_VELT_UpMax 10000.0f
#define SLIDE_VELT_EiMax 5000.0f
#define SLIDE_VELT_SumEMax 5000.0f
#define SLIDE_VELT_UdMax 5000.0f
#define SLIDE_VELT_UMax 10000.0f

#define SLIDE_POS_KP 5.0f
#define SLIDE_POS_KI 0.0f
#define SLIDE_POS_KD 0.f
#define SLIDE_POS_UpMax 7500.0f	  // 45000.0f//45000.0f
#define SLIDE_POS_EiMax 0.5f	  // 45000.0f//45000.0f
#define SLIDE_POS_SumEMax 7500.0f // 45000.0f//45000.0f
#define SLIDE_POS_UdMax 75000.0f  // 45000.0f//45000.0f
#define SLIDE_POS_UMax 8000.0f	  // 45000.0f//45000.0f

// slide_3508
#define LIFT_VELT_KP 200.0f
#define LIFT_VELT_KI 0.f
#define LIFT_VELT_KD 0.f
#define LIFT_VELT_UpMax 10000.0f
#define LIFT_VELT_EiMax 5000.0f
#define LIFT_VELT_SumEMax 5000.0f
#define LIFT_VELT_UdMax 5000.0f
#define LIFT_VELT_UMax 10000.0f

#define LIFT_POS_KP 5.0f
#define LIFT_POS_KI 0.0f
#define LIFT_POS_KD 0.f
#define LIFT_POS_UpMax 7500.0f	 // 45000.0f//45000.0f
#define LIFT_POS_EiMax 0.5f		 // 45000.0f//45000.0f
#define LIFT_POS_SumEMax 7500.0f // 45000.0f//45000.0f
#define LIFT_POS_UdMax 75000.0f	 // 45000.0f//45000.0f
#define LIFT_POS_UMax 8000.0f	 // 45000.0f//45000.0f

#define LIFT_GEAR_RATIO 36.f * 4

class cTable
{
public:
	// 1 is left motor ,2 is right motor.
	C_Motor slide_motor1;
	C_Motor slide_motor2;

	C_Motor lift_motor1;
	C_Motor lift_motor2;

	C_TD td_slide;
	C_TD td_lift;

	cTable()
	{

		td_slide.m_h = ctrl_TS;
		td_slide.m_T = ctrl_TS;
		td_slide.m_r = 50;

		td_lift.m_h = ctrl_TS;
		td_lift.m_T = ctrl_TS;
		td_lift.m_r = 1000;

		slide_motor1.pos_pid.fpUMax = 1000;
		slide_motor1.pos_pid.fpEpMax = 1000;
		slide_motor1.pos_pid.fpEdMax = 1000;
		slide_motor1.pos_pid.fpEiMax = 1000;

		slide_motor1.velt_pid.fpEpMax = 10000;
		slide_motor1.velt_pid.fpUMax = 10000;

		lift_motor1.pos_pid.fpUMax = 1000;
		lift_motor1.pos_pid.fpEpMax = 1000;
		lift_motor1.pos_pid.fpEdMax = 1000;
		lift_motor1.pos_pid.fpEiMax = 1000;
		lift_motor1.velt_pid.fpEpMax = 10000;
		lift_motor1.velt_pid.fpUMax = 10000;

		slide_motor2.pos_pid.fpUMax = 1000;
		slide_motor2.pos_pid.fpEpMax = 1000;
		slide_motor2.pos_pid.fpEdMax = 1000;
		slide_motor2.pos_pid.fpEiMax = 1000;

		slide_motor2.velt_pid.fpEpMax = 10000;
		slide_motor2.velt_pid.fpUMax = 10000;

		lift_motor2.pos_pid.fpUMax = 1000;
		lift_motor2.pos_pid.fpEpMax = 1000;
		lift_motor2.pos_pid.fpEdMax = 1000;
		lift_motor2.pos_pid.fpEiMax = 1000;
		lift_motor2.velt_pid.fpEpMax = 10000;
		lift_motor2.velt_pid.fpUMax = 10000;

		cTable_init();
	};

	void cTable_init()
	{
		slide_motor1.init__motor_PID(SLIDE_POS_KP, SLIDE_POS_KI, SLIDE_POS_KD, SLIDE_VELT_KP, SLIDE_VELT_KI, SLIDE_VELT_KD);
		slide_motor2.init__motor_PID(SLIDE_POS_KP, SLIDE_POS_KI, SLIDE_POS_KD, SLIDE_VELT_KP, SLIDE_VELT_KI, SLIDE_VELT_KD);

		lift_motor1.init__motor_PID(LIFT_POS_KP, LIFT_POS_KI, LIFT_POS_KD, LIFT_VELT_KP, LIFT_POS_KI, LIFT_POS_KD);
		lift_motor2.init__motor_PID(LIFT_POS_KP, LIFT_POS_KI, LIFT_POS_KD, LIFT_VELT_KP, LIFT_POS_KI, LIFT_POS_KD);

		slide_motor1.init__motor_encoder(SLIDE_GEAR_RATIO, ENCODER_NUMBER);
		slide_motor2.init__motor_encoder(SLIDE_GEAR_RATIO, ENCODER_NUMBER);

		lift_motor1.init__motor_encoder(LIFT_GEAR_RATIO, ENCODER_NUMBER);
		lift_motor2.init__motor_encoder(LIFT_GEAR_RATIO, ENCODER_NUMBER);
	};

	void slide_to_aim(void);
	void lift_to_aim(void);
};


extern cTable table;
#endif