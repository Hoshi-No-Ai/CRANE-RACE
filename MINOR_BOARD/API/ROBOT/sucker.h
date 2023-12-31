#ifndef __SUCKER_H__
#define __SUCKER_H__

#include "motor_drive.h"
#include "pid_algorithm.h"

typedef struct {
	
	float sucker_slide;
	float sucker_lift;
	
		float table_slide;
	float table_lift;
	
}DesSet;

#define ENCODER_NUMBER 8191
#define SLIDE_GEAR_RATIO (19.f*42) //3508减速比3591/187，小轮大轮齿比38:21






//slide_3508
#define SLIDE_VELT_KP 	200.0f
#define SLIDE_VELT_KI 	0.f
#define SLIDE_VELT_KD 	0.f
#define SLIDE_VELT_UpMax	10000.0f
#define SLIDE_VELT_EiMax	5000.0f
#define SLIDE_VELT_SumEMax	5000.0f
#define SLIDE_VELT_UdMax	5000.0f
#define SLIDE_VELT_UMax	    10000.0f

#define SLIDE_POS_KP 	    40.0f
#define SLIDE_POS_KI 	    0.0f
#define SLIDE_POS_KD 	    0.f
#define SLIDE_POS_UpMax	7500.0f//45000.0f//45000.0f
#define SLIDE_POS_EiMax	0.5f//45000.0f//45000.0f
#define SLIDE_POS_SumEMax	7500.0f//45000.0f//45000.0f
#define SLIDE_POS_UdMax	75000.0f//45000.0f//45000.0f
#define SLIDE_POS_UMax	8000.0f//45000.0f//45000.0f


//slide_3508
#define LIFT_VELT_KP 	200.0f
#define LIFT_VELT_KI 	0.f
#define LIFT_VELT_KD 	0.f
#define LIFT_VELT_UpMax	10000.0f
#define LIFT_VELT_EiMax	5000.0f
#define LIFT_VELT_SumEMax	5000.0f
#define LIFT_VELT_UdMax	5000.0f
#define LIFT_VELT_UMax	    10000.0f

#define LIFT_POS_KP 	    15.0f
#define LIFT_POS_KI 	    0.0f
#define LIFT_POS_KD 	    0.f
#define LIFT_POS_UpMax	7500.0f//45000.0f//45000.0f
#define LIFT_POS_EiMax	0.5f//45000.0f//45000.0f
#define LIFT_POS_SumEMax	7500.0f//45000.0f//45000.0f
#define LIFT_POS_UdMax	75000.0f//45000.0f//45000.0f
#define LIFT_POS_UMax	8000.0f//45000.0f//45000.0f


#define LIFT_GEAR_RATIO 19

#define ctrl_TS 0.001

class cSucker
{
public:
	
  cMotor_double_td slide_motor;
  cMotor_double_td lift_motor;

	cTD td_slide;
	cTD td_lift;
	
	int Toggle_sucker;

	cSucker() {
	
	cSucker_init();
	};
	
	void cSucker_init()
 {
	 td_slide.h = ctrl_TS;
	 td_slide.T =  ctrl_TS;
	 td_slide.r = 100;
	 
	  td_lift.h = ctrl_TS;
	 td_lift.T =  ctrl_TS;
	 td_lift.r = 700;
	 
	 
	 slide_motor.pos_pid.fpKp = 40;
	  slide_motor.velt_pid.fpKp = 200;
	 
	 lift_motor.pos_pid.fpKp = 10;
	  lift_motor.velt_pid.fpKp = 100;
	 
	 slide_motor.pos_pid.fpUMax = 1000;
	 slide_motor.pos_pid.fpEpMax = 1000;
	  slide_motor.pos_pid.fpEdMax = 1000;
	 slide_motor.pos_pid.fpEiMax = 1000;
	 
	  slide_motor.velt_pid.fpEpMax = 10000;
	  slide_motor.velt_pid.fpUMax = 10000;
	 
	 
	 	 lift_motor.pos_pid.fpUMax = 1000;
	 lift_motor.pos_pid.fpEpMax = 1000;
	  lift_motor.pos_pid.fpEdMax = 1000;
	 lift_motor.pos_pid.fpEiMax = 1000;
	 
	  lift_motor.velt_pid.fpEpMax = 16000;
	  lift_motor.velt_pid.fpUMax = 16000;
	 
	
//	 
//  slide_motor.init__motor_PID(SLIDE_POS_KP,SLIDE_POS_KI,SLIDE_POS_KD,SLIDE_VELT_KP,SLIDE_VELT_KI,SLIDE_VELT_KD);
//  lift_motor.init__motor_PID(LIFT_POS_KP,LIFT_POS_KI,LIFT_POS_KD,LIFT_VELT_KP,LIFT_POS_KI,LIFT_POS_KD);
//// 
//   slide_motor.init__motor_encoder(SLIDE_GEAR_RATIO,ENCODER_NUMBER);
//	 lift_motor.init__motor_encoder(LIFT_GEAR_RATIO,ENCODER_NUMBER);
	 
	 
	 slide_motor.encoder.siGearRatio= SLIDE_GEAR_RATIO;
	  slide_motor.encoder.siNumber = ENCODER_NUMBER;
		 lift_motor.encoder.siGearRatio= LIFT_GEAR_RATIO;
	  lift_motor.encoder.siNumber = ENCODER_NUMBER;
		slide_motor.pos_pid.fpTs = 0.001;
		slide_motor.pos_pid.fpTs = 0.001;
		
		lift_motor.velt_pid.fpTs = 0.001;
		lift_motor.velt_pid.fpTs = 0.001;
		
 };
		
		void slide_to_aim(void);
	  void lift_to_aim(void);

//		void drive_sucker(void)
//		{
//			if(!Toggle_sucker)
//				 GPIO_ResetBits(GPIOF,GPIO_Pin_8);  //蜂鸣器对应引脚GPIOG7拉低， 
//			else
//				 GPIO_SetBits(GPIOF,GPIO_Pin_8);  //蜂鸣器对应引脚GPIOG7拉低， 
//		};
	
};


#endif