#ifndef __FRICTION_BELT_MOTOR_H__
#define __FRICTION_BELT_MOTOR_H__
#include "motor_all.h"
#include "motor_drive.h"
#include "filter_algorithm.h"
#include "action_task.h"
#include "smc_algorithm.h"


//left
#define LEFT_VELT_KP 450//70//4   //500
#define LEFT_VELT_KI 0.35//0.02//0.005
#define LEFT_VELT_KD 0
#define LEFT_VELT_UMax 100000
#define LEFT_VELT_SUMEMAX 30000
#define LEFT_VELT_EMIN  1
#define LEFT_VELT_EMAX 100
#define LEFT_GEAR_RATIO 19

extern ST_LPF friction_lpf;


//right
#define RIGHT_VELT_KP  370//50    //2.8     //500
#define RIGHT_VELT_KI  0.25//0.04   //0.0015
#define RIGHT_VELT_KD 0
#define RIGHT_VELT_UMax 100000
#define RIGHT_VELT_SUMEMAX 45000
#define RIGHT_VELT_EMIN 2
#define RIGHT_VELT_EMAX 100
#define RIGHT_GEAR_RATIO 19




class cFriction_belt_Motor
{
	public:
		cMotor_single friction_belt_left, friction_belt_right;
	  cFriction_belt_Motor(){}
		cFriction_belt_Motor(float left_Kp, float left_Ki, float left_Kd, float left_UpMax, float left_EiMax, float left_SumEMax, 
			                   float left_UdMax, float left_EMin, float left_EMax,float left_gr, int32_t left_num, 
													float right_Kp, float right_Ki, float right_Kd, float right_UpMax, float right_EiMax, float right_SumEMax, 
			                   float right_UdMax, float right_EMin, float right_EMax,float right_gr, int32_t right_num )

	 {
		 friction_belt_left=cMotor_single( left_Kp,  left_Ki,  left_Kd,  left_UpMax,  left_EiMax,  left_SumEMax, 
		        left_UdMax,  left_EMin,  left_EMax, left_gr,  left_num);
		 friction_belt_right=cMotor_single( right_Kp,  right_Ki,  right_Kd,  right_UpMax,  right_EiMax,  right_SumEMax,
                                		 right_UdMax,  right_EMin,  right_EMax, right_gr,  right_num);
	 }

	 
	 
		void friction_belt_to_aim();
    void friction_speed_detection(float threshold_1,float threshold_2);
	
};



extern cFriction_belt_Motor friction_belt_motor;
extern float feedforward;	

extern float current_A;
extern float current_B;
//SMC
#define FRICTION_LEFT_EP 50
#define FRICTION_LEFT_Q  20.0f
#define FRICTION_LEFT_C 70 
#define FRICTION_LEFT_UMAX 60000
#define FRICTION_LEFT_UMIN -60000
#define FRICTION_LEFT_OFF 20.f


#define FRICTION_RIGHT_EP 50
#define FRICTION_RIGHT_Q  14.f
#define FRICTION_RIGHT_C 120
#define FRICTION_RIGHT_UMAX 60000 
#define FRICTION_RIGHT_UMIN -60000
#define FRICTION_RIGHT_OFF 90.f

class SMC_cFriction_belt_Motor
{
	public:
		cSMC friction_belt_left;
	  cSMC friction_belt_right;
	
	
	  SMC_cFriction_belt_Motor(){}
		SMC_cFriction_belt_Motor(float friction_belt_left_EP, float friction_belt_left_Q, float friction_belt_left_C,
                        		 float friction_belt_left_UMAX,float friction_belt_left_UMIN,
		                         float left_off, 
		                         float friction_belt_right_EP,float friction_belt_right_Q,float friction_belt_right_C,
														 float friction_belt_right_UMAX,float friction_belt_right_UMIN,
                             float right_off,
														 float ts)	
	{
			friction_belt_left = cSMC(  friction_belt_left_EP,  friction_belt_left_Q,  friction_belt_left_C,  
		                            friction_belt_left_UMAX,  friction_belt_left_UMIN,
		                          left_off,  ts);
		  friction_belt_right = cSMC(  friction_belt_right_EP,  friction_belt_right_Q,  friction_belt_right_C, 
                           		friction_belt_right_UMAX,  friction_belt_right_UMIN,
		                          right_off,  ts);
	}													 
	  void SMC_friction_belt_to_aim();
	  void SMC_friction_speed_detection(float threshold_1,float threshold_2);
};

extern SMC_cFriction_belt_Motor SMC_friction_belt_motor;

extern float SMC_feedforward1;
extern float SMC_feedforward2;


#endif



