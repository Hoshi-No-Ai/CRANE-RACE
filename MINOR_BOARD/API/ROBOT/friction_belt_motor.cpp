
#include "friction_belt_motor.h"
#include "uart_protocol.h"
ST_LPF friction_left_lpf={0,0,0,1200,0.001};
ST_LPF friction_right_lpf={0,0,0,1200,0.001};
float current_A=0;
float current_B=0;
float feedforward=165;
float TEMPfeedforward=130;
float SMC_feedforward1=0;
float SMC_feedforward2 =0;
#define samp_time 0.001
#define off 1


cFriction_belt_Motor friction_belt_motor{
	/*velt_pid*/			LEFT_VELT_KP, LEFT_VELT_KI, LEFT_VELT_KD, LEFT_VELT_UMax, 200, LEFT_VELT_SUMEMAX, 0, LEFT_VELT_EMIN, LEFT_VELT_EMAX, 
	/*encoder*/				LEFT_GEAR_RATIO, 8191,
	
	/*velt_pid*/			RIGHT_VELT_KP, RIGHT_VELT_KI, RIGHT_VELT_KD, RIGHT_VELT_UMax, 300, RIGHT_VELT_SUMEMAX, 0, RIGHT_VELT_EMIN, RIGHT_VELT_EMAX, 
	/*encoder*/				RIGHT_GEAR_RATIO, 8191,
	
};
void cFriction_belt_Motor::friction_belt_to_aim()
{
	friction_belt_motor.friction_belt_left.velt_pid.fpDes=ClipFloat(DES.FRICTION,0,500);
	friction_belt_motor.friction_belt_right.velt_pid.fpDes=ClipFloat(-DES.FRICTION,-500,0);

	friction_belt_motor.friction_belt_left.velt_pid.CalISeparatedPID();
	friction_belt_motor.friction_belt_right.velt_pid.CalISeparatedPID();
	
	friction_left_lpf.in = friction_belt_motor.friction_belt_left.velt_pid.fpU ;
	friction_right_lpf.in=friction_belt_motor.friction_belt_right.velt_pid.fpU;
//pid¾ÀÆ«
//	friction_left_lpf.in = friction_belt_motor.friction_belt_left.velt_pid.fpU+feedforward ;
//	friction_right_lpf.in=friction_belt_motor.friction_belt_right.velt_pid.fpU+feedforward ;
	LpFilter(&friction_left_lpf);
	LpFilter(&friction_right_lpf);
	current_A = friction_left_lpf.out;
	current_B= friction_right_lpf.out;
//	friction_belt_motor.friction_belt_left.pid_current=friction_belt_motor.friction_belt_left.velt_pid.fpU;
//	friction_belt_motor.friction_belt_right.pid_current=friction_belt_motor.friction_belt_right.velt_pid.fpU;
	
	friction_belt_motor.friction_belt_left.pid_current=0.3*current_A;
	friction_belt_motor.friction_belt_right.pid_current=0.3*current_B;
	
}

//threshold_1±ØÐëÐ¡ÓÚthreshold_2

void cFriction_belt_Motor::friction_speed_detection(float threshold_1,float threshold_2)
{
	if(friction_belt_motor.friction_belt_left.velt_pid.fpFB <= threshold_1 )
	{
		G_flag.detection_flag = 1;
	}
	if( G_flag.detection_flag == 1 && friction_belt_motor.friction_belt_left.velt_pid.fpFB >= threshold_2)
	{
		friction_belt_motor.friction_belt_left.velt_pid.fpSumE =0;
		friction_belt_motor.friction_belt_right.velt_pid.fpSumE =0;
		feedforward = TEMPfeedforward;
		friction_left_lpf.preout=TEMPfeedforward;
		friction_left_lpf.preout=TEMPfeedforward;
		DES.FRICTION = threshold_2;
		G_flag.detection_flag = 0;
	}
}


SMC_cFriction_belt_Motor SMC_friction_belt_motor
{
	FRICTION_LEFT_EP,FRICTION_LEFT_Q,FRICTION_LEFT_C,FRICTION_LEFT_UMAX,FRICTION_LEFT_UMIN,FRICTION_LEFT_OFF,
	FRICTION_RIGHT_EP,FRICTION_RIGHT_Q,FRICTION_RIGHT_C,FRICTION_RIGHT_UMAX,FRICTION_RIGHT_UMIN,FRICTION_RIGHT_OFF,
	0.001f
};


void SMC_cFriction_belt_Motor::SMC_friction_belt_to_aim(void)
{
	friction_belt_left.td_smc.aim = ClipFloat(DES.FRICTION,0,500);
	friction_belt_right.td_smc.aim = ClipFloat(-DES.FRICTION,-500,0);
	rate_monitor.temp_rate[11]++;
	friction_belt_left.CalSMC();
	friction_belt_right.CalSMC();
	
  
	
	friction_belt_left.real_current = (friction_belt_left.realfpU+SMC_feedforward1);
	friction_belt_right.real_current = (friction_belt_right.realfpU+SMC_feedforward2);

}

void SMC_cFriction_belt_Motor::SMC_friction_speed_detection(float threshold_1,float threshold_2)
{
	if( friction_belt_left.FB<= threshold_1 )
	{
		G_flag.detection_flag = 1;
	}
	if( G_flag.detection_flag == 1 && friction_belt_left.FB >= threshold_2)
	{
		
		feedforward = TEMPfeedforward;
		friction_left_lpf.preout=TEMPfeedforward;
		friction_left_lpf.preout=TEMPfeedforward;
		DES.FRICTION = threshold_2;
		G_flag.detection_flag = 0;
	}
}





