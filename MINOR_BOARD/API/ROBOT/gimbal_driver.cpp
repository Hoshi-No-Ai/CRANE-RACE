#include "gimbal_driver.h"
#include "uart_protocol.h"
#include "math.h"
#define pi 3.1415926
int flag_one=0;
int ukk1=1,ukk2=1;

cGimbalMotor Gimbal_motor
{
	
	
	//yaw_motor
	/*aim*/     		YAW_HT_POS_POS, YAW_HT_VELT_VELT,
	/*pos,velt*/    YAW_HT_POS_KP, YAW_HT_VELT_KD,
	/*feed_forward*/YAW_HT_FF,
	/*encoder*/     YAW_HT_GEAR_RATIO,
									YAW_HT_R, YAW_HT_H,
									YAW_Ki,YAW_SumEmax,YAW_Emax,
//pitch_motor
	/*aim*/     		PITCH_HT_POS_POS, PITCH_HT_VELT_VELT,
	/*pos,velt*/    PITCH_HT_POS_KP, PITCH_HT_VELT_KD,
	/*feed_forward*/PITCH_HT_FF,
	/*encoder*/     PITCH_HT_GEAR_RATIO,
									PITCH_HT_R, PITCH_HT_H,
	                PITCH_Ki,PITCH_SumEmax,PITCH_Emax
	
	
	
};

fp32 K =1.0f;


void cGimbalMotor::yaw_to_aim()
{
	  yaw_motor.td.aim = ClipFloat(DES.YAW, -40,40);
		yaw_motor.td.TD_Function();
		yaw_motor.HT_control.des_p = yaw_motor.td.x1;
	  yaw_motor.HT_control.des_v = yaw_motor.td.x2 ;
	
		fpE1 = yaw_motor.HT_control.des_p-yaw_motor.encoder_HT.pos;
	  if(abs(fpE1) >= PITCH_Emax)
		{
			ukk1=0;
		}
		else
		{
			ukk1=1;
			fpSumE1 += fpE1;
		}
	  fpSumE1 =ClipFloat(fpSumE1,-fpSumEmax1,fpSumEmax1);
	  yaw_motor.HT_control.ff = ukk1*Ki1*fpSumE1;
}

void cGimbalMotor::pitch_to_aim()
{
	  pitch_motor.td.aim = ClipFloat(DES.PITCH, 0,60);
		pitch_motor.td.TD_Function();
		pitch_motor.HT_control.des_p = pitch_motor.td.x1;
	  pitch_motor.HT_control.des_v = pitch_motor.td.x2;
	
	
	
	  fpE2 = pitch_motor.HT_control.des_p-pitch_motor.encoder_HT.pos;
	  if(abs(fpE2) >= PITCH_Emax)
		{
			ukk2=0;
		}
		else
		{
			ukk2=1;
			fpSumE2 += fpE2;
		}
	  fpSumE2 =ClipFloat(fpSumE2,-fpSumEmax2,fpSumEmax2);
	  pitch_motor.HT_control.ff = ukk2*Ki2*fpSumE2;
	  
	
} 











