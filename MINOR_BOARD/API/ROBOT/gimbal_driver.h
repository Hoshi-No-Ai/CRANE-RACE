#ifndef __GIMBAL_DRIVER_H__
#define __GIMBAL_DRIVER_H__
#include "motor_all.h"
#include "motor_drive.h"
#include "math_algorithm.h"

#define ENCODER_NUMBER 8191


#define ENCODER_NUMBER 8191


//YAW HT04

#define YAW_HT_POS_KP     500
#define YAW_HT_VELT_KD    5
#define YAW_HT_VELT_VELT  0
#define YAW_HT_POS_POS    0
#define YAW_HT_FF         0
#define YAW_HT_GEAR_RATIO 1
#define YAW_HT_R          800
#define YAW_HT_H          0.001
//PITCH HT04
#define PITCH_HT_POS_KP     350
#define PITCH_HT_VELT_KD    4
#define PITCH_HT_VELT_VELT  0
#define PITCH_HT_POS_POS    0
#define PITCH_HT_FF         0
#define PITCH_HT_GEAR_RATIO 1
#define PITCH_HT_R          500
#define PITCH_HT_H          0.001

#define PITCH_Ki   0.008f
#define PITCH_SumEmax 800.f
#define PITCH_Emax  3.f

#define YAW_Ki   0.001f
#define YAW_SumEmax 2000.f
#define YAW_Emax  2.f






class cGimbalMotor
{
public:
	cMotor_HT_gimbal yaw_motor;
  cMotor_HT_gimbal pitch_motor;
  float fpE1,fpE2;
  float fpSumE1,fpSumE2;
  float Ki1,Ki2;
  float fpSumEmax1,fpSumEmax2;
  float fpEmax1,fpEmax2;
	cGimbalMotor() {}
	cGimbalMotor( fp32 p_pos1, fp32 v_velt1, fp32 pKp1, fp32 vKd1, fp32 gr1, fp32 ff1, fp32 R_p1,fp32 H_p1,
								float ki1,float SumEmax1,float Emax1,
                fp32 p_pos2, fp32 v_velt2, fp32 pKp2, fp32 vKd2, fp32 gr2, fp32 ff2, fp32 R_p2,fp32 H_p2,
		            float ki2,float SumEmax2,float Emax2
              		)
	{
		yaw_motor =  cMotor_HT_gimbal(p_pos1, v_velt1, pKp1, vKd1, gr1, ff1, R_p1, H_p1);
		this->Ki1 = ki1;
		this->fpSumEmax1 = SumEmax1;
		this->fpEmax1 = Emax1;		
		pitch_motor =  cMotor_HT_gimbal(p_pos2, v_velt2, pKp2, vKd2, gr2, ff2, R_p2, H_p2);
		this->Ki2 = ki2;
		this->fpSumEmax2 = SumEmax2;
		this->fpEmax2 = Emax2;
	}
	void yaw_to_aim();
	void pitch_to_aim();
};

extern cGimbalMotor Gimbal_motor;


#endif
