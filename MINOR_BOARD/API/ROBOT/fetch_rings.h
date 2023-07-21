#ifndef __FETCH_RINGS_H__
#define __FETCH_RINGS_H__
#include "motor_drive.h"
#include "pid_algorithm.h"
#include "motor_all.h"
#include "action_task.h"



#define ENCODER_NUMBER 8191
#define SLIDE_GEAR_RATIO (19.f) //3508减速比3591/187，小轮大轮齿比38:21
#define FLEFT_GEAR_RATIO (36.f) 
#define FRIGHT_GEAR_RATIO (36.f) 
#define TURN_GEAR_RATIO (53.0f/21.0f)




//TURN_DM
#define TURN_DM_POS_KP     30
#define TURN_DM_VELT_KD    0.5
#define TURN_DM_VELT_VELT  0
#define TURN_DM_POS_POS    0
#define TURN_DM_FF         0
#define TURN_DM_GEAR_RATIO (53.0f/21.0f)
#define TURN_DM_R          800
#define TURN_DM_H          0.001

//slide_3508
#define SLIDE_VELT_KP 	80.0f
#define SLIDE_VELT_KI 	0.f
#define SLIDE_VELT_KD 	0.f
#define SLIDE_VELT_UpMax	10000.0f
#define SLIDE_VELT_EiMax	5000.0f
#define SLIDE_VELT_SumEMax	5000.0f
#define SLIDE_VELT_UdMax	5000.0f
#define SLIDE_VELT_UMax	10000.0f

#define SLIDE_POS_KP 	23.0f
#define SLIDE_POS_KI 	0.05f
#define SLIDE_POS_KD 	0.f
#define SLIDE_POS_UpMax	7500.0f//45000.0f//45000.0f
#define SLIDE_POS_EiMax	0.5f//45000.0f//45000.0f
#define SLIDE_POS_SumEMax	7500.0f//45000.0f//45000.0f
#define SLIDE_POS_UdMax	75000.0f//45000.0f//45000.0f
#define SLIDE_POS_UMax	8000.0f//45000.0f//45000.0f

#define SLIDE_TD_R 800.0f
#define SLIDE_TD_H 0.0005f

//BOMB 2008
#define FLEFT_VELT_KP 	80.0f
#define FLEFT_VELT_KI 	0.f
#define FLEFT_VELT_KD 	0.f
#define FLEFT_VELT_UpMax	10000.0f
#define FLEFT_VELT_EiMax	5000.0f
#define FLEFT_VELT_SumEMax	5000.0f
#define FLEFT_VELT_UdMax	5000.0f
#define FLEFT_VELT_UMax	10000.0f

#define FLEFT_POS_KP 	23.0f
#define FLEFT_POS_KI 	0.05f
#define FLEFT_POS_KD 	0.f
#define FLEFT_POS_UpMax	7500.0f//45000.0f//45000.0f
#define FLEFT_POS_EiMax	0.5f//45000.0f//45000.0f
#define FLEFT_POS_SumEMax	7500.0f//45000.0f//45000.0f
#define FLEFT_POS_UdMax	75000.0f//45000.0f//45000.0f
#define FLEFT_POS_UMax	8000.0f//45000.0f//45000.0f

#define FLEFT_TD_R 800.0f
#define FLEFT_TD_H 0.0005f



//BOMB 2008
#define FRIGHT_VELT_KP 	80.0f
#define FRIGHT_VELT_KI 	0.f
#define FRIGHT_VELT_KD 	0.f
#define FRIGHT_VELT_UpMax	10000.0f
#define FRIGHT_VELT_EiMax	5000.0f
#define FRIGHT_VELT_SumEMax	5000.0f
#define FRIGHT_VELT_UdMax	5000.0f
#define FRIGHT_VELT_UMax	10000.0f

#define FRIGHT_POS_KP 	23.0f
#define FRIGHT_POS_KI 	0.05f
#define FRIGHT_POS_KD 	0.f
#define FRIGHT_POS_UpMax	7500.0f//45000.0f//45000.0f
#define FRIGHT_POS_EiMax	0.5f//45000.0f//45000.0f
#define FRIGHT_POS_SumEMax	7500.0f//45000.0f//45000.0f
#define FRIGHT_POS_UdMax	75000.0f//45000.0f//45000.0f
#define FRIGHT_POS_UMax	8000.0f//45000.0f//45000.0f

#define FRIGHT_TD_R 800.0f
#define FRIGHT_TD_H 0.0005f



//模型参数
#define INITIAL_STATIC_X 0
#define INITIAL_STATIC_Y 251.6368f //mm
#define start_x -373.f //mm
#define start_y 0

#define turn_length 450.f //mm
#define THETA     64.45f//圆弧段的初始角度
#define START_H   377.f//mm
#define MID_H     105.f//mm
#define END_H     -190.f//mm
#define W          0.1 //度/ms
#define V          0.1 //mm

#define init_h    251.6368f  //最初的位置

#define TRANSLATE_L   -127//MM
#define TRANSLATE_L2  50//MM


typedef enum
{
	no_start,
	take_ring_part1,
	translation1,
  translation2,
	take_ring_part2,
	wait,
	take_ring_part3,
	take_over,
	put_ring_part1,
	put_ring_part2,
	translation3,
	put_ring_part3,
	put_over,
	hold_on,
	reset,
}take_ring_state;



class cRingMotor
{
public:
	cMotor_double_td slide_motor;
  cMotor_HT_gimbal turn_motor;
  cMotor_double_td left_motor,right_motor;
  take_ring_state Take_state;
  float take_ring_time;

  float L;          //臂长
  float theta0;     //初始角度
  float theta; //圆弧段角度
  float delta_x;   //一个时间内走的距离
  float start_h;   //直线截至位置高度
  float mid_h;     //第二段直线的起始高度
  float end_h;    //终点高度

  float w;       //圆弧段的角速度
  float v;       //直线段的速度

  float translate_l; //平移长度
  float translate_2; //往回平移长度
  

  stPoint start_system;  //移动坐标系的初始位置
  stPoint static_system; //静止坐标系下的末端坐标
  stPoint move_system;   //移动坐标系下的走圆弧时的原点坐标

  //解三角
  

	cRingMotor() {}
	cRingMotor(
		float slide_vKp, float slide_vKi, float slide_vKd, float slide_vUpMax, float slide_vEiMax, float slide_vSumEMax, float slide_vUdMax, float slide_vEMin, float slide_vEMax,
		float slide_pKp, float slide_pKi, float slide_pKd, float slide_pUpMax, float slide_pEiMax, float slide_pSumEMax, float slide_pUdMax, float slide_pEmin, float slide_pEMax,
		float slide_gr, int slide_num,
		float slide_R, float slide_H,
		  
		float p_pos, float v_velt, float pKp, float vKd, float gr, float ff, float R_p,float H_p,
	

		float ts,
		float x0,float y0,float x2,float y2,
		float l,float theta, float h,float h1,float h2,float w,float v,float tl,float tl2,
			
		float left_vKp, float left_vKi, float left_vKd, float left_vUpMax, float left_vEiMax, float left_vSumEMax, float left_vUdMax, float left_vEMin, float left_vEMax,
		float left_pKp, float left_pKi, float left_pKd, float left_pUpMax, float left_pEiMax, float left_pSumEMax, float left_pUdMax, float left_pEmin, float left_pEMax,
		float left_gr, int left_num,
		float left_R, float left_H,
			
		float right_vKp, float right_vKi, float right_vKd, float right_vUpMax, float right_vEiMax, float right_vSumEMax, float right_vUdMax, float right_vEMin, float right_vEMax,
		float right_pKp, float right_pKi, float right_pKd, float right_pUpMax, float right_pEiMax, float right_pSumEMax, float right_pUdMax, float right_pEmin, float right_pEMax,
		float right_gr, int right_num,
		float right_R, float right_H
			
		)
	{
		slide_motor = cMotor_double_td(slide_vKp, slide_vKi, slide_vKd, slide_vUpMax, slide_vEiMax, slide_vSumEMax, slide_vUdMax, slide_vEMin, slide_vEMax,
									               slide_pKp, slide_pKi, slide_pKd, slide_pUpMax, slide_pEiMax, slide_pSumEMax, slide_pUdMax, slide_pEmin, slide_pEMax,
									               slide_gr, slide_num, slide_R, slide_H, ts);
		left_motor = cMotor_double_td(left_vKp, left_vKi, left_vKd, left_vUpMax, left_vEiMax, left_vSumEMax, left_vUdMax, left_vEMin, left_vEMax,
									               left_pKp, left_pKi, left_pKd, left_pUpMax, left_pEiMax, left_pSumEMax, left_pUdMax, left_pEmin, left_pEMax,
									               left_gr, left_num, left_R, left_H, ts);
		
		right_motor = cMotor_double_td(right_vKp, right_vKi, right_vKd, right_vUpMax, right_vEiMax, right_vSumEMax, right_vUdMax, right_vEMin, right_vEMax,
									               right_pKp, right_pKi, right_pKd, right_pUpMax, right_pEiMax, right_pSumEMax, right_pUdMax, right_pEmin, right_pEMax,
									               right_gr, right_num, right_R, right_H, ts);
		
		
		turn_motor = cMotor_HT_gimbal(p_pos, v_velt, pKp, vKd, gr, ff, R_p, H_p);
		this->start_system.x = x0;
		this->start_system.y = y0;
		this->static_system.x= x2;
		this->static_system.y= y2;
		this->L = l;
		this->theta0 = theta;
		this->start_h= h;
		this->mid_h=h1;
		this->end_h=h2;
		this->w = w;
    this->v = v;
		this->translate_l = tl;
		this->translate_2 = tl2;
	
	}
	
	void slide_to_aim(void);
	void turn_to_aim(void) ;
	void cRingMotor::interpolation(void);
	void cRingMotor::path_planning(void);
	
	
	void left_to_aim(void);
	void right_to_aim(void);
	
	float tri_angle(float b,float c);
	




};

extern cRingMotor fetch_rings_motor;
#endif



