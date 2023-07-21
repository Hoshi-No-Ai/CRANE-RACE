#include "fetch_rings.h"
#include "air_operated_board.h"

float g=600;
float temp_x =0 ;

cRingMotor fetch_rings_motor
{
		//slider_motor
	/*velt_pid*/			 SLIDE_VELT_KP,  SLIDE_VELT_KI, SLIDE_VELT_KD,  SLIDE_VELT_UpMax, SLIDE_VELT_EiMax, 
                     SLIDE_VELT_SumEMax, SLIDE_VELT_UdMax,  -SLIDE_VELT_UMax, SLIDE_VELT_UMax, 
										 
	/*pos_pid*/				 SLIDE_POS_KP,  SLIDE_POS_KI,  SLIDE_POS_KD,  SLIDE_POS_UpMax, SLIDE_POS_EiMax, 
                     SLIDE_POS_SumEMax, SLIDE_POS_UdMax,  -SLIDE_POS_UMax,  SLIDE_POS_UMax,
										 
	/*encoder*/				 SLIDE_GEAR_RATIO , ENCODER_NUMBER,
	/*td*/						 SLIDE_TD_R, SLIDE_TD_H,
	
	
	//turn_motor
	/*aim*/     		TURN_DM_POS_POS, TURN_DM_VELT_VELT,
	/*pos,velt*/    TURN_DM_POS_KP, TURN_DM_VELT_KD,
	/*feed_forward*/TURN_DM_FF,
	/*encoder*/     TURN_DM_GEAR_RATIO,
									TURN_DM_R, TURN_DM_H,
	                0.001,
	/*canshu*/      start_x,start_y,INITIAL_STATIC_X,INITIAL_STATIC_Y,
	                turn_length,THETA,START_H,MID_H,END_H,W,V,
	                TRANSLATE_L,TRANSLATE_L2,
									
	//left_motor
	/*velt_pid*/			 FLEFT_VELT_KP,  FLEFT_VELT_KI, FLEFT_VELT_KD,  FLEFT_VELT_UpMax, FLEFT_VELT_EiMax, 
                     FLEFT_VELT_SumEMax, FLEFT_VELT_UdMax,  -FLEFT_VELT_UMax, FLEFT_VELT_UMax, 
										 
	/*pos_pid*/				 FLEFT_POS_KP,  FLEFT_POS_KI,  FLEFT_POS_KD,  FLEFT_POS_UpMax, FLEFT_POS_EiMax, 
                     FLEFT_POS_SumEMax, FLEFT_POS_UdMax,  -FLEFT_POS_UMax,  FLEFT_POS_UMax,
										 
	/*encoder*/				 FLEFT_GEAR_RATIO , ENCODER_NUMBER,
	/*td*/						 FLEFT_TD_R, FLEFT_TD_H,
	
	//right_motor
	/*velt_pid*/			 FRIGHT_VELT_KP,  FRIGHT_VELT_KI, FRIGHT_VELT_KD,  FRIGHT_VELT_UpMax, FRIGHT_VELT_EiMax, 
                     FRIGHT_VELT_SumEMax, FRIGHT_VELT_UdMax,  -FRIGHT_VELT_UMax, FRIGHT_VELT_UMax, 
										 
	/*pos_pid*/				 FRIGHT_POS_KP,  FRIGHT_POS_KI,  FRIGHT_POS_KD,  FRIGHT_POS_UpMax, FRIGHT_POS_EiMax, 
                     FRIGHT_POS_SumEMax, FRIGHT_POS_UdMax,  -FRIGHT_POS_UMax,  FRIGHT_POS_UMax,
										 
	/*encoder*/				 FRIGHT_GEAR_RATIO , ENCODER_NUMBER,
	/*td*/						 FRIGHT_TD_R, FRIGHT_TD_H,
	
	
	
};

void cRingMotor::slide_to_aim(void)
{
	slide_motor.td.aim = ClipFloat(DES.SLIDE,0,190);
	slide_motor.td.TD_Function();
	slide_motor.pos_pid.fpDes = slide_motor.td.x1;
	slide_motor.pos_pid.CalComprehensivePID();
	slide_motor.velt_pid.fpDes = slide_motor.pos_pid.fpU;
	slide_motor.velt_pid.CalComprehensivePID();
	slide_motor.pid_current = slide_motor.velt_pid.fpU;
	
}



void cRingMotor::turn_to_aim(void)
{
	turn_motor.td.aim = ClipFloat(DES.TURN, 0, 180);
	turn_motor.td.TD_Function();
	turn_motor.HT_control.des_p = turn_motor.td.x1;
	turn_motor.HT_control.des_v = turn_motor.td.x2;
	
}
//P0 高度为h时的移动坐标系原点 P1时高度为h时相对基座的坐标
void cRingMotor::interpolation(void)
{
	
	float x1;
	switch(Take_state)
	{
		case take_ring_part1:
			  G_flag.get_ring_flag = 0;		 
		    move_system.x = - tri_angle(start_h,L);
				static_system.x=0;
		    static_system.y=init_h+v*take_ring_time;
		    if(abs(static_system.y - start_h) <= 0.05 )
				{
					Take_state = translation1;
					take_ring_time =0;
					theta = asin(start_h/L)/PI*180.f;
				}
		
				
		break;
		case translation1://平移
//			  move_system.x = - tri_angle(start_h,L)+translate_l;
		    static_system.x = -v * take_ring_time;
	      if(fabs(static_system.x - translate_l) <0.05 && G_flag.Is_first_fetch == 0)
				{
					Take_state = translation2;
					take_ring_time =0;
				}
				else if(fabs(static_system.x - translate_l) <0.05)
				{
					G_flag.Wait_flag =1;
					Take_state = wait;
					take_ring_time =0;
					theta = asin(start_h/L)/PI*180.f;
				}
			
		break;
		case translation2:
			static_system.x=translate_l+v*take_ring_time;
		   if(fabs(static_system.x -(translate_l + translate_2)) <0.05)
				{
					Take_state = take_ring_part2;
					take_ring_time =0;
				}
		
			break;
			
		case take_ring_part2://圆弧阶段
      move_system.x = - tri_angle(start_h,L)+translate_l+translate_2;
			theta = asin(start_h/L)/PI*180.f + w*take_ring_time;
			static_system.x = move_system.x+L*cos(theta/180.f*PI);
		  static_system.y = move_system.y + L*sin(theta/180.f*PI);
		  if(mid_h<0)
			{
				if(fabs(theta-(180+asin((-mid_h)/L)/PI*180.f)) < 0.04)
			{
				Take_state = wait;
				take_ring_time =0;
			}
			}
		  else
			{
				if(fabs(theta-(180-asin((mid_h)/L)/PI*180.f)) < 0.04)
				{
				Take_state = wait;
				take_ring_time =0;
				}	
			}
			break;
		case wait:
			take_ring_time =0;

			break;
		case take_ring_part3:
			static_system.y=mid_h-v*take_ring_time;
			if(fabs(static_system.y-end_h) < 0.0001)
			{
				Take_state = take_over;
				take_ring_time =0;
				
				
			}
		break;
		case take_over:

		G_flag.solenoid_valve = 5;
			if(take_ring_time >=1000)
			{
				Take_state = put_ring_part1;
				take_ring_time=0;
				
			}
	
		break;
		case put_ring_part1:
		  static_system.y=end_h+v*take_ring_time;
		  if(fabs(static_system.y - mid_h) <= 0.1)
			{
				  Take_state = put_ring_part2;
					take_ring_time =0;
				if(mid_h < 0)
				{
					 theta = 180+asin(-mid_h/L)/PI*180.f;
				}
				else
				{
					theta = 180-asin(mid_h/L)/PI*180.f;
				}
				 
			}
		break;
		case put_ring_part2:
			if(mid_h <0)
			{
				theta = 180+asin(-mid_h/L)/PI*180.f - w*take_ring_time;
			}
			else
			{
				theta = 180-asin(mid_h/L)/PI*180.f - w*take_ring_time;
			}
			
			static_system.x =  move_system.x+L*cos(theta/180.f*PI);
		  static_system.y = move_system.y + L*sin(theta/180.f*PI);
			
		
				  if(fabs(theta-asin(start_h/L)/PI*180.f)<0.04)
			{
				Take_state = translation3;
        take_ring_time =0;
			}
			
		
		
			break;
		case translation3:
			static_system.x=translate_l+translate_2+v*take_ring_time;
		   if(fabs(static_system.x ) <0.05)
				{
					Take_state = put_ring_part3;
					take_ring_time =0;
				}
		break;
		case put_ring_part3:
//			static_system.x = 0;
		  static_system.y = start_h-v*take_ring_time;
		  if(fabs(static_system.y- init_h) <= 0.04)
			{
				 Take_state = put_over;
					take_ring_time =0;
			}
			
		break;
		case put_over:
		  G_flag.solenoid_valve = 6;
		  
		  
		if(take_ring_time >= 2000)
		{
			Take_state = hold_on;
			take_ring_time = 0;
		}
		break;
		case hold_on:
			G_flag.Is_first_fetch = 1;
		 
		  if(take_ring_time >= 1000)
			{
			    Take_state = take_ring_part1;
				  take_ring_time = 0;
		      G_flag.get_ring_flag = 1;		  //取到环告诉下板
			}
			ktime=1;
		  ctime =1;
		  
		
		break;
		case reset:
			if(take_ring_time >= 2000)
			{
				Take_state = take_ring_part2;
				take_ring_time = 0;
				G_flag.Wait_flag = 0;
				
			}
		break;
			
		default:
			break;
	}
}
//start_system ：移动坐标的初始位置
//static——system ； 静止坐标系下末端坐标

void cRingMotor::path_planning(void)
{
	float turn_des;
	float slide_des;
	interpolation();

	if(Take_state == take_ring_part1 || Take_state == put_ring_part3 ||Take_state == translation1||Take_state == translation2||Take_state == translation3)
	{
		turn_des = asin(static_system.y/L)/PI*180.f;
	}
	else if(Take_state == take_ring_part2 || Take_state == put_ring_part2 )
	{
		turn_des = theta;
	}
	else if(Take_state == wait)
	{
		turn_des = theta;
	}
	else if (Take_state == take_ring_part3 || Take_state == put_ring_part1 )
	{
		if(static_system.y<0)
		{
			turn_des = 180+asin(-static_system.y/L)/PI*180.f;
		}
		else
		{
			turn_des = 180-asin(static_system.y/L)/PI*180.f;
		}
		
	}
	else if(Take_state == take_over)
	{
		turn_des = 180+asin(-end_h/L)/PI*180.f;
	}
	else if(Take_state == put_over)
	{
		turn_des = asin(init_h/L)/PI*180.f;
	}
 
 if(Take_state == hold_on)
 {
//	 	  DES.TURN = 52;
//		  DES.SLIDE = 0;
 }
 else if(Take_state == reset)
 {
//	    DES.TURN = 0;
//		  DES.SLIDE = 0;
 }
 else
 {
	DES.TURN = turn_des - 34.f;
	delta_x = static_system.x-start_system.x - L*cos(turn_des/180.f*PI);
  slide_des=delta_x;
	DES.SLIDE =slide_des;
 }
	
	
}

float cRingMotor::tri_angle(float b,float c)
{
	float a;
	a = sqrt(c*c-b*b);
	return a;
}
	
void cRingMotor::left_to_aim(void)
{
	left_motor.td.aim = ClipFloat(DES.left_fetch ,0,190);
	left_motor.td.TD_Function();
	left_motor.pos_pid.fpDes = left_motor.td.x1;
	left_motor.pos_pid.CalComprehensivePID();
	left_motor.velt_pid.fpDes = left_motor.pos_pid.fpU;
	left_motor.velt_pid.CalComprehensivePID();
	left_motor.pid_current = left_motor.velt_pid.fpU;
	
}



void cRingMotor::right_to_aim(void)
{
	left_motor.td.aim = ClipFloat(DES.right_fetch ,0,190);
	left_motor.td.TD_Function();
	left_motor.pos_pid.fpDes = left_motor.td.x1;
	left_motor.pos_pid.CalComprehensivePID();
	left_motor.velt_pid.fpDes = left_motor.pos_pid.fpU;
	left_motor.velt_pid.CalComprehensivePID();
	left_motor.pid_current = left_motor.velt_pid.fpU;
	
}