#include "gimbal_cal_task.h"

BALL_POSITION ball_position; 

void yaw_to_angle(u8 ball_num)
{ 
	if(ball_num < 6  && ball_num > 0)
	{
		DES.YAW = atan2f(X1_ball_base - DT35_X1,Y1_ball_base + ball_num * BALL_DISTANCE - DT35_Y1)*180/(float)PI - 180 + view_t.fl[2];
	}
	else if(ball_num == 6)
		DES.YAW = atan2f(X1_ball_base - DT35_X2,Y1_ball_base + ball_num * BALL_DISTANCE - DT35_Y1)*180/(float)PI - 180 + view_t.fl[2];
}

void pitch_to_angle(u8 ball_num)
{ 
	if(ball_num < 6  && ball_num > 0)
	{
	DES.PITCH = -atan((float)PITCH_HEIGHT/sqrt((float)(X1_ball_base - DT35_X2)*(X1_ball_base - DT35_X2) 
							 + (Y1_ball_base + ball_num * BALL_DISTANCE - DT35_Y1)*(Y1_ball_base + ball_num * BALL_DISTANCE - DT35_Y1)))
							 / (float )PI * 180 + 21.0f + DES.debug.pitch;
	}
	else if(ball_num == 6)
	DES.PITCH = -atan((float)PITCH_HEIGHT/sqrt((float)(X1_ball_base - DT35_X1)*(X1_ball_base - DT35_X1) 
							 + (Y1_ball_base + ball_num * BALL_DISTANCE - DT35_Y1)*(Y1_ball_base + ball_num * BALL_DISTANCE - DT35_Y1)))
							 / (float )PI * 180 + 21.0f + DES.debug.pitch;
}

u16 send_3;
s8 pitch_3;

void send_to_length(u8 ball_num)
{ 
	if(ball_num < 6  && ball_num > 0)
	{
	DES.SEND = sqrt((float)(X1_ball_base - DT35_X1)*(X1_ball_base - DT35_X1) 
							 + (Y1_ball_base + ball_num * BALL_DISTANCE - DT35_Y1)*(Y1_ball_base + ball_num * BALL_DISTANCE - DT35_Y1) 
							 + PITCH_HEIGHT*PITCH_HEIGHT) * (float)gear_magn
							 - gear_zero;
	}
	else if(ball_num == 6)
	{
		DES.SEND = sqrt((float)(X1_ball_base - DT35_X2)*(X1_ball_base - DT35_X2) 
							 + (Y1_ball_base + ball_num * BALL_DISTANCE - DT35_Y1)*(Y1_ball_base + ball_num * BALL_DISTANCE - DT35_Y1) 
							 + PITCH_HEIGHT*PITCH_HEIGHT) * (float)gear_magn
							 - gear_zero;
	}
}

void fetch_test(u8 ball_num)
{
	DES.YAW = atan((float)(FETCH_PARAMRTER.x_ball_base - ball_num * BALL_DISTANCE + ball_position * FETCH_PARAMRTER.ball_rack_distance - DT35_X1)/
									(DT35_Y2 - FETCH_PARAMRTER.y_ball_base))*180/(float)PI + view_t.fl[2];	
	if(abs(DES.YAW) < 12)
	{
		DES.SEND =sqrt((float)(FETCH_PARAMRTER.x_ball_base - ball_num * FETCH_PARAMRTER.ball_distance + ball_position * FETCH_PARAMRTER.ball_rack_distance - DT35_X1)*
									(FETCH_PARAMRTER.x_ball_base - ball_num * FETCH_PARAMRTER.ball_distance + ball_position * FETCH_PARAMRTER.ball_rack_distance - DT35_X1) + 
									(FETCH_PARAMRTER.y_ball_base - DT35_Y2)*(FETCH_PARAMRTER.y_ball_base
									 - DT35_Y2) + FETCH_PARAMRTER.pitch_height*FETCH_PARAMRTER.pitch_height)
									* (float)FETCH_PARAMRTER.gear_k - FETCH_PARAMRTER.gear_b;
	  DES.PITCH = atan((float)PITCH_HEIGHT/sqrt((float)(FETCH_PARAMRTER.x_ball_base - ball_num * FETCH_PARAMRTER.ball_distance + ball_position * FETCH_PARAMRTER.ball_rack_distance
										- DT35_X1)*(FETCH_PARAMRTER.x_ball_base - ball_num * FETCH_PARAMRTER.ball_distance + ball_position * FETCH_PARAMRTER.ball_rack_distance - DT35_X1) + 
										(FETCH_PARAMRTER.y_ball_base - DT35_Y2)*(FETCH_PARAMRTER.y_ball_base
										 - DT35_Y2))) / (float )PI * 180 + FETCH_PARAMRTER.pitch_zero;
	}
	else
	{
		DES.SEND = 100;
		DES.PITCH = 2;
	}
	if((ball_position == LEFT) && ball_num == 3)
	{
		send_3 = DES.SEND;
		pitch_3 = DES.PITCH;
	}
	if((ball_position == LEFT) && (ball_num < 3))
	{
		DES.SEND = send_3;
		DES.PITCH = pitch_3;
	}
							 
	FETCH_PARAMRTER.pitch_test = DES.PITCH;
	FETCH_PARAMRTER.send_test = DES.SEND;
	FETCH_PARAMRTER.yaw_test = DES.YAW;
}

