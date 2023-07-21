#ifndef __GIMBAL_CAL_TASK_H__
#define __GIMBAL_CAL_TASK_H__

#include "uart_protocol.h"
#include "action_task.h"



#define PI 3.1415926535
 
#define X1_ball_base 5920
#define Y1_ball_base 2920
#define BALL_DISTANCE 299
#define PITCH_HEIGHT  115.61

#define h_pitch_motor 0.27715
#define L_pitch_motor 0.2065
#define m_ball 0.21
#define p_air 1.20		//空气密度(单位kg/m3)
#define D_BALL  0.15
#define Cd 0.3			//空阻系数
#define CL 1.23			//升力系数
#define gravity 9.8		//重力加速度

#define gear_magn     1.682
#define gear_zero     335

#define DT35_X1 DT_data.s[0]
#define DT35_X2 DT_data.s[1]
#define DT35_Y1 DT_data.s[2]
#define DT35_Y2 DT_data.s[3]

typedef enum
{
	RIGHT,
	LEFT,
}BALL_POSITION;

extern BALL_POSITION ball_position;


class fetch_parameter
{
	public:
		float send_test; 
		float pitch_test; 
	
	private:
		u16 pht;
		float pz;
		u16 xbb;
		u16 ybb;
		u16 ball_distance;
		float gear_m;
		float gear_z;
};



void yaw_to_angle(u8 ball_num);
void pitch_to_angle(u8 ball_num);
void send_to_length(u8 ball_num);
void fetch_test(u8 ball_num);

#endif
