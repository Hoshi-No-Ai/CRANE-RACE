#ifndef MOTOT_ALL
#define MOTOT_ALL

#include "motor_drive.h"
#include "fetch_rings.h"
#include "gimbal_driver.h"
#include "open_close_motor.h"
#include "air_operated_board.h"
#include "friction_belt_motor.h"
#include "stm32f4xx_it.h"
#include "action_task.h"
class DEBUG
{
	public:
		fp32 pitch;
		fp32 yaw;
		s16 friction_belt;
		fp32 left_element;
		fp32 right_element;
		u8 ball;
};

class des
{
	public:
		fp32 YAW;
		fp32 PITCH;
    fp32 FRICTION;
	  fp32 TURN;
	  fp32 SLIDE;
	  fp32 PRE_YAW;
	  fp32 Sight_YAW;
	  fp32 left_fetch;
	  fp32 right_fetch;
		
	  des(fp32 yaw,fp32 pitch,fp32 friction,fp32 turn,fp32 slide,fp32 fre_yaw,fp32 sight_yaw)
	{
		
		this->YAW=yaw;
		this->PITCH=pitch;
		this->FRICTION=friction;
		this->TURN=turn;
		this->SLIDE=slide;
    this->PRE_YAW=fre_yaw;
		this->Sight_YAW = sight_yaw;
	}
	
};

typedef enum
{
	SAFE,
	GIMBAL_YAW,
	GIMBAL_PITCH,
	OPEN_CLOSE,
	FRICTION,
	FETCH,
}MOTOR_MODE;


extern int init_flag;
void yaw_control(void);
void pitch_control(void);
void open_close_control(void);
void friction_belt_control(void);
void init_all_motor(void);


extern des DES;

void motor_crl_task(void);

#endif
