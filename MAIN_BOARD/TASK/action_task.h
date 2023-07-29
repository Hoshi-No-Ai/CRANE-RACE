#ifndef __TEMP_CONTROL_TASK_H__
#define __TEMP_CONTROL_TASK_H__

#include "read_remote_ctrl_task.h"
#include "sucker.h"
#include "table.h"

#define LIMIT_DELTA_X 10.0f
#define LIMIT_DELTA_Y 10.0f
#define LIMIT_DELTA_Q 0.5f

enum action_pattern_e
{
    ACTION_INIT,
    ACTION_POS_1,
    ACTION_POS_2,
    ACTION_POS_3,
    ACTION_POS_4,
    ACTION_POS_5,
    ACTION_POS_6,
    ACTION_POS_END,
    ACTION_PUT,
    ACTION_FETCH,
    ACTION_POS_CHECK,
    ACTION_POS_CHANGE,
    ACTION_NONE
};

enum fetch_pattern_e
{
    FETCH_INIT,
    FETCH_AWAIT,
    FETCH_GET,
    FETCH_LOSE,
    FETCH_MOVE,
    FETCH_GET_PRE
};

enum BOX_STATE
{
    none,
    await,
    get_state1,
    get_state2,
    get_state3,
    lose_state0,
    lose_state1,
    lose_state2
};

struct GET_NUM
{
    uint8_t box;
    uint8_t cola;
	
		GET_NUM()
		{
			box=1;
			cola=1;
		}
		
		~GET_NUM(){}
};

extern action_pattern_e action_pattern;
extern fetch_pattern_e fetch_pattern;
extern BOX_STATE box_state;
extern int init_motor;
extern int this_target;
extern GET_NUM target_num;

extern float task_time;
extern int _servo_degree;

void robot_movement(void);
void movement_check(bool if_auto);
void position_check(void);

void handle_box(void);

#endif
