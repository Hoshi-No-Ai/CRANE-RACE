#ifndef __TEMP_CONTROL_TASK_H__
#define __TEMP_CONTROL_TASK_H__

#include "sucker.h"
#include "table.h"

enum action_pattern_e{
    ACTION_INIT,
    ACTION_POS_1,
    ACTION_POS_2,
    ACTION_POS_3,
    ACTION_POS_4,
    ACTION_POS_5,
    ACTION_POS_6,
    ACTION_POS_END,
    ACTION_PUT,
    ACTIO_FETCH
};

enum BOX_STATE{
	none,
	await,
	get_state1,
	get_state2,
	get_state3,
	lose_state0,
	lose_state1,
	lose_state2	
};

struct GET_NUM{
uint8_t box;
uint8_t cola;
};

extern action_pattern_e action_pattern;

void robot_movement(void);
void movement_check(bool if_auto);
	
void handle_box(void);

#endif
