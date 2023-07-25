#ifndef __TEMP_CONTROL_TASK_H__
#define __TEMP_CONTROL_TASK_H__

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

extern action_pattern_e action_pattern;

void robot_movement(void);
void movement_check(bool if_auto);

#endif
