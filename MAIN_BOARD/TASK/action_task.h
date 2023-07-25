#ifndef __TEMP_CONTROL_TASK_H__
#define __TEMP_CONTROL_TASK_H__

typedef enum {
	none,
	await,
	get_state1,
	get_state2,
	get_state3,
	lose_state0,
	lose_state1,
	lose_state2
	
}BOX_STATE;
	

void handle_box();

#endif
