#ifndef __MOTOR_CONTROL_TASK_H__
#define __MOTOR_CONTROL_TASK_H__

#include "action_task.h"
#include "chassis.h"
#include "math_algorithm.h"
#include "pid_algorithm.h"
#include "system_monitor_task.h"
#include "table.h"

#define SAFE_CURRENT_MAX 20000.0f
#define RUN_VELT_MAX 500.0f * PI / 30.0f

void omni_chassis_control(void);
void disable_all_motor(void);

#endif
