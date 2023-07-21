#ifndef ACTION_TASK_H
#define ACTION_TASK_H

#include "fetch_rings.h"
#include "uart_protocol.h"
#include "transmit_task.h"
#include "motor_drive.h"
#include "fetch_rings.h"
#include "gimbal_driver.h"
#include "motor_all.h"
#include "air_operated_board.h"
#include "friction_belt_motor.h"
#include "system_monitor_task.h"
#include "friction_belt_motor.h"
#include "math.h"
#include "stdio.h"

void deal_with_message(void);

void action_detection(void);

#endif