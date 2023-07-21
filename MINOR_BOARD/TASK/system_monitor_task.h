#ifndef __SYSTEM_MONITOR_TASK_H__
#define __SYSTEM_MONITOR_TASK_H__

#include "stm32f4xx.h"
#include "motor_drive.h"
#include "uart_protocol.h"

typedef enum
{
	NONE,
	GIMBAL_ERROR,
	SEND_ERROR,
}error_type_e;

typedef enum
{
	SYS_INIT,
	SYS_RUN,
	SYS_ERROR
}system_state_e;

typedef struct
{
	uint32_t time_base;//在systick中每ims自加一次
	uint16_t temp_rate[20];
	uint16_t real_rate[20];
}rate_monitor_t;

extern void motor_detection(void);
extern void disable_all_motor(void);
extern uint8_t return_normal(void);
extern void send_system_massage2main32(void);

extern error_type_e system_error;
extern rate_monitor_t rate_monitor;
extern system_state_e system_state;

#endif
