#ifndef __DEBUG_TASK__
#define __DEBUG_TASK__

#include "E103_W06.h"
#include "hitcrt_os.h"
#include "navigation_task.h"
#include "path_algorithm.h"
#include "read_remote_ctrl_task.h"
#include "remote_control.h"
#include "string.h"
#include "system_monitor_task.h"
#include "usart_protocol.h"

#define ARM_DEBUG_SIZE 60

struct Frame
{
    fp32 fdata[ARM_DEBUG_SIZE];
    char tail[4];
};

void debug_updata(void);

#endif
