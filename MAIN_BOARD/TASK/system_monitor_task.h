#ifndef __SYSTEM_MONITOR_TASK_H__
#define __SYSTEM_MONITOR_TASK_H__

#include "FPGA_REGS.h"
#include "motor_drive.h"
#include "stm32f4xx.h"
#include "usart_protocol.h"

enum error_type_e {
    NONE,
    CHASSIS_TURN_ERROR,
    CHASSIS_RUN_ERROR,
    CHASSIS_ENCODER_ERROR,
    DT35_ERROR,
    KEY_ERROR,
    MINOR32_ERROR,
};

enum system_state_e { SYS_INIT, SYS_RUN, SYS_ERROR };

enum rate_e {
    CHASSIS_RU,
    CHASSIS_LU,
    CHASSIS_LD,
    CHASSIS_RD,
    ENCODER_RUN,  // 随动轮
    REMOTE_CTRL_WIFI,
    MINOR_32,
    DT35,
    KEY
};

struct rate_monitor_t {
    uint32_t time_base;  // 在systick中每ims自加一次
    uint16_t temp_rate[14];
    uint16_t real_rate[14];
};

class C_Sys_Monitor {
   public:
    error_type_e system_error;
    rate_monitor_t rate_monitor;
    system_state_e system_state;

    C_Sys_Monitor() : system_error(NONE), system_state(SYS_INIT) {}
    ~C_Sys_Monitor() {}

    void motor_detection(void);
    void remote_ctrl_detection(void);
    void error_alarm(void);
    uint8_t return_normal(void);
    void minor32_detection(void);
    void air_operated_board_detection(void);
};

extern C_Sys_Monitor Sys_Monitor;

#endif
