#include "system_monitor_task.h"

C_Sys_Monitor Sys_Monitor;

void C_Sys_Monitor::motor_detection(void) {
    for (int i = 0; i < 4; ++i)  // 电机帧率检测
    {
        if (rate_monitor.real_rate[i] < 900 || rate_monitor.real_rate[i] > 1100) {
            system_error = CHASSIS_RUN_ERROR;
            system_state = SYS_ERROR;
        }
    }
    if (rate_monitor.real_rate[4] < 900 || rate_monitor.real_rate[6] > 1100) {
        system_error = CHASSIS_ENCODER_ERROR;
        system_state = SYS_ERROR;
    }
}

void C_Sys_Monitor::air_operated_board_detection(void) {
    if (rate_monitor.real_rate[DT35] < 50) {
        system_error = DT35_ERROR;
        system_state = SYS_ERROR;
    }
    if (rate_monitor.real_rate[KEY] < 900) {
        system_error = KEY_ERROR;
        system_state = SYS_ERROR;
    }
}

void C_Sys_Monitor::minor32_detection(void) {
    if (rate_monitor.real_rate[MINOR_32] < 95) {
        system_error = MINOR32_ERROR;
        system_state = SYS_ERROR;
    }
    if (!uart3_efr.num[0]) {
        switch (uart3_efr.num[1]) {}
    }
}

uint8_t C_Sys_Monitor::return_normal(void) {
    for (int i = 0; i < 7; ++i) {
        if (rate_monitor.real_rate[i] < 900 || rate_monitor.real_rate[i] > 1100) {
            return 0;
        }
    }
    return 1;
}

void C_Sys_Monitor::error_alarm(void) { BuzzerBeep(BEEP_MODE_2); }
