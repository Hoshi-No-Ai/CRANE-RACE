#ifndef _FLAG_GLOBAL_H_
#define _FLAG_GLOBAL_H_

#include "stm32f4xx.h"

namespace _api_module_
{
    extern u8 flag_tuoluo;
    extern int calculate_flag;
    extern double f_g_error;

    extern uint32_t WIFI_FLAG;
    extern uint8_t wifi_rx_flag;

    // 	extern uint32_t g_uiAirValve;
    //   extern uint32_t g_uiAirValvePre;
    //   extern uint16_t g_usSwitch;
    //   extern uint16_t g_usSwitchPre;
} // namespace _api_module_

namespace _remote_ctrl_
{
    extern bool manual_enable;
    extern bool auto_enable;
}

namespace _navigation_
{
    extern float flag_record;
    extern float calibration_current;
    extern bool vision_enable;
    extern bool vision_true;
} // namespace _navigation_

namespace _action_
{
    extern bool figure_out_object;
}

#endif
