#ifndef _FLAG_GLOBAL_H_
#define _FLAG_GLOBAL_H_

#include "stm32f4xx.h"

namespace _api_module_ {
extern u8 flag_tuoluo;  // 陀螺仪标零漂用
extern int calculate_flag;
extern double f_g_error;

extern uint32_t WIFI_FLAG;
extern uint8_t wifi_rx_flag;

// 	extern uint32_t g_uiAirValve;	   //气缸有关全局变量
//   extern uint32_t g_uiAirValvePre;  //气缸上一次状态
//   extern uint16_t g_usSwitch;     //开关有关全局变量
//   extern uint16_t g_usSwitchPre;//保存上一时刻开关值
}  // namespace _api_module_

namespace _remote_ctrl_ {}

namespace _navigation_ {
extern float flag_record;  // 静态路径初始化标志位
extern float calibration_current;
}  // namespace _navigation_

#endif
