#include "flag_global.h"

u8 _api_module_::flag_tuoluo;
int _api_module_::calculate_flag;
double _api_module_::f_g_error;

uint32_t _api_module_::WIFI_FLAG = 0;
uint8_t _api_module_::wifi_rx_flag = 0;

// //气动板相关全局变量
// uint32_t _api_module_::g_uiAirValve=0;	   //气缸有关全局变量
// uint32_t _api_module_::g_uiAirValvePre=0;  //气缸上一次状态
// uint16_t _api_module_::g_usSwitch=0;     //开关有关全局变量
// uint16_t _api_module_::g_usSwitchPre = 0;//保存上一时刻开关值

bool _remote_ctrl_::manual_enable = 0;

float _navigation_::flag_record = 0;
float _navigation_::calibration_current = 0;
