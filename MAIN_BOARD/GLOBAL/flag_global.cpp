#include "flag_global.h"

u8 _api_module_::flag_tuoluo;
int _api_module_::calculate_flag;
double _api_module_::f_g_error;

uint32_t _api_module_::WIFI_FLAG = 0;
uint8_t _api_module_::wifi_rx_flag = 0;

// //���������ȫ�ֱ���
// uint32_t _api_module_::g_uiAirValve=0;	   //�����й�ȫ�ֱ���
// uint32_t _api_module_::g_uiAirValvePre=0;  //������һ��״̬
// uint16_t _api_module_::g_usSwitch=0;     //�����й�ȫ�ֱ���
// uint16_t _api_module_::g_usSwitchPre = 0;//������һʱ�̿���ֵ

bool _remote_ctrl_::manual_enable = 0;

float _navigation_::flag_record = 0;
float _navigation_::calibration_current = 0;
