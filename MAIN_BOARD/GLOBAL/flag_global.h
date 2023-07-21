#ifndef _FLAG_GLOBAL_H_
#define _FLAG_GLOBAL_H_

#include "stm32f4xx.h"

namespace _api_module_ {
extern u8 flag_tuoluo;  // �����Ǳ���Ư��
extern int calculate_flag;
extern double f_g_error;

extern uint32_t WIFI_FLAG;
extern uint8_t wifi_rx_flag;

// 	extern uint32_t g_uiAirValve;	   //�����й�ȫ�ֱ���
//   extern uint32_t g_uiAirValvePre;  //������һ��״̬
//   extern uint16_t g_usSwitch;     //�����й�ȫ�ֱ���
//   extern uint16_t g_usSwitchPre;//������һʱ�̿���ֵ
}  // namespace _api_module_

namespace _remote_ctrl_ {}

namespace _navigation_ {
extern float flag_record;  // ��̬·����ʼ����־λ
extern float calibration_current;
}  // namespace _navigation_

#endif
