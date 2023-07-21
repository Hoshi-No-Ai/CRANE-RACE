#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include <string.h>

#include "filter_algorithm.h"
#include "navigation_algorithm.h"
#include "nrf.h"
#include "stm32f4xx.h"
#include "usart_protocol.h"

//���ֱ�Ϊ���ģʽʱWlan_JOYSTICK_STATEֵΪ0x73������Ϊ0x41

// qrpucp:ȥ���˱���ǰ��volatile
#define Wlan_JOYSTICK_RESERVED 		(((uint16_t)tmp_buf[3])<<8|tmp_buf[4])
#define Wlan_JOYSTICK_LEFT_X		tmp_buf[5]
#define Wlan_JOYSTICK_LEFT_Y    tmp_buf[6]
#define Wlan_JOYSTICK_RIGTH 		tmp_buf[7]
#define Wlan_JOYSTICK_STATE 		tmp_buf[0]

#define Wlan_PSKEY 	            tmp_buf_11
#define	 JS_MID_POS_X 131//ҡ���м�ֵ     
#define  JS_MID_POS_Y 125

//#define NRF_ENABLE //ʹ��NRFʱ��ע��

class C_JsKey {
   public:
    uint16_t usJsKey;
    uint8_t aucKeyPress[16];        //Ϊ1ʱ����ʾ����������Ч
    uint32_t auiPressDuration[16];  //���³���ʱ�����ֵ�Ƚϣ��ж��Ƿ�Ϊ����
    uint32_t uiStartTime[16];
    uint32_t uiCurTime[16];
    uint16_t usJsState;
	  uint16_t usJsLeft_X;
	 uint16_t usJsLeft_Y;
    uint16_t usJsRight;

    uint16_t usKeyValue;  //����̵�16λֵ

    uint8_t G_SPEED;  //����
    float M_SPEED;    //�޷�
    float W_reduce;   //���ٶȵ���С����

    int coordinate_mode;

    C_JsKey() : G_SPEED(80), M_SPEED(200), W_reduce(100), usKeyValue(0){};
    ~C_JsKey(){};

    void CalSpeed(C_NAV &p_nav);
    void ReadWlanJsValue(void);   //�����ֱ�����������
    void ReadWlanKeyValue(void);  //������̷���������
};

extern C_JsKey JsKey;

#endif
