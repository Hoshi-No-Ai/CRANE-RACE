#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include <string.h>

#include "filter_algorithm.h"
#include "navigation_algorithm.h"
#include "nrf.h"
#include "stm32f4xx.h"
#include "usart_protocol.h"

//当手柄为红灯模式时Wlan_JOYSTICK_STATE值为0x73，否则为0x41

// qrpucp:去掉了变量前的volatile
#define Wlan_JOYSTICK_RESERVED 		(((uint16_t)tmp_buf[3])<<8|tmp_buf[4])
#define Wlan_JOYSTICK_LEFT_X		tmp_buf[5]
#define Wlan_JOYSTICK_LEFT_Y    tmp_buf[6]
#define Wlan_JOYSTICK_RIGTH 		tmp_buf[7]
#define Wlan_JOYSTICK_STATE 		tmp_buf[0]

#define Wlan_PSKEY 	            tmp_buf_11
#define	 JS_MID_POS_X 131//摇杆中间值     
#define  JS_MID_POS_Y 125

//#define NRF_ENABLE //使用NRF时解注释

class C_JsKey {
   public:
    uint16_t usJsKey;
    uint8_t aucKeyPress[16];        //为1时，表示按键按下有效
    uint32_t auiPressDuration[16];  //按下持续时间与该值比较，判断是否为连击
    uint32_t uiStartTime[16];
    uint32_t uiCurTime[16];
    uint16_t usJsState;
	  uint16_t usJsLeft_X;
	 uint16_t usJsLeft_Y;
    uint16_t usJsRight;

    uint16_t usKeyValue;  //存键盘的16位值

    uint8_t G_SPEED;  //死区
    float M_SPEED;    //限幅
    float W_reduce;   //角速度的缩小倍数

    int coordinate_mode;

    C_JsKey() : G_SPEED(80), M_SPEED(200), W_reduce(100), usKeyValue(0){};
    ~C_JsKey(){};

    void CalSpeed(C_NAV &p_nav);
    void ReadWlanJsValue(void);   //处理手柄发来的数据
    void ReadWlanKeyValue(void);  //处理键盘发来的数据
};

extern C_JsKey JsKey;

#endif
