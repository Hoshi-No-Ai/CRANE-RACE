#ifndef __READ_REMOTE_CTRL_TASK_H__
#define __READ_REMOTE_CTRL_TASK_H__

#include "chassis.h"
#include "locate_algorithm.h"
#include "navigation_task.h"
#include "path_algorithm.h"
#include "remote_control.h"
#include "stm32f4xx.h"
#include "usart_protocol.h"

/*手柄 间断按键 宏定义*/
#define PRESS_LEFT (JsKey.aucKeyPress[15] == 1)
#define PRESS_RIGHT (JsKey.aucKeyPress[13] == 1)
#define PRESS_UP (JsKey.aucKeyPress[12] == 1)
#define PRESS_DOWN (JsKey.aucKeyPress[14] == 1)
#define PRESS_B1 (JsKey.aucKeyPress[4] == 1)
#define PRESS_B2 (JsKey.aucKeyPress[5] == 1)
#define PRESS_B3 (JsKey.aucKeyPress[6] == 1)
#define PRESS_B4 (JsKey.aucKeyPress[7] == 1)
#define PRESS_L1 (JsKey.aucKeyPress[2] == 1)
#define PRESS_L2 (JsKey.aucKeyPress[0] == 1)
#define PRESS_R1 (JsKey.aucKeyPress[3] == 1)
#define PRESS_R2 (JsKey.aucKeyPress[1] == 1)
#define PRESS_START (JsKey.aucKeyPress[11] == 1)
#define PRESS_SELECT (JsKey.aucKeyPress[8] == 1)

/*手柄 连续按键 宏定义*/
#define PRESS_LEFT_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 15)) == 0)
#define PRESS_RIGHT_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 13)) == 0)
#define PRESS_UP_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 12)) == 0)
#define PRESS_DOWN_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 14)) == 0)
#define PRESS_B1_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 4)) == 0)
#define PRESS_B2_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 5)) == 0)
#define PRESS_B3_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 6)) == 0)
#define PRESS_B4_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 7)) == 0)

#define PRESS_L1_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 2)) == 0)
#define PRESS_L2_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 0)) == 0)

#define PRESS_R1_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 3)) == 0)
#define PRESS_R2_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 1)) == 0)

// #define PRESS_L1_CONTINUED	    ((Wlan_JOYSTICK_RESERVED & (1 << 2)) == 0)
// #define PRESS_L2_CONTINUED	    ((Wlan_JOYSTICK_RESERVED & (1 << 0)) == 0)

// #define PRESS_R1_CONTINUED	    ((Wlan_JOYSTICK_RESERVED & (1 << 3)) == 0)
// #define PRESS_R2_CONTINUED	    ((Wlan_JOYSTICK_RESERVED & (1 << 1)) == 0)

#define PRESS_START_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 11)) == 0)
#define PRESS_SELECT_CONTINUED ((Wlan_JOYSTICK_RESERVED & (1 << 8)) == 0)

/*老键盘 宏定义*/
#define PRESS_KEY_NUMLOCK (JsKey.usKeyValue == 0xFF77)
#define PRESS_KEY_0 (JsKey.usKeyValue == 0xFF70)
#define PRESS_KEY_1 (JsKey.usKeyValue == 0xFF69)
#define PRESS_KEY_2 (JsKey.usKeyValue == 0xFF72)
#define PRESS_KEY_3 (JsKey.usKeyValue == 0xFF7A)
#define PRESS_KEY_4 (JsKey.usKeyValue == 0xFF6B)
#define PRESS_KEY_5 (JsKey.usKeyValue == 0xFF73)
#define PRESS_KEY_6 (JsKey.usKeyValue == 0xFF74)
#define PRESS_KEY_7 (JsKey.usKeyValue == 0xFF6C)
#define PRESS_KEY_8 (JsKey.usKeyValue == 0xFF75)
#define PRESS_KEY_9 (JsKey.usKeyValue == 0xFF7D)
#define PRESS_KEY_BS (JsKey.usKeyValue == 0xFF66)
#define PRESS_KEY_ENTER (JsKey.usKeyValue == 0xFF5A)
#define PRESS_KEY_INC (JsKey.usKeyValue == 0xFF79)
#define PRESS_KEY_DEC (JsKey.usKeyValue == 0xFF7B)
#define PRESS_KEY_MUL (JsKey.usKeyValue == 0xFF7C)
#define PRESS_KEY_DOT (JsKey.usKeyValue == 0xFF71)
#define PRESS_KEY_DIV (JsKey.usKeyValue == 0xFF4A)
// #define PRESS_KEY_NULL (JsKey.usKeyValue == 0xFFFF)

/*新键盘 宏定义*/
#define PRESS_KEY_0_1 (JsKey.usKeyValue == 0x19)
#define PRESS_KEY_0_2 (JsKey.usKeyValue == 0x1A)
#define PRESS_KEY_1_1 (JsKey.usKeyValue == 0x01)
#define PRESS_KEY_1_2 (JsKey.usKeyValue == 0x04)
#define PRESS_KEY_1_3 (JsKey.usKeyValue == 0x05)
#define PRESS_KEY_1_4 (JsKey.usKeyValue == 0x02)  // 红灯
#define PRESS_KEY_1_5 (JsKey.usKeyValue == 0x06)
#define PRESS_KEY_1_6 (JsKey.usKeyValue == 0x03)
#define PRESS_KEY_2_1 (JsKey.usKeyValue == 0x07)
#define PRESS_KEY_2_2 (JsKey.usKeyValue == 0x0A)
#define PRESS_KEY_2_3 (JsKey.usKeyValue == 0x0B)
#define PRESS_KEY_2_4 (JsKey.usKeyValue == 0x08)
#define PRESS_KEY_2_5 (JsKey.usKeyValue == 0x0C)
#define PRESS_KEY_2_6 (JsKey.usKeyValue == 0x09)
#define PRESS_KEY_3_1 (JsKey.usKeyValue == 0x0D)
#define PRESS_KEY_3_2 (JsKey.usKeyValue == 0x10)
#define PRESS_KEY_3_3 (JsKey.usKeyValue == 0x11)
#define PRESS_KEY_3_4 (JsKey.usKeyValue == 0x0E)
#define PRESS_KEY_3_5 (JsKey.usKeyValue == 0x12)
#define PRESS_KEY_3_6 (JsKey.usKeyValue == 0x0F)
#define PRESS_KEY_4_1 (JsKey.usKeyValue == 0x13)
#define PRESS_KEY_4_2 (JsKey.usKeyValue == 0x16)
#define PRESS_KEY_4_3 (JsKey.usKeyValue == 0x17)
#define PRESS_KEY_4_4 (JsKey.usKeyValue == 0x14)
#define PRESS_KEY_4_5 (JsKey.usKeyValue == 0x18)
#define PRESS_KEY_4_6 (JsKey.usKeyValue == 0x15)
#define PRESS_KEY_NULL (JsKey.usKeyValue == 0x00)

/*老启动键盘 宏定义*/
#define PRESS_STARTKEY_NUMLOCK (g_usstartKeyValue == 0xFF77)
#define PRESS_STARTKEY_0 (g_usstartKeyValue == 0xFF70)
#define PRESS_STARTKEY_1 (g_usstartKeyValue == 0xFF69)
#define PRESS_STARTKEY_2 (g_usstartKeyValue == 0xFF72)
#define PRESS_STARTKEY_3 (g_usstartKeyValue == 0xFF7A)
#define PRESS_STARTKEY_4 (g_usstartKeyValue == 0xFF6B)
#define PRESS_STARTKEY_5 (g_usstartKeyValue == 0xFF73)
#define PRESS_STARTKEY_6 (g_usstartKeyValue == 0xFF74)
#define PRESS_STARTKEY_7 (g_usstartKeyValue == 0xFF6C)
#define PRESS_STARTKEY_8 (g_usstartKeyValue == 0xFF75)
#define PRESS_STARTKEY_9 (g_usstartKeyValue == 0xFF7D)
#define PRESS_STARTKEY_BS (g_usstartKeyValue == 0xFF66)
#define PRESS_STARTKEY_ENTER (g_usstartKeyValue == 0xFF5A)
#define PRESS_STARTKEY_INC (g_usstartKeyValue == 0xFF79)
#define PRESS_STARTKEY_DEC (g_usstartKeyValue == 0xFF7B)
#define PRESS_STARTKEY_MUL (g_usstartKeyValue == 0xFF7C)
#define PRESS_STARTKEY_DOT (g_usstartKeyValue == 0xFF71)
#define PRESS_STARTKEY_DIV (g_usstartKeyValue == 0xFF4A)
// #define PRESS_STARTKEY_NULL (g_usstartKeyValue == 0xFFFF)

enum keyboard_mode_e {
    DEBUG,
    CALIBRATION,
    PATH,
    ACTION,
    PATHPLANNING,
};

void Js_Deal(void);   // 手柄处理函数
void Key_Deal(void);  // 键盘处理函数

extern keyboard_mode_e keyboard_mode;

#endif
