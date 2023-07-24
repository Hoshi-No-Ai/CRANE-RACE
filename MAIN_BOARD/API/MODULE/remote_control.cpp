#include "remote_control.h"

using _api_module_::WIFI_FLAG;
using _api_module_::wifi_rx_flag;

// 手柄键值和摇杆类
C_JsKey JsKey;

/******************************************************************
无线手柄相关函数
**********************************************************************/
uint8_t tmp_buf[33];
uint8_t tmp_buf_11;

/************************************************************
函数功能:通过手柄摇杆值分配速度
************************************************************/
void C_JsKey::CalSpeed(C_NAV &p_nav) {
    uint8_t ucJSTmp;
    int16_t Vx, Vy, Vw;  // Vt;
    const fp32 smooth = 30;

    static C_LPF FJx(smooth, 0.002f);  // 需要pre_out，故需要设为静态变量
    static C_LPF FJy(smooth, 0.002f);
    static C_LPF FJw(smooth, 0.002f);

    // ucJSTmp = usJsLeft>>8;
    ucJSTmp = usJsLeft_X;
    // Vx=(ucJSTmp - JS_MID_POS);
    Vx = (ucJSTmp - JS_MID_POS_X);

    if (fabs((fp32)Vx) < G_SPEED) {
        FJx.m_in = 0;
    } else if (Vx >= G_SPEED) {
        FJx.m_in = (Vx - G_SPEED) * M_SPEED / (JS_MID_POS_X - G_SPEED);
    } else if (Vx <= -G_SPEED) {
        FJx.m_in = (Vx + G_SPEED) * M_SPEED / (JS_MID_POS_X - G_SPEED);
    }

    // ucJSTmp = usJsLeft & (0x00ff);
    ucJSTmp = usJsLeft_Y;

    Vy = (ucJSTmp - JS_MID_POS_Y);

    if (fabs((fp32)Vy) < G_SPEED) {
        FJy.m_in = 0;
    } else if (Vy >= G_SPEED) {
        FJy.m_in = (Vy - G_SPEED) * M_SPEED / (JS_MID_POS_Y - G_SPEED);
    } else if (Vy <= -G_SPEED) {
        FJy.m_in = (Vy + G_SPEED) * M_SPEED / (JS_MID_POS_Y - G_SPEED);
    }
    ucJSTmp = usJsRight;
    Vw = -(ucJSTmp - 0x7B);  // JS_MID_POS;

    if (fabs((fp32)Vw) < G_SPEED) {
        FJw.m_in = 0;
    } else if (Vw >= G_SPEED) {
        FJw.m_in = (Vw - G_SPEED) * M_SPEED / (0x7B - G_SPEED);
    } else if (Vw <= -G_SPEED) {
        FJw.m_in = (Vw + G_SPEED) * M_SPEED / (0x7B - G_SPEED);
    }

    FJx.LpFilter();
    FJy.LpFilter();
    FJw.LpFilter();

    p_nav.expect_robot_global_velt.fpVx = -FJy.m_out;
    p_nav.expect_robot_global_velt.fpVy = FJx.m_out;
    p_nav.expect_robot_global_velt.fpW = FJw.m_out / 700.0f;  // ±?????????10
}

/******************************************************************
函数功能: 手柄键值处理函数
备注:  500内处理为单击，超过后处理为连击，且每100ms使按键有效一次
******************************************************************/
void C_JsKey::ReadWlanJsValue(void) {
#ifdef NRF_ENABLE

    NRF24L01_RX_Mode();
    int i;
    NRF24L01_RX_Mode();        // 配置NRF24L01为接收模式
    for (i = 0; i < 420; i++)  // 必要的延时
    {
    }
    //	memcpy(tmp_buf, uart6_efr.num, 37*sizeof(uint8_t));
    if (NRF24L01_RxPacket(tmp_buf) == 0)  // 接收到数
    {
        wifi_rx_flag = 0;
        if (((tmp_buf[1] == 0x73) || (tmp_buf[1] == 0x41)) && (tmp_buf[2] == 0x5A)) {
            uint8_t i = 0;
            usJsState = Wlan_JOYSTICK_STATE;
            usJsKey = Wlan_JOYSTICK_RESERVED;

            for (i = 0; i < 16; ++i) {
                if ((usJsKey & (1 << i)) == 0) {
                    if (auiPressDuration[i] == 0)  // 按下后，第一次读取
                    {
                        auiPressDuration[i] = 5;
                        uiStartTime[i] = TIM7->CNT;  // T1TC;
                        aucKeyPress[i] = 1;
                    } else {
                        uiCurTime[i] = TIM7->CNT;  // T1TC;
                        if (uiCurTime[i] - uiStartTime[i] >
                            auiPressDuration[i] * 100000)  // 第一次为500ms,之后100ms一次
                        {
                            auiPressDuration[i]++;
                            aucKeyPress[i] = 1;
                        } else {
                            aucKeyPress[i] = 0;
                        }
                    } /*检测按下的时间，判断是否处理为连击*/
                } else {
                    auiPressDuration[i] = 0;
                    uiStartTime[i] = 0;
                    uiCurTime[i] = 0;
                    aucKeyPress[i] = 0;
                } /*检测是否被按下*/
            }

            if (usJsState == 0x80)  // 红灯模式
            {
                usJsLeft = Wlan_JOYSTICK_LEFT;
                usJsRight = Wlan_JOYSTICK_RIGTH;
            } else if (usJsState == 0x41) {
                usJsLeft = 0x8080;
                usJsRight = 0x8080;
            }
        }

    } else {
        usJsKey = 0xFFFF;
        usJsLeft = 0x8080;
        usJsRight = 0x8080;
        usJsState = 0x00;
    }

    if ((tmp_buf[9] == 0xAA) && (tmp_buf[10] == 0x88)) {
        tmp_buf_11 = tmp_buf[11];
    }
#endif
#ifndef NRF_ENABLE
    //	NRF24L01_RX_Mode();
    int i;
    //	NRF24L01_RX_Mode();//配置NRF24L01为接收模式
    for (i = 0; i < 420; i++)  // 必要的延时
    {
    }
    memcpy(tmp_buf, uart6_efr.num, USART6_RX_DATA_LEN * sizeof(uint8_t));
    // if(NRF24L01_RxPacket(tmp_buf)==0)//接收到数
    if (tmp_buf[0] != 0)  // 接收到数
    {
        wifi_rx_flag = 0;
        if (((tmp_buf[0] == 0x73) || (tmp_buf[0] == 0x41)) && (tmp_buf[1] == 0x5A)) {
            uint8_t i = 0;
            usJsState = Wlan_JOYSTICK_STATE;
            usJsKey = Wlan_JOYSTICK_RESERVED;

            //			for(i = 0; i < 16; ++i)
            //			{
            //				if((usJsKey & (1 << i)) == 0)
            //				{
            //					if(auiPressDuration[i] == 0)    //按下后，第一次读取
            //					{
            //						auiPressDuration[i] = 5;
            //						uiStartTime[i] = TIM7->CNT;//T1TC;
            //						aucKeyPress[i] = 1;
            //					}
            //					else
            //					{
            //						uiCurTime[i] = TIM7->CNT;//T1TC;
            //						if(uiCurTime[i] - uiStartTime[i]
            //								> auiPressDuration[i] * 100000)
            ////第一次为500ms,之后100ms一次
            //						{
            //							auiPressDuration[i] ++;
            //							aucKeyPress[i] = 1;
            //						}
            //						else
            //						{
            //							aucKeyPress[i] = 0;
            //						}
            //					}/*检测按下的时间，判断是否处理为连击*/
            //				}
            //				else
            //				{
            //					auiPressDuration[i] = 0;
            //					uiStartTime[i] = 0;
            //					uiCurTime[i] = 0;
            //					aucKeyPress[i] = 0;
            //				}/*检测是否被按下*/
            //			}

            if (usJsState == 0x73)  // 红灯模式
            {
                    usJsLeft_X = Wlan_JOYSTICK_LEFT_X;
                    usJsLeft_Y = Wlan_JOYSTICK_LEFT_Y;
                    usJsRight = Wlan_JOYSTICK_RIGTH;
            } else if (usJsState == 0x41) {
                usJsLeft_X = 128;
                usJsLeft_Y = 130;
                usJsRight = 128;
            }
        }

    } else {
        usJsKey = 0xFFFF;
        usJsLeft_X = 0;
        usJsLeft_Y = 0;
        usJsRight = 0;
        usJsState = 0x00;
    }

    if ((tmp_buf[9] == 0xAA) && (tmp_buf[10] == 0x88)) {
        tmp_buf_11 = tmp_buf[11];
    }
#endif
}

static uint8_t Wlan_ucKeyStatic = 0;
static uint32_t Wlan_uiStartTime = 0;
static uint16_t Wlan_usKeyPre = 0;
uint32_t uiCurTime = 0;
uint16_t usKey = 0;
/******************************************************************
函数功能: WLAN键盘键值处理函数
******************************************************************/
void C_JsKey::ReadWlanKeyValue(void) {
    usKey = Wlan_PSKEY;
    if (Wlan_ucKeyStatic == 0)  // 自由状态
    {
        if (usKey != 0 && usKey != Wlan_usKeyPre) {
            usKeyValue = usKey;
            //            Wlan_uiStartTime = TIM7->CNT;
            Wlan_ucKeyStatic = 1;  // 有数状态
        }
        Wlan_usKeyPre = usKey;
    } else if (Wlan_ucKeyStatic == 1) {
        usKeyValue = 0;
        Wlan_ucKeyStatic = 2;  // 等待状态
    } else if (Wlan_ucKeyStatic == 2) {
        //        uiCurTime = TIM7->CNT;
        //        if(uiCurTime - Wlan_uiStartTime > 20000)
        //        {
        Wlan_ucKeyStatic = 0;
        //        }
    }
}

uint8_t wifi_rx_packet(void) {
    if (wifi_rx_flag) {
        wifi_rx_flag = 0;
        WIFI_FLAG = 0;
    }
    if (WIFI_FLAG > 200) {
        return 1;
    } else {
        return 0;
    }
}
