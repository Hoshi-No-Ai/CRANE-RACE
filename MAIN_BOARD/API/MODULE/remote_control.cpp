#include "remote_control.h"

using _api_module_::WIFI_FLAG;
using _api_module_::wifi_rx_flag;

// �ֱ���ֵ��ҡ����
C_JsKey JsKey;

/******************************************************************
�����ֱ���غ���
**********************************************************************/
uint8_t tmp_buf[33];
uint8_t tmp_buf_11;

/************************************************************
��������:ͨ���ֱ�ҡ��ֵ�����ٶ�
************************************************************/
void C_JsKey::CalSpeed(C_NAV &p_nav) {
    uint8_t ucJSTmp;
    int16_t Vx, Vy, Vw;  // Vt;
    const fp32 smooth = 30;

    static C_LPF FJx(smooth, 0.002f);  // ��Ҫpre_out������Ҫ��Ϊ��̬����
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
    p_nav.expect_robot_global_velt.fpW = FJw.m_out / 700.0f;  // ��?????????10
}

/******************************************************************
��������: �ֱ���ֵ������
��ע:  500�ڴ���Ϊ��������������Ϊ��������ÿ100msʹ������Чһ��
******************************************************************/
void C_JsKey::ReadWlanJsValue(void) {
#ifdef NRF_ENABLE

    NRF24L01_RX_Mode();
    int i;
    NRF24L01_RX_Mode();        // ����NRF24L01Ϊ����ģʽ
    for (i = 0; i < 420; i++)  // ��Ҫ����ʱ
    {
    }
    //	memcpy(tmp_buf, uart6_efr.num, 37*sizeof(uint8_t));
    if (NRF24L01_RxPacket(tmp_buf) == 0)  // ���յ���
    {
        wifi_rx_flag = 0;
        if (((tmp_buf[1] == 0x73) || (tmp_buf[1] == 0x41)) && (tmp_buf[2] == 0x5A)) {
            uint8_t i = 0;
            usJsState = Wlan_JOYSTICK_STATE;
            usJsKey = Wlan_JOYSTICK_RESERVED;

            for (i = 0; i < 16; ++i) {
                if ((usJsKey & (1 << i)) == 0) {
                    if (auiPressDuration[i] == 0)  // ���º󣬵�һ�ζ�ȡ
                    {
                        auiPressDuration[i] = 5;
                        uiStartTime[i] = TIM7->CNT;  // T1TC;
                        aucKeyPress[i] = 1;
                    } else {
                        uiCurTime[i] = TIM7->CNT;  // T1TC;
                        if (uiCurTime[i] - uiStartTime[i] >
                            auiPressDuration[i] * 100000)  // ��һ��Ϊ500ms,֮��100msһ��
                        {
                            auiPressDuration[i]++;
                            aucKeyPress[i] = 1;
                        } else {
                            aucKeyPress[i] = 0;
                        }
                    } /*��ⰴ�µ�ʱ�䣬�ж��Ƿ���Ϊ����*/
                } else {
                    auiPressDuration[i] = 0;
                    uiStartTime[i] = 0;
                    uiCurTime[i] = 0;
                    aucKeyPress[i] = 0;
                } /*����Ƿ񱻰���*/
            }

            if (usJsState == 0x80)  // ���ģʽ
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
    //	NRF24L01_RX_Mode();//����NRF24L01Ϊ����ģʽ
    for (i = 0; i < 420; i++)  // ��Ҫ����ʱ
    {
    }
    memcpy(tmp_buf, uart6_efr.num, USART6_RX_DATA_LEN * sizeof(uint8_t));
    // if(NRF24L01_RxPacket(tmp_buf)==0)//���յ���
    if (tmp_buf[0] != 0)  // ���յ���
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
            //					if(auiPressDuration[i] == 0)    //���º󣬵�һ�ζ�ȡ
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
            ////��һ��Ϊ500ms,֮��100msһ��
            //						{
            //							auiPressDuration[i] ++;
            //							aucKeyPress[i] = 1;
            //						}
            //						else
            //						{
            //							aucKeyPress[i] = 0;
            //						}
            //					}/*��ⰴ�µ�ʱ�䣬�ж��Ƿ���Ϊ����*/
            //				}
            //				else
            //				{
            //					auiPressDuration[i] = 0;
            //					uiStartTime[i] = 0;
            //					uiCurTime[i] = 0;
            //					aucKeyPress[i] = 0;
            //				}/*����Ƿ񱻰���*/
            //			}

            if (usJsState == 0x73)  // ���ģʽ
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
��������: WLAN���̼�ֵ������
******************************************************************/
void C_JsKey::ReadWlanKeyValue(void) {
    usKey = Wlan_PSKEY;
    if (Wlan_ucKeyStatic == 0)  // ����״̬
    {
        if (usKey != 0 && usKey != Wlan_usKeyPre) {
            usKeyValue = usKey;
            //            Wlan_uiStartTime = TIM7->CNT;
            Wlan_ucKeyStatic = 1;  // ����״̬
        }
        Wlan_usKeyPre = usKey;
    } else if (Wlan_ucKeyStatic == 1) {
        usKeyValue = 0;
        Wlan_ucKeyStatic = 2;  // �ȴ�״̬
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
