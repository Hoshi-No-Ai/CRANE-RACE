#include "read_remote_ctrl_task.h"

using _navigation_::calibration_current;

keyboard_mode_e keyboard_mode = DEBUG;

void Js_Deal(void)
{
    // static bool calibration_status = 0;
    JsKey.ReadWlanJsValue();

    if (PRESS_START)
    {
        if (keyboard_mode == ACTION)
        {
            nav.auto_path.m_point_end.point_set(0, 0, 0);
            SET_NAV_PATH_AUTO(1);
        }
        else if (keyboard_mode == PATH)
        {
        }
        else if (keyboard_mode == DEBUG)
        {
        }
    }
    else if (PRESS_SELECT)
    {
    }
    else if (PRESS_L1)
    {
        if (keyboard_mode == DEBUG || keyboard_mode == PATH || keyboard_mode == ACTION ||
            keyboard_mode == PATHPLANNING)
        {
            JsKey.M_SPEED += 200;
            JsKey.M_SPEED = ClipFloat(JsKey.M_SPEED, 200, 2000);
        }
    }
    else if (PRESS_L2)
    {
        if (keyboard_mode == DEBUG || keyboard_mode == PATH || keyboard_mode == ACTION ||
            keyboard_mode == PATHPLANNING)
        {
            JsKey.M_SPEED -= 200;
            JsKey.M_SPEED = ClipFloat(JsKey.M_SPEED, 200, 2000);
        }
    }
    else if (PRESS_R1)
    {
    }
    else if (PRESS_B1)
    {
        keyboard_mode = ACTION;
    }
    else if (PRESS_B2)
    {
        nav.state = NAV_STOP;
        keyboard_mode = DEBUG;
    }
    else if (PRESS_B3)
    {
        BuzzerBeep(BEEP_MODE_1);
        keyboard_mode = DEBUG;
        nav.state = NAV_STOP;
    }
    else if (PRESS_B4)
    {
        nav.state = NAV_STOP;
        keyboard_mode = DEBUG;
    }
    else if (PRESS_UP)
    {
        keyboard_mode = PATH;
    }
    else if (PRESS_DOWN)
    {
    }
    else if (PRESS_LEFT)
    {
        keyboard_mode = ACTION;
        nav.auto_path.m_velt_acc.Velt_Acc_Set(500, 500, 500);
    }
    else if (PRESS_RIGHT)
    {
        uart3_eft.num[1] = 10;
        USART3_DMA_Tx();
    }
}

void Key_Deal(void)
{
    JsKey.ReadWlanKeyValue();
    JsKey.ReadWlanJsValue();

    if (PRESS_KEY_1_1)
    {
        if (keyboard_mode == DEBUG)
        {
            nav.auto_path.run_time = 0;
            cRobot.cFollowoerWheel.stPot.fpPosX = 0;
            cRobot.cFollowoerWheel.stPot.fpPosX1 = 0;
            cRobot.cFollowoerWheel.stPot.fpPosY = 0;
            cRobot.cFollowoerWheel.stPot.fpPosY1 = 0;
            cRobot.stPot.fpPosX = fpStartX;
            cRobot.stPot.fpPosX1 = fpStartX;
            cRobot.stPot.fpPosY = fpStartY;
            cRobot.stPot.fpPosY1 = fpStartY;
            cRobot.stPot.fpPosQ = 0;
            cRobot.stPot.fpPosQ1 = 0;
        }
        else if (keyboard_mode == CALIBRATION)
        {
            if (nav.state == NAV_CALIBRATION_1)
            { // 0A
                calibration_current = 4000;
            }
        }
    }
    else if (PRESS_KEY_1_2)
    {
        if (keyboard_mode == DEBUG)
        {
            nav.state=NAV_MANUAL;
        }
        else if (keyboard_mode == CALIBRATION)
        {
            if (nav.state == NAV_CALIBRATION_1)
            { // 0A
                calibration_current = 8000;
            }
        }
    }
    else if (PRESS_KEY_1_3)
    {
        if (keyboard_mode == DEBUG)
        {
            nav.state=NAV_NEW_MANUAL;
        }
        else if (keyboard_mode == CALIBRATION)
        {
            if (nav.state == NAV_CALIBRATION_1)
            {
                calibration_current = 12000;
            }
        }
    }
    else if (PRESS_KEY_1_4)
    {
        if (keyboard_mode == DEBUG)
        {
            nav.state=NAV_CALIBRATION_1;
        }
        else if (keyboard_mode == CALIBRATION)
        {
            if (nav.state == NAV_CALIBRATION_1)
            { // 0A
                calibration_current = 16000;
            }
        }
    }
    else if (PRESS_KEY_1_5)
    {
        if (keyboard_mode == ACTION)
        {
        }
        else if (keyboard_mode == DEBUG)
        {
            JsKey.M_SPEED+=200;
            JsKey.M_SPEED=ClipFloat(JsKey.M_SPEED,200,3000);
        }
        else if (keyboard_mode == PATH)
        {
        }
        else if (keyboard_mode == CALIBRATION)
        {
            if (nav.state == NAV_CALIBRATION_1)
            { // 0A
                calibration_current = 20000;
            }
        }
    }
    else if (PRESS_KEY_1_6)
    {
        if (keyboard_mode == DEBUG)
        {
            JsKey.M_SPEED-=200;
            JsKey.M_SPEED=ClipFloat(JsKey.M_SPEED,200,3000);
        }
        else if (keyboard_mode == CALIBRATION)
        {
            if (nav.state == NAV_CALIBRATION_1)
            { // 0A
                calibration_current = 24000;
            }
        }
        else if (keyboard_mode == PATHPLANNING)
        {
        }
    }
    else if (PRESS_KEY_2_1)
    {
        if (keyboard_mode == DEBUG)
        {
        }
        else if (keyboard_mode == PATH)
        {
        }
        else if (keyboard_mode == CALIBRATION)
        {
            if (nav.state == NAV_CALIBRATION_1)
            { // 0A
                calibration_current = 28000;
            }
        }
        else if (keyboard_mode == PATHPLANNING)
        {
        }
    }

    else if (PRESS_KEY_2_2)
    {
        if (keyboard_mode == DEBUG)
        {
        }
        else if (keyboard_mode == PATH)
        {
        }
        else if (keyboard_mode == CALIBRATION)
        {
            if (nav.state == NAV_CALIBRATION_1)
            { // 0A
                calibration_current = 32000;
            }
        }
    }
    else if (PRESS_KEY_2_3)
    {
    }
    else if (PRESS_KEY_2_4)
    {
    }
    else if (PRESS_KEY_2_5)
    {
    }
    else if (PRESS_KEY_2_6)
    {
    }
    else if (PRESS_KEY_3_1)
    {
    }
    else if (PRESS_KEY_3_2)
    {
    }
    else if (PRESS_KEY_3_3)
    {
        cRobot.stPot.fpPosQ = 0;
        cRobot.stPot.fpPosQ1 = 0;
        cRobot.DT35_relocation_new();
    }
    else if (PRESS_KEY_3_4)
    {
    }
    else if (PRESS_KEY_3_5)
    {
    }
    else if (PRESS_KEY_3_6)
    {
    }
    else if (PRESS_KEY_4_1)
    {
        nav.state = NAV_STOP;
        keyboard_mode = DEBUG;
    }
    else if (PRESS_KEY_4_2)
    {
        keyboard_mode = PATH;
    }
    else if (PRESS_KEY_4_3)
    {
        keyboard_mode = ACTION;
    }
    else if (PRESS_KEY_4_4)
    {
    }
    else if (PRESS_KEY_4_5)
    {
    }
    else if (PRESS_KEY_4_6)
    {
    }
}
