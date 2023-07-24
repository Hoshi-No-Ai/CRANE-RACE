#include "navigation_task.h"

using _api_module_::flag_tuoluo;
using _navigation_::calibration_current;
using _remote_ctrl_::manual_enable;

static bool path_with_pos_pid = 1;

void navigation(void) {
    switch (nav.state) {
        case NAV_INIT:
            for (int i = 0; i < 4; i++) {
                Omni_chassis[i].m_run_motor.motor_pid_state = VELT_LOOP;
                Omni_chassis[i].m_run_motor.feed_forward_current = WITHOUT_FORWARD;
            }
            break;

        case NAV_OFF:
            nav.disable_omni_chassis();
            break;

        case NAV_STOP:  // ����״̬���˶��������˫��
            nav.stop_omni_chassis();
            break;

        case NAV_LOCK:
            break;

        case NAV_MANUAL:
            for (int i = 0; i < 4; i++) {
                Omni_chassis[i].m_run_motor.motor_pid_state = VELT_LOOP;
                Omni_chassis[i].m_run_motor.feed_forward_current = WITHOUT_FORWARD;
            }

            // if (manual_enable)
            // {
                JsKey.CalSpeed(nav);
            // }       
            
            nav.Omni_chassis_SpeedDistribute();

            break;

        case NAV_NEW_MANUAL:
            for (int i = 0; i < 4; i++) {
                Omni_chassis[i].m_run_motor.motor_pid_state = VELT_LOOP;
                Omni_chassis[i].m_run_motor.feed_forward_current = WITHOUT_FORWARD;
            }

            // if (manual_enable)
            // {
                JsKey.CalSpeed(nav);
            // }  

            nav.Omni_chassis_SpeedDistribute();

            break;

        case NAV_AUTO_PATH:
            for (int i = 0; i < 4; i++) {
                Omni_chassis[i].m_run_motor.motor_pid_state = VELT_LOOP;
                Omni_chassis[i].m_run_motor.feed_forward_current = WITH_FORWARD;
            }

            nav.auto_path.pos_pid.x.fpFB = cRobot.stPot.fpPosX;
            nav.auto_path.pos_pid.y.fpFB = cRobot.stPot.fpPosY;
            nav.auto_path.pos_pid.w.fpFB = cRobot.stPot.fpPosQ;

            nav.auto_path.velt_pid.x.fpFB = cRobot.stVelt.fpVx;
            nav.auto_path.velt_pid.y.fpFB = cRobot.stVelt.fpVy;
            nav.auto_path.velt_pid.w.fpFB = cRobot.stVelt.fpW;

            // �滮����·��

            nav.auto_path.Path_Choose();
            nav.auto_path.pos_pid.x.fpKp = 5.0f;
            nav.auto_path.pos_pid.y.fpKp = 5.0f;
            nav.auto_path.pos_pid.w.fpKp = 5.0f;

            nav.auto_path.basic_velt.fpW *= RADIAN;
            nav.auto_path.pos_pid.w.fpFB *= RADIAN;
            nav.auto_path.pos_pid.w.fpDes *= RADIAN;

            if (path_with_pos_pid) {
                nav.auto_path.pos_pid.x.CalPID();
                nav.auto_path.pos_pid.y.CalPID();
                nav.auto_path.pos_pid.w.CalPID();

                nav.expect_robot_global_velt.fpVx =
                    nav.auto_path.pos_pid.x.fpU + nav.auto_path.basic_velt.fpVx;
                nav.expect_robot_global_velt.fpVy =
                    nav.auto_path.pos_pid.y.fpU + nav.auto_path.basic_velt.fpVy;
                nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU + nav.auto_path.basic_velt.fpW;
            } else {
                nav.expect_robot_global_velt.fpVx = nav.auto_path.basic_velt.fpVx;
                nav.expect_robot_global_velt.fpVy = nav.auto_path.basic_velt.fpVy;
                nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW;
            }

            nav.Omni_chassis_SpeedDistribute();
            nav.Omni_chassis_Cal_Feedforward();

            break;

        /*�궨�������ӵ�ֱ�߼��ٶ�ϵ��*/
        case NAV_CALIBRATION_1:
            for (int i = 0; i < 4; i++) {
                Omni_chassis[i].m_run_motor.motor_pid_state = VELT_LOOP;
                Omni_chassis[i].m_run_motor.feed_forward_current = WITHOUT_FORWARD;
            }

            /*����ʵ��Ҫ�ߵķ������*/
            for (int i; i < 4; i++) {
                Omni_chassis[i].m_run_motor.velt_pid.fpU = calibration_current;
            }

            if (fabs(cRobot.stPot.fpPosY) > 2000 || fabs(cRobot.stPot.fpPosX) > 2000) {
                nav.state = NAV_STOP;
            }
            break;

        /*�궨�������ӵ���ת���ٶ�ϵ��*/
        case NAV_CALIBRATION_2:
            for (int i = 0; i < 4; i++) {
                Omni_chassis[i].m_run_motor.motor_pid_state = VELT_LOOP;
                Omni_chassis[i].m_run_motor.feed_forward_current = WITH_FORWARD;
            }

            for (int i; i < 4; i++) {
                Omni_chassis[i].m_run_motor.velt_pid.fpU = calibration_current;
            }
            break;

        default:
            break;
    }
}
