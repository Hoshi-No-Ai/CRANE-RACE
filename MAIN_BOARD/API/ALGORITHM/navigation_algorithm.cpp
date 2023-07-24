#include "navigation_algorithm.h"

C_NAV nav;

fp32 velt_w[4], velt_x[4], velt_y[4];
  C_VECTOR pos_chassis[4];
void C_NAV::Omni_chassis_SpeedDistribute(void) {
    fp32 fpQ;
//    fp32 velt_w[4], velt_x[4], velt_y[4];
//    C_VECTOR pos_chassis[4];
    // 右上，左上，左下，右下
    pos_chassis[0] = C_VECTOR(L_HALF, B_HALF, CARTESIAN);
    pos_chassis[1] = C_VECTOR(-L_HALF, B_HALF, CARTESIAN);
    pos_chassis[2] = C_VECTOR(-L_HALF, -B_HALF, CARTESIAN);
    pos_chassis[3] = C_VECTOR(L_HALF, -B_HALF, CARTESIAN);

    if (state == NAV_MANUAL) {
        fpQ = 0;
    } else {
        fpQ = ConvertAngle(cRobot.stPot.fpPosQ * RADIAN_10);
    }

    C_VECTOR::Concert_coorindnate(expect_robot_global_velt, expect_robot_local_Velt, fpQ);
    expect_robot_local_Velt.fpW = expect_robot_global_velt.fpW;

    for (int i = 0; i < 4; i++) {
        velt_w[i] = R_HALF * expect_robot_local_Velt.fpW;
        velt_x[i] = pos_chassis[i].CalRadialProjection(C_VECTOR(expect_robot_local_Velt.fpVx, 0, CARTESIAN),
                                                       pos_chassis[i]);
        velt_y[i] = pos_chassis[i].CalRadialProjection(C_VECTOR(0, expect_robot_local_Velt.fpVy, CARTESIAN),
                                                       pos_chassis[i]);
        Omni_chassis[i].m_run_motor.velt_pid.fpDes = (velt_w[i] + velt_x[i] + velt_y[i]) / R_WHEEL;
			if(i==RIGHTUP)
			{
				Omni_chassis[i].m_run_motor.velt_pid.fpDes=-Omni_chassis[i].m_run_motor.velt_pid.fpDes;
			}
			else if(i==LEFTUP)
			{
				Omni_chassis[i].m_run_motor.velt_pid.fpDes=-Omni_chassis[i].m_run_motor.velt_pid.fpDes;
			}
			else if(i==LEFTDOWN)
			{
				Omni_chassis[i].m_run_motor.velt_pid.fpDes=-Omni_chassis[i].m_run_motor.velt_pid.fpDes;
			}
			else if(i==RIGHTDOWN)
			{
				Omni_chassis[i].m_run_motor.velt_pid.fpDes=-Omni_chassis[i].m_run_motor.velt_pid.fpDes;
			}
    }
}

void C_NAV::Omni_chassis_Cal_Feedforward(void) {
    C_VECTOR Omni_velt, Omni_pre_velt;

    fp32 fpQ;
    fpQ = ConvertAngle(cRobot.stPot.fpPosQ * RADIAN_10);
    C_VECTOR::Concert_coorindnate(expect_robot_global_velt, expect_robot_local_Velt, fpQ);

    for (int i = 0; i < 4; i++) {
        Omni_chassis[i].m_run_motor.feed_forward_current =
            Omni_chassis[i].cal_single_feed_forward(expect_robot_local_Velt, i);
    }

    Omni_pre_velt = Omni_velt;
}

void C_NAV::disable_omni_chassis(void) {
    for (int i = 0; i < 4; i++) {
        Omni_chassis[i].m_run_motor.motor_pid_state = OPEN_LOOP;
        Omni_chassis[i].m_run_motor.feed_forward_state = WITHOUT_FORWARD;
        Omni_chassis[i].m_run_motor.velt_pid.fpU = 0;
    }
}

void C_NAV::stop_omni_chassis(void) {
    for (int i = 0; i < 4; i++) {
        Omni_chassis[i].m_run_motor.motor_pid_state = VELT_LOOP;
        Omni_chassis[i].m_run_motor.feed_forward_state = WITHOUT_FORWARD;
        Omni_chassis[i].m_run_motor.velt_pid.fpDes = 0;
    }
}