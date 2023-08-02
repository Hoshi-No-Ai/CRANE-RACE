#include "debug_task.h"

extern u8 udp_demo_sendbuf[ARM_DEBUG_SIZE * 4];

Frame debug_frame = {
    {0},
    {0x00, 0x00, 0x80, 0x7f},
};

void debug_updata(void)
{
    static float t;
    t += 100;
    debug_frame.fdata[0] = cRobot.stVelt.fpVx;
    debug_frame.fdata[1] = cRobot.stVelt.fpVy;
    debug_frame.fdata[2] = cRobot.stVelt.fpW;

    debug_frame.fdata[3] = cRobot.stPot.fpPosX;
    debug_frame.fdata[4] = cRobot.stPot.fpPosY;
    debug_frame.fdata[5] = cRobot.stPot.fpPosQ;

    // 导航路径相关
    debug_frame.fdata[6] = nav.expect_robot_global_velt.fpVx;
    debug_frame.fdata[7] = nav.expect_robot_global_velt.fpVy;
    debug_frame.fdata[8] = nav.expect_robot_global_velt.fpW;

    debug_frame.fdata[9] = nav.auto_path.basic_velt.fpVx;
    debug_frame.fdata[10] = nav.auto_path.basic_velt.fpVy;
    debug_frame.fdata[11] = nav.auto_path.basic_velt.fpW;

    debug_frame.fdata[12] = nav.auto_path.pos_pid.x.fpDes;
    debug_frame.fdata[13] = nav.auto_path.pos_pid.y.fpDes;
    debug_frame.fdata[14] = nav.auto_path.pos_pid.w.fpDes;

    debug_frame.fdata[15] = nav.auto_path.run_time;

    // 底盘运动电机相关
    // 速度环
    debug_frame.fdata[16] = Omni_chassis[0].m_run_motor.velt_pid.fpDes;
    debug_frame.fdata[17] = Omni_chassis[1].m_run_motor.velt_pid.fpDes;
    debug_frame.fdata[18] = Omni_chassis[2].m_run_motor.velt_pid.fpDes;
    debug_frame.fdata[19] = Omni_chassis[3].m_run_motor.velt_pid.fpDes;

    debug_frame.fdata[20] = Omni_chassis[0].m_run_motor.velt_pid.fpFB;
    debug_frame.fdata[21] = Omni_chassis[1].m_run_motor.velt_pid.fpFB;
    debug_frame.fdata[22] = Omni_chassis[2].m_run_motor.velt_pid.fpFB;
    debug_frame.fdata[23] = Omni_chassis[3].m_run_motor.velt_pid.fpFB;

    // 电流
    debug_frame.fdata[24] = Omni_chassis[0].m_run_motor.velt_pid.fpU;
    debug_frame.fdata[25] = Omni_chassis[1].m_run_motor.velt_pid.fpU;
    debug_frame.fdata[26] = Omni_chassis[2].m_run_motor.velt_pid.fpU;
    debug_frame.fdata[27] = Omni_chassis[3].m_run_motor.velt_pid.fpU;

    debug_frame.fdata[28] = Omni_chassis[0].m_run_motor.feed_forward_current;
    debug_frame.fdata[29] = Omni_chassis[1].m_run_motor.feed_forward_current;
    debug_frame.fdata[30] = Omni_chassis[2].m_run_motor.feed_forward_current;
    debug_frame.fdata[31] = Omni_chassis[3].m_run_motor.feed_forward_current;

    debug_frame.fdata[32] = Omni_chassis[0].m_run_motor.real_current;
    debug_frame.fdata[33] = Omni_chassis[1].m_run_motor.real_current;
    debug_frame.fdata[34] = Omni_chassis[2].m_run_motor.real_current;
    debug_frame.fdata[35] = Omni_chassis[3].m_run_motor.real_current;

    // 电机的帧率
    debug_frame.fdata[40] = Sys_Monitor.rate_monitor.real_rate[0];
    debug_frame.fdata[41] = Sys_Monitor.rate_monitor.real_rate[1];
    debug_frame.fdata[42] = Sys_Monitor.rate_monitor.real_rate[2];
    debug_frame.fdata[43] = Sys_Monitor.rate_monitor.real_rate[3];
    debug_frame.fdata[44] = Sys_Monitor.rate_monitor.real_rate[4];

    debug_frame.fdata[46] = cRobot.stDt35_now.dt35_x1;
    debug_frame.fdata[47] = cRobot.stDt35_now.dt35_x2;
    debug_frame.fdata[48] = cRobot.stDt35_now.dt35_y1;
    debug_frame.fdata[49] = cRobot.stDt35_now.dt35_y2;

    debug_frame.fdata[50] = cRobot.stDt35_now.robot_x;
    debug_frame.fdata[51] = cRobot.stDt35_now.robot_y;

    debug_frame.fdata[52] = cRobot.stDt35_now.pro_x1;
    debug_frame.fdata[53] = cRobot.stDt35_now.pro_x2;
    debug_frame.fdata[54] = cRobot.stDt35_now.pro_y1;
    debug_frame.fdata[55] = cRobot.stDt35_now.pro_y2;
}
