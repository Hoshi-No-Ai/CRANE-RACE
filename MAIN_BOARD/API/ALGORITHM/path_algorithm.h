#ifndef __PATH_ALGORITHM_H__
#define __PATH_ALGORITHM_H__

#include "flag_global.h"
#include "basic_type.h"
#include "math.h"
#include "math_algorithm.h"
#include "pid_algorithm.h"

using _navigation_::flag_record;

#define POS_1_X -589.50f//-570.81f
#define POS_1_Y 740.16f//749.69f
#define POS_1_Q -89.68f//-89.89f

#define POS_2_X -497.93f//-470.80f
#define POS_2_Y 1090.47f//1093.09f
#define POS_2_Q -124.59f//-126.02f

#define POS_3_X -206.73f//-188.26f
#define POS_3_Y 1304.45f//1300.14f
#define POS_3_Q -160.73f//-161.35f

#define POS_4_X 154.49f//166.53f
#define POS_4_Y 1329.19f//1327.13f
#define POS_4_Q -197.28f//-197.28f

#define POS_5_X 470.13f//462.530f
#define POS_5_Y 1116.34f//1125.420f
#define POS_5_Q -232.70f//-232.80f

#define POS_6_X 592.51f//575.66f
#define POS_6_Y 771.52f//793.58f
#define POS_6_Q -268.23f//-268.33f

#define POS_END_X 630.15f//640.0f
#define POS_END_Y -1753.73f//-1794.4f
#define POS_END_Q -357.19f//-359.0f

class C_POINT
{
public:
    float m_x;
    float m_y;
    float m_q;
    C_POINT() {}
    C_POINT(float x, float y, float q)
    {
        m_x = x;
        m_y = y;
        m_q = q;
    }
    ~C_POINT() {}
    void point_set(float x, float y, float q)
    {
        m_x = x;
        m_y = y;
        m_q = q;
    }
};

class C_VELT_ACC
{
public:
    float Vmax;
    float A_up;
    float A_down;
    float Wmax;
    C_VELT_ACC() {}
    C_VELT_ACC(float vm, float wm, float au, float ad)
    {
        Vmax = vm;
        A_up = au;
        A_down = ad;
    }
    ~C_VELT_ACC() {}
    void Velt_Acc_Set(float vm, float wm, float au, float ad)
    {
        Vmax = vm;
        Wmax = wm;
        A_up = au;
        A_down = ad;
    }
};

class Robot_pid_t
{
public:
    C_PID x;
    C_PID y;
    C_PID w;
};

class C_AUTO_PATH
{
public:
    Robot_pid_t pos_pid, velt_pid;
    ST_VELT basic_velt;
    uint32_t run_time;
    uint8_t m_number;

    C_POINT m_point_end;
    C_VELT_ACC m_velt_acc;
    bool flag_path_end;

    C_AUTO_PATH() : m_point_end(0, 0, 0), m_velt_acc(2000, 90, 2000, 2000)
    {
        pos_pid.x.fpKp = 7.0f;
        pos_pid.x.fpUMax = 8000.0f;
        pos_pid.y.fpKp = 7.0f;
        pos_pid.y.fpUMax = 8000.0f;
        pos_pid.w.fpUMax = 90 * RADIAN;
    }
    ~C_AUTO_PATH() {}

    bool Path_Choose(void);

private:
    void path_straight(void);
    void path_x_test(void);
    void path_y_test(void);
};

#endif
