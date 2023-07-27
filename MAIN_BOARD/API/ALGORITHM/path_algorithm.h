#ifndef __PATH_ALGORITHM_H__
#define __PATH_ALGORITHM_H__

#include "flag_global.h"
#include "basic_type.h"
#include "math.h"
#include "math_algorithm.h"
#include "pid_algorithm.h"

using _navigation_::flag_record;

#define POS_1_X  -568.21f
#define POS_1_Y  739.69f
#define POS_1_Q  -90.71f

#define POS_2_X  -489.0f
#define POS_2_Y  1091.0f
#define POS_2_Q  -125.41f

#define POS_3_X  -207.8f
#define POS_3_Y  1313.0f
#define POS_3_Q  -160.53f

#define POS_4_X  164.0f
#define POS_4_Y  1344.5f
#define POS_4_Q  -196.05f

#define POS_5_X  459.0f
#define POS_5_Y  1135.0f
#define POS_5_Q  -232.50f

#define POS_6_X  581.5f
#define POS_6_Y  779.5f
#define POS_6_Q  -269.56f

#define POS_END_X  640.0f
#define POS_END_Y  -1794.4f
#define POS_END_Q  -359.0f

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
    C_PID x; // �?坐标X（单位：mm�?
    C_PID y; // 竖坐标Y（单位：mm�?
    C_PID w; // �?向�?�Q（单位：0.1度）
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
