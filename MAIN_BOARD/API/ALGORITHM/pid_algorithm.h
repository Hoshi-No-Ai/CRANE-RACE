#ifndef __PID_ALGORITHM_H__
#define __PID_ALGORITHM_H__

#define SQUARE(x) ((x) * (x))

#include "basic_type.h"
#include "filter_algorithm.h"
#include "math.h"
#include "math_algorithm.h"
#include "stm32f4xx.h"

class C_PID
{
public:
    fp32 fpDes; // 控制变量目标值
    fp32 fpFB;  // 控制变量反馈值

    fp32 fpKp; // 比例系数Kp
    fp32 fpKi; // 积分系数Ki
    fp32 fpKd; // 微分系数Kd

    fp32 fpE;    // 本次偏差
    fp32 fpPreE; // 上次偏差
    fp32 fpSumE; // 总偏差
    fp32 fpInput;
    fp32 fpInputpre;
    fp32 fpOutput;
    fp32 fpOutputpre;
    fp32 fpEpMax;   // 比例项输出最大值
    fp32 fpEiMax;   // 积分项输出最大值
    fp32 fpEdMax;   // 微分项输出最大值
    fp32 fpEMin;    // 积分下限
    fp32 fpEMax;    // 积分上限
    fp32 fpSumEMax; // 积分项sumE最大值

    fp32 fpUp; // 比例输出
    fp32 fpUi; // 积分输出
    fp32 fpUd; // 微分输出
    fp32 fpU;  // 本次PID运算结果
    fp32 fpUpre;
    fp32 fpUMax; // PID运算后输出最大值及做遇限削弱时的上限值
    fp32 fpTs;   // PID控制周期，单位：s

    C_PID() {}
    C_PID(fp32 Kp, fp32 Ki, fp32 Kd, fp32 UMax, fp32 UiMax, fp32 UdMax, fp32 ts)
        : fpKp(Kp),
          fpKi(Ki),
          fpKd(Kd),
          fpUMax(UMax),
          fpEpMax(UMax),
          fpEiMax(UiMax),
          fpEdMax(UdMax),
          fpTs(ts) {} // initialize PID
    ~C_PID() {}

    void CalPID(void);
    void CalISeparatedPID(void);
    void CalIResistedPID(void);
    void CalIWeakenPID(void);
    void CalFilterPID(void);
    void CalComprehensivePID(void);
};

class C_Tr
{
public:
    fp32 fpInput1;
    fp32 fpInput2;
    fp32 fpInput3;
    fp32 fpInputpre1;
    fp32 fpInputpre2;
    fp32 fpInputpre3;
    fp32 fpOutput1;
    fp32 fpOutput2;
    fp32 fpOutput3;
    fp32 fpOutputpre1;
    fp32 fpOutputpre2;
    fp32 fpOutputpre3;
    fp32 fpTs;

    C_Tr() {}
    C_Tr(fp32 Ts) : fpTs(Ts) {} // 采样周期
    ~C_Tr() {}

    void TrF1(fp32 t1, fp32 t2);
    void TrF2(fp32 t);
    void TrF3(fp32 t);
    void LagCompensator(fp32 gain, fp32 t1, fp32 t2);
};

// 滑模相关
class C_TD
{
public:
    float m_x1;
    float m_x2;
    float m_x3;
    float m_x;
    float m_r;
    float m_h;
    float m_T;
    float m_aim;

    C_TD() {}
    C_TD(float r, float h, float T) : m_r(r), m_h(h), m_T(T) {}

    void TD_Function(void);

private:
    s32 Sign_Judge(float fp_Judge_Number) { return fp_Judge_Number >= 0 ? 1 : -1; }
};

#endif
