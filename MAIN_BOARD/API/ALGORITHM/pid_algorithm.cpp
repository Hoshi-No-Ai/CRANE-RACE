/**********************************************************************************************************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：PID_Algorithm.c
最近修改日期：2016.10.13
版本：1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
模块描述：
函数列表：
----------------------------------------------------------------------------------------------------------------------------------------------------------
修订记录：
     作者        	时间            版本     	说明
  JIN    2022.10.13      	2.0      划分建立此模块
**********************************************************************************************************************************************************/

#include "pid_algorithm.h"

/*以下为位置型PID*/
/*******************************************************************
函数名称：CalPID(ST_PID *this)
函数功能：普通的PID算法计算PID量
备    注：
********************************************************************/
void C_PID::CalPID(void)
{
    fpE = fpDes - fpFB;      // 计算当前偏差
    if (fabs(fpE) <= fpEMin) // 偏差死区限制
    {
        fpE = 0;
        fpSumE = 0;
    }
    fpSumE += fpE;
    /*位置式PID计算公式*/
    fpUp = fpKp * fpE;
    fpUi = fpKi * fpSumE;
    fpUd = fpKd * (fpE - fpPreE) / 0.001f;
    fpU = fpUp + fpUi + fpUd;
    fpPreE = fpE; // 保存本次偏差
    /*PID运算输出限幅*/
    fpU = ClipFloat(fpU, -fpUMax, fpUMax);
}

/*******************************************************************
函数名称：CalISeparatedPID(ST_PID *this)
函数功能：积分分离式PID算法计算PID量
备    注：积分分离式PID改进算法可减小启动、停止或大幅度增减时较大偏差
          对积分项的积累，从而避免出现较大的超调及振荡现象。
********************************************************************/
void C_PID::CalISeparatedPID(void)
{
    uint8_t uck = 1;

    fpE = fpDes - fpFB;      // 计算当前偏差
    if (fabs(fpE) <= fpEMin) // 偏差死区限制
    {
        fpE = 0;
    }
    fpSumE += fpE; // 计算偏差累积
    /*若偏差过大，则积分项不累积偏差*/
    if (fabs(fpE) > fpEMax) // 判断是否满足积分分离
    {
        fpSumE = 0;
        uck = 0;
    }
    /*位置式PID计算公式*/
    fpU = fpKp * fpE + fpKi * fpSumE * uck + fpKd * (fpE - fpPreE);
    fpPreE = fpE; // 保存本次偏差
    /*PID运算输出限幅*/
    fpU = ClipFloat(fpU, -fpUMax, fpUMax);
}

/*******************************************************************
函数名称：CalIResistedPID(ST_PID *this)
函数功能：抗积分饱和PID算法
备    注：系统往一个方向运动会产生较大积分误差，会在几个周期内产生振荡或超调
********************************************************************/
void C_PID::CalIResistedPID(void)
{
    fpE = fpDes - fpFB; // 计算当前偏差
    fpSumE += fpE;      // 计算偏差累积

    fpSumE = ClipFloat(fpSumE, -fpEiMax, fpEiMax);
    fpUi = fpKi * fpSumE;

    fpUp = ClipFloat(fpKp * fpE, -fpEpMax, fpEpMax);
    fpUd = ClipFloat(fpKd * (fpE - fpPreE), -fpEdMax, fpEdMax);

    /*若偏差在死区之内，则清零积分累计项*/
    if (fabs(fpE) < fpEMin) // 判断是否满足积分饱和条件
    {
        fpSumE = 0; // 清除偏差累积
    }
    /*位置式PID计算公式*/
    fpU = fpUp + fpUi + fpUd;

    fpPreE = fpE; // 保存本次偏差
    /*PID运算输出限幅*/
    fpU = ClipFloat(fpU, -fpUMax, fpUMax);
}
/*******************************************************************
函数名称：CalIWeakenPID(ST_PID *this)
函数功能：遇限削弱积分PID改进算法计算PID量
备    注：
********************************************************************/
void C_PID::CalIWeakenPID(void)
{
    fpE = fpDes - fpFB; // 计算当前偏差

    if (((fpU <= fpUMax && fpE > 0) || (fpU >= -fpUMax && fpE < 0)) && fabs(fpE) < fpEMin)
    {
        fpSumE += fpE; // 计算偏差累积
    }

    fpSumE = ClipFloat(fpSumE, -fpEiMax, fpEiMax);
    fpUi = fpKi * fpSumE;

    fpUp = ClipFloat(fpKp * fpE, -fpEpMax, fpEpMax);
    fpUd = ClipFloat(fpKd * (fpE - fpPreE), -fpEdMax, fpEdMax);

    /*位置式PID计算公式*/
    fpU = fpUp + fpUi + fpUd;

    fpPreE = fpE; // 保存本次偏差

    /*PID运算输出限幅*/
    fpU = ClipFloat(fpU, -fpUMax, fpUMax);
}

/*******************************************************************
函数名称：CalFilterPID(ST_PID *this)
函数功能：微分项PID输出
备    注：
********************************************************************/
void C_PID::CalFilterPID(void)
{
    //=======计算当前偏差===========
    fpE = fpDes - fpFB;
    //=======偏差死区限制========
    if (fabs(fpE) <= fpEMin)
    {
        fpE = 0.0f;
        fpUi = 0;
    }
    /*======位置式PID计算公式======*/
    fpUp = fpKp * fpE;         // 比例项输出
    fpUi += fpKi * fpE * fpTs; // 积分项输出
    // 微分项输出及微分滤波
    fpUd = fpKd * (fpE - fpPreE) / fpTs;

    //===========更新上次偏差===========
    fpPreE = fpE;
    //===========PID总输出===============
    fpU = fpUp + fpUi + fpUd;
    /*=========输出限幅============*/
    fpU = ClipFloat(fpU, -fpUMax, fpUMax);
}

///*******************************************************************
// 函数名称：CalComprehensivePID(ST_PID *this)
// 函数功能：综合PID输出
// 备    注：为调试所用，可参照以上PID进行综合，功能全面
//********************************************************************/
void C_PID::CalComprehensivePID(void)
{
    uint8_t uck = 0;
    static u8 kkk = 2;

    fpE = fpDes - fpFB; // 计算当前偏差
    fpSumE += fpE;
    /*积分抗饱和*/
    fpSumE = ClipFloat(fpSumE, -fpSumEMax, fpSumEMax);
    /*三个输出项限幅*/
    fpUp = ClipFloat(fpKp * fpE, -fpEpMax, fpEpMax);
    fpUi = ClipFloat(fpKi * fpSumE, -fpEiMax, fpEiMax);
    fpUd = ClipFloat(fpKd * (fpE - fpPreE), -fpEdMax, fpEdMax);
    /*积分分离*/
    if (fabs(fpE) >= fpEMax) // 判断是否满足积分分离
    {
        uck = 0;
    }
    else
        uck = 1;
    /*位置式PID计算公式*/
    if (fpUd > 0.1f)
    {
        lpf_PID.m_in = fpUd;
    }

    lpf_PID.LpFilter();

    if (fabs(fpE) <= fpEMin)
    {
        fpU = fpUp + kkk * fpUi * uck + lpf_PID.m_out;
    }
    else
    {
        fpU = fpUp + fpUi * uck + lpf_PID.m_out;
    }

    fpPreE = fpE; // 保存本次偏差
    /*PID运算输出限幅*/
    fpU = ClipFloat(fpU, -fpUMax, fpUMax);
}

void C_Tr::TrF1(fp32 t1, fp32 t2)
{
    fpOutput1 =
        ((2 * t1 + fpTs) * fpInput1 + (fpTs - 2 * t1) * fpInputpre1 - (fpTs - 2 * t2) * fpOutputpre1) /
        (2 * t2 + fpTs);
    fpInputpre1 = fpInput1;
    fpOutputpre1 = fpOutput1;
}

void C_Tr::TrF2(fp32 t)
{
    fpOutput2 = 2 * fpInput2 - 2 * fpInputpre2 - (fpTs - 2 * t) * fpOutputpre2;
    fpInputpre2 = fpInput2;
    fpOutputpre2 = fpOutput2;
}

void C_Tr::TrF3(fp32 t)
{
    fpOutput3 = fpInput3 - fpInputpre3 - (fpTs - 2 * t) * fpOutputpre3;
    fpInputpre3 = fpInput3;
    fpOutputpre3 = fpOutput3;
}

void C_Tr::LagCompensator(fp32 gain, fp32 t1, fp32 t2) // 滞后校正，t2为控制周期取
{
    fpOutput1 =
        ((2 * t1 + t2) * fpInput1 + (t2 - 2 * t1) * fpInputpre1 + (2 * gain * t1 - t2) * fpOutputpre1) /
        (2 * t1 * gain + t2);
    fpInputpre1 = fpInput1;
    fpOutputpre1 = fpOutput1;
}

void C_TD::TD_Function(void)
{
    float d, d0, y, a0, a = 0;
    m_x = m_x1 - m_aim;
    d = m_r * m_h;
    d0 = m_h * d;
    y = m_x + m_h * m_x2;
    a0 = sqrt(d * d + 8 * m_r * fabs(y));

    if (fabs(y) > d0)
        a = m_x2 + (a0 - d) * Sign_Judge(y) / 2;
    else
        a = m_x2 + y / m_h;

    if (fabs(a) > d)
        y = -1 * m_r * Sign_Judge(a);
    else
        y = -1 * m_r * a / d;

    m_x1 += 0.001f * m_x2;
    m_x2 += 0.001f * y;
}
