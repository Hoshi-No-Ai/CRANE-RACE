#include "locate_algorithm.h"

/****************定位使用******************/
fp32 fpPosXOffset = 0; // X方向纠偏量，勿动
fp32 fpPosYOffset = 0; // Y方向纠偏量，勿动
fp32 fpQOffset = 0;    // 角度Q纠偏量，勿动

fp32 fpStartX = 0; // 5557.5f; 底盘半宽+导轮
fp32 fpStartY = 0; // 545.0f;

fp32 cha_x; // 插值因子

/***************测速使用*******************/
static fp32 afpRobot_Q[4] = {0};       // 存储机器人姿态角
static uint32_t auiTIM2_Time[4] = {0}; // 存储TIM2计数值
static fp32 afpRobot_PosX[4] = {0};    // 存储机器人X位姿
static fp32 afpRobot_PosY[4] = {0};    // 存储机器人Y位姿

C_ROBOT cRobot;

/*******************************************************************************************
函数名称：Push_Q_In()
函数功能：将新的角度值存入数组中
输入：	  fpQ  待存入的角度值
输出：	  无
备注：
*******************************************************************************************/
void Push_Q_In(fp32 fpQ)
{
    uint8_t i;
    for (i = 3; i >= 1; i--)
    {
        afpRobot_Q[i] = afpRobot_Q[i - 1];
    }
    afpRobot_Q[0] = fpQ;
}
/*******************************************************************************************
函数名称：Push_Time_In()
函数功能：将新的计数值存入数组中
输入：	  uiT  待存入的计数值
输出：	  无
备注：
*******************************************************************************************/
void Push_Time_In(uint32_t uiT)
{
    uint8_t i;
    for (i = 3; i >= 1; i--)
    {
        auiTIM2_Time[i] = auiTIM2_Time[i - 1];
    }
    auiTIM2_Time[0] = uiT;
}
/*******************************************************************************************
函数名称：Push_RobotPosX_In()
函数功能：将机器人新的X位姿存入数组
输入：	  fpX  待存入的计数值
输出：	  无
备注：
*******************************************************************************************/
void Push_RobotPosX_In(fp32 fpX)
{
    uint8_t i;
    for (i = 3; i >= 1; i--)
    {
        afpRobot_PosX[i] = afpRobot_PosX[i - 1];
    }
    afpRobot_PosX[0] = fpX;
}
/*******************************************************************************************
函数名称：Push_RobotPosY_In()
函数功能：将机器人新的Y位姿存入数组
输入：	  fpX  待存入的计数值
输出：	  无
备注：
*******************************************************************************************/
void Push_RobotPosY_In(fp32 fpY)
{
    uint8_t i;
    for (i = 3; i >= 1; i--)
    {
        afpRobot_PosY[i] = afpRobot_PosY[i - 1];
    }
    afpRobot_PosY[0] = fpY;
}

fp32 C_GYRO::get_gyro_value(void)
{
    static bool change_flag = 0;
    static int32_t gyro_value[2] = {0}; // 该数组为最后两次的陀螺仪数值，去掉所有重复的数值
    int32_t gyro_value_now;

    gyro_value_now = TestAngle2;

    if (gyro_value_now != gyro_value[1]) // 如果陀螺仪数据发生了变化，则更新该数组
    {
        gyro_value[0] = gyro_value[1];
        gyro_value[1] = gyro_value_now;
        change_flag = 1;
    }
    else
    {
        change_flag = 0;
    }

    if (change_flag == 1)
    {
        return (fp32)(gyro_value[0] + gyro_value[1]) / 2;
    }
    else
    {
        return (fp32)(gyro_value[1]);
    }
}

/*******************************************************************************************
函数名称：Calibrate_Robot_Degree()
函数功能：由陀螺数据得到机器人的姿态角
备注：    1.函数中涉及的角度单位都是0.1度(以后的函数中Degree代表角度数，Angle则代表弧度数)
          2.逆时针和顺时针旋转均有标定系数在陀螺仪的结构体中声明
          3.机器人姿态角是指机器人坐标系y轴与全局坐标系Y轴的夹角
          4.随动轮中心姿态角是指机器人中心至随动轮中心的向量与全局坐标系Y轴的夹角
*******************************************************************************************/
static int temp_vo, temp_v, temp_q;
void C_GYRO::Calibrate_Robot_Degree(C_ROBOT &Robot_loc)
{
    //    SB维特
    //    temp_vo = (int)((uart1_efr.num1[3] << 8) | uart1_efr.num1[2]);
    //    temp_v = (int)((uart1_efr.num1[5] << 8) | uart1_efr.num1[4]);
    //    temp_q = (int)((uart1_efr.num2[5] << 8) | uart1_efr.num2[4]);

    //    fpW_orgin = temp_vo / 32768.0f * 2000.0f;
    //    fpW = temp_v / 32768.0f * 2000.0f;
    //    fpQ = temp_q / 32768.0f * 180.0f;

    fpQ_Cur = get_gyro_value();
#if DigtalGyro // 数字

    if (fabs(fpQ_Cur - fpQ_Pre) > 100) // 2ms转10度，不可能
    {
        Robot_loc.stPot.fpPosQ1 += 0;
    }
    else if (fpQ_Cur - fpQ_Pre >= 0) // 逆时针
    {
        Robot_loc.stPot.fpPosQ1 += (fpQ_Cur - fpQ_Pre) * fpAntiClock;
    }
    else // 顺时针
    {
        Robot_loc.stPot.fpPosQ1 += (fpQ_Cur - fpQ_Pre) * fpClock;
    }

    if (fpQ_Pre != fpQ_Cur)
    {
        fps_Gyro = 500 / cnt_Gyro;
        cnt_Gyro = 0;
    }
    else
        cnt_Gyro++;
    fpQ_Pre = fpQ_Cur; // 陀螺角度数据更新

#endif

#if AnologGyro              // 模拟
    if (fpQ_Cur >= fpQ_Pre) // 逆时针
    {
        Robot_loc.stPot.fpPosQ1 += (fpQ_Cur - fpQ_Pre) * fpAntiClock;
    }
    else // 顺时针
    {
        Robot_loc.stPot.fpPosQ1 += (fpQ_Cur - fpQ_Pre) * fpClock;
    }
    fpQ_Pre = fpQ_Cur; // 陀螺角度数据更新
#endif
}

// float pre_Q, now_Q, d_Q;
// void C_ROBOT::Cal_Robot_Degree(void)
//{

//    cGyro.Calibrate_Robot_Degree(*this);

//    now_Q = cGyro.fpQ;
//    d_Q = now_Q - pre_Q;

//    if (d_Q != 0)
//    {
//        d_Q = d_Q;
//    }

//    if (d_Q > 180.0f)
//    {
//        d_Q -= 360.0f;
//    }
//    else if (d_Q < -180.0f)
//    {
//        d_Q += 360.0f;
//    }

//    stPot.fpPosQ += d_Q;
//}

void C_ROBOT::Cal_Robot_Degree(void)
{
    //    SB维特
    //    temp_vo = (int)((uart1_efr.num1[3] << 8) | uart1_efr.num1[2]);
    //    temp_v = (int)((uart1_efr.num1[5] << 8) | uart1_efr.num1[4]);
    //    temp_q = (int)((uart1_efr.num2[5] << 8) | uart1_efr.num2[4]);

    //    fpW_orgin = temp_vo / 32768.0f * 2000.0f;
    //    fpW = temp_v / 32768.0f * 2000.0f;
    //    fpQ = temp_q / 32768.0f * 180.0f;

    cGyro.fpQ_Cur = cGyro.get_gyro_value();
#if DigtalGyro // 数字

    if (fabs(cGyro.fpQ_Cur - cGyro.fpQ_Pre) > 100) // 2ms转10度，不可能
    {
        stPot.fpPosQ1 += 0;
    }
    else if (cGyro.fpQ_Cur - cGyro.fpQ_Pre >= 0) // 逆时针
    {
        stPot.fpPosQ1 += (cGyro.fpQ_Cur - cGyro.fpQ_Pre) * cGyro.fpAntiClock;
    }
    else // 顺时针
    {
        stPot.fpPosQ1 += (cGyro.fpQ_Cur - cGyro.fpQ_Pre) * cGyro.fpClock;
    }

    if (cGyro.fpQ_Pre != cGyro.fpQ_Cur)
    {
        cGyro.fps_Gyro = 500 / cGyro.cnt_Gyro;
        cGyro.cnt_Gyro = 0;
    }
    else
        cGyro.cnt_Gyro++;
    cGyro.fpQ_Pre = cGyro.fpQ_Cur; // 陀螺角度数据更新

#endif

#if AnologGyro              // 模拟
    if (fpQ_Cur >= fpQ_Pre) // 逆时针
    {
        Robot_loc.stPot.fpPosQ1 += (fpQ_Cur - fpQ_Pre) * fpAntiClock;
    }
    else // 顺时针
    {
        Robot_loc.stPot.fpPosQ1 += (fpQ_Cur - fpQ_Pre) * fpClock;
    }
    fpQ_Pre = fpQ_Cur; // 陀螺角度数据更新
#endif
}

/*******************************************************************************************
函数名称：RobotLocation()
函数功能：由双随动轮和陀螺仪数据得到机器人的坐标和姿态角（单位：mm，0.1°）
输入：	  1.Robot_loc 指向机器人总结构体的指针
          2.pstFW	 指向随动轮总结构体的指针
          3.pstGyro  指向陀螺总结构体的指针
输出：	  1.机器人中心位姿，包括坐标和姿态角
          2.随动轮中心位姿	包括坐标
备注：    1.此函数中的运算都是在弧度为单位的情况下进行的
          2.ALPHA_A为A随动轮逆时针旋转时线速度方向与机器人局部坐标系y正方向的夹角
              3.ALPHA_B为B随动轮逆时针旋转时线速度方向与机器人局部坐标系y正方向的夹角
          4.以上两个角度就是从y轴逆旋为正
*******************************************************************************************/
void C_ROBOT::RobotLocation()
{
    static C_LPF Fpx(20, 0.002);
    static C_LPF Fpy(20, 0.002);
    static fp32 temp_x, temp_y;
    fp32 ALPHA_B;
    fp32 ALPHA_A;
    fp32 Sin_B_A;
    fp32 fpDeltaA, fpDeltaB;  // 随动轮走过的距离
    fp32 Convert_Array[2][2]; // 距离转换矩阵，实际是3*3的，但是我们只用2*2
    fp32 fpQ;                 // 机器人姿态角临时变量(单位：弧度)

    /*******************获取角度*********************/
    Cal_Robot_Degree();                        // 给stPot.fpPosQ1赋值,单位0.1度
    stPot.fpPosQ = stPot.fpPosQ1 + fpQOffset;  // 单位0.1度
    fpQ = ConvertAngle(stPot.fpPosQ * RADIAN); // 单位:弧度

    /**************获取随动轮方向位移****************/
    cFollowoerWheel.m_CoderACur = cFollowoerWheel.degreeA;
    cFollowoerWheel.m_CoderBCur = cFollowoerWheel.degreeB;

    if (my_intabs(cFollowoerWheel.m_CoderACur - cFollowoerWheel.m_CoderAPre) > 1000 ||
        my_intabs(cFollowoerWheel.m_CoderBCur - cFollowoerWheel.m_CoderBPre) > 1000) // 2ms 10cm不可能
    {
        cFollowoerWheel.m_CoderAPre = cFollowoerWheel.m_CoderACur;
        cFollowoerWheel.m_CoderBPre = cFollowoerWheel.m_CoderBCur;
        return;
    }

    /*计算随动轮走过的距离*/
    if (cFollowoerWheel.m_CoderACur >= cFollowoerWheel.m_CoderAPre) // A轮正转，正转编码器值变小
    {
        fpDeltaA = (cFollowoerWheel.m_CoderACur - cFollowoerWheel.m_CoderAPre) * FW_Len_A_Inc;
        ALPHA_A = ALPHA_A_Inc;
    }
    else // A轮反转
    {
        fpDeltaA = (cFollowoerWheel.m_CoderACur - cFollowoerWheel.m_CoderAPre) * FW_Len_A_Dec;
        ALPHA_A = ALPHA_A_Dec;
    }
    if (cFollowoerWheel.m_CoderBCur >= cFollowoerWheel.m_CoderBPre) // B轮正转
    {
        fpDeltaB = (cFollowoerWheel.m_CoderBCur - cFollowoerWheel.m_CoderBPre) * FW_Len_B_Inc;
        ALPHA_B = ALPHA_B_Inc;
    }
    else // B轮反转
    {
        fpDeltaB = (cFollowoerWheel.m_CoderBCur - cFollowoerWheel.m_CoderBPre) * FW_Len_B_Dec;
        ALPHA_B = ALPHA_B_Dec;
    }

    /**************解算随动轮中心坐标****************/
    Sin_B_A = sinf(ALPHA_B - ALPHA_A); // 转换矩阵分母系数sin(B-A)

    Convert_Array[0][0] = cosf(ALPHA_B + fpQ) / Sin_B_A;
    Convert_Array[0][1] = cosf(ALPHA_A + fpQ) / Sin_B_A;
    Convert_Array[1][0] = sinf(ALPHA_B + fpQ) / Sin_B_A;
    Convert_Array[1][1] = sinf(ALPHA_A + fpQ) / Sin_B_A;

    cFollowoerWheel.stPot.fpPosX += Convert_Array[0][0] * fpDeltaA - Convert_Array[0][1] * fpDeltaB;
    cFollowoerWheel.stPot.fpPosY += Convert_Array[1][0] * fpDeltaA - Convert_Array[1][1] * fpDeltaB;

    /**************解算机器人中心坐标****************/
    /*任何位置都适用，其中要注意FW_rob_Alpha为机器人中心指向随动轮中心的矢量(同样要求从y轴开始逆时针旋转为正)，该角度范围为[0，2*pi)*/
    stPot.fpPosX =
        fpStartX + cFollowoerWheel.stPot.fpPosX -
        (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ)) * cFollowoerWheel.m_VectorFWCen_RobCen.fpLength;
    stPot.fpPosY =
        fpStartY + cFollowoerWheel.stPot.fpPosY +
        (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ)) * cFollowoerWheel.m_VectorFWCen_RobCen.fpLength;

    stPot.fpPosX = stPot.fpPosX + fpPosXOffset;
    stPot.fpPosY = stPot.fpPosY + fpPosYOffset;

    temp_x = stPot.fpPosY;
    temp_y = -stPot.fpPosX;

    stPot.fpPosX = temp_x;
    stPot.fpPosY = temp_y;

    //	Fpx.in = stPot.fpPosX;
    //	Fpy.in = stPot.fpPosY;
    //
    //	LpFilter(&Fpx);
    //	LpFilter(&Fpy);
    //
    //	stPot.fpPosX = Fpx.out;
    //	stPot.fpPosY = Fpy.out;
    //

    /*随动轮编码器数据保存*/
    cFollowoerWheel.m_CoderAPre = cFollowoerWheel.m_CoderACur;
    cFollowoerWheel.m_CoderBPre = cFollowoerWheel.m_CoderBCur;
}

/*******************************************************************************************
函数名称：CalibrateRobotVelt()
函数功能：校准机器人的速度，由求得的坐标增量计算机器人在全局坐标系下的速度（单位mm/s）
输入：	  1.Robot_loc 指向机器人总结构体的指针
输出：	  1.机器人在全局坐标系下的速度
备注：    1.函数中角速度的单位是(0.1度/秒)
*******************************************************************************************/
void C_ROBOT::Cal_RobotVelt(void)
{
    static C_LPF Fvx(200, 0.002);
    static C_LPF Fvy(200, 0.002);
    static C_LPF Fw(200, 0.002);

    /*存储机器人坐标、姿态角、计数器计数值的数组更新*/
    Push_Q_In(stPot.fpPosQ);
    Push_Time_In(TIM2->CNT);
    Push_RobotPosX_In(stPot.fpPosX);
    Push_RobotPosY_In(stPot.fpPosY);

    /*滤波*/
    Fvx.m_in = (afpRobot_PosX[0] - afpRobot_PosX[3]) /
               ((auiTIM2_Time[0] - auiTIM2_Time[3]) * TIM2_BASE_TIME); // x方向速度
    Fvy.m_in = (afpRobot_PosY[0] - afpRobot_PosY[3]) /
               ((auiTIM2_Time[0] - auiTIM2_Time[3]) * TIM2_BASE_TIME); // y方向速度
    Fw.m_in = (afpRobot_Q[0] - afpRobot_Q[3]) / ((auiTIM2_Time[0] - auiTIM2_Time[3]) * TIM2_BASE_TIME) /
              10.0f; // 角速度，方向从y轴逆转为正

    Fvx.LpFilter();
    Fvy.LpFilter();
    Fw.LpFilter();

    stVelt.fpVx = Fvx.m_out;
    stVelt.fpVy = Fvy.m_out;
    stVelt.fpW = Fw.m_out;
}

float temp_x_dt35, temp_y_dt35;
float temp_x1_dt35, temp_y1_dt35;
float temp_x2_dt35, temp_y2_dt35;
float temp_y_hide_dt35, temp_y1_hide_dt35, temp_y2_hide_dt35;
float temp_x_hide_dt35, temp_x1_hide_dt35, temp_x2_hide_dt35;
bool flag_x_dt35, flag_y_dt35;

void C_ROBOT::DT35_relocation_new(void)
{
    // 四个DT35，在rightdown轮对应的边沿x正方向为y1,y2；在up轮对应的边沿y正方向为x2,x1
    static fp32 fpQ_now, fpQ_save;
    stDt35_now.robot_q = stPot.fpPosQ;
    // x:4du
    fpQ_save = ConvertAngle(stDt35_save.robot_q * RADIAN); // 弧度
    fpQ_now = ConvertAngle(stDt35_now.robot_q * RADIAN);   // 弧度

    // dt35到墙的距离，
    stDt35_save.dt35_fix_x1 = fabs(K_DT35_X1 * stDt35_save.dt35_x1 + B_DT35_X1);
    stDt35_save.dt35_fix_x2 = fabs(K_DT35_X2 * stDt35_save.dt35_x2 + B_DT35_X2);
    stDt35_now.dt35_fix_x1 = fabs(K_DT35_X1 * stDt35_now.dt35_x1 + B_DT35_X1);
    stDt35_now.dt35_fix_x2 = fabs(K_DT35_X2 * stDt35_now.dt35_x2 + B_DT35_X2);

    stDt35_save.dt35_fix_y1 = fabs(K_DT35_Y1 * stDt35_save.dt35_y1 + B_DT35_Y1);
    stDt35_save.dt35_fix_y2 = fabs(K_DT35_Y2 * stDt35_save.dt35_y2 + B_DT35_Y2);
    stDt35_now.dt35_fix_y1 = fabs(K_DT35_Y1 * stDt35_now.dt35_y1 + B_DT35_Y1);
    stDt35_now.dt35_fix_y2 = fabs(K_DT35_Y2 * stDt35_now.dt35_y2 + B_DT35_Y2);

    // dt35所在边的中心的x或y距离

    stDt35_save.pro_x1 = stDt35_save.dt35_fix_x1 * cosf(fpQ_save) - DIS_X1_DT35_TO_C1 * sinf(fpQ_save) +
                         DIS_X1_DT35_TO_C2 * cosf(fpQ_save);
    stDt35_now.pro_x1 = stDt35_now.dt35_fix_x1 * cosf(fpQ_now) - DIS_X1_DT35_TO_C1 * sinf(fpQ_now) +
                        DIS_X1_DT35_TO_C2 * cosf(fpQ_now);
    stDt35_save.pro_x2 = stDt35_save.dt35_fix_x2 * cosf(fpQ_save) + DIS_X2_DT35_TO_C1 * sinf(fpQ_save) +
                         DIS_X2_DT35_TO_C2 * cosf(fpQ_save);
    stDt35_now.pro_x2 = stDt35_now.dt35_fix_x2 * cosf(fpQ_now) + DIS_X2_DT35_TO_C1 * sinf(fpQ_now) +
                        DIS_X2_DT35_TO_C2 * cosf(fpQ_now);

    stDt35_save.pro_y1 = stDt35_save.dt35_fix_y1 * cosf(fpQ_save) + DIS_Y1_DT35_TO_C1 * sinf(fpQ_save) +
                         DIS_Y1_DT35_TO_C2 * cosf(fpQ_save);
    stDt35_now.pro_y1 = stDt35_now.dt35_fix_y1 * cosf(fpQ_now) + DIS_Y1_DT35_TO_C1 * sinf(fpQ_now) +
                        DIS_Y1_DT35_TO_C2 * cosf(fpQ_now);
    stDt35_save.pro_y2 = stDt35_save.dt35_fix_y2 * cosf(fpQ_save) - DIS_Y2_DT35_TO_C1 * sinf(fpQ_save) +
                         DIS_Y2_DT35_TO_C2 * cosf(fpQ_save);
    stDt35_now.pro_y2 = stDt35_now.dt35_fix_y2 * cosf(fpQ_now) - DIS_Y2_DT35_TO_C1 * sinf(fpQ_now) +
                        DIS_Y2_DT35_TO_C2 * cosf(fpQ_now);

    // 注意坐标轴正负方向
    stDt35_now.robot_x =
        stDt35_save.robot_x -
        (stDt35_now.pro_x1 - stDt35_save.pro_x1 + stDt35_now.pro_x2 - stDt35_save.pro_x2) / 2;
    stDt35_now.robot_x1 = stDt35_save.robot_x - (stDt35_now.pro_x1 - stDt35_save.pro_x1);
    stDt35_now.robot_x2 = stDt35_save.robot_x - (stDt35_now.pro_x2 - stDt35_save.pro_x2);

    stDt35_now.robot_y =
        stDt35_save.robot_y +
        (stDt35_now.pro_y2 - stDt35_save.pro_y2 + stDt35_now.pro_y1 - stDt35_save.pro_y1) / 2;
    stDt35_now.robot_y1 = stDt35_save.robot_y + (stDt35_now.pro_y1 - stDt35_save.pro_y1);
    stDt35_now.robot_y2 = stDt35_save.robot_y + (stDt35_now.pro_y2 - stDt35_save.pro_y2);

    temp_x_dt35 = -stDt35_now.robot_y;
    temp_y_dt35 = stDt35_now.robot_x1;

    flag_x_dt35 = 1;
    flag_y_dt35 = 1;
    if (fabs(stDt35_now.robot_x - stPot.fpPosX) <= LIMIT_X &&
        fabs(stDt35_now.pro_x1 - stDt35_now.pro_x2) <= LIMIT_DIS_X) // 防止阶跃
    {
        stDt35_now.robot_x = stDt35_now.robot_x;
    }
    else if (fabs(stDt35_now.robot_x1 - stPot.fpPosX) <= LIMIT_X1)
    {
        stDt35_now.robot_x = stDt35_now.robot_x1;
    }
    else if (fabs(stDt35_now.robot_x2 - stPot.fpPosX) <= LIMIT_X2)
    {
        stDt35_now.robot_x = stDt35_now.robot_x2;
    }
    else
    {
        flag_x_dt35 = 0;
    }

    if (fabs(stDt35_now.robot_y - stPot.fpPosY) <= LIMIT_Y &&
        fabs(stDt35_now.pro_y1 - stDt35_now.pro_y2) <= LIMIT_DIS_Y)
    {
        stDt35_now.robot_y = stDt35_now.robot_y;
    }
    else if (fabs(stDt35_now.robot_y1 - stPot.fpPosY) <= LIMIT_Y1)
    {
        stDt35_now.robot_y = stDt35_now.robot_y1;
    }
    else if (fabs(stDt35_now.robot_y2 - stPot.fpPosY) <= LIMIT_Y2)
    {
        stDt35_now.robot_y = stDt35_now.robot_y2;
    }
    else
    {
        flag_y_dt35 = 0;
    }

    if (flag_y_dt35) // 非耦合时可用
    {
        temp_x_dt35 = -stDt35_now.robot_y;
        cFollowoerWheel.stPot.fpPosX = temp_x_dt35 + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) *
                                                         cFollowoerWheel.m_VectorFWCen_RobCen.fpLength;
        cFollowoerWheel.stPot.fpPosX1 = temp_x_dt35 + (-sinf(-FW_rob_Alpha) + sinf(-FW_rob_Alpha - fpQ_now)) *
                                                          cFollowoerWheel.m_VectorFWCen_RobCen.fpLength;
    }
    if (flag_x_dt35)
    {
        temp_y_dt35 = stDt35_now.robot_x;
        cFollowoerWheel.stPot.fpPosY = temp_y_dt35 - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) *
                                                         cFollowoerWheel.m_VectorFWCen_RobCen.fpLength;
        cFollowoerWheel.stPot.fpPosY1 = temp_y_dt35 - (cosf(-FW_rob_Alpha) - cosf(-FW_rob_Alpha - fpQ_now)) *
                                                          cFollowoerWheel.m_VectorFWCen_RobCen.fpLength;
    }
}

void C_ROBOT::DT35_Q_relocation(void)
{
    static float DT35_Q;
    stDt35_now.dt35_fix_x1 = fabs(K_DT35_X1 * stDt35_now.dt35_x1 + B_DT35_X1);
    stDt35_now.dt35_fix_x2 = fabs(K_DT35_X2 * stDt35_now.dt35_x2 + B_DT35_X2);

    DT35_Q = atan2f(stDt35_now.dt35_fix_x1 - stDt35_now.dt35_fix_x2, DIS_X1_DT35_TO_C1 + DIS_X2_DT35_TO_C1) /
             RADIAN;
}
