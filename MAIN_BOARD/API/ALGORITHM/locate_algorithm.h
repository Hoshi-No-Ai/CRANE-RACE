#ifndef __LOCATE_ALGORITHM_H__
#define __LOCATE_ALGORITHM_H__

#include "flag_global.h"
#include "FPGA_REGS.h"
#include "basic_type.h"
#include "usart_protocol.h"
#include "filter_algorithm.h"
#include "math_algorithm.h"
#include "movement_info.h"
#include "stm32f4xx.h"
#include "tim.h"

/***************************************************定位********************************************************/
/*模拟陀螺、数字陀螺切换*/
#define DigtalGyro 1
#define AnologGyro 0

/*陀螺仪系数*/
#define K_ANTICLOCK 1.0237613751263902932254802831142568250758f
#define K_CLOCK 1.0237613751263902932254802831142568250758f

/*随动轮系数*/
//2.3561944901923449288469825374596271631478
//0.7853981633974483096156608458198757210492
#define ALPHA_A_Inc -2.410565098551039f //-0.810965294108738f//2.3561944901923449288469825374596271631478f
#define ALPHA_A_Dec -2.410108197924082f //-0.808016683626560f//2.3561944901923449288469825374596271631478f
#define ALPHA_B_Inc 2.386849871549865f //0.790117598576705f //-2.3561944901923449288469825374596271631478f
#define ALPHA_B_Dec 2.385656091669817f //0.790250194252695f//-2.3561944901923449288469825374596271631478f

#define FW_Len_A_Inc -0.229548363662008f //-0.222884815442030f //-0.4376413099375781414136154601985196378958f
#define FW_Len_A_Dec -0.230235332900968f //-0.222745633406768f //-0.4376413099375781414136154601985196378958f
#define FW_Len_B_Inc 0.228858132233363f //-0.222875462099823f //-0.4376413099375781414136154601985196378958f
#define FW_Len_B_Dec 0.229011487586866f //-0.222404068161941f //-0.4376413099375781414136154601985196378958f

/*机器人中心至随动轮中心向量角度(全局Y)和模*/
#define FW_Rob_Len 118.18f  // 随动轮中心与机器人中心的距离
#define FW_rob_Alpha 0.0f // 随动轮坐标与机器人坐标的夹角（单位：弧度）

/* 视觉重定位参数 */
#define POS_1_X_RECLOC -578.84f
#define POS_1_Y_RECLOC 789.40f
#define POS_1_Q_RECLOC -89.58f

#define POS_2_X_RECLOC -466.69f
#define POS_2_Y_RECLOC 1146.99f
#define POS_2_Q_RECLOC -125.41f

#define POS_3_X_RECLOC -167.61f
#define POS_3_Y_RECLOC 1562.55f
#define POS_3_Q_RECLOC -159.09f

#define POS_4_X_RECLOC 203.35f
#define POS_4_Y_RECLOC 1367.31f
#define POS_4_Q_RECLOC -196.56f

#define POS_5_X_RECLOC 489.78f
#define POS_5_Y_RECLOC 1152.22f
#define POS_5_Q_RECLOC -234.34f

#define POS_6_X_RECLOC 596.72f
#define POS_6_Y_RECLOC 795.55f
#define POS_6_Q_RECLOC -268.33f

#define POS_END_X_RECLOC 590.26f
#define POS_END_Y_RECLOC -1673.39f
#define POS_END_Q_RECLOC -359.36f

/*
DT35系数
                |y+
                |dt35_y
                |
x+——————————————————————————dt35_x
                |
                |
                |
*/

#define DT35_SAVE_ROBOTX 0.0f
#define DT35_SAVE_ROBOTY 0.0f
#define DT35_SAVE_ROBOTQ 0.0f
#define DT35_SAVE_X1 0.0f
#define DT35_SAVE_X2 0.0f
#define DT35_SAVE_Y1 0.0f
#define DT35_SAVE_Y2 0.0f

// DT35系数
#define K_DT35_X1 -1.112127110963998f
#define B_DT35_X1 -8.118602110400346f

#define K_DT35_X2 -1.003652393891736f
#define B_DT35_X2 -1.330248591758925e+02f

#define K_DT35_Y1 1.012553393969609f
#define B_DT35_Y1 1.238437436213362e+02f

#define K_DT35_Y2 1.043767942624321f
#define B_DT35_Y2 60.577194894797600f

// 现用
#define DIS_X1_DT35_TO_C1 109.34f // 293.275f
#define DIS_X1_DT35_TO_C2 172.98f // 297.25f
#define DIS_X2_DT35_TO_C1 60.96f  // 293.275f
#define DIS_X2_DT35_TO_C2 222.98f // 297.25f
#define DIS_Y1_DT35_TO_C1 158.15f // 261.0f
#define DIS_Y1_DT35_TO_C2 116.81f // 174.99f
#define DIS_Y2_DT35_TO_C1 158.15f // 110.0f
#define DIS_Y2_DT35_TO_C2 116.81f // 334.99f

/*dt35与旋转中心的各个距离*/
#define DIS_X1_DT35_TO_MID (586.55f / 2.0)
#define DIS_X2_DT35_TO_MID (586.55f / 2.0)
#define DIS_Y1_DT35_TO_MID 305.0f
#define DIS_Y2_DT35_TO_MID 65.0f

#define LIMIT_X 30.0f
#define LIMIT_DIS_X 40.0f
#define LIMIT_X1 50.0f
#define LIMIT_X2 50.0f
#define LIMIT_Y 30.0f
#define LIMIT_DIS_Y 40.0f
#define LIMIT_Y1 50.0f
#define LIMIT_Y2 50.0f

enum pos_state_e {
    POS_1,
    POS_2,
    POS_3,
    POS_4,
    POS_5,
    POS_6,
    POS_END
};

extern fp32 fpStartX;
extern fp32 fpStartY;

class C_ROBOT;

/*随动轮码盘过线计数及随动轮相关结构体*/
class C_FOLLOWER_WHEEL
{
public:
    int32_t m_CoderACur; // 当前码盘A读数
    int32_t m_CoderAPre; // 上一次码盘A读数，判断随动轮旋转方向
    int32_t m_CoderBCur; // 当前码盘B读数
    int32_t m_CoderBPre; // 上一次码盘B读数

    float degreeA;
    float degreeB;

    C_VECTOR m_VectorFWCen_RobCen; // 随动轮中心与机器人中心的向量（主要是y轴相对角度和距离）

    ST_POT stPot;    // 随动轮中心坐标姿态
    ST_POT stPotPre; // 随动轮上次中心坐标

    C_FOLLOWER_WHEEL() : m_VectorFWCen_RobCen(C_VECTOR(FW_Rob_Len, FW_rob_Alpha, POLAR)){};
    ~C_FOLLOWER_WHEEL(){};
};

/*陀螺结构体*/
class C_GYRO
{
public:
    //    SB维特
    //    fp32 fpQ;       //陀螺当前数据读数
    //		fp32 fpW_orgin; //陀螺仪原始角速度
    //	  fp32 fpW;       //陀螺仪校正后角速度

    fp32 fpClock;     // 顺时针系数
    fp32 fpAntiClock; // 逆时针系数
    fp32 fpQ_Cur;     // 陀螺当前数据读数
    fp32 fpQ_Pre;     // 陀螺上一次数据读数，判断旋转方向

    uint32_t cnt_Gyro;
    uint32_t fps_Gyro;

    C_GYRO()
    {
        fpClock = K_CLOCK;
        fpAntiClock = K_ANTICLOCK;
    };
    ~C_GYRO(){};

    void Calibrate_Robot_Degree(C_ROBOT &cRobot);

    static fp32 get_gyro_value(void);
};

/*DT35定位结构体*/
// 四个DT35，在UP边上两个DT35沿x轴正向为y1、y2；在LEFT边上两个DT35沿y轴正向为x1、x2
struct ST_DT35
{
    fp32 robot_x, robot_y, robot_q;
    fp32 dt35_x1, dt35_x2, dt35_y1, dt35_y2;
    fp32 dt35_fix_x1, dt35_fix_x2, dt35_fix_y1, dt35_fix_y2;
    fp32 pro_x1, pro_x2, pro_y1, pro_y2;
    fp32 robot_x1, robot_x2, robot_y1, robot_y2;
};

class C_ROBOT
{
public:
    ST_POT stPot;     // 机器人中心坐标姿态
    ST_POT stPotPre;  // 机器人上次中心坐标姿态
    ST_POT stPotFeed; // 机器人前馈运算后的坐标
    ST_VELT stVelt;   // 机器人中心在全场坐标系下的速度

    C_GYRO cGyro;
    C_FOLLOWER_WHEEL cFollowoerWheel;
    ST_DT35 stDt35_save;
    ST_DT35 stDt35_now;

    C_ROBOT()
    {
        stDt35_save.robot_x = DT35_SAVE_ROBOTX;
        stDt35_save.robot_y = DT35_SAVE_ROBOTY;
        stDt35_save.robot_q = DT35_SAVE_ROBOTQ;
        stDt35_save.dt35_x1 = DT35_SAVE_X1;
        stDt35_save.dt35_x2 = DT35_SAVE_X2;
        stDt35_save.dt35_y1 = DT35_SAVE_Y1;
        stDt35_save.dt35_y2 = DT35_SAVE_Y2;
    };
    ~C_ROBOT(){};

    void Cal_Robot_Degree(void);
    void RobotLocation(void);
    void Cal_RobotVelt(void);
    void Aruco_relocation(aruco &aruco_rec, int pos_num, bool if_q_rec);
    void DT35_relocation_new(void);
    void DT35_Q_relocation(void);
};

extern C_ROBOT cRobot;

#endif
