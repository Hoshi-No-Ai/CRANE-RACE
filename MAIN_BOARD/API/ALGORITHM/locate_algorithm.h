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

/***************************************************��λ********************************************************/
/*ģ�����ݡ����������л�*/
#define DigtalGyro 1
#define AnologGyro 0

/*������ϵ��*/
#define K_ANTICLOCK 1.0237613751263902932254802831142568250758f
#define K_CLOCK 1.0237613751263902932254802831142568250758f

/*�涯��ϵ��*/
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

/*�������������涯�����������Ƕ�(ȫ��Y)��ģ*/
#define FW_Rob_Len 118.18f  // �涯����������������ĵľ���
#define FW_rob_Alpha 0.0f // �涯�����������������ļнǣ���λ�����ȣ�

/* �Ӿ��ض�λ���� */
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
DT35ϵ��
                |y+
                |dt35_y
                |
x+����������������������������������������������������dt35_x
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

// DT35ϵ��
#define K_DT35_X1 -1.112127110963998f
#define B_DT35_X1 -8.118602110400346f

#define K_DT35_X2 -1.003652393891736f
#define B_DT35_X2 -1.330248591758925e+02f

#define K_DT35_Y1 1.012553393969609f
#define B_DT35_Y1 1.238437436213362e+02f

#define K_DT35_Y2 1.043767942624321f
#define B_DT35_Y2 60.577194894797600f

// ����
#define DIS_X1_DT35_TO_C1 109.34f // 293.275f
#define DIS_X1_DT35_TO_C2 172.98f // 297.25f
#define DIS_X2_DT35_TO_C1 60.96f  // 293.275f
#define DIS_X2_DT35_TO_C2 222.98f // 297.25f
#define DIS_Y1_DT35_TO_C1 158.15f // 261.0f
#define DIS_Y1_DT35_TO_C2 116.81f // 174.99f
#define DIS_Y2_DT35_TO_C1 158.15f // 110.0f
#define DIS_Y2_DT35_TO_C2 116.81f // 334.99f

/*dt35����ת���ĵĸ�������*/
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

/*�涯�����̹��߼������涯����ؽṹ��*/
class C_FOLLOWER_WHEEL
{
public:
    int32_t m_CoderACur; // ��ǰ����A����
    int32_t m_CoderAPre; // ��һ������A�������ж��涯����ת����
    int32_t m_CoderBCur; // ��ǰ����B����
    int32_t m_CoderBPre; // ��һ������B����

    float degreeA;
    float degreeB;

    C_VECTOR m_VectorFWCen_RobCen; // �涯����������������ĵ���������Ҫ��y����ԽǶȺ;��룩

    ST_POT stPot;    // �涯������������̬
    ST_POT stPotPre; // �涯���ϴ���������

    C_FOLLOWER_WHEEL() : m_VectorFWCen_RobCen(C_VECTOR(FW_Rob_Len, FW_rob_Alpha, POLAR)){};
    ~C_FOLLOWER_WHEEL(){};
};

/*���ݽṹ��*/
class C_GYRO
{
public:
    //    SBά��
    //    fp32 fpQ;       //���ݵ�ǰ���ݶ���
    //		fp32 fpW_orgin; //������ԭʼ���ٶ�
    //	  fp32 fpW;       //������У������ٶ�

    fp32 fpClock;     // ˳ʱ��ϵ��
    fp32 fpAntiClock; // ��ʱ��ϵ��
    fp32 fpQ_Cur;     // ���ݵ�ǰ���ݶ���
    fp32 fpQ_Pre;     // ������һ�����ݶ������ж���ת����

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

/*DT35��λ�ṹ��*/
// �ĸ�DT35����UP��������DT35��x������Ϊy1��y2����LEFT��������DT35��y������Ϊx1��x2
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
    ST_POT stPot;     // ����������������̬
    ST_POT stPotPre;  // �������ϴ�����������̬
    ST_POT stPotFeed; // ������ǰ������������
    ST_VELT stVelt;   // ������������ȫ������ϵ�µ��ٶ�

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
