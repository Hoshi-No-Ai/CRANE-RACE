#include "locate_algorithm.h"

/****************��λʹ��******************/
fp32 fpPosXOffset = 0; // X�����ƫ������
fp32 fpPosYOffset = 0; // Y�����ƫ������
fp32 fpQOffset = 0;    // �Ƕ�Q��ƫ������

fp32 fpStartX = 0; // 5557.5f; ���̰��+����
fp32 fpStartY = 0; // 545.0f;

fp32 cha_x; // ��ֵ����

/***************����ʹ��*******************/
static fp32 afpRobot_Q[4] = {0};       // �洢��������̬��
static uint32_t auiTIM2_Time[4] = {0}; // �洢TIM2����ֵ
static fp32 afpRobot_PosX[4] = {0};    // �洢������Xλ��
static fp32 afpRobot_PosY[4] = {0};    // �洢������Yλ��

C_ROBOT cRobot;

/*******************************************************************************************
�������ƣ�Push_Q_In()
�������ܣ����µĽǶ�ֵ����������
���룺	  fpQ  ������ĽǶ�ֵ
�����	  ��
��ע��
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
�������ƣ�Push_Time_In()
�������ܣ����µļ���ֵ����������
���룺	  uiT  ������ļ���ֵ
�����	  ��
��ע��
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
�������ƣ�Push_RobotPosX_In()
�������ܣ����������µ�Xλ�˴�������
���룺	  fpX  ������ļ���ֵ
�����	  ��
��ע��
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
�������ƣ�Push_RobotPosY_In()
�������ܣ����������µ�Yλ�˴�������
���룺	  fpX  ������ļ���ֵ
�����	  ��
��ע��
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
    static int32_t gyro_value[2] = {0}; // ������Ϊ������ε���������ֵ��ȥ�������ظ�����ֵ
    int32_t gyro_value_now;

    gyro_value_now = TestAngle2;

    if (gyro_value_now != gyro_value[1]) // ������������ݷ����˱仯������¸�����
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
�������ƣ�Calibrate_Robot_Degree()
�������ܣ����������ݵõ������˵���̬��
��ע��    1.�������漰�ĽǶȵ�λ����0.1��(�Ժ�ĺ�����Degree����Ƕ�����Angle���������)
          2.��ʱ���˳ʱ����ת���б궨ϵ���������ǵĽṹ��������
          3.��������̬����ָ����������ϵy����ȫ������ϵY��ļн�
          4.�涯��������̬����ָ�������������涯�����ĵ�������ȫ������ϵY��ļн�
*******************************************************************************************/
static int temp_vo, temp_v, temp_q;
void C_GYRO::Calibrate_Robot_Degree(C_ROBOT &Robot_loc)
{
    //    SBά��
    //    temp_vo = (int)((uart1_efr.num1[3] << 8) | uart1_efr.num1[2]);
    //    temp_v = (int)((uart1_efr.num1[5] << 8) | uart1_efr.num1[4]);
    //    temp_q = (int)((uart1_efr.num2[5] << 8) | uart1_efr.num2[4]);

    //    fpW_orgin = temp_vo / 32768.0f * 2000.0f;
    //    fpW = temp_v / 32768.0f * 2000.0f;
    //    fpQ = temp_q / 32768.0f * 180.0f;

    fpQ_Cur = get_gyro_value();
#if DigtalGyro // ����

    if (fabs(fpQ_Cur - fpQ_Pre) > 100) // 2msת10�ȣ�������
    {
        Robot_loc.stPot.fpPosQ1 += 0;
    }
    else if (fpQ_Cur - fpQ_Pre >= 0) // ��ʱ��
    {
        Robot_loc.stPot.fpPosQ1 += (fpQ_Cur - fpQ_Pre) * fpAntiClock;
    }
    else // ˳ʱ��
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
    fpQ_Pre = fpQ_Cur; // ���ݽǶ����ݸ���

#endif

#if AnologGyro              // ģ��
    if (fpQ_Cur >= fpQ_Pre) // ��ʱ��
    {
        Robot_loc.stPot.fpPosQ1 += (fpQ_Cur - fpQ_Pre) * fpAntiClock;
    }
    else // ˳ʱ��
    {
        Robot_loc.stPot.fpPosQ1 += (fpQ_Cur - fpQ_Pre) * fpClock;
    }
    fpQ_Pre = fpQ_Cur; // ���ݽǶ����ݸ���
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
    //    SBά��
    //    temp_vo = (int)((uart1_efr.num1[3] << 8) | uart1_efr.num1[2]);
    //    temp_v = (int)((uart1_efr.num1[5] << 8) | uart1_efr.num1[4]);
    //    temp_q = (int)((uart1_efr.num2[5] << 8) | uart1_efr.num2[4]);

    //    fpW_orgin = temp_vo / 32768.0f * 2000.0f;
    //    fpW = temp_v / 32768.0f * 2000.0f;
    //    fpQ = temp_q / 32768.0f * 180.0f;

    cGyro.fpQ_Cur = cGyro.get_gyro_value();
#if DigtalGyro // ����

    if (fabs(cGyro.fpQ_Cur - cGyro.fpQ_Pre) > 100) // 2msת10�ȣ�������
    {
        stPot.fpPosQ1 += 0;
    }
    else if (cGyro.fpQ_Cur - cGyro.fpQ_Pre >= 0) // ��ʱ��
    {
        stPot.fpPosQ1 += (cGyro.fpQ_Cur - cGyro.fpQ_Pre) * cGyro.fpAntiClock;
    }
    else // ˳ʱ��
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
    cGyro.fpQ_Pre = cGyro.fpQ_Cur; // ���ݽǶ����ݸ���

#endif

#if AnologGyro              // ģ��
    if (fpQ_Cur >= fpQ_Pre) // ��ʱ��
    {
        Robot_loc.stPot.fpPosQ1 += (fpQ_Cur - fpQ_Pre) * fpAntiClock;
    }
    else // ˳ʱ��
    {
        Robot_loc.stPot.fpPosQ1 += (fpQ_Cur - fpQ_Pre) * fpClock;
    }
    fpQ_Pre = fpQ_Cur; // ���ݽǶ����ݸ���
#endif
}

/*******************************************************************************************
�������ƣ�RobotLocation()
�������ܣ���˫�涯�ֺ����������ݵõ������˵��������̬�ǣ���λ��mm��0.1�㣩
���룺	  1.Robot_loc ָ��������ܽṹ���ָ��
          2.pstFW	 ָ���涯���ܽṹ���ָ��
          3.pstGyro  ָ�������ܽṹ���ָ��
�����	  1.����������λ�ˣ������������̬��
          2.�涯������λ��	��������
��ע��    1.�˺����е����㶼���ڻ���Ϊ��λ������½��е�
          2.ALPHA_AΪA�涯����ʱ����תʱ���ٶȷ���������˾ֲ�����ϵy������ļн�
              3.ALPHA_BΪB�涯����ʱ����תʱ���ٶȷ���������˾ֲ�����ϵy������ļн�
          4.���������ǶȾ��Ǵ�y������Ϊ��
*******************************************************************************************/
void C_ROBOT::RobotLocation()
{
    static C_LPF Fpx(20, 0.002);
    static C_LPF Fpy(20, 0.002);
    static fp32 temp_x, temp_y;
    fp32 ALPHA_B;
    fp32 ALPHA_A;
    fp32 Sin_B_A;
    fp32 fpDeltaA, fpDeltaB;  // �涯���߹��ľ���
    fp32 Convert_Array[2][2]; // ����ת������ʵ����3*3�ģ���������ֻ��2*2
    fp32 fpQ;                 // ��������̬����ʱ����(��λ������)

    /*******************��ȡ�Ƕ�*********************/
    Cal_Robot_Degree();                        // ��stPot.fpPosQ1��ֵ,��λ0.1��
    stPot.fpPosQ = stPot.fpPosQ1 + fpQOffset;  // ��λ0.1��
    fpQ = ConvertAngle(stPot.fpPosQ * RADIAN); // ��λ:����

    /**************��ȡ�涯�ַ���λ��****************/
    cFollowoerWheel.m_CoderACur = cFollowoerWheel.degreeA;
    cFollowoerWheel.m_CoderBCur = cFollowoerWheel.degreeB;

    if (my_intabs(cFollowoerWheel.m_CoderACur - cFollowoerWheel.m_CoderAPre) > 1000 ||
        my_intabs(cFollowoerWheel.m_CoderBCur - cFollowoerWheel.m_CoderBPre) > 1000) // 2ms 10cm������
    {
        cFollowoerWheel.m_CoderAPre = cFollowoerWheel.m_CoderACur;
        cFollowoerWheel.m_CoderBPre = cFollowoerWheel.m_CoderBCur;
        return;
    }

    /*�����涯���߹��ľ���*/
    if (cFollowoerWheel.m_CoderACur >= cFollowoerWheel.m_CoderAPre) // A����ת����ת������ֵ��С
    {
        fpDeltaA = (cFollowoerWheel.m_CoderACur - cFollowoerWheel.m_CoderAPre) * FW_Len_A_Inc;
        ALPHA_A = ALPHA_A_Inc;
    }
    else // A�ַ�ת
    {
        fpDeltaA = (cFollowoerWheel.m_CoderACur - cFollowoerWheel.m_CoderAPre) * FW_Len_A_Dec;
        ALPHA_A = ALPHA_A_Dec;
    }
    if (cFollowoerWheel.m_CoderBCur >= cFollowoerWheel.m_CoderBPre) // B����ת
    {
        fpDeltaB = (cFollowoerWheel.m_CoderBCur - cFollowoerWheel.m_CoderBPre) * FW_Len_B_Inc;
        ALPHA_B = ALPHA_B_Inc;
    }
    else // B�ַ�ת
    {
        fpDeltaB = (cFollowoerWheel.m_CoderBCur - cFollowoerWheel.m_CoderBPre) * FW_Len_B_Dec;
        ALPHA_B = ALPHA_B_Dec;
    }

    /**************�����涯����������****************/
    Sin_B_A = sinf(ALPHA_B - ALPHA_A); // ת�������ĸϵ��sin(B-A)

    Convert_Array[0][0] = cosf(ALPHA_B + fpQ) / Sin_B_A;
    Convert_Array[0][1] = cosf(ALPHA_A + fpQ) / Sin_B_A;
    Convert_Array[1][0] = sinf(ALPHA_B + fpQ) / Sin_B_A;
    Convert_Array[1][1] = sinf(ALPHA_A + fpQ) / Sin_B_A;

    cFollowoerWheel.stPot.fpPosX += Convert_Array[0][0] * fpDeltaA - Convert_Array[0][1] * fpDeltaB;
    cFollowoerWheel.stPot.fpPosY += Convert_Array[1][0] * fpDeltaA - Convert_Array[1][1] * fpDeltaB;

    /**************�����������������****************/
    /*�κ�λ�ö����ã�����Ҫע��FW_rob_AlphaΪ����������ָ���涯�����ĵ�ʸ��(ͬ��Ҫ���y�Ὺʼ��ʱ����תΪ��)���ýǶȷ�ΧΪ[0��2*pi)*/
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

    /*�涯�ֱ��������ݱ���*/
    cFollowoerWheel.m_CoderAPre = cFollowoerWheel.m_CoderACur;
    cFollowoerWheel.m_CoderBPre = cFollowoerWheel.m_CoderBCur;
}

/*******************************************************************************************
�������ƣ�CalibrateRobotVelt()
�������ܣ�У׼�����˵��ٶȣ�����õ��������������������ȫ������ϵ�µ��ٶȣ���λmm/s��
���룺	  1.Robot_loc ָ��������ܽṹ���ָ��
�����	  1.��������ȫ������ϵ�µ��ٶ�
��ע��    1.�����н��ٶȵĵ�λ��(0.1��/��)
*******************************************************************************************/
void C_ROBOT::Cal_RobotVelt(void)
{
    static C_LPF Fvx(200, 0.002);
    static C_LPF Fvy(200, 0.002);
    static C_LPF Fw(200, 0.002);

    /*�洢���������ꡢ��̬�ǡ�����������ֵ���������*/
    Push_Q_In(stPot.fpPosQ);
    Push_Time_In(TIM2->CNT);
    Push_RobotPosX_In(stPot.fpPosX);
    Push_RobotPosY_In(stPot.fpPosY);

    /*�˲�*/
    Fvx.m_in = (afpRobot_PosX[0] - afpRobot_PosX[3]) /
               ((auiTIM2_Time[0] - auiTIM2_Time[3]) * TIM2_BASE_TIME); // x�����ٶ�
    Fvy.m_in = (afpRobot_PosY[0] - afpRobot_PosY[3]) /
               ((auiTIM2_Time[0] - auiTIM2_Time[3]) * TIM2_BASE_TIME); // y�����ٶ�
    Fw.m_in = (afpRobot_Q[0] - afpRobot_Q[3]) / ((auiTIM2_Time[0] - auiTIM2_Time[3]) * TIM2_BASE_TIME) /
              10.0f; // ���ٶȣ������y����תΪ��

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
    // �ĸ�DT35����rightdown�ֶ�Ӧ�ı���x������Ϊy1,y2����up�ֶ�Ӧ�ı���y������Ϊx2,x1
    static fp32 fpQ_now, fpQ_save;
    stDt35_now.robot_q = stPot.fpPosQ;
    // x:4du
    fpQ_save = ConvertAngle(stDt35_save.robot_q * RADIAN); // ����
    fpQ_now = ConvertAngle(stDt35_now.robot_q * RADIAN);   // ����

    // dt35��ǽ�ľ��룬
    stDt35_save.dt35_fix_x1 = fabs(K_DT35_X1 * stDt35_save.dt35_x1 + B_DT35_X1);
    stDt35_save.dt35_fix_x2 = fabs(K_DT35_X2 * stDt35_save.dt35_x2 + B_DT35_X2);
    stDt35_now.dt35_fix_x1 = fabs(K_DT35_X1 * stDt35_now.dt35_x1 + B_DT35_X1);
    stDt35_now.dt35_fix_x2 = fabs(K_DT35_X2 * stDt35_now.dt35_x2 + B_DT35_X2);

    stDt35_save.dt35_fix_y1 = fabs(K_DT35_Y1 * stDt35_save.dt35_y1 + B_DT35_Y1);
    stDt35_save.dt35_fix_y2 = fabs(K_DT35_Y2 * stDt35_save.dt35_y2 + B_DT35_Y2);
    stDt35_now.dt35_fix_y1 = fabs(K_DT35_Y1 * stDt35_now.dt35_y1 + B_DT35_Y1);
    stDt35_now.dt35_fix_y2 = fabs(K_DT35_Y2 * stDt35_now.dt35_y2 + B_DT35_Y2);

    // dt35���ڱߵ����ĵ�x��y����

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

    // ע����������������
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
        fabs(stDt35_now.pro_x1 - stDt35_now.pro_x2) <= LIMIT_DIS_X) // ��ֹ��Ծ
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

    if (flag_y_dt35) // �����ʱ����
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
