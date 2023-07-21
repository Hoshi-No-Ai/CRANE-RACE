/**********************************************************************************************************************************************************
��Ȩ������HITCRT(�����󾺼�������Э��)
�ļ�����PID_Algorithm.c
����޸����ڣ�2016.10.13
�汾��1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
ģ��������
�����б�
----------------------------------------------------------------------------------------------------------------------------------------------------------
�޶���¼��
     ����        	ʱ��            �汾     	˵��
  JIN    2022.10.13      	2.0      ���ֽ�����ģ��
**********************************************************************************************************************************************************/

#include "pid_algorithm.h"

/*����Ϊλ����PID*/
/*******************************************************************
�������ƣ�CalPID(ST_PID *this)
�������ܣ���ͨ��PID�㷨����PID��
��    ע��
********************************************************************/
void C_PID::CalPID(void) {
    fpE = fpDes - fpFB;       //���㵱ǰƫ��
    if (fabs(fpE) <= fpEMin)  //ƫ����������
    {
        fpE = 0;
        fpSumE = 0;
    }
    fpSumE += fpE;
    /*λ��ʽPID���㹫ʽ*/
		fpUp = fpKp * fpE;
		fpUi = fpKi * fpSumE ;
		fpUd =  fpKd * (fpE - fpPreE) / 0.001f;
    fpU =fpUp+ fpUi + fpUd;
    fpPreE = fpE;  //���汾��ƫ��
    /*PID��������޷�*/
    fpU = ClipFloat(fpU, -fpUMax, fpUMax);
}

/*******************************************************************
�������ƣ�CalISeparatedPID(ST_PID *this)
�������ܣ����ַ���ʽPID�㷨����PID��
��    ע�����ַ���ʽPID�Ľ��㷨�ɼ�С������ֹͣ����������ʱ�ϴ�ƫ��
          �Ի�����Ļ��ۣ��Ӷ�������ֽϴ�ĳ�����������
********************************************************************/
void C_PID::CalISeparatedPID(void) {
    uint8_t uck = 1;

    fpE = fpDes - fpFB;       //���㵱ǰƫ��
    if (fabs(fpE) <= fpEMin)  //ƫ����������
    {
        fpE = 0;
    }
    fpSumE += fpE;  //����ƫ���ۻ�
    /*��ƫ������������ۻ�ƫ��*/
    if (fabs(fpE) > fpEMax)  //�ж��Ƿ�������ַ���
    {
        fpSumE = 0;
        uck = 0;
    }
    /*λ��ʽPID���㹫ʽ*/
    fpU = fpKp * fpE + fpKi * fpSumE * uck + fpKd * (fpE - fpPreE);
    fpPreE = fpE;  //���汾��ƫ��
    /*PID��������޷�*/
    fpU = ClipFloat(fpU, -fpUMax, fpUMax);
}

/*******************************************************************
�������ƣ�CalIResistedPID(ST_PID *this)
�������ܣ������ֱ���PID�㷨
��    ע��ϵͳ��һ�������˶�������ϴ���������ڼ��������ڲ����񵴻򳬵�
********************************************************************/
void C_PID::CalIResistedPID(void) {
    fpE = fpDes - fpFB;  //���㵱ǰƫ��
    fpSumE += fpE;       //����ƫ���ۻ�

    fpSumE = ClipFloat(fpSumE, -fpEiMax, fpEiMax);
    fpUi = fpKi * fpSumE;

    fpUp = ClipFloat(fpKp * fpE, -fpEpMax, fpEpMax);
    fpUd = ClipFloat(fpKd * (fpE - fpPreE), -fpEdMax, fpEdMax);

    /*��ƫ��������֮�ڣ�����������ۼ���*/
    if (fabs(fpE) < fpEMin)  //�ж��Ƿ�������ֱ�������
    {
        fpSumE = 0;  //���ƫ���ۻ�
    }
    /*λ��ʽPID���㹫ʽ*/
    fpU = fpUp + fpUi + fpUd;

    fpPreE = fpE;  //���汾��ƫ��
                   /*PID��������޷�*/
    fpU = ClipFloat(fpU, -fpUMax, fpUMax);
}
/*******************************************************************
�������ƣ�CalIWeakenPID(ST_PID *this)
�������ܣ�������������PID�Ľ��㷨����PID��
��    ע��
********************************************************************/
void C_PID::CalIWeakenPID(void) {
    fpE = fpDes - fpFB;  //���㵱ǰƫ��

    if (((fpU <= fpUMax && fpE > 0) || (fpU >= -fpUMax && fpE < 0)) && fabs(fpE) < fpEMin) {
        fpSumE += fpE;  //����ƫ���ۻ�
    }

    fpSumE = ClipFloat(fpSumE, -fpEiMax, fpEiMax);
    fpUi = fpKi * fpSumE;

    fpUp = ClipFloat(fpKp * fpE, -fpEpMax, fpEpMax);
    fpUd = ClipFloat(fpKd * (fpE - fpPreE), -fpEdMax, fpEdMax);

    /*λ��ʽPID���㹫ʽ*/
    fpU = fpUp + fpUi + fpUd;

    fpPreE = fpE;  //���汾��ƫ��

    /*PID��������޷�*/
    fpU = ClipFloat(fpU, -fpUMax, fpUMax);
}

/*******************************************************************
�������ƣ�CalFilterPID(ST_PID *this)
�������ܣ�΢����PID���
��    ע��
********************************************************************/
void C_PID::CalFilterPID(void) {
    //=======���㵱ǰƫ��===========
    fpE = fpDes - fpFB;
    //=======ƫ����������========
    if (fabs(fpE) <= fpEMin) {
        fpE = 0.0f;
        fpUi = 0;
    }
    /*======λ��ʽPID���㹫ʽ======*/
    fpUp = fpKp * fpE;          //���������
    fpUi += fpKi * fpE * fpTs;  //���������
    //΢���������΢���˲�
    fpUd = fpKd * (fpE - fpPreE) / fpTs;

    //===========�����ϴ�ƫ��===========
    fpPreE = fpE;
    //===========PID�����===============
    fpU = fpUp + fpUi + fpUd;
    /*=========����޷�============*/
    fpU = ClipFloat(fpU, -fpUMax, fpUMax);
}

///*******************************************************************
//�������ƣ�CalComprehensivePID(ST_PID *this)
//�������ܣ��ۺ�PID���
//��    ע��Ϊ�������ã��ɲ�������PID�����ۺϣ�����ȫ��
//********************************************************************/
void C_PID::CalComprehensivePID(void) {
    uint8_t uck = 0;
    static u8 kkk = 2;

    fpE = fpDes - fpFB;  //���㵱ǰƫ��
    fpSumE += fpE;
    /*���ֿ�����*/
    fpSumE = ClipFloat(fpSumE, -fpSumEMax, fpSumEMax);
    /*����������޷�*/
    fpUp = ClipFloat(fpKp * fpE, -fpEpMax, fpEpMax);
    fpUi = ClipFloat(fpKi * fpSumE, -fpEiMax, fpEiMax);
    fpUd = ClipFloat(fpKd * (fpE - fpPreE), -fpEdMax, fpEdMax);
    /*���ַ���*/
    if (fabs(fpE) >= fpEMax)  //�ж��Ƿ�������ַ���
    {
        uck = 0;
    } else
        uck = 1;
    /*λ��ʽPID���㹫ʽ*/
    if (fpUd > 0.1f) {
        lpf_PID.m_in = fpUd;
    }

    lpf_PID.LpFilter();

    if (fabs(fpE) <= fpEMin) {
        fpU = fpUp + kkk * fpUi * uck + lpf_PID.m_out;
    } else {
        fpU = fpUp + fpUi * uck + lpf_PID.m_out;
    }

    fpPreE = fpE;  //���汾��ƫ��
    /*PID��������޷�*/
    fpU = ClipFloat(fpU, -fpUMax, fpUMax);
}

void C_Tr::TrF1(fp32 t1, fp32 t2) {
    fpOutput1 =
        ((2 * t1 + fpTs) * fpInput1 + (fpTs - 2 * t1) * fpInputpre1 - (fpTs - 2 * t2) * fpOutputpre1) /
        (2 * t2 + fpTs);
    fpInputpre1 = fpInput1;
    fpOutputpre1 = fpOutput1;
}

void C_Tr::TrF2(fp32 t) {
    fpOutput2 = 2 * fpInput2 - 2 * fpInputpre2 - (fpTs - 2 * t) * fpOutputpre2;
    fpInputpre2 = fpInput2;
    fpOutputpre2 = fpOutput2;
}

void C_Tr::TrF3(fp32 t) {
    fpOutput3 = fpInput3 - fpInputpre3 - (fpTs - 2 * t) * fpOutputpre3;
    fpInputpre3 = fpInput3;
    fpOutputpre3 = fpOutput3;
}

void C_Tr::LagCompensator(fp32 gain, fp32 t1, fp32 t2)  //�ͺ�У����t2Ϊ��������ȡ
{
    fpOutput1 =
        ((2 * t1 + t2) * fpInput1 + (t2 - 2 * t1) * fpInputpre1 + (2 * gain * t1 - t2) * fpOutputpre1) /
        (2 * t1 * gain + t2);
    fpInputpre1 = fpInput1;
    fpOutputpre1 = fpOutput1;
}

void C_TD::TD_Function(void) {
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
