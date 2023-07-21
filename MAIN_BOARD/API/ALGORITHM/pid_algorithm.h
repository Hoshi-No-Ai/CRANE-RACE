#ifndef __PID_ALGORITHM_H__
#define __PID_ALGORITHM_H__

#define SQUARE(x) ((x) * (x))

#include "basic_type.h"
#include "filter_algorithm.h"
#include "math.h"
#include "math_algorithm.h"
#include "stm32f4xx.h"

class C_PID {
   public:
    fp32 fpDes;  //���Ʊ���Ŀ��ֵ
    fp32 fpFB;   //���Ʊ�������ֵ

    fp32 fpKp;  //����ϵ��Kp
    fp32 fpKi;  //����ϵ��Ki
    fp32 fpKd;  //΢��ϵ��Kd

    fp32 fpE;     //����ƫ��
    fp32 fpPreE;  //�ϴ�ƫ��
    fp32 fpSumE;  //��ƫ��
    fp32 fpInput;
    fp32 fpInputpre;
    fp32 fpOutput;
    fp32 fpOutputpre;
    fp32 fpEpMax;    //������������ֵ
    fp32 fpEiMax;    //������������ֵ
    fp32 fpEdMax;    //΢����������ֵ
    fp32 fpEMin;     //��������
    fp32 fpEMax;     //��������
    fp32 fpSumEMax;  //������sumE���ֵ

    fp32 fpUp;  //�������
    fp32 fpUi;  //�������
    fp32 fpUd;  //΢�����
    fp32 fpU;   //����PID������
    fp32 fpUpre;
    fp32 fpUMax;  // PID�����������ֵ������������ʱ������ֵ
    fp32 fpTs;    // PID�������ڣ���λ��s

    C_PID() {}
    C_PID(fp32 Kp, fp32 Ki, fp32 Kd, fp32 UMax, fp32 UiMax, fp32 UdMax, fp32 ts)
        : fpKp(Kp),
          fpKi(Ki),
          fpKd(Kd),
          fpUMax(UMax),
          fpEpMax(UMax),
          fpEiMax(UiMax),
          fpEdMax(UdMax),
          fpTs(ts) {}  // initialize PID
    ~C_PID() {}

    void CalPID(void);
    void CalISeparatedPID(void);
    void CalIResistedPID(void);
    void CalIWeakenPID(void);
    void CalFilterPID(void);
    void CalComprehensivePID(void);
};

class C_Tr {
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
    C_Tr(fp32 Ts) : fpTs(Ts) {}  //��������
    ~C_Tr() {}

    void TrF1(fp32 t1, fp32 t2);
    void TrF2(fp32 t);
    void TrF3(fp32 t);
    void LagCompensator(fp32 gain, fp32 t1, fp32 t2);
};

//��ģ���
class C_TD {
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
