#ifndef __MOTOR_DRIVE_H__
#define __MOTOR_DRIVE_H__

#include "basic_type.h"
#include "pid_algorithm.h"
#include "stm32f4xx.h"

#define ENCODER_NUMBER 8191
#define MOTOR_TS 0.001f

enum motor_pid_state_e {
    VELT_LOOP,
    POS_LOOP,
    DOUBLE_LOOP,
    OPEN_LOOP  //����״̬�£�����������ڵ������������������PID����
};

enum motor_feed_forward_state_e { WITH_FORWARD, WITHOUT_FORWARD };

//����������ṹ��
class C_Encoder {
   public:
    int32_t siRawValue;     //���α�������ԭʼֵ
    int32_t siPreRawValue;  //��һ�α�������ԭʼֵ
    int32_t siDiff;         //����������ԭʼֵ�Ĳ�ֵ
    int32_t siSumValue;     //�������ۼ�ֵ
    fp32 siGearRatio;       //������������ٱ�
    int32_t siNumber;       //����������
    fp32 fpSpeed;           //��������������ת�٣���λ��r/min

    C_Encoder() {}
    C_Encoder(fp32 gr, int32_t num) : siGearRatio(gr), siNumber(num) {}
    ~C_Encoder() {}

    void Encoder_Process(int32_t value, uint8_t type);
    // void GetPosition(void);
    static int32_t Get_Encoder_Number(CanRxMsg* rx_message);
    static int32_t Get_Speed(CanRxMsg* rx_message);
};

class C_Motor {
   public:
		 float pid_current;
    int32_t real_current;
    int32_t feed_forward_current;
    C_PID velt_pid;
    C_PID pos_pid;
    C_Encoder encoder;
    motor_pid_state_e motor_pid_state;
    motor_feed_forward_state_e feed_forward_state;

    C_Motor() {}
    C_Motor(fp32 vKp, fp32 vKi, fp32 vKd, fp32 vUpMax, fp32 vUiMax, fp32 vUdMax, fp32 pKp, fp32 pKi, fp32 pKd,
            fp32 pUpMax, fp32 pUiMax, fp32 pUdMax, fp32 gr, int32_t num = ENCODER_NUMBER,
            fp32 ts = MOTOR_TS) {
        velt_pid = C_PID(vKp, vKi, vKd, vUpMax, vUiMax, vUdMax, ts);
        pos_pid = C_PID(pKp, pKi, pKd, pUpMax, pUiMax, pUdMax, ts);
        encoder = C_Encoder(gr, num);
    }
    C_Motor(C_PID v_pid, C_PID p_pid, C_Encoder c_encoder)
        : velt_pid(v_pid), pos_pid(p_pid), encoder(c_encoder) {}
    C_Motor(C_PID v_pid, C_PID p_pid, C_Encoder c_encoder, motor_pid_state_e mps,
            motor_feed_forward_state_e mffs)
        : velt_pid(v_pid),
          pos_pid(p_pid),
          encoder(c_encoder),
          motor_pid_state(mps),
          feed_forward_state(mffs) {}
    ~C_Motor(){};


void init__motor_PID(float p_kp,float p_ki,float p_kd,float v_kp,float v_ki,float v_kd,float ts=0.001)
		{
			pos_pid.fpKp = p_kp;
		pos_pid.fpKi = p_ki;
			pos_pid.fpKd = p_kd;
pos_pid.fpTs = ts;
			velt_pid.fpKp = v_kp;
			velt_pid.fpKi = v_ki;
			velt_pid.fpKd = v_kd;
		}
		
		void init__motor_encoder(fp32 gr, int32_t num = ENCODER_NUMBER)
		{
		encoder = C_Encoder(gr, num);
		}
		
		
    static void can_send_data(CAN_TypeDef* CANx, uint32_t StdID, int16_t ssMotor1, int16_t ssMotor2,
                              int16_t ssMotor3, int16_t ssMotor4);
};

#endif
