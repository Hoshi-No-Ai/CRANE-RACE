#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "motor_drive.h"
#include "pid_algorithm.h"

// 8ʸ��������Ҫ�ĳߴ磬��λmm
#define L_HALF 308.63f / 1.414f  // �복����ָX����
#define B_HALF 308.63f / 1.414f  // �복��ָY����
#define R_HALF 308.63f
#define R_WHEEL 76.0f  // 44.0f//�ʵ���СR_WHEEL����������ʹexcept_global_veltС��velt_fb���ﵽ��Ҫ��Ч��

#define OMNI_GEAR_RATIO 19.0f  // ȫ���ּ��ٱ�

// ȫ���ֲ���
#define OMNI_VELT_KP 1500  // 450
#define OMNI_VELT_KI 5    // 0
#define OMNI_VELT_KD 0
#define OMNI_VELT_UMax 20000.0f
#define OMNI_VELT_Ts 0.001f

/*feedforward*/  // 24.5072

#define K_RU_STRAIGHT 0.0f
#define K_LU_STRAIGHT 0.0f
#define K_LD_STRAIGHT 0.0f
#define K_RD_STRAIGHT 0.0f

#define START_CURRENT_RU 0.0f
#define START_CURRENT_LU 0.0f
#define START_CURRENT_LD 0.0f
#define START_CURRENT_RD 0.0f

#define K_LU_TURN 0.0f
#define K_RU_TURN 0.0f
#define K_LD_TURN 0.0f
#define K_RD_TURN 0.0f

#define K_UP_STRAIGHT 0.0f
#define K_UP_TURN 0.0f

#define START_CURRENT_UP 2.180650738014755e+03

enum tri_chassis_e { T_UP, T_RIGHTDOWN, T_LEFTDOWN };

enum chassis_e { RIGHTUP, LEFTUP, LEFTDOWN, RIGHTDOWN };

class C_Omniwheel_Motors {
   public:
    C_Motor m_run_motor;

    C_Omniwheel_Motors()
        : m_run_motor(C_PID(OMNI_VELT_KP, 0, 0, OMNI_VELT_UMax, 0, 0, OMNI_VELT_Ts),
                      C_PID(0, 0, 0, 0, 0, 0, 0), C_Encoder(OMNI_GEAR_RATIO, ENCODER_NUMBER), VELT_LOOP,
                      WITHOUT_FORWARD) {}
    ~C_Omniwheel_Motors() {}

    float cal_single_feed_forward(C_VECTOR &expect_Velt, const int num);
};

extern C_Omniwheel_Motors Omni_chassis[4];

#endif
