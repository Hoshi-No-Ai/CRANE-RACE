#ifndef __NAVIGATION_ALGORITHM_H__
#define __NAVIGATION_ALGORITHM_H__

#include "basic_type.h"
#include "chassis.h"
#include "locate_algorithm.h"
#include "math.h"
#include "math_algorithm.h"
#include "movement_info.h"
#include "path_algorithm.h"
#include "pid_algorithm.h"

#define SET_NAV_PATH_AUTO(Name)    \
    nav.auto_path.m_number = Name; \
    nav.state = NAV_AUTO_PATH;     \
    nav.auto_path.run_time = 0;    \
    flag_record = 1

enum nav_state_e {
    NAV_INIT,
    NAV_MANUAL,  // 手动控制
    NAV_NEW_MANUAL,
    NAV_AUTO_PATH,  // 自动路径导航
    NAV_PATHPLANNING,
    NAV_OFF,
    NAV_STOP,
    NAV_LOCK,
    NAV_CALIBRATION_1,  // 跑开环，测启动电流与加速度系数
    NAV_CALIBRATION_2,  // 跑最大加速度，修正加速度系数
};

class C_NAV {
   public:
    nav_state_e state;
    C_VECTOR expect_robot_global_velt;
    C_VECTOR expect_robot_local_Velt;

    C_AUTO_PATH auto_path;

    C_NAV() {}
    ~C_NAV() {}

    void Omni_chassis_SpeedDistribute(void);
    void Omni_chassis_Cal_Feedforward(void);

    void disable_omni_chassis(void);
    void stop_omni_chassis(void);
};

extern C_NAV nav;

#endif
