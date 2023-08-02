#include "chassis.h"

C_Omniwheel_Motors Omni_chassis[4];

float C_Omniwheel_Motors::cal_single_feed_forward(C_VECTOR &expect_Velt, const int num) {
    static C_VECTOR d_straight_velt, pre_straight_velt, straight_velt;
    float d_straight_velt_dis;
    float feed_forward_current;
    straight_velt = expect_Velt;
    // 得到加速度d_straight_velt，单位cm/s2
    d_straight_velt = C_VECTOR((straight_velt.fpLength - pre_straight_velt.fpLength) / 0.002f,
                               straight_velt.fpthetha, POLAR);
    // 将加速度(x，y方向的分量)沿四个轮的方向进行分解
    switch (num) {
        case RIGHTUP:
            d_straight_velt_dis =
                d_straight_velt.fpVy / 2.0f / L_HALF * R_HALF - d_straight_velt.fpVx / 2.0f / B_HALF * R_HALF;
            feed_forward_current = d_straight_velt_dis * K_RU_STRAIGHT + START_CURRENT_RU;
            break;
        case LEFTUP:
            d_straight_velt_dis = -d_straight_velt.fpVy / 2.0f / L_HALF * R_HALF -
                                  d_straight_velt.fpVx / 2.0f / B_HALF * R_HALF;
            feed_forward_current = d_straight_velt_dis * K_LU_STRAIGHT + START_CURRENT_LU;
            break;
        case LEFTDOWN:
            d_straight_velt_dis = -d_straight_velt.fpVy / 2.0f / L_HALF * R_HALF +
                                  d_straight_velt.fpVx / 2.0f / B_HALF * R_HALF;
            feed_forward_current = d_straight_velt_dis * K_LD_STRAIGHT + START_CURRENT_LD;
            break;
        case RIGHTDOWN:
            d_straight_velt_dis =
                d_straight_velt.fpVy / 2.0f / L_HALF * R_HALF + d_straight_velt.fpVx / 2.0f / B_HALF * R_HALF;
            feed_forward_current = d_straight_velt_dis * K_RD_STRAIGHT + START_CURRENT_RD;
            break;
        default:
            break;
    }
    pre_straight_velt = straight_velt;
    return feed_forward_current;
}