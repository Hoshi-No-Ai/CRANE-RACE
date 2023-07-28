#include "vision_algorithm.h"
fp32 K;
fp32 VisionY;
fp32 VisionX;
fp32 ConvertArray[2][2];

// 机器人反馈位置到目标位置的距离
Delta_2D delta_fb_des;
// 机器人反馈位置到aruco码的距离（机器人坐标系）
static Delta_2D delta_fb_aruco_r;
// TODO: aruco码坐标系下目标位置的坐标表示，x,y信息在宏定义里修改
//  aruco码到机器人目标位置的距离（aruco码坐标系）
static Delta_2D delta_aruco_des_a = {DES_ARUCO_CENTER, DES_CENTER_DES};

// 取可乐时目标点的位置变化
Delta_2D delta_des_cola_w;
static Delta_2D delta_des_cola_r = {DELTA_COLA, 0};

void DealVision(void)
{
    fp32 R;
    fp32 theata;
    R = sqrt(pow(VisionX, 2) + pow(VisionY, 2));
    theata = asin(R / K) / 2;
}

Delta_2D Coord_transformation(float theta, Delta_2D Delta_r)
{
    Delta_2D Delta_w;
    Delta_w.delta_x = Delta_r.delta_x * cosf(theta) - Delta_r.delta_y * sinf(theta);
    Delta_w.delta_y = Delta_r.delta_x * sinf(theta) + Delta_r.delta_y * cosf(theta);
    return Delta_w;
}

bool des_base_aruco(aruco &aruco_ref)
{
    // TODO: 可能视觉参数的正负方向得调一下
    float fpQ = 0.1 * cRobot.stPot.fpPosQ;
    float theta_w_r = -fpQ;
    float theta_w_a = -(fpQ + aruco_ref.thetaz);

    delta_fb_aruco_r.delta_x = aruco_ref.x;
    delta_fb_aruco_r.delta_y = aruco_ref.y;

    Delta_2D delta_fb_aruco_w = Coord_transformation(theta_w_r, delta_fb_aruco_r);
    Delta_2D delta_aruco_des_w = Coord_transformation(theta_w_a, delta_aruco_des_a);

    delta_fb_des.delta_x = delta_fb_aruco_w.delta_x + delta_aruco_des_w.delta_x;
    delta_fb_des.delta_y = delta_fb_aruco_w.delta_y + delta_aruco_des_w.delta_y;

    return aruco_ref.if_detect;
}



void delta_des_cola(int num)
{
    Delta_2D delta_des_r;
    switch (num)
    {
    case 1:
        delta_des_r.delta_x=delta_des_cola_r.delta_x;
        delta_des_r.delta_y=delta_des_cola_r.delta_y;
        break;
    case 2:
        delta_des_r.delta_x=0;
        delta_des_r.delta_y=0;
        break;
    case 3:
        delta_des_r.delta_x=-delta_des_cola_r.delta_x;
        delta_des_r.delta_y=-delta_des_cola_r.delta_y;
        break;
    default:
        break;
    }

    float fpQ = nav.auto_path.m_point_end.m_q * RADIAN;
    float theta_w_r = fpQ;
		
    delta_des_cola_w = Coord_transformation(theta_w_r, delta_des_r);
}

bool Identify_box_cola(int &target)
{
    if(nav.state==NAV_STOPX)
    {
        target=temp_target_detect;
        return true;
    }
    else
    {
        return false;
    }
}