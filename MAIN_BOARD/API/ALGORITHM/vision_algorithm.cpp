#include "vision_algorithm.h"
fp32 K;
fp32 VisionY;
fp32 VisionX;
fp32 ConvertArray[2][2];

// �����˷���λ�õ�Ŀ��λ�õľ���
Delta_2D delta_fb_des;
// �����˷���λ�õ�aruco��ľ��루����������ϵ��
static Delta_2D delta_fb_aruco_r;
// TODO: aruco������ϵ��Ŀ��λ�õ������ʾ��x,y��Ϣ�ں궨�����޸�
//  aruco�뵽������Ŀ��λ�õľ��루aruco������ϵ��
static Delta_2D delta_aruco_des_a = {DES_ARUCO_CENTER, DES_CENTER_DES};

// ȡ����ʱĿ����λ�ñ仯
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
    // TODO: �����Ӿ���������������õ�һ��
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