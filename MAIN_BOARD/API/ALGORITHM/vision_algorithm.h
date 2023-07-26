#ifndef __VISION_ALGORITHM_H__
#define __VISION_ALGORITHM_H__

#include "basic_type.h"
#include "math.h"
#include "usart_protocol.h"
#include "locate_algorithm.h"

#define DES_ARUCO_CENTER 0.0f
#define DES_CENTER_DES 0.0f

struct ST_CAMERA
{
    fp32 fpQx;    // 相机安�?��?�度与水平方向的夹�?�（单位�?0.1度，理想值是0�?
    fp32 fpQy;    // 相机安�?��?�度与竖直方向的夹�?�（单位�?0.1度，理想值是0�?
    fp32 fpQz;    // 相机坐标系Y方向与机器人正前方向的夹角（单位�?0.1度，理想值是0�?
    fp32 fpPosX0; // 相机�?心相对于机器人中心的X坐标
    fp32 fpPosY0; // 相机�?心相对于机器人中心的Y坐标
    fp32 fpPosZ0; // 相机�?心相对于机器人中心的Z坐标
    fp32 fpPosX1; // �?标相对于相机坐标系的X坐标
    fp32 fpPosY1; // �?标相对于相机坐标系的Y坐标
    fp32 fpPosZ1; // �?标相对于相机坐标系的Z坐标
    fp32 fpPosX2; // �?标相对于全局坐标系的X坐标
    fp32 fpPosY2; // �?标相对于全局坐标系的Y坐标
    fp32 fpPosZ2; // �?标相对于全局坐标系的Z坐标
};

// 二维齐�?�变换矩�?
struct SE2
{
    float theta;
    float deltax;
    float deltay;
};

struct Delta_2D
{
    float delta_x;
    float delta_y;
};

bool des_base_aruco(aruco &aruco_ref);

extern Delta_2D delta_fb_des;

#endif
