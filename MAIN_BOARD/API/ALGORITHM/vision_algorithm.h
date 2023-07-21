#ifndef __VISION_ALGORITHM_H__
#define __VISION_ALGORITHM_H__

#include "basic_type.h"
#include "math.h"

struct ST_CAMERA {
    fp32 fpQx;  //相机安装角度与水平方向的夹角（单位：0.1度，理想值是0）
    fp32 fpQy;  //相机安装角度与竖直方向的夹角（单位：0.1度，理想值是0）
    fp32 fpQz;  //相机坐标系Y方向与机器人正前方向的夹角（单位：0.1度，理想值是0）
    fp32 fpPosX0;  //相机中心相对于机器人中心的X坐标
    fp32 fpPosY0;  //相机中心相对于机器人中心的Y坐标
    fp32 fpPosZ0;  //相机中心相对于机器人中心的Z坐标
    fp32 fpPosX1;  //目标相对于相机坐标系的X坐标
    fp32 fpPosY1;  //目标相对于相机坐标系的Y坐标
    fp32 fpPosZ1;  //目标相对于相机坐标系的Z坐标
    fp32 fpPosX2;  //目标相对于全局坐标系的X坐标
    fp32 fpPosY2;  //目标相对于全局坐标系的Y坐标
    fp32 fpPosZ2;  //目标相对于全局坐标系的Z坐标
};

#endif
