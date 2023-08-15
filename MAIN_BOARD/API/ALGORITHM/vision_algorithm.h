#ifndef __VISION_ALGORITHM_H__
#define __VISION_ALGORITHM_H__

#include "basic_type.h"
#include "math.h"
#include "usart_protocol.h"
#include "locate_algorithm.h"
#include "navigation_algorithm.h"

#define ARUCO_2_DES_X -2.750164264970416e+02f//-2.887154954258607e+02f
#define ARUCO_2_DES_Y 4.095191999176823e+02f//4.116275204611565e+02f

#define DELTA_COLA 90.0f

// //世界坐标中，码到机器人目标点的距离.
// #define DELTA_ARUCO_DES_X 0.0f
// #define DELTA_ARUCO_DES_Y 0.0f

struct ST_CAMERA
{
    fp32 fpQx;
    fp32 fpQy;
    fp32 fpQz;
    fp32 fpPosX0;
    fp32 fpPosY0;
    fp32 fpPosZ0;
    fp32 fpPosX1;
    fp32 fpPosY1;
    fp32 fpPosZ1;
    fp32 fpPosX2;
    fp32 fpPosY2;
    fp32 fpPosZ2;
};
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
void delta_des_cola(int num);
bool Identify_box_cola(int &target);
bool Detect_Object(float *temp_result, int *final_result);

extern Delta_2D delta_fb_des;
extern Delta_2D delta_des_cola_w;
extern Delta_2D delta_fb_aruco_w;
extern Delta_2D delta_aruco_des_w;

#endif
