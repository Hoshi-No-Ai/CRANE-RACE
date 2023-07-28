#ifndef __VISION_ALGORITHM_H__
#define __VISION_ALGORITHM_H__

#include "basic_type.h"
#include "math.h"
#include "usart_protocol.h"
#include "locate_algorithm.h"
#include "navigation_algorithm.h"

#define DES_ARUCO_CENTER 0.0f
#define DES_CENTER_DES 0.0f

#define DELTA_COLA 68.0f

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

extern Delta_2D delta_fb_des;
extern Delta_2D delta_des_cola_w;

#endif
