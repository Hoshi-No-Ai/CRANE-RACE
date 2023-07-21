/*---------------------------------------------------------------------
版权声明：HITCRT(哈工大竞技机器人队)
文件名：math_algorithm.h
最近修改日期：2022.10.14
版本：1.0
---------------------------------------------------------------------*/
#ifndef __MATH_ALGORITHM_H__
#define __MATH_ALGORITHM_H__

#include "basic_type.h"
#include "global_math.h"
#include "movement_info.h"
#include "stm32f4xx.h"

/*点坐标*/
struct stPoint {
    fp32 x;
    fp32 y;
};

struct stDT35 {
    fp32 x1;
    fp32 y1;
    fp32 x2;
    fp32 y2;
};

struct ANGLE {
    fp32 s;
    fp32 e;
};

struct T_fasedown {
    fp32 t11;         //加速时间
    fp32 t1;          //减速时间
    ST_VELT Accup;    //加速加速度
    ST_VELT Accdown;  //减速加速度
    ST_VELT VIN;      //开始减速时的初速度
};                    //减速结构体，先加速再减速

/*2021计算路径用*/
struct point_t {
    fp32 x, y;
};

struct line_t {
    fp32 A, B, C;  // Ax + By + C = 0
};

int32_t Round(fp32 fpValue);     //浮点数四舍五入转换为整型数函数
fp32 ConvertAngle(fp32 fpAngA);  //将角度转换为全局坐标系的航向角范围[-PI,PI)(单位：弧度)
int16_t ConvertDeg(int16_t fpDegA);  //将角度转换为全局坐标系的航向角范围[-1800,1800)(单位：0.1°)
void ramp_signal(float &Output, float DesValue, float Step);
int32_t Clip(int32_t siValue, int32_t siMin,
             int32_t siMax);  //整型削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值
uint8_t ClipChar(uint8_t ucValue, uint8_t ucMin, uint8_t ucMax);
fp32 ClipFloat(fp32 fpValue, fp32 fpMin,
               fp32 fpMax);  //浮点数削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值
fp32 Fabs(fp32 fpNum);                               //计算绝对值函数
uint16_t crc16(uint8_t *pucData, uint8_t ucLength);  //对固定长度的数组生成CRC校验码
uint8_t sum8(uint8_t *data, uint8_t length);         //求和函数
void us_delay(uint32_t uiTime);                      //延时us

stPoint tangentdot(stPoint ptCenter, stPoint ptOutside, fp32 Radious, int mode);
fp32 SLine(stPoint p1, stPoint p2);
stPoint Cirpoint(stPoint p1, stPoint p2, stPoint p3, fp32 Radius);
fp32 Angle(stPoint p0, stPoint p1);
fp32 SCircle(stPoint p1, stPoint p2, stPoint p0);
fp32 CalTime_min(fp32 A1, fp32 A2, fp32 S, fp32 V);
fp32 CalTime_mid(fp32 A1, fp32 A2, fp32 S, fp32 V);
fp32 CalRushTime(fp32 A, fp32 S, fp32 V);
void CalLineTime(fp32 A1, fp32 A2, fp32 S, fp32 *T1, fp32 *T11);
void CalT_dST(T_fasedown *T_dST, stPoint Start, stPoint End, ST_VELT Vin, fp32 A);
void Trapezoid_Speed(fp32 A, fp32 Vmax, fp32 S, fp32 *T1, fp32 *T11, fp32 *T12);
int8_t sign_judge(float fp_Judge_Number);

int32_t my_intabs(int32_t num);
fp32 cal_min_angle(fp32 des, fp32 fdb);

inline fp32 Geometric_mean(fp32 a, fp32 b) { return (sqrt(pow(a, 2) + pow(b, 2))); }
inline float COS(float a) { return (cosf(a * RADIAN)); }
inline float SIN(float a) { return (sinf(a * RADIAN)); }

#endif
