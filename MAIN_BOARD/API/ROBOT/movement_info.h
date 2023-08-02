#ifndef __MOVEMENT_INFO_H__
#define __MOVEMENT_INFO_H__

#include "basic_type.h"
#include "global_math.h"
#include "math.h"

/*坐标标定结构体*/
struct ST_POS
{
    fp32 X; // 横坐标X（单位：mm）
    fp32 Y; // 竖坐标Y（单位：mm）
    fp32 Q; // 航向角Q（单位：0.1度）
};

/*坐标姿态结构体*/
struct ST_POT
{
    fp32 fpPosX; // 横坐标X（单位：mm）
    fp32 fpPosY; // 竖坐标Y（单位：mm）
    fp32 fpPosQ; // 航向角Q（单位：度）
    fp32 fpPosX1;
    fp32 fpPosY1;
    fp32 fpPosQ1;
};

/*速度结构体*/
struct ST_VELT
{
    fp32 fpVx; // Ｘ方向速度（单位mm/s）
    fp32 fpVy; // Y方向速度（单位：mm/s）
    fp32 fpW;  // 角速度（单位0.1度/s）
};

enum COORDINATE
{
    CARTESIAN, // 笛卡尔坐标系
    POLAR      // 极坐标系
};

/*向量有关结构体*/
class C_VECTOR
{
public:
    fp32 fpVx;       // X方向差
    fp32 fpVy;       // Y方向差
    fp32 fpW;        // 旋转速度
    fp32 fpLength;   // 向量长度（单位mm）
    fp32 fpthetha;   // 向量与X轴角度（单位:弧度）
    COORDINATE type; // 坐标系类型

    void CalAngle(void);
    void CalLength(void);
    void CalfpVx(void);
    void CalfpVy(void);

    void Covert_coordinate(COORDINATE coor_type);

    C_VECTOR() {}
    C_VECTOR(fp32 x, fp32 y, COORDINATE coor_type = CARTESIAN);
    ~C_VECTOR(){};

    static C_VECTOR Vector_minus(C_VECTOR &a, C_VECTOR &b);
    static C_VECTOR Vector_Plus(C_VECTOR &a, C_VECTOR &b);

    static C_VECTOR Vector_cross(C_VECTOR &r, fp32 w);

    fp32 CalRadialProjection(const C_VECTOR &stAim,
                             const C_VECTOR &stBase);                        // 计算一向量在基准向量法向方向投影
    fp32 CalNormalProjection(const C_VECTOR &stAim, const C_VECTOR &stBase); // 计算一向量在基准向量方向投影

    static void Concert_coorindnate(C_VECTOR &global, C_VECTOR &local,
                                    fp32 fpQ); // 将全局坐标系转化为本地坐标系
};

#endif
