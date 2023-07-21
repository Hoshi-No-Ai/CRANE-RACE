#include "movement_info.h"
/*******************************************************************
函数名称：CalAngle()
函数功能：计算一向量与ssPosY轴正半轴夹角
输入：    stAim:目标向量
输出：    目标向量与ssPosY轴正半轴夹角(RADIAN)
备注：    逆时针为正，顺时针为负，计算夹角范围为：[-PI,PI)
********************************************************************/
void C_VECTOR::CalAngle(void) {
    if (fabs(fpVx) < EPS && fabs(fpVy) < EPS) {
        fpthetha = 0;
    } else {
        fpthetha = atan2f(fpVy, fpVx);
    }
}

void C_VECTOR::CalLength(void) { fpLength = sqrtf(fpVx * fpVx + fpVy * fpVy); }

void C_VECTOR::CalfpVx(void) { fpVx = fpLength * cosf(fpthetha); }

void C_VECTOR::CalfpVy(void) { fpVy = fpLength * sinf(fpthetha); }

void C_VECTOR::Covert_coordinate(COORDINATE coor_type) {
    if (type == POLAR) {
        CalfpVx();
        CalfpVy();
    } else {
        CalLength();
        CalAngle();
    }
    type = coor_type;
}

C_VECTOR::C_VECTOR(fp32 x, fp32 y, COORDINATE coor_type) : type(coor_type) {
    if (type == CARTESIAN) {
        fpVx = x;
        fpVy = y;
        CalLength();
        CalAngle();
    } else if (type == POLAR) {
        fpLength = x;
        fpthetha = y;
        CalfpVx();
        CalfpVy();
    }
}

C_VECTOR C_VECTOR::Vector_minus(C_VECTOR &a, C_VECTOR &b) {
    return C_VECTOR(a.fpVx - b.fpVx, a.fpVy - b.fpVy);
}

C_VECTOR C_VECTOR::Vector_Plus(C_VECTOR &a, C_VECTOR &b) {
    return C_VECTOR(a.fpVx + b.fpVx, a.fpVy + b.fpVy);
}

/****************************************************************************************************
函数名称: Vector_cross(C_VECTOR *r,fp32 w,C_VECTOR *c)
函数功能: 外积
输入参数:
返回参数:
备    注:
****************************************************************************************************/
C_VECTOR C_VECTOR::Vector_cross(C_VECTOR &r, fp32 w) { return C_VECTOR(-w * r.fpVy, w * r.fpVx); }

/*******************************************************************
    函数名称：CalNormalProjection()
    函数功能：计算一向量在基准向量方向投影
    输入：    stAim:目标向量，即需要投影的向量
              stBase:基准向量
    输出：    目标向量在基准向量上的投影
    备注：    当目标点在基准直线之间时，投影值为正，反之为负
    ********************************************************************/
fp32 C_VECTOR::CalNormalProjection(const C_VECTOR &stAim, const C_VECTOR &stBase) {
    return (fp32)(stBase.fpVx * stAim.fpVx + stBase.fpVy * stAim.fpVy) / (stBase.fpLength);
}

/*******************************************************************
函数名称：CalRadialProjection()
函数功能：计算一向量在基准向量法向方向投影
输入：    stAim:目标向量，即需要投影的向量
        stBase:基准向量
输出：    目标向量在基准向量法向方向上的投影
备注：    基准向量的法向向量为基准向量逆时针旋转90°得到
        当目标点在基准直线左侧时（以基准向量方向为正方向），投影值为正，反之为负
********************************************************************/
fp32 C_VECTOR::CalRadialProjection(const C_VECTOR &stAim, const C_VECTOR &stBase) {
    return (fp32)(-stBase.fpVy * stAim.fpVx + stBase.fpVx * stAim.fpVy) / (stBase.fpLength);
}

// 将全局坐标下的速度转化为局部坐标下的速度
void C_VECTOR::Concert_coorindnate(C_VECTOR &global, C_VECTOR &local, fp32 fpQ) {
    local.fpVx = global.fpVx * cosf(fpQ) + global.fpVy * sinf(fpQ);
    local.fpVy = -global.fpVx * sinf(fpQ) + global.fpVy * cosf(fpQ);
    local.CalLength();
    local.CalAngle();
    local.fpW = global.fpW;
}
