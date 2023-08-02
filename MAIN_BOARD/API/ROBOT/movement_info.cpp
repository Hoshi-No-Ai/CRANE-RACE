#include "movement_info.h"

void C_VECTOR::CalAngle(void)
{
    if (fabs(fpVx) < EPS && fabs(fpVy) < EPS)
    {
        fpthetha = 0;
    }
    else
    {
        fpthetha = atan2f(fpVy, fpVx);
    }
}

void C_VECTOR::CalLength(void) { fpLength = sqrtf(fpVx * fpVx + fpVy * fpVy); }

void C_VECTOR::CalfpVx(void) { fpVx = fpLength * cosf(fpthetha); }

void C_VECTOR::CalfpVy(void) { fpVy = fpLength * sinf(fpthetha); }

void C_VECTOR::Covert_coordinate(COORDINATE coor_type)
{
    if (type == POLAR)
    {
        CalfpVx();
        CalfpVy();
    }
    else
    {
        CalLength();
        CalAngle();
    }
    type = coor_type;
}

C_VECTOR::C_VECTOR(fp32 x, fp32 y, COORDINATE coor_type) : type(coor_type)
{
    if (type == CARTESIAN)
    {
        fpVx = x;
        fpVy = y;
        CalAngle();
    }
    else if (type == POLAR)
    {
        fpLength = x;
        fpthetha = y;
        CalfpVx();
        CalfpVy();
    }
}

C_VECTOR C_VECTOR::Vector_minus(C_VECTOR &a, C_VECTOR &b)
{
    return C_VECTOR(a.fpVx - b.fpVx, a.fpVy - b.fpVy);
}

C_VECTOR C_VECTOR::Vector_Plus(C_VECTOR &a, C_VECTOR &b)
{
    return C_VECTOR(a.fpVx + b.fpVx, a.fpVy + b.fpVy);
}

C_VECTOR C_VECTOR::Vector_cross(C_VECTOR &r, fp32 w) { return C_VECTOR(-w * r.fpVy, w * r.fpVx); }

fp32 C_VECTOR::CalNormalProjection(const C_VECTOR &stAim, const C_VECTOR &stBase)
{
    return (fp32)(stBase.fpVx * stAim.fpVx + stBase.fpVy * stAim.fpVy) / (stBase.fpLength);
}

fp32 C_VECTOR::CalRadialProjection(const C_VECTOR &stAim, const C_VECTOR &stBase)
{
    return (fp32)(-stBase.fpVy * stAim.fpVx + stBase.fpVx * stAim.fpVy) / (stBase.fpLength);
}

void C_VECTOR::Concert_coorindnate(C_VECTOR &global, C_VECTOR &local, fp32 fpQ)
{
    local.fpVx = global.fpVx * cosf(fpQ) + global.fpVy * sinf(fpQ);
    local.fpVy = -global.fpVx * sinf(fpQ) + global.fpVy * cosf(fpQ);
    local.CalLength();
    local.CalAngle();
    local.fpW = global.fpW;
}
