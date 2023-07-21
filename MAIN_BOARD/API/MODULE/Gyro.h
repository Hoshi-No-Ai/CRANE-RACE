#ifndef __GYRO_H__
#define __GYRO_H__

#include "basic_type.h"

#define GAINBtw50hz (3.450423889e+02f)
#define AD_Num 160

extern int GyroCount, CalFlag;
extern double AD_Table[AD_Num];
extern double AD_Table_a[AD_Num];
extern double AD_Table_b[AD_Num];
extern fp64 GyroData, gyroGz, Angel_d, Angel_v, RobAngel, TestAngle, TestAngle2;
extern fp64 GyroTab[3];
extern fp64 GyrotestTab[3];

extern fp32 value_door;

struct Bw50HzLPFTypeDef {
    float xv[4];
    float yv[4];
    float input;
    float output;
};

extern Bw50HzLPFTypeDef GyroLPF;

void Butterworth50HzLPF(Bw50HzLPFTypeDef* pLPF);
fp64 Gyro_TableLookUp(fp64 DataAccu);

#endif
