#include "vision_algorithm.h"
fp32 K;                   //角度距离系数
fp32 VisionY;             //转换后Y
fp32 VisionX;             //转换后X
fp32 ConvertArray[2][2];  //相机中心转换矩阵
void DealVision(void) {
    fp32 R;
    fp32 theata;
    R = sqrt(pow(VisionX, 2) + pow(VisionY, 2));
    theata = asin(R / K) / 2;
}
