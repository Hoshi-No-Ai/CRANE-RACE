#include "vision_algorithm.h"
fp32 K;                   //�ǶȾ���ϵ��
fp32 VisionY;             //ת����Y
fp32 VisionX;             //ת����X
fp32 ConvertArray[2][2];  //�������ת������
void DealVision(void) {
    fp32 R;
    fp32 theata;
    R = sqrt(pow(VisionX, 2) + pow(VisionY, 2));
    theata = asin(R / K) / 2;
}
