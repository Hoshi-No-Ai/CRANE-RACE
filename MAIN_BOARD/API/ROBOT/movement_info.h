#ifndef __MOVEMENT_INFO_H__
#define __MOVEMENT_INFO_H__

#include "basic_type.h"
#include "global_math.h"
#include "math.h"

/*����궨�ṹ��*/
struct ST_POS {
    fp32 X;  // ������X����λ��mm��
    fp32 Y;  // ������Y����λ��mm��
    fp32 Q;  // �����Q����λ��0.1�ȣ�
};

/*������̬�ṹ��*/
struct ST_POT {
    fp32 fpPosX;  // ������X����λ��mm��
    fp32 fpPosY;  // ������Y����λ��mm��
    fp32 fpPosQ;  // �����Q����λ���ȣ�
    fp32 fpPosX1;
    fp32 fpPosY1;
    fp32 fpPosQ1;
};

/*�ٶȽṹ��*/
struct ST_VELT {
    fp32 fpVx;  // �ط����ٶȣ���λmm/s��
    fp32 fpVy;  // Y�����ٶȣ���λ��mm/s��
    fp32 fpW;   // ���ٶȣ���λ0.1��/s��
};

enum COORDINATE {
    CARTESIAN,  // �ѿ�������ϵ
    POLAR       // ������ϵ
};

/*�����йؽṹ��*/
class C_VECTOR {
   public:
    fp32 fpVx;        // X�����
    fp32 fpVy;        // Y�����
    fp32 fpW;         // ��ת�ٶ�
    fp32 fpLength;    // �������ȣ���λmm��
    fp32 fpthetha;    // ������X��Ƕȣ���λ:���ȣ�
    COORDINATE type;  // ����ϵ����

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
                             const C_VECTOR &stBase);  // ����һ�����ڻ�׼����������ͶӰ
    fp32 CalNormalProjection(const C_VECTOR &stAim, const C_VECTOR &stBase);  // ����һ�����ڻ�׼��������ͶӰ

    static void Concert_coorindnate(C_VECTOR &global, C_VECTOR &local,
                                    fp32 fpQ);  // ��ȫ������ϵת��Ϊ��������ϵ
};

#endif
