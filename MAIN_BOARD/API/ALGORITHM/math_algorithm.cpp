/*******************************************************************
��Ȩ������HITCRT(�����󾺼������˶�)
�ļ�����math_algorithm.cpp
����޸����ڣ�2022.10.14
�汾����N��
���ߣ�JIN
**********************************************************************/

#include "math_algorithm.h"

/*******************************************************************
�������ƣ�Round()
�������ܣ����������������룬����32λ������
���룺    fpValue
�����    ��������󷵻ص�������
��ע��
********************************************************************/
int32_t Round(fp32 fpValue) {
    if (fpValue >= 0) {
        return (int32_t)(fpValue + 0.5f);
    } else {
        return (int32_t)(fpValue - 0.5f);
    }
}

/*******************************************************************
�������ƣ�ConvertAngle()
�������ܣ����Ƕ�ת��Ϊȫ������ϵ�ĺ���Ƿ�Χ[-PI,PI)
���룺    ang��Ŀ��Ƕ�(RADIAN)
�����    ת����ĽǶ�(RADIAN)
��ע��    ��ʱ��Ϊ����˳ʱ��Ϊ�������ʺ϶ԽǶ�ֵ�ϴ��ֵ��ת��
********************************************************************/
fp32 ConvertAngle(fp32 fpAngA) {
    do {
        if (fpAngA >= PI) {
            fpAngA -= PI2;
        } else if (fpAngA < -PI) {
            fpAngA += PI2;
        }
    } while (fpAngA >= PI || fpAngA < -PI);
    return fpAngA;
}

/*******************************************************************
�������ƣ�ConvertDeg()
�������ܣ����Ƕ�ת��Ϊȫ������ϵ�ĺ���Ƿ�Χ[-1800,1800)(��λ��0.1��)
���룺    ang��Ŀ��Ƕ�(��λ��0.1��)
�����    ת����ĽǶ�(��λ��0.1��)
��ע��    ��ʱ��Ϊ����˳ʱ��Ϊ�������ʺ϶ԽǶ�ֵ�ϴ��ֵ��ת��
********************************************************************/
int16_t ConvertDeg(int16_t fpDegA) {
    do {
        if (fpDegA >= 1800) {
            fpDegA -= 3600;
        } else if (fpDegA < -1800) {
            fpDegA += 3600;
        }
    } while (fpDegA >= 1800 || fpDegA < -1800);
    return fpDegA;
}

//�������ܣ�б�������ź�
void ramp_signal(float Output, float DesValue, float Step) {
    if (fabs(Output - DesValue) <= Step) {
        Output = DesValue;
    } else {
        if (Output < DesValue) {
            Output += Step;
        } else {
            Output -= Step;
        }
    }
}

/*******************************************************************
�������ƣ�Clip()
�������ܣ�����������ȥ���������ֵ����Сֵ֮���ֵ����֮��������Сֵ
���룺    siValue:ʵ��ֵ
          siMin:����ֵ
          siMax:����ֵ
�����    siValue���������ֵ
��ע��	  ���������ͱ���������
********************************************************************/
int32_t Clip(int32_t siValue, int32_t siMin, int32_t siMax) {
    if (siValue < siMin) {
        return siMin;
    } else if (siValue > siMax) {
        return siMax;
    } else {
        return siValue;
    }
}

/*******************************************************************
�������ƣ�ClipChar()
�������ܣ�����������ȥ���������ֵ����Сֵ֮���ֵ����֮��������Сֵ
���룺    ucValue:ʵ��ֵ
          ucMin:����ֵ
          ucMax:����ֵ
�����    ucValue���������ֵ
��ע��	  �������޷����ַ�������
********************************************************************/
uint8_t ClipChar(uint8_t ucValue, uint8_t ucMin, uint8_t ucMax) {
    if (ucValue < ucMin) {
        return ucMin;
    } else if (ucValue > ucMax) {
        return ucMax;
    } else {
        return ucValue;
    }
}

/*******************************************************************
�������ƣ�ClipFloat()
�������ܣ�����������ȥ���������ֵ����Сֵ֮���ֵ����֮��������Сֵ
���룺    fpValue:ʵ��ֵ
          fpMin:����ֵ
          fpMax:����ֵ
�����    fpValue���������ֵ
��ע��	  �����ڸ���������������
********************************************************************/
fp32 ClipFloat(fp32 fpValue, fp32 fpMin, fp32 fpMax) {
    if (fpValue < fpMin) {
        return fpMin;
    } else if (fpValue > fpMax) {
        return fpMax;
    } else {
        return fpValue;
    }
}
/*******************************************************************
�������ƣ�Fabs()
�������ܣ��������ֵ����
���룺    fpNum: ʵ��ֵ
�����    fpNum���������ֵ
��ע��
********************************************************************/
fp32 Fabs(fp32 fpNum) {
    if (fpNum < 0) {
        return -fpNum;
    }

    return fpNum;
}

// CRC��λ�ֽ�ֵ��
const uint8_t aucCRCHighTable[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};
// CRC��λ�ֽ�ֵ��
const uint8_t aucCRCLowTable[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40};
/***********************************************************************************
�������ƣ�crc16()
�������ܣ��Թ̶����ȵ���������CRCУ����
��ڲ�����Ҫ����У���������׵�ַ������ĳ���
����ֵ��  У��Ľ��Ϊ16λ uint16_t(�޷���16λ���ͱ���)
************************************************************************************/
uint16_t crc16(uint8_t *pucData, uint8_t ucLength) {
    uint8_t ucCRCHi = 0xFF;  // ��CRC�ֽڳ�ʼ��
    uint8_t ucCRCLo = 0xFF;  // ��CRC�ֽڳ�ʼ��
    uint32_t uiIndex;        // CRCѭ���е�����

    while (ucLength--) {
        // ����CRC
        uiIndex = ucCRCLo ^ *pucData++;
        ucCRCLo = ucCRCHi ^ aucCRCHighTable[uiIndex];
        ucCRCHi = aucCRCLowTable[uiIndex];
    }
    return ((ucCRCHi << 8) | ucCRCLo);
}

/****************************************************************************************************
�������ƣ�us_delay()
�������ܣ���ʱus
��ڲ�������Ҫ��ʱ��ʱ��
****************************************************************************************************/
void us_delay(uint32_t uiTime) {
    uint32_t uiDelay;
    while (uiTime--) {
        uiDelay = 25;
        while (uiDelay--)
            ;
    }
}

/***********************************************************************************
�������ƣ�sum8()
�������ܣ��Թ̶����ȵ���������SUMУ����
��ڲ�����Ҫ����У���������׵�ַ������ĳ���
����ֵ��  У��Ľ��Ϊ8λ uint16_t(�޷���8λ�ַ���)
************************************************************************************/
uint8_t sum8(uint8_t *data, uint8_t length) {
    uint8_t chSUM = 0x00;
    while (length--)
    //����sum
    {
        chSUM += *data;
        data++;
    }
    return (chSUM);
}

/*-------------------------------------------------------------------------------------------------
�� �� ����tangentdot
�������ܣ����Բ��һ���ֱ����Բ���е�
��    ע��ptCenterԲ�����꣬ptOutsideԲ��㣬RadiousԲ�뾶��modeģʽ�������е�����
-------------------------------------------------------------------------------------------------*/
stPoint tangentdot(stPoint ptCenter, stPoint ptOutside, fp32 Radious, int mode) {
    stPoint E, F, G, H;
    fp32 r = Radious;
    fp32 a;

    E.x = ptOutside.x - ptCenter.x;
    E.y = ptOutside.y - ptCenter.y;

    fp32 t = r / sqrt(E.x * E.x + E.y * E.y);
    F.x = E.x * t;
    F.y = E.y * t;  //�������뵼�����ı���������һ���油����λ��һ���油����������ƫ��

    if (mode)  // mode=1,�е���Բ��ָ��Բ��һ�����������ʱ�뷽��һ��
        a = acos(t);
    else
        a = -acos(t);
    G.x = F.x * cos(a) - F.y * sin(a);
    G.y = F.x * sin(a) + F.y * cos(a);

    H.x = G.x + ptCenter.x;
    H.y = G.y + ptCenter.y;

    return H;
}

fp32 Angle(stPoint p0, stPoint p1) {
    stPoint p;
    fp32 alpha;
    p.x = p1.x - p0.x;
    p.y = p1.y - p0.y;

    if (p.y == 0)  //��ĸΪ0
    {
        if (p.x > 0) {
            return 0;
        } else if (p.x < 0) {
            return PI;
        } else {
            return 0;
        }
    } else {
        if (p.x == 0) {
            if (p.y > 0) {
                return PI_2;
            } else if (p.y < 0) {
                return -PI_2;
            } else {
                return 0;
            }
        } else {
            alpha = atanf((fp32)p.y / (fp32)p.x);
            if (p.x < 0)  // 3��4����
            {
                if (alpha >= 0)  // 4����
                {
                    alpha -= PI;
                } else if (alpha < 0)  // 3
                {
                    alpha += PI;
                }
            }
            return alpha;
        }
    }
}

//���㳤��
fp32 SLine(stPoint p1, stPoint p2) {
    return (fp32)sqrtf((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}
//����Բ������
fp32 SCircle(stPoint p1, stPoint p2, stPoint p0) {
    fp32 theta, theta1, theta2, Radius;
    Radius = SLine(p1, p0);
    theta1 = Angle(p0, p1);
    theta2 = Angle(p0, p2);

    if ((theta2 - theta1) > PI)
        theta2 -= PI2;
    else if ((theta2 - theta1) < -PI)
        theta2 += PI2;
    theta = fabs(theta2 - theta1);
    return theta * Radius;
}
/*******************************************************************
�������ƣ�Cirpoint()
�������ܣ�����Բ������
���룺    p1��p2��p3ΪԲ��3�㣬RadiusΪ�뾶��С
�����    Բ������
��ע��
********************************************************************/
stPoint Cirpoint(stPoint p1, stPoint p2, stPoint p3, fp32 Radius) {
    // ax+by=m
    // cx+dy+n
    fp32 a, b, c, d;
    fp32 m[4], n[4];
    stPoint p;
    int i;
    fp32 A, B, C;
    a = p2.y - p1.y;
    b = p1.x - p2.x;
    c = p2.y - p3.y;
    d = p3.x - p2.x;

    m[0] = (p2.y - p1.y) * p2.x - (p2.x - p1.x) * p2.y + SLine(p1, p2) * Radius;
    n[0] = (p2.y - p3.y) * p2.x - (p2.x - p3.x) * p2.y + SLine(p3, p2) * Radius;

    m[1] = (p2.y - p1.y) * p2.x - (p2.x - p1.x) * p2.y + SLine(p1, p2) * Radius;
    n[1] = (p2.y - p3.y) * p2.x - (p2.x - p3.x) * p2.y - SLine(p3, p2) * Radius;

    m[2] = (p2.y - p1.y) * p2.x - (p2.x - p1.x) * p2.y - SLine(p1, p2) * Radius;
    n[2] = (p2.y - p3.y) * p2.x - (p2.x - p3.x) * p2.y - SLine(p3, p2) * Radius;

    m[3] = (p2.y - p1.y) * p2.x - (p2.x - p1.x) * p2.y - SLine(p1, p2) * Radius;
    n[3] = (p2.y - p3.y) * p2.x - (p2.x - p3.x) * p2.y + SLine(p3, p2) * Radius;

    for (i = 0; i <= 3; ++i) {
        p.x = (m[i] * d - b * n[i]) / (a * d - b * c);
        p.y = (a * n[i] - m[i] * c) / (a * d - b * c);
        A = Angle(p2, p1);
        B = Angle(p2, p);
        C = Angle(p2, p3);
        if (fabs(A - C) < PI) {
            if (fabs(B - (A + C) / 2.0f) < 0.0001f) return p;
        } else {
            if (A < 0) A = A + 2.0f * PI;
            if (B < 0) B = B + 2.0f * PI;
            if (C < 0) C = C + 2.0f * PI;
            if (fabs(B - (A + C) / 2.0f) < 0.0001f) return p;
        }
    }
    return p;
}

/*******************************************************************
�������ƣ�CalTime_min()
�������ܣ�������ĳһ�ٶȵ���ĳһ�����̼���ʱ��ͼ���ʱ��
���룺    A1�����ٶȣ�A2�����ٶȣ�S���룬VĿ���ٶ�
�����    �����ʱ��T_min
��ע��    ��0��ʼ����
********************************************************************/
fp32 CalTime_min(fp32 A1, fp32 A2, fp32 S, fp32 V) {
    fp32 T_min;
    T_min = (-V * A1 + sqrt(V * V * A1 * A1 + A1 * A2 * V * V + 2 * A1 * A2 * S * (A1 + A2))) / (A1 * A2);
    return T_min;
}

/*******************************************************************
�������ƣ�CalTime_mid()
�������ܣ�������ĳһ�ٶȵ���ĳһ�����̼���ʱ��ͼ���ʱ��
���룺    A1�����ٶȣ�A2�����ٶȣ�S���룬VĿ���ٶ�
�����    T_min�еļ���ʱ��
��ע��    ��0��ʼ����
********************************************************************/
fp32 CalTime_mid(fp32 A1, fp32 A2, fp32 S, fp32 V) {
    fp32 T_min;
    fp32 T_mid;
    T_min = CalTime_min(A1, A2, S, V);
    T_mid = (A2 * T_min + V) / (A1 + A2);
    return T_mid;
}

/*******************************************************************
�������ƣ�CalLineTime()
�������ܣ��Գ���Ϊ0�����ٶ�A1���١�A2���٣�ֹͣ�������Sλ�ô�����Ҫ�ļ���ʱ��
���룺    A1���ٶȣ�A2���ٶȣ�S����
�����    ����ʱT1������ʱ��T11
��ע��
********************************************************************/
void CalLineTime(fp32 A1, fp32 A2, fp32 S, fp32 *T1, fp32 *T11) {
    *T1 = sqrt(2.0f * (A1 + A2) * S / (A1 * A2));
    *T11 = *T1 * A2 / (A1 + A2);
}

/*******************************************************************
�������ƣ�Trapezoid_Speed()
�������ܣ���0���ٵģ�������ٶ������µ������ٶȹ滮����δ�ﵽ����������������ٶȹ滮
���룺    A���ٶȣ�S���룬Vmax����ٶ�����
�����    T1��ʱ�䡢T11����ʱ�䡢T12����ʱ���������ʱ��
��ע��    A����Ϊ0
********************************************************************/
void Trapezoid_Speed(fp32 A, fp32 Vmax, fp32 S, fp32 *T1, fp32 *T11, fp32 *T12) {
    fp32 V_expect;  //ȫ�̼Ӽ���ʱ���˶������е�����ٶ�
    fp32 S_acc;     //�Ӽ���·��
    V_expect = sqrt(A * S);
    if (V_expect < Vmax) {
        CalLineTime(A, A, S, T1, T11);
        *T12 = *T11;
    } else {
        *T11 = Vmax / A;
        S_acc = Vmax * Vmax / A;
        *T12 = *T11 + (S - S_acc) / Vmax;
        *T1 = *T12 + *T11;
    }
}
/*******************************************************************
�������ƣ�CalRushTime()
�������ܣ�������ĳһ���ٶȼ��ٵ���ĳһ��ĳ��ʱ��
���룺    A���ٶȣ�S���룬V���ٶ�
�����    ����ʱ��T_Rush
��ע��    A����Ϊ0
********************************************************************/
fp32 CalRushTime(fp32 A, fp32 S, fp32 V) {
    fp32 T_Rush;
    T_Rush = (sqrt(V * V + 2.0f * A * S) - V) / A;
    return T_Rush;
}

/*******************************************************************
�������ƣ�CalT_dST()
�������ܣ���ĳһ���ٶ��Լ��ٶ�A�����ټ��������END��ͣ�£���Լ��ٽṹ��ļӼ���ʱ�䡢ʸ�������ٶȡ��Ӽ��ٵļ��ٶȸ�ֵ
���룺    T_dST���ٽṹ�塢Start�����㡢End�յ㡢Vin���ٶȡ�A���ٶ�
�����
��ע��    ���Vin��ֱ�߷�����ͶӰ�ټ��㣬�����򲻶�������ʵ���ϵ�����
********************************************************************/
void CalT_dST(T_fasedown *T_dST, stPoint Start, stPoint End, ST_VELT Vin, fp32 A) {
    fp32 S;
    static ST_VELT S_vector;
    static fp32 Vin_real;  //ʵ�ʳ��ٶ��������·�������ϵ�ͶӰ
    static fp32 T11;
    S = sqrt(pow(End.x - Start.x, 2) + pow(End.y - Start.y, 2));
    S_vector.fpVx = End.x - Start.x;  //��·��������
    S_vector.fpVy = End.y - Start.y;
    Vin_real = fabs((Vin.fpVx * S_vector.fpVx + Vin.fpVy * S_vector.fpVy) / S);
    T_dST->VIN.fpVx = Vin_real * S_vector.fpVx / S;
    T_dST->VIN.fpVy = Vin_real * S_vector.fpVy / S;
    T_dST->t1 = CalTime_min(A, A, S, Vin_real);  //�����ܼ���ʱ��
    T11 = CalTime_mid(A, A, S, Vin_real);
    T_dST->t11 = T_dST->t1 - T11;  //�������ʱ��
    T_dST->Accup.fpVx =
        (2 * (End.x - Start.x) - T_dST->VIN.fpVx * (T_dST->t1 + T_dST->t11)) / (T_dST->t11 * T_dST->t1);
    T_dST->Accup.fpVy =
        (2 * (End.y - Start.y) - T_dST->VIN.fpVy * (T_dST->t1 + T_dST->t11)) / (T_dST->t11 * T_dST->t1);
    T_dST->Accdown.fpVx = (T_dST->VIN.fpVx + T_dST->Accup.fpVx * T_dST->t11) / (T_dST->t1 - T_dST->t11);
    T_dST->Accdown.fpVy = (T_dST->VIN.fpVy + T_dST->Accup.fpVy * T_dST->t11) / (T_dST->t1 - T_dST->t11);
    if (T_dST->t11 < 0)  //�����ټ�����A�ļ��ٶ����޷�ʵ�֣�����ǿ���٣�ͻ�Ƽ��ٶ�����
    {
        T_dST->t1 = 2.0f * S / Vin_real;
        T_dST->Accdown.fpVx = 2.0f * S_vector.fpVx / (T_dST->t1 * T_dST->t1);
        T_dST->Accdown.fpVy = 2.0f * S_vector.fpVy / (T_dST->t1 * T_dST->t1);
        T_dST->Accup.fpVx = 0;
        T_dST->Accup.fpVy = 0;
    }
}

int32_t my_intabs(int32_t num) {
    if (num >= 0) {
        return num;
    } else {
        return -num;
    }
}

/*-------------------------------------------------------------------
�������ܣ��ж�����
��    ע������ֵΪ1��-1�����ı����ķ���
-------------------------------------------------------------------*/
int8_t sign_judge(float fp_Judge_Number) { return fp_Judge_Number >= 0 ? 1 : -1; }

/*�����des��ΧΪ0��360�ȣ����ؾ���fdb����ĽǶ�ֵ*/
fp32 cal_min_angle(fp32 des, fp32 fdb) {
    fp32 abs_fdb;

    abs_fdb = ((int32_t)fdb + 720000) % 360 + fdb - (int32_t)fdb;

    if (fabs(des - abs_fdb) > 180) {
        if (des > abs_fdb) {
            des -= 360;
        } else {
            des += 360;
        }
    }
    return (fdb + des - abs_fdb);
}