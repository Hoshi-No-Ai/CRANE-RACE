#ifndef __GIMBAL_CONTROL_TYPES__
#define __GIMBAL_CONTROL_TYPES__

#include "basic_type.h"

#define Cali_Rol_Coe 	1
#define Cali_Pit_Coe 	1
#define Cali_Yaw_Coe 	1

#define Cali_Rol_CoeA 1
#define Cali_Pit_CoeA 1
#define Cali_Yaw_CoeA 1

#define mat         arm_matrix_instance_f32
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

#define Main_Buf_Length 18

//typedef struct
//{
//    float raw_value;
//    float filtered_value[2];
//    mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
//} kalman_filter_t;

typedef struct
{
    UINT32 SysTickIRQ_cnt;

    USHORT16 IMUSample_cnt;
    USHORT16 IMUSample_fps;

    USHORT16 TempControl_cnt;
    USHORT16 TempControl_fps;

    USHORT16 MotionCtrl_cnt;
    USHORT16 MotionCtrl_fps;

    USHORT16 CAN_cnt;
    USHORT16 CAN_fps;

    USHORT16 USART_cnt;
    USHORT16 USART_fps;
} SYS_MNT;

typedef struct
{
    float raw_value;
    float filtered_value[2];
    float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
    float P_data[4];
    float AT_data[4], HT_data[4];
    float A_data[4];
    float H_data[4];
    float Q_data[4];
    float R_data[4];
} kalman_filter_init_t;

/*PID�������ṹ��*/
typedef struct
{
    FP32 fpDes;//���Ʊ���Ŀ��ֵ
    FP32 fpFB;//���Ʊ�������ֵ

    FP32 fpKp;//����ϵ��Kp
    FP32 fpKi;//����ϵ��Ki
    FP32 fpKd;//΢��ϵ��Kd

    FP32 fpUp;//�������
    FP32 fpUi;//�������
    FP32 fpUd;//΢�����

    FP32 fpE;//����ƫ��
    FP32 fpPreE;//�ϴ�ƫ��
    FP32 fpSumE;//��ƫ��
    FP32 fpU;//����PID������

    FP32 fpUMax;//PID�����������ֵ������������ʱ������ֵ
    FP32 fpEpMax;//������������ֵ
    FP32 fpEiMax;//������������ֵ
    FP32 fpEdMax;//΢����������ֵ
    FP32 fpEMin;//��������

    FP32 fpDt;//��������
} ST_PID;

typedef struct
{
    UCHAR8 Head1;
    UCHAR8 Head2;
    UCHAR8 Head3;
    UCHAR8 Head4;
    float Pitch_FB;
    float Yaw_FB;
    float Supply_FB;
    UCHAR8 Tail1;
    UCHAR8 Tail2;
} STMAINDATA;
typedef union
{
    STMAINDATA stMainData;
    UCHAR8 Main_Buf[Main_Buf_Length];				//#define Main_Buf_Length 18
} MAIN_SEND_DATA;

//С�������ݴ���ṹ��
typedef struct
{
    struct
    {
        u8 head[2];             //2
        u8 id;                  //1
        u8 num;                 //1
        float PitchPos_FB;      //4
        float YawPos_FB;        //4
        float BenjaminPos_FB;   //4
        u8 tail[2];             //2
    } Send;                     //total:18

    struct
    {
        u8 head[2];             //2
        u8 id;                  //1
        u8 num;                 //1
        float PitchPos_Des;     //4
        float YawPos_Des;       //4
        float ShooterPos_Des;   //4
        u8 tail[2];             //2
    } Receive;                  //total:18
} ST_MAINCONTROL;

typedef enum
{
    Idtfy_pitch,
    Idtfy_yaw,
    None,
    Complete,
} EM_IDTFY_MODE;

typedef struct
{
    FP32 fpJpp;
    FP32 fpJpy;
    FP32 fpJyy;
    FP32 fpFp;
    FP32 fpFy;
    FP32 fpGs;
    FP32 fpGc;
    EM_IDTFY_MODE emIdtfyMode;
    FunctionalState emSampleState;
} ST_IDTFY_CTRL;

/*������̽ṹ��*/
typedef struct
{
    SINT32 siRawValue;//���α�������ԭʼֵ
    SINT32 siPreRawValue;//��һ�α�������ԭʼֵ
    SINT32 siDiff;//����������ԭʼֵ�Ĳ�ֵ
    SINT32 siSumValue;//�������ۼ�ֵ
    SINT32 siGearRatio;//������������ٱ�
    SINT32 siNumber;//����������
    FP32   fpSpeed;//��������������ת�٣���λ��r/min
} ST_ENCODER;

typedef struct
{
    FP32 Acc;
    FP32 Spd;
    FP32 Cos_theta;
    FP32 Sin_theta;
    FP32 Sign_Spd;
    FP32 Input;
} IDTFY_DATA;

//typedef struct
//{
//    USART_TypeDef* USARTx;
//    DMA_Stream_TypeDef* DMAy_Streamx;
//    UCHAR8* pMailbox;
//    __IO UCHAR8* pDMAbuf;
//    USHORT16 MbLen;
//    USHORT16 DMALen;
//    USHORT16 rxConter;
//    USHORT16 rxBufferPtr;
//    USHORT16 rxSize;
//} USART_RX_TypeDef;

//typedef struct
//{
//    USART_TypeDef* USARTx;
//    DMA_Stream_TypeDef* DMAy_Streamx;
//    UCHAR8* pMailbox;
//    __IO UCHAR8* pDMAbuf;
//    USHORT16 MbLen;
//    USHORT16 DMALen;
//} USART_TX_TypeDef;

//typedef struct
//{
//    float preout;
//    float out;
//    float in;
//    float off_freq;
//    float samp_tim;
//} ST_LPF;

//typedef struct
//{
//    FP32 x1;
//    FP32 x2;
//    FP32 x;
//    FP32 r;
//    float h;
//    FP32 T;
//    FP32 aim;
//} TD;


enum
{
    A_X = 0,
    A_Y,
    A_Z,
    G_X,
    G_Y,
    G_Z,
    TEM,
    MPU_ITEMS,
};


enum
{
    CH_ROL = 0,
    CH_PIT,
    CH_THR,
    CH_YAW,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    CH_NUM,
};

enum
{
    m1=0,
    m2,
    m3,
    m4,
    m5,
    m6,
    m7,
    m8,

};

enum
{
    MPU_6050_0 = 0,
    MPU_6050_1,

};

enum
{
    X = 0,
    Y = 1,
    Z = 2,
    VEC_XYZ,
};

enum
{
    ROL = 0,
    PIT = 1,
    YAW = 2,
    VEC_RPY,
};

typedef struct
{
    float q0;//q0;
    float q1;//q1;
    float q2;//q2;
    float q3;//q3;

    float gkp;
    float gki;

    float x_vec[VEC_XYZ];
    float y_vec[VEC_XYZ];
    float z_vec[VEC_XYZ];

    float a_acc[VEC_XYZ];
    float gacc_deadzone[VEC_XYZ];
    float gra_acc[VEC_XYZ];

    float rol;
    float pit;
    float yaw;
} _imu_st ;



#define USART5_RX_STREAM        DMA1_Stream0
#define USART3_RX_STREAM        DMA1_Stream1
#define UART4_RX_STREAM         DMA1_Stream2
#define USART3_TX_STREAM        DMA1_Stream3
#define UART4_TX_STREAM         DMA1_Stream4
#define USART2_RX_STREAM        DMA1_Stream5
#define USART2_TX_STREAM        DMA1_Stream6
#define USART5_TX_STREAM        DMA1_Stream7
#define UART8_TX_STREAM         DMA1_Stream0
#define UART7_TX_STREAM         DMA1_Stream1
#define UART7_RX_STREAM         DMA1_Stream3
#define UART8_RX_STREAM         DMA1_Stream6
#define USART6_RX_STREAM        DMA2_Stream1
#define USART1_RX_STREAM        DMA2_Stream5
#define USART6_TX_STREAM        DMA2_Stream6
#define USART1_TX_STREAM        DMA2_Stream7

#define USART5_RX_CHANNEL       DMA_Channel_4
#define USART3_RX_CHANNEL       DMA_Channel_4
#define UART4_RX_CHANNEL        DMA_Channel_4
#define USART3_TX_CHANNEL       DMA_Channel_4
#define UART4_TX_CHANNEL        DMA_Channel_4
#define USART2_RX_CHANNEL       DMA_Channel_4
#define USART2_TX_CHANNEL       DMA_Channel_4
#define USART5_TX_CHANNEL       DMA_Channel_4
#define UART8_TX_CHANNEL        DMA_Channel_5
#define UART7_TX_CHANNEL        DMA_Channel_5
#define UART7_RX_CHANNEL        DMA_Channel_5
#define UART8_RX_CHANNEL        DMA_Channel_5
#define USART1_TX_CHANNEL       DMA_Channel_4
#define USART1_RX_CHANNEL       DMA_Channel_4
#define USART6_TX_CHANNEL       DMA_Channel_5
#define USART6_RX_CHANNEL       DMA_Channel_5

/*����1ͨ�Ż��峤��*/
#define USART3_TXDMA_LEN           40
#define USART3_TXMB_LEN            20

#define USART3_RXDMA_LEN           40
#define USART3_RXMB_LEN            20

#define RC_ON (system_monitor.USART4_fps>950)

#endif
