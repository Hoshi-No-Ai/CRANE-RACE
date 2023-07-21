#ifndef __MOTOR_DRIVE_H__
#define __MOTOR_DRIVE_H__

#include "stm32f4xx.h"
#include "basic_type.h"
#include "pid_algorithm.h"
#include "filter_algorithm.h"
#include "math_algorithm.h"
#include "hitcrt_os.h"
#include "system_monitor_task.h"

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

#define DEVICE_STD_ID						(0x140)

//HT_03
#define HT_03 0

#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f    
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
//DM

#define DM 1

 #define P_MIN2 -30.0f
 #define P_MAX2 30.0f
 #define V_MIN2 -30.0f
 #define V_MAX2 30.0f
 #define KP_MIN2 0.0f
 #define KP_MAX2 500.0f
 #define KD_MIN2 0.0f
 #define KD_MAX2 5.0f
 #define T_MIN2 -10.0f
 #define T_MAX2 10.0f

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

/*����������ֵ��ؽṹ��*/
class cEncoder
{
		public:
		int32_t 	siRawValue;//���α�������ԭʼֵ
		int32_t 	siPreRawValue;//��һ�α�������ԭʼֵ
		int32_t 	siDiff;//����������ԭʼֵ�Ĳ�ֵ
		int32_t 	siSumValue;//�������ۼ�ֵ
		float 	  siGearRatio;//������������ٱ�
		int32_t 	siNumber;//����������
		float   	fpSpeed;//��������������ת�٣���λ��r/min
		int       motor_flag;
		cEncoder(){}
		cEncoder( float gr, int32_t num )
		{
			this->siNumber = num;
			this->siGearRatio = gr;
			this->motor_flag = 0;
			
		}
	void Encoder_Process(uint8_t type);    //3508���

};


/*˫�����Ƶ����PID��*/
class cMotor_double
{
	public:
	float real_current;
	float pid_current;
	cPID velt_pid;
	cPID pos_pid;
	cEncoder encoder;
	
	
	cMotor_double(){}
	cMotor_double(float vKp, float vKi, float vKd, float vUpMax, float vEiMax, float vSumEMax, float vUdMax, float vEMin,float vEMax,
				 float pKp, float pKi, float pKd, float pUpMax, float pEiMax, float pSumEMax, float pUdMax, float pEMin,float pEMax,
				 float gr, int32_t num,float ts)
	{
		velt_pid = cPID(vKp, vKi, vKd, vUpMax, vEiMax, vSumEMax, vUdMax, vEMin, vEMax, ts);
		pos_pid  = cPID(pKp, pKi, pKd, pUpMax, pEiMax, pSumEMax, pUdMax, pEMin, pEMax, ts);
		encoder  = cEncoder(gr, num);
	}
};


/*˫�����Ƶ����pid+td��*/
class cMotor_double_td
{
	public:

	float real_current;
	float pid_current;

	cPID velt_pid;
	cPID pos_pid;

	cTD td;

	cEncoder encoder;
	
	cMotor_double_td(){}
	cMotor_double_td(
	/*�ٶȻ�*/	          float vKp, float vKi, float vKd, float vUpMax, float vEiMax, float vSumEMax, float vUdMax, float vEMin, float vEMax,
	/*λ�û�*/  		      float pKp, float pKi, float pKd, float pUpMax, float pEiMax,  float pSumEMax, float pUdMax, float pEMin, float pEMax,
	/*����ֵ*/			      float gr, int32_t num,
	/*TD*/     						float R, float H, float ts)
	{

	velt_pid = cPID(vKp, vKi, vKd, vUpMax, vEiMax, vSumEMax, vUdMax, vEMin, vEMax, ts);
	pos_pid  = cPID(pKp, pKi, pKd, pUpMax, pEiMax, pSumEMax, pUdMax, pEMin, pEMax, ts);
	encoder  = cEncoder(gr, num);
  td = cTD(R,H);
	
	}
};
extern cMotor_double_td motor_double_td;

/*�������Ƶ��*/
class cMotor_single
{
	public:

	float real_current;
	float pid_current;

	cPID velt_pid;
	
	cEncoder encoder;
	
	cMotor_single(){}
	cMotor_single(float Kp, float Ki, float Kd, float UpMax, float EiMax, float SumEMax, float UdMax, float EMin, float EMax,
				      float gr, int32_t num)
	{

	velt_pid = cPID(Kp, Ki, Kd, UpMax, EiMax, SumEMax, UdMax, EMin, EMax,0);
	encoder  = cEncoder(gr, num);

	}
};

//��̨�����(PID+TD+ǰ��)
class cMotor_gimbal
{
	public:

	float real_current;
	float pid_current;
	float feed_forward_current;
	
	cPID velt_pid;
	cPID pos_pid;

	cTD td;
	
	cFeedForward g_ff;
	
	cEncoder encoder;
	
	cMotor_gimbal(){}
	cMotor_gimbal(float vKp, float vKi, float vKd, float vUpMax, float vEiMax, float vSumEMax, float vUdMax, float vEMin,float vEMax,
				        float pKp, float pKi, float pKd, float pUpMax, float pEiMax, float pSumEMax, float pUdMax, float pEMin,float pEMax,
				        float Kf_sin, float Kf_cos, float FFangle, float ffUMax, float gr, int32_t num,
								float R, float H, float ts)
	{
	
	velt_pid = cPID(vKp, vKi, vKd, vUpMax, vEiMax, vSumEMax, vUdMax, vEMin,vEMax, ts);
	pos_pid  = cPID(pKp, pKi, pKd, pUpMax, pEiMax, pSumEMax, pUdMax, pEMin,pEMax, ts);
	encoder  = cEncoder(gr, num);    //gr���ٱȣ�num����
	td = cTD(R,H);
	g_ff = cFeedForward(Kf_sin, Kf_cos, FFangle, ffUMax);
	
	}
};


//��̩
class HT_Cal
{
	public:
		float des_p;
		float des_v;
		float pKp;
		float vKd;
		float ff;
	
	HT_Cal(){}
	HT_Cal(float d_p, float d_v, float Kp, float Kd, float f)
	{
		this -> des_p = d_p;
		this -> des_v = d_v;
		this -> pKp = Kp;
		this -> vKd = Kd;
		this -> ff = f;
	}
};

class cEncoder_HT
{
	public:
		float pos;
		float velt;
		float current;
};

class cMotor_HT_gimbal
{
	public:

	u8 HT_cmd;
	
	
	HT_Cal HT_control;
	cEncoder_HT encoder_HT;
	cTD td;
	cMotor_HT_gimbal(){}
	cMotor_HT_gimbal(float pos, float velt, float pKp, float vKd, float ff, float gr, float R, float H)
	{
		HT_control = HT_Cal(pos, velt, pKp, vKd, ff);
		td = cTD(R,H);
	}
	
	void pitch_to_angle();
	
};





//겿ص��
extern void multi_position_command(CAN_TypeDef* CANx, uint8_t motorId, int32_t  angleControl);
extern void zero_position_command(CAN_TypeDef* CANx, uint8_t motorId);
extern void get_position_command(CAN_TypeDef* CANx, uint8_t motorId);
extern void run_command(CAN_TypeDef* CANx, uint8_t motorId);
extern void clear_command(CAN_TypeDef* CANx, uint8_t motorId);

extern int64_t get_position(CanRxMsg* rx_message);


//3508
extern int32_t Get_Encoder_Number(CanRxMsg* rx_message);
extern int32_t Get_Speed(CanRxMsg* rx_message);
extern void can_send_data(CAN_TypeDef* CANx, uint32_t StdID, int16_t ssMotor1, int16_t ssMotor2, 
															int16_t ssMotor3, int16_t ssMotor4);
//��̩���
extern float Get_HT_04_Position(CanRxMsg* rx_message,bool type);
extern float Get_HT_04_Speed(CanRxMsg* rx_message,bool type);
extern int16_t Get_HT_04_Current(CanRxMsg* rx_message);
extern void CanComm_SendControlPara(CAN_TypeDef* CANx, float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t StdId,bool type);
extern void CanComm_ControlCmd(CAN_TypeDef* CANx, uint8_t cmd,uint32_t StdId);



#endif
