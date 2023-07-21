/**********************************************************************************************************************************************************
��Ȩ������HITCRT(�����󾺼�������Э��)
�ļ�����PID_Algorithm.c
����޸����ڣ�2022.01.02
�汾��1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
ģ��������
�����б�
----------------------------------------------------------------------------------------------------------------------------------------------------------
�޶���¼��
	 ����        	ʱ��            �汾     	˵��
pyx & Chris    2016.10.13      	1.0      ���ֽ�����ģ��
    wyy        2022.01.02       1.1      ���ƴ���ṹ����PID��������
**********************************************************************************************************************************************************/

#include "pid_algorithm.h"
#include "filter_algorithm.h"
#include "friction_belt_motor.h"
/*����Ϊλ����PID*/
/*******************************************************************
�������ƣ�CalPID(ST_PID *this)
�������ܣ���ͨ��PID�㷨����PID��
��    ע��
********************************************************************/
void cPID::CalPID(void)
{
  this->fpE = this->fpDes - this->fpFB;//���㵱ǰƫ��
	if(fabs(this->fpE) <= this->fpEMin)//ƫ����������
	{
	    this->fpE = 0;
	}
	this->fpSumE += this->fpE;
	/*λ��ʽPID���㹫ʽ*/
	this->fpU = this->fpKp * this->fpE 
	            + this->fpKi * this->fpSumE 
				+ this->fpKd * (this->fpE - this->fpPreE)/0.002f;
	this->fpPreE = this->fpE;//���汾��ƫ��
  /*PID��������޷�*/
  this->fpU = ClipFloat(this->fpU, -this->fpUMax, this->fpUMax);
}

/*******************************************************************
�������ƣ�CalISeparatedPID(ST_PID *this)
�������ܣ����ַ���ʽPID�㷨����PID��
��    ע�����ַ���ʽPID�Ľ��㷨�ɼ�С������ֹͣ����������ʱ�ϴ�ƫ��
          �Ի�����Ļ��ۣ��Ӷ�������ֽϴ�ĳ�����������
********************************************************************/
void cPID::CalISeparatedPID(void)
{   
	uint8_t uck=1;
  if(this->fpPreKi != this->fpKi )
	{
		this->fpSumE = 0;
	}
	this->fpE=this->fpDes-this->fpFB;//���㵱ǰƫ��
 	if(fabs(this->fpE) <= this->fpEMin)//ƫ����������
	{
		this->fpE = 0;
	}
	
	/*��ƫ������������ۻ�ƫ��*/
	if(fabs(this->fpE) < this->fpEMax )//�ж��Ƿ�������ַ���
	{
//		uck=0;
  this->fpSumE += this->fpE;//����ƫ���ۻ� 		
	}
	/*�����޷�*/
	if(this->fpSumE > this->fpSumEMax)
	{
		this->fpSumE=this->fpSumEMax ;
	}
	if(this->fpSumE<-this->fpSumEMax )
	{
		this->fpSumE =-this->fpSumEMax;
	}
	
	
//	friction_left_lpf.in = friction_belt_motor.friction_belt_left.velt_pid.fpU+feedforward ;
//	friction_right_lpf.in=friction_belt_motor.friction_belt_right.velt_pid.fpU+feedforward ;
//	LpFilter(&friction_left_lpf);
//	LpFilter(&friction_right_lpf);
//	current_A = friction_left_lpf.out;
//	current_B= friction_right_lpf.out;
	
	
	/*�����޷�*/
	this->fpUi = Clip(this->fpKi * this->fpSumE, -this->fpEiMax, this->fpEiMax);
	this->fpUp = Clip(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
	this->fpUd = Clip(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);

	/*λ��ʽPID���㹫ʽ*/
	this->fpU = this->fpUp + this->fpUi * uck - this->fpUd ;
	this->fpPreE = this->fpE;//���汾��ƫ��
  /*PID��������޷�*/
  this->fpU = ClipFloat(this->fpU, -this->fpUMax, this->fpUMax);
	
	this->fpPreKi=this->fpKi;
}

/*******************************************************************
�������ƣ�CalIResistedPID(ST_PID *this)
�������ܣ������ֱ���PID�㷨
��    ע��ϵͳ��һ�������˶�������ϴ���������ڼ��������ڲ����񵴻򳬵�
********************************************************************/
void cPID::CalIResistedPID(void)
{   

	this->fpE=this->fpDes-this->fpFB;   //���㵱ǰƫ��
	this->fpSumE += this->fpE;   //����ƫ���ۻ�
	
	this->fpSumE = ClipFloat(this->fpSumE, -this->fpEiMax, this->fpEiMax);
	this->fpUi = this->fpKi * this->fpSumE;
	
	this->fpUp = ClipFloat(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
	this->fpUd = ClipFloat(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);

	
	/*��ƫ��������֮�ڣ�����������ۼ���*/
	if(fabs(this->fpE) < this->fpEMin)   //�ж��Ƿ�������ֱ�������
	{
		this->fpSumE = 0;   //���ƫ���ۻ�
	}
	/*λ��ʽPID���㹫ʽ*/
	this->fpU = this->fpUp + this->fpUi + this->fpUd;
	
	this->fpPreE = this->fpE;//���汾��ƫ��
    /*PID��������޷�*/	
	this->fpU = ClipFloat(this->fpU, -this->fpUMax, this->fpUMax);
}
/*******************************************************************
�������ƣ�CalIWeakenPID(ST_PID *this)
�������ܣ�������������PID�Ľ��㷨����PID��
��    ע��
********************************************************************/
void cPID::CalIWeakenPID(void)
{

	this->fpE=this->fpDes-this->fpFB;//���㵱ǰƫ��
	
	if(((this->fpU <= this->fpUMax && this->fpE > 0) || (this->fpU >= -this->fpUMax && this->fpE < 0)) \
		    && fabs(this->fpE) < this->fpEMin)
	{
		this->fpSumE += this->fpE;//����ƫ���ۻ�
	}
	
	this->fpSumE = Clip(this->fpSumE, -this->fpEiMax, this->fpEiMax);
	this->fpUi = this->fpKi * this->fpSumE;
	
	this->fpUp = Clip(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
	this->fpUd = Clip(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);
	
	/*λ��ʽPID���㹫ʽ*/
	this->fpU = this->fpUp + this->fpUi + this->fpUd;
	
	this->fpPreE = this->fpE;//���汾��ƫ��
	
    /*PID��������޷�*/	
	this->fpU = Clip(this->fpU, -this->fpUMax, this->fpUMax);
}

/*******************************************************************
�������ƣ�CalFilterPID(ST_PID *this)
�������ܣ�΢����PID���
��    ע��
********************************************************************/
void cPID::CalFilterPID(void)
{
  //=======���㵱ǰƫ��===========
	this->fpE = this->fpDes - this->fpFB;
	//=======ƫ����������========
	if(fabs(this->fpE) <= this->fpEMin)
	{
		this->fpE = 0.0f;
		this->fpUi = 0;
	}
	/*======λ��ʽPID���㹫ʽ======*/
	this->fpUp = this->fpKp * this->fpE;//���������
	this->fpUi += this->fpKi * this->fpE * this->fpTs;//���������
	//΢���������΢���˲�
	this->fpUd = this->fpKd * (this->fpE - this->fpPreE) / this->fpTs;
  
	//===========�����ϴ�ƫ��===========
	this->fpPreE = this->fpE;
	//===========PID�����===============
	this->fpU = this->fpUp + this->fpUi + this->fpUd;
  /*=========����޷�============*/
  if(this->fpU > this->fpUMax){this->fpU = this->fpUMax;}
	if(this->fpU < -this->fpUMax){this->fpU = -this->fpUMax;}
}
/*******************************************************************
�������ƣ�CalComprehensivePID(ST_PID *this)
�������ܣ��ۺ�PID���
��    ע��Ϊ�������ã��ɲ�������PID�����ۺϣ�����ȫ��
********************************************************************/
u8 kkk = 0;

void cPID::CalComprehensivePID(void)
{ 
	uint8_t uck = 0;
	
	this->fpE = this->fpDes - this->fpFB;//���㵱ǰƫ��
	this->fpSumE += this->fpE;
	/*���ֿ�����*/
	this->fpSumE = ClipFloat(this->fpSumE, -this->fpSumEMax, this->fpSumEMax);
	/*����������޷�*/
	this->fpUp = ClipFloat(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
	this->fpUi = ClipFloat(this->fpKi * this->fpSumE, -this->fpEiMax, this->fpEiMax);
	this->fpUd = ClipFloat(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);
	/*���ַ���*/
	if(fabs(this->fpE) >= this->fpEMax )//�ж��Ƿ�������ַ���
	{
		uck = 0;	
	}
	else
	{
		
	}
		uck = 1;
	/*λ��ʽPID���㹫ʽ*/
	if(this->fpUd > 0.1f)
	{ 
		lpf_PID.in = this->fpUd;
	}
	
	LpFilter(&lpf_PID);
	
	if(fabs(this->fpE) <= this->fpEMin)
	{
	  this->fpU = this->fpUp + kkk * this->fpUi * uck + lpf_PID.out;
	}
	else
	{
		this->fpU = this->fpUp + this->fpUi * uck + lpf_PID.out;
	}

	this->fpPreE = this->fpE;//���汾��ƫ��
  /*PID��������޷�*/
  this->fpU = ClipFloat(this->fpU, -this->fpUMax, this->fpUMax);
}

void cFeedForward::CalGFeedforward(float angle)
{
	this->ffU = -this->Kf_sin * sin(angle/360*3.14f*2)+ this->Kf_cos * cos(angle/360*3.14f*2);
    this->ffU = ClipFloat(this->ffU, -this->ffU_Max, this->ffU_Max);
}


//void TD_Function(TD *ptd)
//{
//	float d, d0, y, a0, a = 0;
//	ptd->x = ptd->x1 - ptd->aim;
//	d = ptd->r * ptd->h;
//	d0 = ptd->h * d;
//	y = ptd->x + ptd->h * ptd->x2;
//	a0 = sqrt(d * d + 8 * ptd->r * fabs(y));

//	if (fabs(y) > d0)
//		a = ptd->x2 + (a0 - d) * Sign_Judge(y) / 2;
//	else
//		a = ptd->x2 + y / ptd->h;

//	if (fabs(a) > d)
//		y = -1 * ptd->r * Sign_Judge(a);
//	else
//		y = -1 * ptd->r * a / d;

//	ptd->x1 += 0.001f * ptd->x2;
//	ptd->x2 += 0.001f * y;
//}


void Clip_TD_Function(TD *pstTd, fp32 lim_x2)
{
	float d, d0, y, a0, a = 0;
	pstTd->x = pstTd->x1 - pstTd->aim;
	d = pstTd->r * pstTd->h;
	d0 = pstTd->h * d;
	y = pstTd->x + pstTd->h * pstTd->x2;
	a0 = sqrt(d * d + 8 * pstTd->r * fabs(y));

	if (fabs(y) > d0)
		a = pstTd->x2 + (a0 - d) * Sign_Judge(y) / 2;
	else
		a = pstTd->x2 + y / pstTd->h;

	if (fabs(a) > d)
		y = -1 * pstTd->r * Sign_Judge(a);
	else
		y = -1 * pstTd->r * a / d;
	lim_x2 = fabs(lim_x2);
	if (pstTd->x2 > lim_x2)
	{
		pstTd->x2 = lim_x2;
	}
	else if (pstTd->x2 < -lim_x2)
	{
		pstTd->x2 = -lim_x2;
	}
	pstTd->x1 += pstTd->T * pstTd->x2;
	pstTd->x2 += pstTd->T * y;
	pstTd->x3 = y;
}



