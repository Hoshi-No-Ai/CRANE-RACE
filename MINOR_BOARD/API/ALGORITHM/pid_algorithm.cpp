/**********************************************************************************************************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：PID_Algorithm.c
最近修改日期：2022.01.02
版本：1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
模块描述：
函数列表：
----------------------------------------------------------------------------------------------------------------------------------------------------------
修订记录：
	 作者        	时间            版本     	说明
pyx & Chris    2016.10.13      	1.0      划分建立此模块
    wyy        2022.01.02       1.1      完善代码结构增加PID计算类型
**********************************************************************************************************************************************************/

#include "pid_algorithm.h"
#include "filter_algorithm.h"
#include "friction_belt_motor.h"
/*以下为位置型PID*/
/*******************************************************************
函数名称：CalPID(ST_PID *this)
函数功能：普通的PID算法计算PID量
备    注：
********************************************************************/
void cPID::CalPID(void)
{
  this->fpE = this->fpDes - this->fpFB;//计算当前偏差
	if(fabs(this->fpE) <= this->fpEMin)//偏差死区限制
	{
	    this->fpE = 0;
	}
	this->fpSumE += this->fpE;
	/*位置式PID计算公式*/
	this->fpU = this->fpKp * this->fpE 
	            + this->fpKi * this->fpSumE 
				+ this->fpKd * (this->fpE - this->fpPreE)/0.002f;
	this->fpPreE = this->fpE;//保存本次偏差
  /*PID运算输出限幅*/
  this->fpU = ClipFloat(this->fpU, -this->fpUMax, this->fpUMax);
}

/*******************************************************************
函数名称：CalISeparatedPID(ST_PID *this)
函数功能：积分分离式PID算法计算PID量
备    注：积分分离式PID改进算法可减小启动、停止或大幅度增减时较大偏差
          对积分项的积累，从而避免出现较大的超调及振荡现象。
********************************************************************/
void cPID::CalISeparatedPID(void)
{   
	uint8_t uck=1;
  if(this->fpPreKi != this->fpKi )
	{
		this->fpSumE = 0;
	}
	this->fpE=this->fpDes-this->fpFB;//计算当前偏差
 	if(fabs(this->fpE) <= this->fpEMin)//偏差死区限制
	{
		this->fpE = 0;
	}
	
	/*若偏差过大，则积分项不累积偏差*/
	if(fabs(this->fpE) < this->fpEMax )//判断是否满足积分分离
	{
//		uck=0;
  this->fpSumE += this->fpE;//计算偏差累积 		
	}
	/*积分限幅*/
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
	
	
	/*三项限幅*/
	this->fpUi = Clip(this->fpKi * this->fpSumE, -this->fpEiMax, this->fpEiMax);
	this->fpUp = Clip(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
	this->fpUd = Clip(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);

	/*位置式PID计算公式*/
	this->fpU = this->fpUp + this->fpUi * uck - this->fpUd ;
	this->fpPreE = this->fpE;//保存本次偏差
  /*PID运算输出限幅*/
  this->fpU = ClipFloat(this->fpU, -this->fpUMax, this->fpUMax);
	
	this->fpPreKi=this->fpKi;
}

/*******************************************************************
函数名称：CalIResistedPID(ST_PID *this)
函数功能：抗积分饱和PID算法
备    注：系统往一个方向运动会产生较大积分误差，会在几个周期内产生振荡或超调
********************************************************************/
void cPID::CalIResistedPID(void)
{   

	this->fpE=this->fpDes-this->fpFB;   //计算当前偏差
	this->fpSumE += this->fpE;   //计算偏差累积
	
	this->fpSumE = ClipFloat(this->fpSumE, -this->fpEiMax, this->fpEiMax);
	this->fpUi = this->fpKi * this->fpSumE;
	
	this->fpUp = ClipFloat(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
	this->fpUd = ClipFloat(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);

	
	/*若偏差在死区之内，则清零积分累计项*/
	if(fabs(this->fpE) < this->fpEMin)   //判断是否满足积分饱和条件
	{
		this->fpSumE = 0;   //清除偏差累积
	}
	/*位置式PID计算公式*/
	this->fpU = this->fpUp + this->fpUi + this->fpUd;
	
	this->fpPreE = this->fpE;//保存本次偏差
    /*PID运算输出限幅*/	
	this->fpU = ClipFloat(this->fpU, -this->fpUMax, this->fpUMax);
}
/*******************************************************************
函数名称：CalIWeakenPID(ST_PID *this)
函数功能：遇限削弱积分PID改进算法计算PID量
备    注：
********************************************************************/
void cPID::CalIWeakenPID(void)
{

	this->fpE=this->fpDes-this->fpFB;//计算当前偏差
	
	if(((this->fpU <= this->fpUMax && this->fpE > 0) || (this->fpU >= -this->fpUMax && this->fpE < 0)) \
		    && fabs(this->fpE) < this->fpEMin)
	{
		this->fpSumE += this->fpE;//计算偏差累积
	}
	
	this->fpSumE = Clip(this->fpSumE, -this->fpEiMax, this->fpEiMax);
	this->fpUi = this->fpKi * this->fpSumE;
	
	this->fpUp = Clip(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
	this->fpUd = Clip(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);
	
	/*位置式PID计算公式*/
	this->fpU = this->fpUp + this->fpUi + this->fpUd;
	
	this->fpPreE = this->fpE;//保存本次偏差
	
    /*PID运算输出限幅*/	
	this->fpU = Clip(this->fpU, -this->fpUMax, this->fpUMax);
}

/*******************************************************************
函数名称：CalFilterPID(ST_PID *this)
函数功能：微分项PID输出
备    注：
********************************************************************/
void cPID::CalFilterPID(void)
{
  //=======计算当前偏差===========
	this->fpE = this->fpDes - this->fpFB;
	//=======偏差死区限制========
	if(fabs(this->fpE) <= this->fpEMin)
	{
		this->fpE = 0.0f;
		this->fpUi = 0;
	}
	/*======位置式PID计算公式======*/
	this->fpUp = this->fpKp * this->fpE;//比例项输出
	this->fpUi += this->fpKi * this->fpE * this->fpTs;//积分项输出
	//微分项输出及微分滤波
	this->fpUd = this->fpKd * (this->fpE - this->fpPreE) / this->fpTs;
  
	//===========更新上次偏差===========
	this->fpPreE = this->fpE;
	//===========PID总输出===============
	this->fpU = this->fpUp + this->fpUi + this->fpUd;
  /*=========输出限幅============*/
  if(this->fpU > this->fpUMax){this->fpU = this->fpUMax;}
	if(this->fpU < -this->fpUMax){this->fpU = -this->fpUMax;}
}
/*******************************************************************
函数名称：CalComprehensivePID(ST_PID *this)
函数功能：综合PID输出
备    注：为调试所用，可参照以上PID进行综合，功能全面
********************************************************************/
u8 kkk = 0;

void cPID::CalComprehensivePID(void)
{ 
	uint8_t uck = 0;
	
	this->fpE = this->fpDes - this->fpFB;//计算当前偏差
	this->fpSumE += this->fpE;
	/*积分抗饱和*/
	this->fpSumE = ClipFloat(this->fpSumE, -this->fpSumEMax, this->fpSumEMax);
	/*三个输出项限幅*/
	this->fpUp = ClipFloat(this->fpKp * this->fpE, -this->fpEpMax, this->fpEpMax);
	this->fpUi = ClipFloat(this->fpKi * this->fpSumE, -this->fpEiMax, this->fpEiMax);
	this->fpUd = ClipFloat(this->fpKd * (this->fpE - this->fpPreE), -this->fpEdMax, this->fpEdMax);
	/*积分分离*/
	if(fabs(this->fpE) >= this->fpEMax )//判断是否满足积分分离
	{
		uck = 0;	
	}
	else
	{
		
	}
		uck = 1;
	/*位置式PID计算公式*/
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

	this->fpPreE = this->fpE;//保存本次偏差
  /*PID运算输出限幅*/
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



