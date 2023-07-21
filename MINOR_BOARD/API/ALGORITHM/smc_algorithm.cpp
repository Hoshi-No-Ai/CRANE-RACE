#include "smc_algorithm.h"
void cSMC::CalSMC(void)
{

	td_smc.TD_Function(); 
	this->aim = td_smc.x1;
	this->x1 = this->aim - this->FB;     //λ��
//	this->x2 = (this->x1 - this->prex1);    
	this->x2 = (this->prex1 - this->x1);
	this->prex1 = this->x1;

	this->s = this->c * this->x1 +this->x2;
	this->fpUi = this->c * this->x2 + this->epsilon * SGN(this->s) +this->q * this->s;//��ⵥ�����ֵ
//this->fpUi =this->epsilon * SGN(this->s) + this->q * this->s - this->c * this->x2 ;
 

	this->lpf_smc.in = this->fpUi;
	LpFilter(&lpf_smc);

	
	this->realfpU = ClipFloat((this->lpf_smc.out),this ->fpUMin,this ->fpUMax);//�����������������ֵ��Χ

 
	
	
}
	

float cSMC::SGN(float a)
{
	if(a>0)
	{
		return 1;
  }
	if(a<0)
	{return -1;
	}
	else
	{return 0;}
}
