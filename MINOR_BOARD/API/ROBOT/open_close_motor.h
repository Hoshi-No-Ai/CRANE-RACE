#ifndef __OPEN_CLOSE_H__
#define __OPEN_CLOSE_H__

#include "motor_all.h"
#include "motor_drive.h"
#include "math_algorithm.h"

#define OC_VELT_KP 50     //500
#define OC_VELT_KI 0
#define OC_VELT_KD 0
#define OC_VELT_UMax 16000
#define OC_VELT_SUMEMAX 0
#define OC_VELT_EMIN 0
#define OC_VELT_EMAX 0

#define OC_POS_KP 20      //6
#define OC_POS_KI 0
#define OC_POS_KD 0
#define OC_POS_UMax 8000
#define OC_POS_SUMEMAX 0
#define OC_POS_EMIN 0
#define OC_POS_EMAX 0

#define OC_GEAR_RATIO 19

#define OC_R 500
#define OC_H 0.001

#define OC_POS_MIN 0
#define OC_POS_MAX 250




class cOpencloseMotor
{
	public:
	cMotor_double_td OC_motor;
	cOpencloseMotor(){}
	cOpencloseMotor(fp32 oc_vKp, fp32 oc_vKi, fp32 oc_vKd, fp32 oc_vUpMax, fp32 oc_vEiMax, fp32 oc_vSumEMax, fp32 oc_vUdMax, fp32 oc_vEMin, fp32 oc_vEMax,
		              fp32 oc_pKp, fp32 oc_pKi, fp32 oc_pKd, fp32 oc_pUpMax, fp32 oc_pEiMax, fp32 oc_pSumEMax, fp32 oc_pUdMax, fp32 oc_pEmin, fp32 oc_pEMax,
		              fp32 oc_gr, int32_t oc_num,
		              fp32 oc_R, fp32 oc_H,fp32 ts)
		{
			OC_motor=cMotor_double_td(oc_vKp, oc_vKi, oc_vKd, oc_vUpMax, oc_vEiMax, oc_vSumEMax, oc_vUdMax, oc_vEMin, oc_vEMax,
									 oc_pKp, oc_pKi, oc_pKd, oc_pUpMax, oc_pEiMax, oc_pSumEMax, oc_pUdMax, oc_pEmin, oc_pEMax,
									 oc_gr, oc_num, oc_R, oc_H, ts);
		}
		void openclose_to_aim();
		
		
		
		
};



extern cOpencloseMotor Openclose_motor;






#endif
