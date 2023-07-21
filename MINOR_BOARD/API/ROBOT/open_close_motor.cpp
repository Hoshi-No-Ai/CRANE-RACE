#include "open_close_motor.h"
#include "uart_protocol.h"

cOpencloseMotor Openclose_motor
{
	//开合电机
	/*velt_pid*/			OC_VELT_KP, OC_VELT_KI, OC_VELT_KD, OC_VELT_UMax, 0, OC_VELT_SUMEMAX, 0, OC_VELT_EMIN, OC_VELT_EMAX, 
	/*pos_pid*/				OC_POS_KP, OC_POS_KI, OC_POS_KD, OC_POS_UMax, 0, OC_POS_SUMEMAX, 0, OC_POS_EMIN,  OC_POS_EMAX,
	/*encoder*/				OC_GEAR_RATIO, 8191,
	/*td*/						OC_R, OC_H,
	/*ts*/            0.001f
};

void cOpencloseMotor::openclose_to_aim()
{
	OC_motor.td.aim = ClipFloat(DES.OC, -30,0);    //限位
	OC_motor.td.TD_Function();
	OC_motor.pos_pid.fpDes =OC_motor.td.x1;
	OC_motor.pos_pid.CalComprehensivePID();
	OC_motor.velt_pid.fpDes = OC_motor.pos_pid.fpU + OC_motor.td.x2;
	OC_motor.velt_pid.CalComprehensivePID();
	OC_motor.pid_current = OC_motor.velt_pid.fpU;

	OC_motor.real_current = OC_motor.pid_current;
	
	
	
}
