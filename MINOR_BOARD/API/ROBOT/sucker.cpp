#include "sucker.h"


DesSet DES_qzj;
cSucker sucker;

void cSucker::slide_to_aim(void)
{
//	slide_motor.td.aim = ClipFloat(DES.SLIDE,0,146);
//	slide_motor.td.TD_Function();
//	slide_motor.pos_pid.fpDes = slide_motor.td.aim;
	td_slide.aim = DES_qzj.sucker_slide;
	td_slide.TD_Function();
	slide_motor.pos_pid.fpDes = ClipFloat(td_slide.x1,-50,0);
	slide_motor.pos_pid.CalComprehensivePID();
	slide_motor.velt_pid.fpDes = slide_motor.pos_pid.fpU;
	slide_motor.velt_pid.CalComprehensivePID();
	slide_motor.pid_current = slide_motor.velt_pid.fpU;
	
}

void cSucker::lift_to_aim(void)
{
	td_lift.aim = DES_qzj.sucker_lift;
	td_lift.TD_Function();
	lift_motor.pos_pid.fpDes = ClipFloat(td_lift.x1,0,1000);
	lift_motor.pos_pid.CalComprehensivePID();
	lift_motor.velt_pid.fpDes = lift_motor.pos_pid.fpU;
	lift_motor.velt_pid.CalComprehensivePID();
	lift_motor.pid_current = lift_motor.velt_pid.fpU;
}