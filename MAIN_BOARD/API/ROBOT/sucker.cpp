#include "sucker.h"


DesSet DES;
cSucker sucker;

void cSucker::slide_to_aim(void)
{
//	slide_motor.td.aim = ClipFloat(DES.SLIDE,0,146);
//	slide_motor.td.TD_Function();
//	slide_motor.pos_pid.fpDes = slide_motor.td.aim;
	td_slide.m_aim = DES.sucker_slide;
	td_slide.TD_Function();
	slide_motor.pos_pid.fpDes = ClipFloat(td_slide.m_x1,0,50);
	slide_motor.pos_pid.CalComprehensivePID();
	slide_motor.velt_pid.fpDes = slide_motor.pos_pid.fpU;
	slide_motor.velt_pid.CalComprehensivePID();
	slide_motor.pid_current = slide_motor.velt_pid.fpU;
	
}

void cSucker::lift_to_aim(void)
{
	td_lift.m_aim = DES.sucker_lift;
	td_lift.TD_Function();
	lift_motor.pos_pid.fpDes = ClipFloat(td_lift.m_x1,0,500);
	lift_motor.pos_pid.CalComprehensivePID();
	lift_motor.velt_pid.fpDes = lift_motor.pos_pid.fpU;
	lift_motor.velt_pid.CalComprehensivePID();
	lift_motor.pid_current = lift_motor.velt_pid.fpU;
}