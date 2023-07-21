#include "table.h"

cTable table;
extern DesSet DES;
void cTable::slide_to_aim(void)
{
	//	slide_motor.td.aim = ClipFloat(DES.SLIDE,0,146);
	//	slide_motor.td.TD_Function();
	//	slide_motor.pos_pid.fpDes = slide_motor.td.aim;
	td_slide.m_aim = DES.table_slide;
	td_slide.TD_Function();

	slide_motor1.pos_pid.fpDes = ClipFloat(td_slide.m_x1, -36, 0);
	slide_motor1.pos_pid.CalComprehensivePID();
	slide_motor1.velt_pid.fpDes = slide_motor1.pos_pid.fpU;
	slide_motor1.velt_pid.CalComprehensivePID();
	slide_motor1.pid_current = slide_motor1.velt_pid.fpU;

	slide_motor2.pos_pid.fpDes = ClipFloat(-td_slide.m_x1, 0, 36);
	slide_motor2.pos_pid.CalComprehensivePID();
	slide_motor2.velt_pid.fpDes = slide_motor2.pos_pid.fpU;
	slide_motor2.velt_pid.CalComprehensivePID();
	slide_motor2.pid_current = slide_motor2.velt_pid.fpU;
}

void cTable::lift_to_aim(void)
{

	td_lift.m_aim = DES.table_lift;
	td_lift.TD_Function();

	lift_motor1.pos_pid.fpDes = ClipFloat(td_lift.m_x1, -2000, -100);
	lift_motor1.pos_pid.CalComprehensivePID();
	lift_motor1.velt_pid.fpDes = lift_motor1.pos_pid.fpU;
	lift_motor1.velt_pid.CalComprehensivePID();
	lift_motor1.pid_current = lift_motor1.velt_pid.fpU;

	lift_motor2.pos_pid.fpDes = ClipFloat(td_lift.m_x1, -2000, -100);
	lift_motor2.pos_pid.CalComprehensivePID();
	lift_motor2.velt_pid.fpDes = lift_motor2.pos_pid.fpU;
	lift_motor2.velt_pid.CalComprehensivePID();
	lift_motor2.pid_current = lift_motor2.velt_pid.fpU;
}