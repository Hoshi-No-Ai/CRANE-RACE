#include "motor_control_task.h"

flag_and_mode flag_mode;

void select_motor_pattern()
{
    if(flag_mode.motor_mode == SAFE)
    {
      disable_all_motor();
    }
		
    if(flag_mode.motor_mode == GIMBAL_YAW)
    {
			Gimbal_motor.yaw_motor.td.aim = DES.YAW;
			Gimbal_motor.yaw_motor.pos_pid.fpDes=ClipFloat(DES.YAW, -110 + YAW_ZERO, 110 + YAW_ZERO);
//      Gimbal_motor.yaw_to_aim();
      can_send_data(CAN1,0x200,0,0,0,0);
			can_send_data(CAN1,0x1FF,Gimbal_motor.yaw_motor.pos_pid.fpDes*Gimbal_motor.yaw_motor.encoder.siGearRatio,view_r.fl[2],0,0);
//      can_send_data(CAN1,0x1FF,Gimbal_motor.yaw_motor.real_current,view_r.fl[2],0,0);
			can_send_data(CAN2, 0x200, 0, 0, 0, 0);
	    can_send_data(CAN2, 0x1FF, 0, 0, 0, 0);
    }
		
    if(flag_mode.motor_mode == GIMBAL_PITCH)
    {
      Gimbal_motor.pitch_motor.td.aim = DES.PITCH;
			Gimbal_motor.pitch_to_aim();
			can_send_data(CAN1,0x200,0,0,0,0);
      can_send_data(CAN1,0x1FF,0,0,0,0);
			can_send_data(CAN2, 0x200, 0, 0, 0, 0);
	    can_send_data(CAN2, 0x1FF, 0, 0, 0, 0);
//			if(Gimbal_motor.pitch_motor.HT_cmd)
//			{
//				CanComm_ControlCmd(CAN1, Gimbal_motor.pitch_motor.HT_cmd);
//				Gimbal_motor.pitch_motor.HT_cmd = 0;
//			}
//			CanComm_SendControlPara(CAN1, Gimbal_motor.pitch_motor.HT_control.des_p * (float)PI / 180, 
//																		Gimbal_motor.pitch_motor.HT_control.des_v  * (float)PI / 180, 
//																		Gimbal_motor.pitch_motor.HT_control.pKp, 
//																		Gimbal_motor.pitch_motor.HT_control.vKd, 
//																		Gimbal_motor.pitch_motor.HT_control.ff);
		}
		
		if(flag_mode.motor_mode == MOTOR2SHOOT)
    {
			fetch_ball_motor.fetch_right_put.pos_pid.fpDes = -DES.PUT;
			fetch_ball_motor.fetch_left_put.pos_pid.fpDes = DES.PUT;
      fetch_ball_motor.enable_put();
			can_send_data(CAN1,0x200,0,fetch_ball_motor.fetch_left_put.real_current,0,0);
      can_send_data(CAN2,0x200,0,fetch_ball_motor.fetch_right_put.real_current,0,0);
    }
		
    if(flag_mode.motor_mode == MOTOR2SEND)
    {
			fetch_ball_motor.fetch_left_send.pos_pid.fpDes = DES.SEND;
			fetch_ball_motor.fetch_right_send.pos_pid.fpDes = -DES.SEND;
      fetch_ball_motor.enable_send();
      can_send_data(CAN2,0x200,0,0,fetch_ball_motor.fetch_left_send.real_current,0);
      can_send_data(CAN2,0x1FF,0,0,fetch_ball_motor.fetch_right_send.real_current,0);
    }
    
    if(flag_mode.motor_mode == COMPREHENSIVE)
    { 			
//			if(Gimbal_motor.pitch_motor.HT_cmd)
//			{
//				CanComm_ControlCmd(CAN1, Gimbal_motor.pitch_motor.HT_cmd);
//				Gimbal_motor.pitch_motor.HT_cmd = 0;
//			}
			friction_belt.cal_velt_differ(friction_belt.friction_belt_left.v_fb, friction_belt.friction_belt_right.v_fb, &friction_belt.average_differ);
//			Gimbal_motor.yaw_to_aim();
			Gimbal_motor.yaw_motor.pos_pid.fpDes=ClipFloat(DES.YAW, -110 + YAW_ZERO, 110 + YAW_ZERO);
			Gimbal_motor.pitch_to_aim();
			friction_belt.friction_to_aim();
			fetch_ball_motor.enable_put();
			fetch_ball_motor.enable_send();
			fetch_ball_motor.enable_deal();
			fetch_ball_motor.enable_cut();
			
			CanComm_SendControlPara(CAN1, PITCH_HT_GEAR_RATIO * Gimbal_motor.pitch_motor.HT_control.des_p * (float)PI / 180, 
																		PITCH_HT_GEAR_RATIO * Gimbal_motor.pitch_motor.HT_control.des_v  * (float)PI / 180, 
																		Gimbal_motor.pitch_motor.HT_control.pKp, 
																		Gimbal_motor.pitch_motor.HT_control.vKd, 
																		Gimbal_motor.pitch_motor.HT_control.ff);
      can_send_data(CAN1,0x200,fetch_ball_motor.deal_motor.real_current,fetch_ball_motor.fetch_right_put.real_current,fetch_ball_motor.fetch_right_send.real_current,0);
//      can_send_data(CAN1,0x200,0,fetch_ball_motor.fetch_right_put.real_current,fetch_ball_motor.fetch_right_send.real_current,0);
			can_send_data(CAN1,0x1FF,0,0,0,0);												
			can_send_data(CAN2,0x200,0,fetch_ball_motor.fetch_left_put.real_current,fetch_ball_motor.fetch_left_send.real_current,fetch_ball_motor.cut_motor.real_current);
      can_send_data(CAN2,0x1FF,0,0,0,0);
		}
}

void init_all_motor(void)
{
    //给所有电机设定初始位置（上电后恢复的状态，绝对式编码器存初始值）
	if(system_state == SYS_RUN)
	{ 
		friction_belt.friction_loop_flag = 1;
		friction_belt.friction_belt_left.loop_flag = 1;
		friction_belt.friction_belt_right.loop_flag = 1;
		Gimbal_motor.pitch_motor.HT_cmd = 0x01;
		if(Gimbal_motor.pitch_motor.HT_cmd)
			{
				CanComm_ControlCmd(CAN1, Gimbal_motor.pitch_motor.HT_cmd);
				Gimbal_motor.pitch_motor.HT_cmd = 0;
			}
		flag_mode.motor_mode = COMPREHENSIVE;
	}
    //计算电流
    //发电流
}
