#include "action_task.h"
#include "hitcrt_os.h"

action_pattern_e action_pattern = ACTION_NONE;
fetch_pattern_e fetch_pattern = FETCH_INIT;
static int pos_i;

void robot_movement(void)
{
    static action_pattern_e action_pattern_pre = ACTION_NONE;
    if (action_pattern != action_pattern_pre)
    {
        switch (action_pattern)
        {
        case ACTION_NONE:
            break;
        case ACTION_INIT:
            // 上层机构初始化
            box_state = await;
            break;
        case ACTION_FETCH:
            // 上层抓取
            box_state = get_state1;
            break;
        case ACTION_PUT:
            // 上层放置
            box_state = lose_state1;
            break;
        case ACTION_POS_1:
            // 跑到第一个点位
            nav.auto_path.m_point_end.point_set(POS_1_X, POS_1_Y, POS_1_Q);
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 90, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        case ACTION_POS_2:
            nav.auto_path.m_point_end.point_set(POS_2_X, POS_2_Y, POS_2_Q);
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 90, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        case ACTION_POS_3:
            nav.auto_path.m_point_end.point_set(POS_3_X, POS_3_Y, POS_3_Q);
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 90, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        case ACTION_POS_4:
            nav.auto_path.m_point_end.point_set(POS_4_X, POS_4_Y, POS_4_Q);
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 90, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        case ACTION_POS_5:
            nav.auto_path.m_point_end.point_set(POS_5_X, POS_5_Y, POS_5_Q);
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 90, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        case ACTION_POS_6:
            nav.auto_path.m_point_end.point_set(POS_6_X, POS_6_Y, POS_6_Q);
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 90, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        case ACTION_POS_END:
            nav.auto_path.m_point_end.point_set(POS_END_X, POS_END_Y, POS_END_Q);
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 90, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        default:
            break;
        }
        action_pattern_pre = action_pattern;
    }
}

void movement_check(bool if_auto)
{
    if (if_auto)
    {
        switch (action_pattern)
        {
        case ACTION_INIT:
            // 上层发送初始化成功flag标志位
            if (fetch_pattern == FETCH_AWAIT)
            {
                action_pattern = ACTION_POS_1;
            }
            break;
        case ACTION_FETCH:
            if (fetch_pattern == FETCH_GET && pos_i < 7 && pos_i > 0)
            {
                action_pattern = (action_pattern_e)(pos_i + 1);
            }
            break;
        case ACTION_POS_1:
            // 判断底盘是否跑到点位
            if (pos_i == ACTION_POS_1)
            {
                action_pattern = ACTION_FETCH;
            }
            break;
        case ACTION_POS_2:
            if (pos_i == ACTION_POS_2)
            {
                action_pattern = ACTION_FETCH;
            }
            break;
        case ACTION_POS_3:
            if (pos_i == ACTION_POS_3)
            {
                action_pattern = ACTION_FETCH;
            }
            break;
        case ACTION_POS_4:
            if (pos_i == ACTION_POS_4)
            {
                action_pattern = ACTION_FETCH;
            }
            break;
        case ACTION_POS_5:
            if (pos_i == ACTION_POS_5)
            {
                action_pattern = ACTION_FETCH;
            }
            break;
        case ACTION_POS_6:
            if (pos_i == ACTION_POS_6)
            {
                action_pattern = ACTION_FETCH;
            }
            break;
        case ACTION_POS_END:
            if (pos_i == ACTION_POS_END)
            {
                action_pattern = ACTION_PUT;
            }
            break;
        case ACTION_PUT:
            break;
        default:
            break;
        }
    }
}

void position_check(void)
{
    static C_POINT point_fb;
    point_fb.m_x = cRobot.stPot.fpPosX;
    point_fb.m_y = cRobot.stPot.fpPosY;
    point_fb.m_q = 0.1f * cRobot.stPot.fpPosQ;

    // 如果停止状态变成STOP_X，记得要改这里！
    if (nav.state == NAV_STOP || nav.state == NAV_STOPX)
    {
        if (fabs(point_fb.m_x - POS_1_X) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_1_Y) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_1_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 1;
        }
        else if (fabs(point_fb.m_x - POS_2_X) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_2_Y) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_2_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 2;
        }
        else if (fabs(point_fb.m_x - POS_3_X) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_3_Y) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_3_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 3;
        }
        else if (fabs(point_fb.m_x - POS_4_X) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_4_Y) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_4_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 4;
        }
        else if (fabs(point_fb.m_x - POS_5_X) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_5_Y) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_5_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 5;
        }
        else if (fabs(point_fb.m_x - POS_6_X) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_6_Y) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_6_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 6;
        }
        else if (fabs(point_fb.m_x - POS_END_X) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_END_Y) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_END_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 7;
        }
        else
        {
            pos_i = -1;
        }
    }
    else
    {
        pos_i = -1;
    }
}

GET_NUM target_num;
BOX_STATE box_state, pre_box_state;
float height_box = 220;

float sucker_lift_box_await = 1100, sucker_slide_await = 0;
float sucker_lift_box_get_state1 = 210, sucker_slide_get_state1 = -5;
float sucker_lift_box_get_state2 = 500, sucker_slide_get_state2 = -45;
float table_lift_up = -1500, table_lift_down = -800, talbe_lift_await = -10;
float table_slide_out = -30, table_slide_in = 0;
float table_slide_await = -10;
float sucker_out = 1150;
float sucker_out2 = 1050;

int init_motor;
extern int _servo_degree;
int this_target = 0; // box 1,cola 2
extern float task_time;
void handle_box(void)
{
	OS_ERR err;
	if(fetch_pattern !=FETCH_GET_PRE)
	{
		fetch_pattern=FETCH_MOVE;
	}
     
    switch (box_state)
    {
    case none:
        if (init_motor)
        {
            DES.table_lift = -2100;
            if (fabs(table.lift_motor1.pos_pid.fpFB - DES.table_lift) < 1 && fabs(table.lift_motor2.pos_pid.fpFB - DES.table_lift) < 1)
            {
                DES.table_lift = -10;
                table.lift_motor1.encoder.siSumValue = -6298099;
                table.lift_motor2.encoder.siSumValue = -5797689;
                table.td_lift.m_x1 = -1750;
                table.td_lift.m_x2 = 0;
                init_motor = 0;
            }
        }
        break;
    case await:
        sucker.Toggle_sucker = 1;
        DES.table_slide = table_slide_in;
        DES.table_lift = talbe_lift_await;
        DES.sucker_lift = sucker_lift_box_await;
		
				//可乐高度
				if(fetch_pattern==FETCH_GET_PRE && sucker.lift_motor.pos_pid.fpFB>height_box)
				{
					fetch_pattern=FETCH_GET;
					this_target=0;
				}
		
        if (fabs(DES.sucker_lift - sucker.lift_motor.pos_pid.fpFB) < 5)
        {
            DES.sucker_slide = sucker_slide_await;
            // TODO：调节衔接时间
						fetch_pattern = FETCH_AWAIT;
        }
        break;

    case get_state1:
        sucker.Toggle_sucker = 0;
        // DES.table_lift = table_lift_up;
        DES.table_slide = table_slide_in;

        if (this_target == 1) // box
        {
            DES.sucker_slide = sucker_slide_get_state1;
            DES.sucker_lift = sucker_lift_box_get_state1;
        }
        else if (this_target == 2)
        {
            DES.sucker_slide = 0;
            DES.sucker_lift = -5;
        }

        if (fabs(DES.sucker_lift - sucker.lift_motor.pos_pid.fpFB) < 5)
        {
            if (this_target == 1)
						{
							box_state = get_state2;
						}
            else if (this_target == 2)
						{
							box_state = await;
							fetch_pattern = FETCH_GET_PRE;
						}
        }
        break;

    case get_state2:
        DES.sucker_lift = sucker_lift_box_await;
        if (fabs(DES.sucker_lift - sucker.lift_motor.pos_pid.fpFB) < 5)
        {
            box_state = get_state3;
        }
        break;

    case get_state3:

        DES.sucker_slide = sucker_slide_get_state2;
        if (fabs(DES.sucker_slide - sucker.slide_motor.pos_pid.fpFB) < 5)
        {
            DES.sucker_lift = sucker_lift_box_get_state2 + target_num.box * height_box;
            if (fabs(DES.sucker_lift - sucker.lift_motor.pos_pid.fpFB) < 5)
            {
                sucker.Toggle_sucker = 1;
                // TODO：调节衔接时间
								OSTimeDly_ms(2000);
								box_state = await;
								if(this_target==2)
								{
									fetch_pattern = FETCH_GET;
									this_target=0;
								}            
            }
        }
        break;

    case lose_state0:
        sucker.Toggle_sucker = 1;
        DES.sucker_lift = sucker_out;
        DES.table_lift = table_lift_up;
        if (fabs(DES.table_lift - table.td_lift.m_x1) < 5)
        {
            box_state = lose_state1;
        }
        break;

    case lose_state1:

        DES.table_slide = table_slide_out;
        //  DES.sucker_lift = 1300;
        DES.sucker_slide = 0;
        if (fabs(DES.sucker_slide - sucker.slide_motor.pos_pid.fpFB) < 2)
        {
            if (fabs(DES.table_slide - table.td_slide.m_x1) < 5)
            {
                DES.table_lift = table_lift_down;

                if (fabs(DES.table_lift - table.td_lift.m_x1) < 5)
                {
                    DES.sucker_lift = sucker_out2;

                    if (fabs(DES.sucker_lift - sucker.lift_motor.pos_pid.fpFB) < 5)
                    {

                        if (_servo_degree > 0)
                        {
                            task_time = 0;
                        }
                        _servo_degree = 0;

                        if (task_time > 0.5)
                        {
                            box_state = lose_state2;
                        }
                    }
                }
            }
        }
        break;

    case lose_state2:
        DES.table_lift = table_lift_down;
        DES.sucker_lift = 1300;
        if (fabs(DES.table_lift - table.td_lift.m_x1) < 5)
        {
            DES.table_slide = table_slide_in;
        }
        break;

    default:
        break;
    }

    pre_box_state = box_state;
}