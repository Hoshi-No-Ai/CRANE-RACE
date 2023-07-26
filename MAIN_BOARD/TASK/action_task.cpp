#include "action_task.h"

action_pattern_e action_pattern = ACTION_INIT;

void robot_movement(void)
{
    static action_pattern_e action_pattern_pre = ACTION_INIT;
    if (action_pattern != action_pattern_pre)
    {
        switch (action_pattern)
        {
        case ACTION_INIT:
            // 上层机构初始化

            break;
        case ACTION_POS_1:
            // 跑到第一个点位

            break;
        case ACTION_POS_2:
            // 跑到第二个点位
            break;
        case ACTION_POS_3:
            // 跑到第三个点位
            break;
        case ACTION_POS_4:
            // 跑到第四个点位
            break;
        case ACTION_POS_5:
            // 跑到第五个点位
            break;
        case ACTION_POS_6:
            // 跑到第六个点位
            break;
        case ACTION_POS_END:
            // 跑到终点
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
            //  if (/* condition */)
            //  {
            //      action_pattern=ACTION_POS_1;
            //  }
            break;
        case ACTIO_FETCH:
            // 上层发送抓取成功flag标志位
            //  if (/* condition */&& action_pattern<7 && action_pattern>0)
            //  {
            //      action_pattern=pos_i+1;
            //  }
            break;
        case ACTION_POS_1:
            // 判断底盘是否跑到点位
            // if (pos_i==ACTION_POS_1)
            // {
            //     action_pattern=ACTIO_FETCH;
            // }
            break;
        case ACTION_POS_2:
            // if (pos_i==ACTION_POS_2)
            // {
            //     action_pattern=ACTIO_FETCH;
            // }
            break;
        case ACTION_POS_3:
            // if (pos_i==ACTION_POS_3)
            // {
            //     action_pattern=ACTIO_FETCH;
            // }
            break;
        case ACTION_POS_4:
            // if (pos_i==ACTION_POS_4)
            // {
            //     action_pattern=ACTIO_FETCH;
            // }
            break;
        case ACTION_POS_5:
            // if (pos_i==ACTION_POS_5)
            // {
            //     action_pattern=ACTIO_FETCH;
            // }
            break;
        case ACTION_POS_6:
            // if (pos_i==ACTION_POS_6)
            // {
            //     action_pattern=ACTIO_FETCH;
            // }
            break;
        case ACTION_POS_END:
            // if (pos_i==ACTION_POS_END)
            // {
            //     action_pattern=ACTIO_FETCH;
            // }
            break;
        case ACTION_PUT:
            break;
        default:
            break;
        }
    }
}

GET_NUM target_num;
BOX_STATE box_state, pre_box_state;
float height_box = 205;
extern cTable table;
#include "table.h"

float sucker_lift_box_await = 1100, sucker_slide_await = 0;
float sucker_lift_box_get_state1 = 200, sucker_slide_get_state1 = -5;
float sucker_lift_box_get_state2 = 500, sucker_slide_get_state2 = -45;
float table_lift_up = -1500, table_lift_down = -800, talbe_lift_await = -10;
float table_slide_out = -30, table_slide_in = 0;
float table_slide_await = -10;

extern int _servo_degree;
int this_target = 0; // box 1,cola 2
void handle_box()
{
    switch (box_state)
    {

    case await:
        sucker.Toggle_sucker = 1;
        DES.table_slide = table_slide_in;
        DES.table_lift = talbe_lift_await;
        DES.sucker_lift = sucker_lift_box_await;
        if (fabs(DES.sucker_lift - sucker.lift_motor.pos_pid.fpFB) < 5)
        {

            DES.sucker_slide = sucker_slide_await;
        }
        break;

    case get_state1:
        sucker.Toggle_sucker = 0;
        // DES.table_lift = table_lift_up;
        DES.table_slide = table_slide_in;
        DES.sucker_slide = sucker_slide_get_state1;
        if (this_target == 1)
            DES.sucker_lift = sucker_lift_box_get_state1;
        else if (this_target == 2)
            DES.sucker_lift = 0;
        if (fabs(DES.sucker_lift - sucker.lift_motor.pos_pid.fpFB) < 5)
        {
            if (this_target == 1)
                box_state = get_state2;
            else if (this_target == 2)
                box_state = await;
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
        if (pre_box_state != box_state)
        {
            target_num.box++;
        }
        DES.sucker_slide = sucker_slide_get_state2;
        if (fabs(DES.sucker_slide - sucker.slide_motor.pos_pid.fpFB) < 5)
        {
            DES.sucker_lift = sucker_lift_box_get_state2 + target_num.box * height_box;
            if (fabs(DES.sucker_lift - sucker.lift_motor.pos_pid.fpFB) < 5)
            {
                sucker.Toggle_sucker = 1;
            }
        }
        break;

    case lose_state0:
        DES.sucker_lift = 1170;
        DES.table_lift = table_lift_up;
        if (fabs(DES.table_lift - table.td_lift.m_x1) < 5)
        {
            box_state = lose_state1;
        }
        break;

    case lose_state1:
        DES.table_slide = table_slide_out;
        DES.sucker_lift = 1170;
        DES.sucker_slide = 0;
        if (fabs(DES.sucker_lift - sucker.lift_motor.pos_pid.fpFB) < 5)
        {
            if (fabs(DES.table_slide - table.td_slide.m_x1) < 5)
            {
                DES.table_lift = table_lift_down;

                if (fabs(DES.table_lift - table.td_lift.m_x1) < 5)
                {
                    _servo_degree = 180;
                    box_state = lose_state2;
                }
            }
        }
        break;

    case lose_state2:
        DES.table_lift = table_lift_down;
        DES.sucker_lift = 1000;
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

void get_cola()
{
}