#include "action_task.h"
#include "hitcrt_os.h"
#define ssfdb sucker.slide_motor.pos_pid.fpFB
#define slfdb sucker.lift_motor.pos_pid.fpFB

#define tsfdb table.slide_motor.pos_pid.fpFB
#define tslfdb table.lift_motor.pos_pid.fpFB

using _action_::figure_out_object;
using _action_::flag_fetch_cola;
using _action_::flag_stop_wait;
using _navigation_::vision_enable;
using _navigation_::vision_true;

action_pattern_e action_pattern = ACTION_NONE;
fetch_pattern_e fetch_pattern = FETCH_INIT;
global_pattern_e global_pattern = WITH_GLOBAL;
aruco_pattern_e aruco_pattern = WITH_ARUCO;
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
            // 上层机构初化
            box_state = await;
            break;
        case ACTION_FETCH:
            // 上层抓取
            box_state = get_state1;
            break;
        case ACTION_PUT:
            // 上层放置
            box_state = lose_state0;
            break;
        case ACTION_POS_1:
            // 跑到点位
            this_target = target_global[0];
            nav.auto_path.m_point_end.point_set(POS_1_X, POS_1_Y, POS_1_Q);
            if (this_target == 2)
            {
                delta_des_cola(target_num.cola);
                nav.auto_path.m_point_end.m_x = nav.auto_path.m_point_end.m_x + delta_des_cola_w.delta_x;
                nav.auto_path.m_point_end.m_y = nav.auto_path.m_point_end.m_y + delta_des_cola_w.delta_y;
                nav.auto_path.m_point_end.m_q = nav.auto_path.m_point_end.m_q;
            }
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 60, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        case ACTION_POS_2:
            if (global_pattern == WITH_GLOBAL)
            {
                this_target = target_global[1];
            }
            nav.auto_path.m_point_end.point_set(POS_2_X, POS_2_Y, POS_2_Q);
            if (this_target == 2)
            {
                delta_des_cola(target_num.cola);
                nav.auto_path.m_point_end.m_x = nav.auto_path.m_point_end.m_x + delta_des_cola_w.delta_x;
                nav.auto_path.m_point_end.m_y = nav.auto_path.m_point_end.m_y + delta_des_cola_w.delta_y;
                nav.auto_path.m_point_end.m_q = nav.auto_path.m_point_end.m_q;
            }
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 60, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        case ACTION_POS_3:
            if (global_pattern == WITH_GLOBAL)
            {
                this_target = target_global[2];
            }
            nav.auto_path.m_point_end.point_set(POS_3_X, POS_3_Y, POS_3_Q);
            if (this_target == 2)
            {
                delta_des_cola(target_num.cola);
                nav.auto_path.m_point_end.m_x = nav.auto_path.m_point_end.m_x + delta_des_cola_w.delta_x;
                nav.auto_path.m_point_end.m_y = nav.auto_path.m_point_end.m_y + delta_des_cola_w.delta_y;
                nav.auto_path.m_point_end.m_q = nav.auto_path.m_point_end.m_q;
            }
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 60, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        case ACTION_POS_4:
            if (global_pattern == WITH_GLOBAL)
            {
                this_target = target_global[3];
            }
            nav.auto_path.m_point_end.point_set(POS_4_X, POS_4_Y, POS_4_Q);
            if (this_target == 2)
            {
                delta_des_cola(target_num.cola);
                nav.auto_path.m_point_end.m_x = nav.auto_path.m_point_end.m_x + delta_des_cola_w.delta_x;
                nav.auto_path.m_point_end.m_y = nav.auto_path.m_point_end.m_y + delta_des_cola_w.delta_y;
                nav.auto_path.m_point_end.m_q = nav.auto_path.m_point_end.m_q;
            }
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 60, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        case ACTION_POS_5:
            if (global_pattern == WITH_GLOBAL)
            {
                this_target = target_global[4];
            }
            nav.auto_path.m_point_end.point_set(POS_5_X, POS_5_Y, POS_5_Q);
            if (this_target == 2)
            {
                delta_des_cola(target_num.cola);
                nav.auto_path.m_point_end.m_x = nav.auto_path.m_point_end.m_x + delta_des_cola_w.delta_x;
                nav.auto_path.m_point_end.m_y = nav.auto_path.m_point_end.m_y + delta_des_cola_w.delta_y;
                nav.auto_path.m_point_end.m_q = nav.auto_path.m_point_end.m_q;
            }
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 60, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        case ACTION_POS_6:
            if (global_pattern == WITH_GLOBAL)
            {
                this_target = target_global[5];
            }
            nav.auto_path.m_point_end.point_set(POS_6_X, POS_6_Y, POS_6_Q);
            if (this_target == 2)
            {
                delta_des_cola(target_num.cola);
                nav.auto_path.m_point_end.m_x = nav.auto_path.m_point_end.m_x + delta_des_cola_w.delta_x;
                nav.auto_path.m_point_end.m_y = nav.auto_path.m_point_end.m_y + delta_des_cola_w.delta_y;
                nav.auto_path.m_point_end.m_q = nav.auto_path.m_point_end.m_q;
            }
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 60, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        case ACTION_POS_END:
            nav.auto_path.m_point_end.point_set(POS_END_X, POS_END_Y, POS_END_Q);
            nav.auto_path.m_velt_acc.Velt_Acc_Set(1500, 60, 1500, 1500);
            SET_NAV_PATH_AUTO(1);
            break;
        case ACTION_POS_CHECK:
            break;
        case ACTION_POS_CHANGE:
            // 取可乐后改变坐标
            nav.auto_path.m_point_end.m_x = nav.auto_path.pos_pid.x.fpFB + delta_fb_des.delta_x;
            nav.auto_path.m_point_end.m_y = nav.auto_path.pos_pid.y.fpFB + delta_fb_des.delta_y;
            nav.auto_path.m_point_end.m_q = 0.1f * cRobot.stPot.fpPosQ + aruco_fdb.thetaz;
            if (this_target == 2)
            {
                delta_des_cola(target_num.cola);
                nav.auto_path.m_point_end.m_x = nav.auto_path.m_point_end.m_x + delta_des_cola_w.delta_x;
                nav.auto_path.m_point_end.m_y = nav.auto_path.m_point_end.m_y + delta_des_cola_w.delta_y;
                nav.auto_path.m_point_end.m_q = nav.auto_path.m_point_end.m_q;
            }
            //            nav.auto_path.m_velt_acc.Velt_Acc_Set(500, 60, 500, 500);
            //            SET_NAV_PATH_AUTO(1);
            flag_fetch_cola = 1;
            break;
        default:
            break;
        }
        action_pattern_pre = action_pattern;
    }
}

int test_enable = 0;
int change_time = 0;
int stable_time = 0;

void movement_check(bool if_auto)
{
    static C_POINT point_fb;
    point_fb.m_x = cRobot.stPot.fpPosX;
    point_fb.m_y = cRobot.stPot.fpPosY;
    point_fb.m_q = 0.1f * cRobot.stPot.fpPosQ;
    if (if_auto)
    {
        switch (action_pattern)
        {
        case ACTION_NONE:
            if (!Detect_Object(temp_data, target_global))
            {
                global_pattern = WITHOUT_GLOBAL;
            }
            else
            {
                global_pattern = WITH_GLOBAL;
            }
            break;
        case ACTION_INIT:
            // 上层发送初始化成功flag标志
            action_pattern = ACTION_POS_1;
            break;
        case ACTION_FETCH:
            if (fetch_pattern == FETCH_GET && pos_i < 7 && pos_i > 0)
            {
                action_pattern = (action_pattern_e)(pos_i + 1);
            }
            break;
        case ACTION_POS_1:
            // 判断底盘是否跑到点
            if (pos_i == ACTION_POS_1)
            {
                action_pattern = ACTION_POS_CHECK;
            }
            break;
        case ACTION_POS_2:
            if (pos_i == ACTION_POS_2)
            {
                action_pattern = ACTION_POS_CHECK;
            }
            break;
        case ACTION_POS_3:
            if (pos_i == ACTION_POS_3)
            {
                action_pattern = ACTION_POS_CHECK;
            }
            break;
        case ACTION_POS_4:
            if (pos_i == ACTION_POS_4)
            {
                action_pattern = ACTION_POS_CHECK;
            }
            break;
        case ACTION_POS_5:
            if (pos_i == ACTION_POS_5)
            {
                action_pattern = ACTION_POS_CHECK;
            }
            break;
        case ACTION_POS_6:
            if (pos_i == ACTION_POS_6)
            {
                action_pattern = ACTION_POS_CHECK;
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
        case ACTION_POS_CHECK:
            // TODO:激光传感器给出识别到的信号
            if (aruco_pattern == WITHOUT_ARUCO && global_pattern == WITH_GLOBAL)
            {
                action_pattern = ACTION_FETCH;
                stable_time = 0;
            }
            else
            {
                stable_time++;
                if (stable_time > 100)
                {
                    vision_true = 1;
                    figure_out_object = 1;
                    if (aruco_pattern == WITH_ARUCO)
                    {
                        vision_true = des_base_aruco(aruco_fdb);
                    }
                    if (global_pattern == WITHOUT_GLOBAL)
                    {
                        figure_out_object = Identify_box_cola(this_target);
                    }

                    if (figure_out_object && vision_true)
                    {
                        test_enable = 0;
                        if (this_target == 1)
                        {
                            action_pattern = ACTION_POS_CHANGE;
                            figure_out_object = 0;
                            vision_true = 0;
                        }
                        else if (this_target == 2)
                        {
                            action_pattern = ACTION_POS_CHANGE;
                            figure_out_object = 0;
                            vision_true = 0;
                        }
                        stable_time = 0;
                    }
                }
            }

            break;
        case ACTION_POS_CHANGE:
            if (fabs(nav.auto_path.pos_pid.x.fpDes - nav.auto_path.pos_pid.x.fpFB) < LIMIT_DELTA_X && fabs(nav.auto_path.pos_pid.y.fpDes - nav.auto_path.pos_pid.y.fpFB) < LIMIT_DELTA_Y && fabs(nav.auto_path.pos_pid.w.fpDes - nav.auto_path.pos_pid.w.fpFB) < LIMIT_DELTA_Q_RAD)
            {
                change_time++;
                if (change_time > 50)
                {
                    action_pattern = ACTION_FETCH;
                    change_time = 0;
                }
            }
            break;
        default:
            break;
        }
    }
}

void position_check(void)
{
    static C_POINT point_fb;
    float dx, dy;
    point_fb.m_x = cRobot.stPot.fpPosX;
    point_fb.m_y = cRobot.stPot.fpPosY;
    point_fb.m_q = 0.1f * cRobot.stPot.fpPosQ;

    if (nav.state == NAV_STOP || nav.state == NAV_STOPX)
    {
        if (this_target == 1)
        {
            dx = 0;
            dy = 0;
        }
        else if (this_target == 2)
        {
            dx = delta_des_cola_w.delta_x;
            dy = delta_des_cola_w.delta_y;
        }

        if (fabs(point_fb.m_x - POS_1_X - dx) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_1_Y - dy) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_1_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 1;
        }
        else if (fabs(point_fb.m_x - POS_2_X - dx) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_2_Y - dy) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_2_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 2;
        }
        else if (fabs(point_fb.m_x - POS_3_X - dx) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_3_Y - dy) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_3_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 3;
        }
        else if (fabs(point_fb.m_x - POS_4_X - dx) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_4_Y - dy) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_4_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 4;
        }
        else if (fabs(point_fb.m_x - POS_5_X - dx) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_5_Y - dy) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_5_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 5;
        }
        else if (fabs(point_fb.m_x - POS_6_X - dx) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_6_Y - dy) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_6_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 6;
        }
        else if (fabs(point_fb.m_x - POS_END_X) < LIMIT_DELTA_X && fabs(point_fb.m_y - POS_END_Y) < LIMIT_DELTA_Y && fabs(point_fb.m_q - POS_END_Q) < LIMIT_DELTA_Q)
        {
            pos_i = 7;
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
float sucker_lift_box_get_state1 = 220, sucker_slide_get_state1 = -7;
float sucker_lift_box_get_state2 = 570, sucker_slide_get_state2 = -45;
float table_lift_up = -1500, table_lift_down = -800, talbe_lift_await = -700;
float table_slide_out = -30, table_slide_in = 0;
float table_slide_await = -10;
float sucker_out = 1150;
float sucker_out2 = 700; // 1050
float sucker_yajin = 990;
int init_motor;
int this_target = 0; // box 1,cola 2
int target_global[6] = {2, 1, 1, 2, 1, 2};

int cola_finish = 0, box_finish = 0;
float cal_distance_by_sensor;

int detect_box = 0;
float cal_sssssssss;

uint8_t omtor_mode1 = 0;
float pre_motor_sucker;
float velt_sucker;
uint8_t final_target;
extern float sucker_lift_r;
extern float sucker_slide_r;
extern float handle_time;
float s_time_down;

float cal_time(float x_des, float x_fdb, float a)
{
    return sqrt(fabs(x_des - x_fdb) / a);
}
void handle_box(void)
{
    OS_ERR err;
    if (fetch_pattern != FETCH_GET_PRE)
    {
        fetch_pattern = FETCH_MOVE;
    }
    switch (box_state)
    {
    case none:
        sucker.Toggle_sucker = 1;
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
        pre_box_state = none;
        break;

    case await:
        sucker.Toggle_sucker = 1;
        DES.table_slide = table_slide_in;
        DES.table_lift = talbe_lift_await;
        //   DES.sucker_lift = sucker_lift_box_await;
        if (!(box_finish * cola_finish))
        {
            sucker_lift_r = 3000;
            sucker_slide_r = 50;
            DES.sucker_slide = 0;

            if (fabs(sucker.slide_motor.pos_pid.fpFB - DES.sucker_slide) < 5)
            {
                DES.sucker_lift = 400;
            }
            else
            {
                if (!box_finish)
                {
                    DES.sucker_lift = sucker_lift_box_get_state2 + (target_num.box - 1) * height_box + 200;
                }
                else
                {
                    DES.sucker_lift = sucker_out;
                }
            }
        }
        if (box_finish * cola_finish)
        {
            sucker_lift_r = 3000;
            sucker_slide_r = 200;
            if (box_finish * cola_finish && sucker.lift_motor.pos_pid.fpFB < 700 && !final_target)
            { // the last target is cola
                final_target = 2;
            }
            else if (box_finish * cola_finish && sucker.lift_motor.pos_pid.fpFB > 700 && !final_target)
            {
                final_target = 1;
            }
            if (final_target == 2)
            {
                DES.sucker_lift = 1180;
                if (fabs(sucker.lift_motor.pos_pid.fpFB - DES.sucker_lift) < 5)
                {
                    DES.sucker_slide = sucker_slide_get_state2;
                    if (fabs(sucker.slide_motor.pos_pid.fpFB - DES.sucker_slide) < 5)
                    {
                        box_state = zhengli;
                    }
                }
            }
            else if (final_target == 1)
            {
                box_state = zhengli;
            }
        }

        if (fetch_pattern == FETCH_GET_PRE && sucker.lift_motor.pos_pid.fpFB > height_box && this_target == 2)
        {
            fetch_pattern = FETCH_GET;
            this_target = 0;
            target_num.cola++;
            if (target_num.cola > 3)
            {
                target_num.cola = 1;
                cola_finish = 1;
            }
        }
        else if (this_target == 1 && fetch_pattern == FETCH_GET_PRE)
        {
            fetch_pattern = FETCH_GET;
            this_target = 0;
            target_num.box++;
            if (target_num.box > 3)
            {
                target_num.box = 1;
                box_finish = 1;
            }
        }

        pre_box_state = await;
        break;

    case get_state1:
        sucker_lift_r = 3000;
        sucker_slide_r = 200;
        sucker.Toggle_sucker = 0;
        // DES.table_lift = table_lift_up;
        DES.table_slide = table_slide_in;
        if (box_state != pre_box_state)
        {
            if (this_target == 1) // box
            {
                if (sucker.lift_motor.pos_pid.fpFB > 300)
                {
                    DES.sucker_slide = cal_distance_by_sensor;

                    //  cal_sssssssss = cal_distance_by_sensor;
                }
                DES.sucker_lift = sucker_lift_box_get_state1;
            }
            else if (this_target == 2)
            {
                DES.sucker_slide = 0;
                DES.sucker_lift = -5;
            }
            s_time_down = cal_time(DES.sucker_lift, slfdb, sucker.td_lift.m_r);
            handle_time = 0;
        }

        if (handle_time > s_time_down)
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
        pre_box_state = get_state1;
        break;
    case get_state2:
        DES.sucker_lift = sucker_lift_box_get_state2 + (target_num.box - 1) * height_box + 200;
        if (fabs(DES.sucker_lift - sucker.lift_motor.pos_pid.fpFB) < 5)
        {
            box_state = get_state3;
        }
        pre_box_state = get_state2;

        break;

    case get_state3:

        DES.sucker_slide = sucker_slide_get_state2;
        if (fabs(DES.sucker_slide - sucker.slide_motor.pos_pid.fpFB) < 5)
        {
            DES.sucker_lift = sucker_lift_box_get_state2 + (target_num.box - 1) * height_box;
            if (box_finish)
            {
                DES.sucker_lift = sucker_lift_box_get_state2 + (3 - 1) * height_box;
            }
            if (fabs(DES.sucker_lift - sucker.lift_motor.pos_pid.fpFB) < 5)
            {
                sucker.Toggle_sucker = 1;
                // TODO：调节衔接时间
                fetch_pattern = FETCH_GET_PRE;
                box_state = await;
            }
        }
        pre_box_state = get_state3;
        break;
    case zhengli:
        DES.sucker_slide = sucker_slide_get_state2;
        if (fabs(sucker.slide_motor.pos_pid.fpFB - DES.sucker_slide) < 5)
        {
            DES.sucker_lift = sucker_yajin;
        }
        pre_box_state = zhengli;

        break;
    case lose_state0:
        sucker.Toggle_sucker = 1;
        DES.sucker_lift = sucker_out;
        DES.table_lift = table_lift_up;
        if (fabs(DES.table_lift - table.td_lift.m_x1) < 5)
        {
            box_state = lose_state1;
        }

        pre_box_state = lose_state0;
        break;

    case lose_state1:

        DES.table_slide = table_slide_out;
        //  DES.sucker_lift = 1300;
        DES.sucker_slide = -5;
        if (fabs(DES.sucker_slide - sucker.slide_motor.pos_pid.fpFB) < 2)
        {
            if (fabs(DES.table_slide - table.td_slide.m_x1) < 5)
            {
                DES.table_lift = table_lift_down;

                if (fabs(DES.table_lift - table.td_lift.m_x1) < 5)
                {
                    DES.sucker_lift = sucker_out2;
                    omtor_mode1 = 1;
                    if (fabs(velt_sucker) < 20)
                    {
                        //	DES.sucker_lift =  sucker.lift_motor.pos_pid.fpFB;
                        detect_box++;
                        if (detect_box > 50)
                        {
                            if (_servo_degree > 90)
                            {
                                task_time = 0;
                            }
                            _servo_degree = 40;

                            if (task_time > 1.0)
                            {
                                box_state = lose_state2;
                            }
                        }
                    }
                }
            }
        }

        pre_box_state = lose_state1;

        break;

    case lose_state2:
        omtor_mode1 = 0;
        DES.table_lift = table_lift_down;
        DES.sucker_lift = 1300;
        table.td_slide.m_r = 200;

        if (fabs(DES.table_lift - table.td_lift.m_x1) < 5)
        {
            DES.table_slide = table_slide_in;
        }
        pre_box_state = lose_state2;

        break;

    default:
        break;
    }

    pre_box_state = box_state;
}