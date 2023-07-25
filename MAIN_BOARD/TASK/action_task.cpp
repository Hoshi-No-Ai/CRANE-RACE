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