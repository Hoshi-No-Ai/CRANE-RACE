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
            // �ϲ������ʼ��

            break;
        case ACTION_POS_1:
            // �ܵ���һ����λ

            break;
        case ACTION_POS_2:
            // �ܵ��ڶ�����λ
            break;
        case ACTION_POS_3:
            // �ܵ���������λ
            break;
        case ACTION_POS_4:
            // �ܵ����ĸ���λ
            break;
        case ACTION_POS_5:
            // �ܵ��������λ
            break;
        case ACTION_POS_6:
            // �ܵ���������λ
            break;
        case ACTION_POS_END:
            // �ܵ��յ�
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
            // �ϲ㷢�ͳ�ʼ���ɹ�flag��־λ
            //  if (/* condition */)
            //  {
            //      action_pattern=ACTION_POS_1;
            //  }
            break;
        case ACTIO_FETCH:
            // �ϲ㷢��ץȡ�ɹ�flag��־λ
            //  if (/* condition */&& action_pattern<7 && action_pattern>0)
            //  {
            //      action_pattern=pos_i+1;
            //  }
            break;
        case ACTION_POS_1:
            // �жϵ����Ƿ��ܵ���λ
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