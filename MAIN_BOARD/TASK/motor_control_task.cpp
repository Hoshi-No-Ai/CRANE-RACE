#include "motor_control_task.h"

void omni_chassis_control(void)
{
    static motor_pid_state_e pre_turn_pid_state;
    static motor_pid_state_e pre_run_pid_state;

    static int16_t turn_current[4];
    static fp32 run_current[4];

    switch (Omni_chassis[RIGHTUP].m_run_motor.motor_pid_state)
    {
    case DOUBLE_LOOP:
        for (int i = 0; i < 4; i++)
        {
            Omni_chassis[i].m_run_motor.pos_pid.CalFilterPID();
            Omni_chassis[i].m_run_motor.velt_pid.fpDes = Omni_chassis[i].m_run_motor.pos_pid.fpU;
            Omni_chassis[i].m_run_motor.velt_pid.CalPID();
        }
        break;
    case POS_LOOP:
        break;
    case VELT_LOOP:
        for (int i = 0; i < 4; i++)
        {
            Omni_chassis[i].m_run_motor.velt_pid.fpDes =
                ClipFloat(Omni_chassis[i].m_run_motor.velt_pid.fpDes, -RUN_VELT_MAX, RUN_VELT_MAX);
            Omni_chassis[i].m_run_motor.velt_pid.CalFilterPID();
        }

        break;
    case OPEN_LOOP:
        if (pre_run_pid_state != OPEN_LOOP)
        {
            for (int i = 0; i < 4; i++)
            {
                Omni_chassis[i].m_run_motor.velt_pid.fpU = 0;
            }
        }
        break;
    }

    for (int i = 0; i < 4; i++)
    {
        run_current[i] = Omni_chassis[i].m_run_motor.velt_pid.fpU;
        if (Omni_chassis[i].m_run_motor.feed_forward_state == WITH_FORWARD)
        {
            run_current[i] += Omni_chassis[i].m_run_motor.feed_forward_current;
        }
        run_current[i] = ClipFloat(run_current[i], -SAFE_CURRENT_MAX, SAFE_CURRENT_MAX);
    }

    C_Motor::can_send_data(CAN1, 0x200, (int16_t)run_current[RIGHTUP], (int16_t)run_current[LEFTUP], (int16_t)run_current[LEFTDOWN], (int16_t)run_current[RIGHTDOWN]);

  //  C_Motor::can_send_data(CAN2, 0x200, table.slide_motor1.pid_current, table.slide_motor2.pid_current, table.lift_motor1.pid_current, table.lift_motor2.pid_current);

    pre_run_pid_state = Omni_chassis[RIGHTUP].m_run_motor.motor_pid_state;
}

void disable_all_motor(void)
{
    C_Motor::can_send_data(CAN1, 0x1FF, 0, 0, 0, 0);
    C_Motor::can_send_data(CAN1, 0x200, 0, 0, 0, 0);
    C_Motor::can_send_data(CAN2, 0x1FF, 0, 0, 0, 0);
    C_Motor::can_send_data(CAN2, 0x200, 0, 0, 0, 0);
}
