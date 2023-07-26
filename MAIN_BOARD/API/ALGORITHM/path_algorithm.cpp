#include "path_algorithm.h"
using _navigation_::flag_record;

static float StartX, StartY, StartQ, DELTA_X, DELTA_Y, DELTA_Q, t_run, Ts = 0.001;
static fp32 t1, t2, t3, t4, t5, t6, t7, t8, t_w;

void turn_Q(fp32 &des, fp32 &fb)
{
    float des_forward, des_backward, fb_feedback;
    des = ((int32_t)des + 720000) % 360 + des - (int32_t)des;
    if (des > 180)
    {
        des_forward = des - 360;
    }
    else
    {
        des_forward = des;
    }

    fb_feedback = ((int32_t)fb + 720000) % 360 + fb - (int32_t)fb;
    if (fb_feedback > 180)
    {
        fb_feedback = fb_feedback - 360;
    }
    else
    {
        fb_feedback = fb_feedback;
    }

    des = fb + des_forward - fb_feedback;
}

// 直线x+方向测试
void C_AUTO_PATH::path_x_test(void)
{
    static fp32 S;
    static fp32 S1, S2, S3;
    static fp32 A_up;
    static fp32 A_down;
    static fp32 R;
    static fp32 Vmax;
    if (flag_record) // 初始化加速度，运动时间，位移etc
    {
        A_up = 4000;   //-4500;
        A_down = 4000; //-4500;
        Vmax = 4000;
        DELTA_X = 4000;
        DELTA_Y = 0;

        StartX = pos_pid.x.fpFB;
        StartY = pos_pid.y.fpFB;
        StartQ = pos_pid.w.fpFB;

        S1 = Vmax * Vmax / (2 * A_up);
        S3 = Vmax * Vmax / (2 * A_down);
        S2 = DELTA_X - S1 - S3;
        t1 = Vmax / A_up;
        t3 = Vmax / A_down;
        t2 = S2 / Vmax;

        pos_pid.x.fpSumE = 0;
        pos_pid.y.fpSumE = 0;
        pos_pid.w.fpSumE = 0;

        flag_path_end = 0;
        flag_record = 0;
    }

    t_run = Ts * run_time;
    if (t_run < t1)
    {
        pos_pid.x.fpDes = StartX + 0.5f * A_up * t_run * t_run;
        pos_pid.y.fpDes = StartY;
        basic_velt.fpVx = A_up * t_run;
        basic_velt.fpVy = 0;
        pos_pid.w.fpDes = StartQ;
        basic_velt.fpW = 0;
        // acceleration = A_up;
    }
    else if (t_run < t1 + t2)
    {
        pos_pid.x.fpDes = StartX + S1 + Vmax * (t_run - t1);
        pos_pid.y.fpDes = StartY;
        basic_velt.fpVx = Vmax;
        basic_velt.fpVy = 0;
        // acceleration    = 0;
        pos_pid.w.fpDes = StartQ;
        basic_velt.fpW = 0;
    }
    else if (t_run < t1 + t2 + t3)
    {
        pos_pid.x.fpDes = StartX + DELTA_X - 0.5f * A_down * (t1 + t2 + t3 - t_run) * (t1 + t2 + t3 - t_run);
        pos_pid.y.fpDes = StartY;
        basic_velt.fpVx = Vmax - A_down * (t_run - t1 - t2);
        basic_velt.fpVy = 0;
        // acceleration    = 0;
        pos_pid.w.fpDes = StartQ;
        basic_velt.fpW = 0;
    }
    else if (t_run < t1 + t2 + t3 + 0.1f)
    {
        pos_pid.x.fpDes = StartX + DELTA_X;
        pos_pid.y.fpDes = StartY + DELTA_Y;
        basic_velt.fpVx = 0;
        basic_velt.fpVy = 0;
        // acceleration    = 0;
        pos_pid.w.fpDes = StartQ;
        basic_velt.fpW = 0;
    }
    else
    {
        run_time = 0;
        flag_path_end = 1;

        pos_pid.x.fpSumE = 0;
        pos_pid.y.fpSumE = 0;
        pos_pid.w.fpSumE = 0;
    }
    // basic_velt.fpW *= 0.5f;
}

// 直线y+方向测试
void C_AUTO_PATH::path_y_test(void)
{
    static fp32 S;
    static fp32 S1, S2, S3;
    static fp32 A_up;
    static fp32 A_down;
    static fp32 R;
    static fp32 Vmax;
    if (flag_record) // 初始化加速度，运动时间，位移etc
    {
        A_up = 4000;   //-4500;
        A_down = 4000; //-4500;
        Vmax = 4000;
        DELTA_X = 0;
        DELTA_Y = 4000;

        StartX = pos_pid.x.fpFB;
        StartY = pos_pid.y.fpFB;
        StartQ = pos_pid.w.fpFB;

        S1 = Vmax * Vmax / (2 * A_up);
        S3 = Vmax * Vmax / (2 * A_down);
        S2 = DELTA_Y - S1 - S3;
        t1 = Vmax / A_up;
        t3 = Vmax / A_down;
        t2 = S2 / Vmax;

        pos_pid.x.fpSumE = 0;
        pos_pid.y.fpSumE = 0;
        pos_pid.w.fpSumE = 0;

        flag_path_end = 0;
        flag_record = 0;
    }

    t_run = Ts * run_time;
    if (t_run < t1)
    {
        pos_pid.x.fpDes = StartX;
        pos_pid.y.fpDes = StartY + 0.5f * A_up * t_run * t_run;
        basic_velt.fpVx = 0;
        basic_velt.fpVy = A_up * t_run;
        pos_pid.w.fpDes = StartQ;
        basic_velt.fpW = 0;
        // acceleration = A_up;
    }
    else if (t_run < t1 + t2)
    {
        pos_pid.x.fpDes = StartX;
        pos_pid.y.fpDes = StartY + S1 + Vmax * (t_run - t1);
        basic_velt.fpVx = 0;
        basic_velt.fpVy = Vmax;
        // acceleration    = 0;
        pos_pid.w.fpDes = StartQ;
        basic_velt.fpW = 0;
    }
    else if (t_run < t1 + t2 + t3)
    {
        pos_pid.x.fpDes = StartX;
        pos_pid.y.fpDes = StartY + DELTA_Y - 0.5f * A_down * (t1 + t2 + t3 - t_run) * (t1 + t2 + t3 - t_run);
        basic_velt.fpVx = 0;
        basic_velt.fpVy = Vmax - A_down * (t_run - t1 - t2);
        // acceleration    = 0;
        pos_pid.w.fpDes = StartQ;
        basic_velt.fpW = 0;
    }
    else if (t_run < t1 + t2 + t3 + 0.1f)
    {
        pos_pid.x.fpDes = StartX + DELTA_X;
        pos_pid.y.fpDes = StartY + DELTA_Y;
        basic_velt.fpVx = 0;
        basic_velt.fpVy = 0;
        // acceleration    = 0;
        pos_pid.w.fpDes = StartQ;
        basic_velt.fpW = 0;
    }
    else
    {
        run_time = 0;
        flag_path_end = 1;

        pos_pid.x.fpSumE = 0;
        pos_pid.y.fpSumE = 0;
        pos_pid.w.fpSumE = 0;
    }
    // basic_velt.fpW *= 0.5f;
}

void C_AUTO_PATH::path_straight(void)
{
    static fp32 S;
    static fp32 S1, S2, S3;
    static fp32 A_up;
    static fp32 A_down;
    static fp32 Vmax;
    static fp32 Wmax;
    static fp32 alpha;
    static fp32 S_move;
    static fp32 V_move;
    static fp32 W_move;

    if (flag_record) // 初始化加速度，运动时间，位移etc
    {
        A_up = m_velt_acc.A_up;     // 3000;
        A_down = m_velt_acc.A_down; // 3000;
        Vmax = m_velt_acc.Vmax;     // 3000;
        Wmax = m_velt_acc.Wmax;

        StartX = pos_pid.x.fpFB;
        StartY = pos_pid.y.fpFB;
        StartQ = pos_pid.w.fpFB;

        // 类似转向的控制
//        turn_Q(m_point_end.m_q, StartQ);

        DELTA_X = m_point_end.m_x - StartX;
        DELTA_Y = m_point_end.m_y - StartY;
        DELTA_Q = m_point_end.m_q - StartQ;

        // 可能要调整绝对式Q大小
        if (fabs(DELTA_Q) < 5)
        {
            StartQ = m_point_end.m_q;
            DELTA_Q = 0;
        }

        S = Geometric_mean(DELTA_X, DELTA_Y);
        alpha = atan2f(DELTA_Y, DELTA_X);

        if (S <= (Vmax * Vmax / (2 * A_up) + Vmax * Vmax / (2 * A_down)))
        {
            S1 = S / (A_up + A_down) * A_down;
            S2 = 0;
            S3 = S / (A_up + A_down) * A_up;
            t1 = sqrt(2 * S1 / A_up);
            t2 = 0;
            t3 = sqrt(2 * S3 / A_down);
            Vmax = A_up * t1;
        }
        else
        {
            t1 = Vmax / A_up;
            S1 = Vmax * Vmax / (2 * A_up);
            t3 = Vmax / A_down;
            S3 = Vmax * Vmax / (2 * A_down);
            S2 = S - S1 - S3;
            t2 = S2 / Vmax;
        }

        W_move = DELTA_Q / (0.8f * (t1 + t2 + t3));
        t_w = 0.8f * (t1 + t2 + t3);
				if(W_move>Wmax)
				{
					W_move=Wmax;
					t_w=DELTA_Q/Wmax;
				}
					else if(W_move<-Wmax)
					{
						W_move=-Wmax;
						t_w=DELTA_Q/-Wmax;
					}


        pos_pid.x.fpSumE = 0;
        pos_pid.y.fpSumE = 0;
        pos_pid.w.fpSumE = 0;

        flag_path_end = 0;
        flag_record = 0;
    }

    t_run = Ts * run_time;
    if (t_run < t_w)
    {
        pos_pid.w.fpDes = StartQ + W_move * t_run;
        basic_velt.fpW = W_move;
    }
    else if (t_run < t_w + 0.1f || t1 + t2 + t3 + 0.1f)
    {
        pos_pid.w.fpDes = StartQ + DELTA_Q;
        basic_velt.fpW = 0;
    }
    else
    {
        pos_pid.w.fpSumE = 0;
    }

    if (t_run < t1)
    {
        S_move = 0.5f * A_up * t_run * t_run;
        V_move = A_up * t_run;
        pos_pid.x.fpDes = StartX + S_move * cosf(alpha);
        pos_pid.y.fpDes = StartY + S_move * sinf(alpha);
        basic_velt.fpVx = V_move * cosf(alpha);
        basic_velt.fpVy = V_move * sinf(alpha);
    }
    else if (t_run < t1 + t2)
    {
        S_move = S1 + Vmax * (t_run - t1);
        V_move = Vmax;
        pos_pid.x.fpDes = StartX + S_move * cosf(alpha);
        pos_pid.y.fpDes = StartY + S_move * sinf(alpha);
        basic_velt.fpVx = V_move * cosf(alpha);
        basic_velt.fpVy = V_move * sinf(alpha);
    }
    else if (t_run < t1 + t2 + t3)
    {
        S_move = S1 + S2 + Vmax * (t_run - t1 - t2) - 0.5f * A_down * (t_run - t1 - t2) * (t_run - t1 - t2);
        V_move = Vmax - A_down * (t_run - t1 - t2);
        pos_pid.x.fpDes = StartX + S_move * cosf(alpha);
        pos_pid.y.fpDes = StartY + S_move * sinf(alpha);
        basic_velt.fpVx = V_move * cosf(alpha);
        basic_velt.fpVy = V_move * sinf(alpha);
    }
    else if (t_run < t1 + t2 + t3 + 0.1f || t_run < t_w + 0.1f)
    {
        pos_pid.x.fpDes = StartX + DELTA_X;
        pos_pid.y.fpDes = StartY + DELTA_Y;
        basic_velt.fpVx = 0;
        basic_velt.fpVy = 0;
    }
    else
    {
        pos_pid.x.fpSumE = 0;
        pos_pid.y.fpSumE = 0;
    }

    if (t_run > t1 + t2 + t3 + 0.1f && t_run > t_w + 0.1f)
    {
        run_time = 0;
        flag_path_end = 1;
    }
}

bool C_AUTO_PATH::Path_Choose(void)
{
    if (flag_path_end)
    {
        return false;
    }
    else
    {
        switch (m_number)
        {
        case 0:
            // none
            break;
        case 1:
            path_straight();
            break;
        case 10:
            path_x_test();
            break;
        case 11:
            path_y_test();
            break;
        default:
            break;
        }
    }
}
