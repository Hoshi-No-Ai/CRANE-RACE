#include "path_discarded.h"
/*******************************************************************
版权声明：HITCRT(哈工大竞技机器人队)
文件名：path_discarded.c
最近修改日期：2022.5.21
版本：第一版
作者：JIN
该.c文件用于存储一些较有借鉴意义的路径
**********************************************************************/
////路径规划还需的依赖变量
// double t;
// double _start_ptx = 6000, _start_pty = 2000; //在路径仿真中全局坐标下车的起始位置
// double time_sum;

// const int _n_segment = 5;
// double _time[_n_segment] = {3.78571, 1.85485, 2.92989, 1.08537, 1.01497};
// const int _order[_n_segment] = {6, 6, 6, 6, 6};
// double _coefx[5][7] =
//     {
//         {1.58491, 1.58491, 1.58491, 1.68897, 1.89159, 2.13565, 2.37027},
//         {4.83768, 5.07229, 5.30228, 5.51543, 5.70025, 5.84936, 5.96276},
//         {3.77488, 3.88828, 3.94526, 3.94585, 3.91928, 3.89778, 3.89039},
//         {10.5019, 10.4945, 10.4923, 10.4966, 10.5075, 10.5233, 10.5402},
//         {11.2713, 11.2882, 11.3062, 11.322, 11.3304, 11.3304, 11.3304},
// };
// double _coefy[5][7] =
//     {
//         {0.528302, 0.528302, 0.528302, 0.492125, 0.434685, 0.390824, 0.384139},
//         {0.784022, 0.777337, 0.788867, 0.824276, 0.887908, 0.980987, 1.09982},
//         {0.696271, 0.815106, 0.974624, 1.16562, 1.35951, 1.52639, 1.65306},
//         {4.46232, 4.58899, 4.70075, 4.7958, 4.87318, 4.93312, 4.97735},
//         {5.32261, 5.36685, 5.39641, 5.41283, 5.4189, 5.4189, 5.4189},
// };

// int CList[7] = {1, 6, 15, 20, 15, 6, 1};
// int CvList[7] = {1, 5, 10, 10, 5, 1};
// int CaList[7] = {1, 4, 6, 4, 1};

// static void path_bezier(C_NAV *p_nav)
//{
//     if (flag_record) //初始化加速度，运动时间，位移etc
//     {
//         time_sum = 0;
//         for (int i = 0; i < _n_segment; i++)
//         {
//             time_sum += _time[i];
//         }

//        StartX = p_nav->auto_path.pos_pid.x.fpFB;
//        StartY = p_nav->auto_path.pos_pid.y.fpFB;
//        StartQ = p_nav->auto_path.pos_pid.w.fpFB;

//        DELTA_X = 5500;
//        DELTA_Y = 3500;

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//        flag_record = 0;

//        //		if ((fabs(StartX) < 250) && (fabs(StartY) < 250) && (fabs(StartQ) < 30))
//        //		{
//        //			StartX = 0;
//        //			StartY = 0;
//        //			StartQ = 0;
//        //		}
//    }

//    t_run = Ts * p_nav->auto_path.run_time;
//    t = t_run;

//    if (t_run < time_sum)
//    {
//        for (int idx = 0; idx < _n_segment; ++idx)
//        {

//            if (t > _time[idx] && idx + 1 < _n_segment)
//            {
//                t -= _time[idx];
//            }
//            else
//            {
//                t /= _time[idx];

//                p_nav->auto_path.pos_pid.x.fpDes = 0.0;
//                p_nav->auto_path.pos_pid.y.fpDes = 0.0;

//                p_nav->auto_path.basic_velt.fpVx = 0.0;
//                p_nav->auto_path.basic_velt.fpVy = 0.0;

//                int cur_order = _order[idx];
//                int cur_poly_num = cur_order + 1;

//                for (int i = 0; i < cur_poly_num; i++)
//                {
//                    p_nav->auto_path.pos_pid.x.fpDes += _time[idx] * 1000 * CList[i] * _coefx[idx][i] *
//                    pow(t, i) * pow((1 - t), (cur_order - i)); p_nav->auto_path.pos_pid.y.fpDes +=
//                    _time[idx] * 1000 * CList[i] * _coefy[idx][i] * pow(t, i) * pow((1 - t), (cur_order -
//                    i));

//                    if (i < (cur_poly_num - 1))
//                    {
//                        p_nav->auto_path.basic_velt.fpVx += 1000 * CvList[i] * cur_order * (_coefx[idx][i +
//                        1] - _coefx[idx][i]) * pow(t, i) * pow((1 - t), (cur_order - 1 - i));

//                        p_nav->auto_path.basic_velt.fpVy += 1000 * CvList[i] * cur_order * (_coefy[idx][i +
//                        1] - _coefy[idx][i]) * pow(t, i) * pow((1 - t), (cur_order - 1 - i));
//                    }

//                    //					if (i < (cur_poly_num - 2))
//                    //					{
//                    //						p_nav->auto_path.acceleration_x += 1.0 / _time[idx] *
//                    1000*CaList[i]
//                    * cur_order * (cur_order - 1) * (_coefx[idx][i + 2] - 2 * _coefx[idx][i + 1] +
//                    _coefx[idx][i]) * pow(t, i) * pow((1 - t), (cur_order - 2 - i));

//                    //						p_nav->auto_path.acceleration_y+= 1.0 / _time[idx] *
//                    1000*CaList[i]
//                    * cur_order * (cur_order - 1) * (_coefy[idx][i + 2] - 2 * _coefy[idx][i + 1] +
//                    _coefy[idx][i]) * pow(t, i) * pow((1 - t), (cur_order - 2 - i));
//                    //					}
//                }
//                break;
//            }
//        }
//        p_nav->auto_path.pos_pid.x.fpDes -= _start_ptx;
//        p_nav->auto_path.pos_pid.y.fpDes -= _start_pty;
//        p_nav->auto_path.pos_pid.x.fpDes += StartX;
//        p_nav->auto_path.pos_pid.y.fpDes += StartY;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < time_sum + 0.1f)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X;
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y;
//        p_nav->auto_path.basic_velt.fpVx = 0;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else
//    {
//        p_nav->auto_path.run_time = 0;
//        p_nav->state = NAV_STOP;

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//    }
//}

// void Call_curvature(ST_Trajectory *track)
//{
//     C_VECTOR temp_a;
//     C_VECTOR temp_b;
//     fp32 an, R, v_2;
//     v_2 = track->velt_ref.fpVx * track->velt_ref.fpVx + track->velt_ref.fpVy * track->velt_ref.fpVy;

//    if (fabs(v_2) < 10)
//    {
//        v_2 = 10;
//    }

//    temp_a.fpVx = track->velt_ref.fpVy / sqrt(v_2);
//    temp_a.fpVy = -track->velt_ref.fpVx / sqrt(v_2);

//    an = temp_a.fpVx * track->acc_ref.fpVx + temp_a.fpVy * track->acc_ref.fpVy;

//    if (fabs(an) < 10)
//    {
//        an = 10;
//    }

//    R = v_2 / an;
//    temp_b.fpVx = R * temp_a.fpVx;
//    temp_b.fpVy = R * temp_a.fpVy;
//    temp_b.fpLength = Geometric_mean(temp_b.fpVx, temp_b.fpVy);
//    if (fabs(temp_b.fpLength) < 0.1)
//    {
//        track->state = LINE;
//        temp_b.fpLength = 0.1;
//        track->curvature.fpLength = temp_b.fpLength;
//        track->curvature.fpVy = track->curvature.fpLength * temp_a.fpVy;
//        track->curvature.fpVx = track->curvature.fpLength * temp_a.fpVx;
//    }
//    if (fabs(temp_b.fpLength) > 5000)
//    {
//        track->state = LINE;
//        track->curvature.fpLength = 10000;
//        track->curvature.fpVy = track->curvature.fpLength * temp_a.fpVy;
//        track->curvature.fpVx = track->curvature.fpLength * temp_a.fpVx;
//    }
//    else
//    {
//        track->state = CURVE;
//        track->curvature.fpVy = temp_b.fpVy;
//        track->curvature.fpVx = temp_b.fpVx;
//        track->curvature.fpLength = temp_b.fpLength;
//    }
//}

// static void path_bezier_traj(C_NAV *p_nav, ST_Trajectory *track)
//{
//     ST_Trajectory traj;
//     fp32 Ts = 0.001;
//     if (flag_record) //初始化加速度，运动时间，位移etc
//     {
//         time_sum = 0;
//         for (int i = 0; i < _n_segment; i++)
//         {
//             time_sum += _time[i];
//         }

//        StartX = p_nav->auto_path.pos_pid.x.fpFB;
//        StartY = p_nav->auto_path.pos_pid.y.fpFB;
//        StartQ = p_nav->auto_path.pos_pid.w.fpFB;

//        DELTA_X = 5500; //
//        DELTA_Y = 3750; //

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//        flag_record = 0;

//        if ((fabs(StartX) < 250) && (fabs(StartY) < 250) && (fabs(StartQ) < 30))
//        {
//            StartX = 0;
//            StartY = 0;
//            StartQ = 0;
//        }
//    }

//    t_run = Ts * p_nav->auto_path.run_time;
//    t = t_run;

//    if (t_run < time_sum)
//    {
//        for (int idx = 0; idx < _n_segment; ++idx)
//        {

//            if (t > _time[idx] && idx + 1 < _n_segment)
//            {
//                t -= _time[idx];
//            }
//            else
//            {
//                t /= _time[idx];
//                traj.pos_ref.X = 0.0;
//                traj.pos_ref.Y = 0.0;
//                traj.pos_ref.Q = 0;
//                traj.velt_ref.fpLength = 0;
//                traj.velt_ref.fpVx = 0;
//                traj.velt_ref.fpVy = 0;
//                traj.acc_ref.fpVx = 0;
//                traj.acc_ref.fpVy = 0;
//                int cur_order = _order[idx];
//                int cur_poly_num = cur_order + 1;

//                for (int i = 0; i < cur_poly_num; i++)
//                {
//                    traj.pos_ref.X += _time[idx] * CList[i] * 1000 * _coefx[idx][i] * pow(t, i) * pow((1 -
//                    t), (cur_order - i)); traj.pos_ref.Y += _time[idx] * CList[i] * 1000 * _coefy[idx][i] *
//                    pow(t, i) * pow((1 - t), (cur_order - i));

//                    if (i < (cur_poly_num - 1))
//                    {
//                        traj.velt_ref.fpVx += CvList[i] * cur_order * 1000 * (_coefx[idx][i + 1] -
//                        _coefx[idx][i]) * pow(t, i) * pow((1 - t), (cur_order - 1 - i)); traj.velt_ref.fpVy
//                        += CvList[i] * cur_order * 1000 * (_coefy[idx][i + 1] - _coefy[idx][i]) * pow(t, i)
//                        * pow((1 - t), (cur_order - 1 - i));
//                    }

//                    if (i < (cur_poly_num - 2))
//                    {
//                        traj.acc_ref.fpVx += 1.0 / _time[idx] * CaList[i] * 1000 * cur_order * (cur_order -
//                        1) * (_coefx[idx][i + 2] - 2 * _coefx[idx][i + 1] + _coefx[idx][i]) * pow(t, i) *
//                        pow((1 - t), (cur_order - 2 - i));

//                        traj.acc_ref.fpVy += 1.0 / _time[idx] * CaList[i] * 1000 * cur_order * (cur_order -
//                        1) * (_coefy[idx][i + 2] - 2 * _coefy[idx][i + 1] + _coefy[idx][i]) * pow(t, i) *
//                        pow((1 - t), (cur_order - 2 - i));
//                    }
//                }
//                break;
//            }
//        }
//        traj.pos_ref.X -= _start_ptx;
//        traj.pos_ref.Y -= _start_pty;
//        traj.pos_ref.X += StartX;
//        traj.pos_ref.Y += StartY;

//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < time_sum + 0.2f)
//    {
//        traj.pos_ref.X = StartX + DELTA_X;
//        traj.pos_ref.Y = StartY + DELTA_Y;
//        traj.velt_ref.fpVx = 0;
//        traj.velt_ref.fpVy = 0;
//        traj.acc_ref.fpVx = 0;
//        traj.acc_ref.fpVy = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else
//    {
//        p_nav->auto_path.run_time = 0;
//        p_nav->state = NAV_STOP;

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//    }
//    track->pos_ref.X = traj.pos_ref.X;
//    track->pos_ref.Y = traj.pos_ref.Y;
//    track->velt_ref.fpVx = traj.velt_ref.fpVx;
//    track->velt_ref.fpVy = traj.velt_ref.fpVy;
//    track->acc_ref.fpVx = traj.acc_ref.fpVx;
//    track->acc_ref.fpVy = traj.acc_ref.fpVy;
//    Call_curvature(track);
//    p_nav->auto_path.pos_pid.x.fpDes = track->pos_ref.X;
//    p_nav->auto_path.pos_pid.y.fpDes = track->pos_ref.Y;
//    p_nav->auto_path.basic_velt.fpVx = track->velt_ref.fpVx;
//    p_nav->auto_path.basic_velt.fpVy = track->velt_ref.fpVy;
//}

// static void path_test_circle(C_NAV *p_nav, ST_Trajectory *track)
//{
//     const fp32 t_base = 0.001f;
//     fp32 t_now;
//     fp32 theta;
//     static fp32 s_test = 4, s_acc, s_uni;
//     static fp32 t1_1, t1_2, t2, t3_1, t3_2;
//     static fp32 acc = 4, r = 2, v_max = 4;
//     static ST_POS start_pos;
//     if (flag_record)
//     {
//         start_pos.X = cRobot.stPot.fpPosX;
//         start_pos.Y = cRobot.stPot.fpPosX;
//         start_pos.Q = cRobot.stPot.fpPosQ;
//         t_now = p_nav->auto_path.run_time * t_base;
//         t1_1 = v_max / acc;
//         s_acc = (v_max * v_max) / (2 * acc);
//         s_uni = s_test - s_acc;
//         t1_2 = s_uni / v_max;
//         t2 = r * 3.1415 / v_max;
//         t3_1 = t1_2;
//         t3_2 = t1_1;

//        flag_record = 1;
//    }

//    if (t_now < t1_1)
//    {
//        track->pos_ref.X = 0.5 * acc * t_now * t_now;
//        track->pos_ref.Y = 0;
//        track->pos_ref.Q = 0;
//        track->velt_ref.fpLength = acc * t_now;
//        track->velt_ref.fpVx = acc * t_now;
//        track->velt_ref.fpVy = 0;
//        track->curvature.fpVy = 100000000;
//        track->curvature.fpLength = 100000000;
//    }
//    if (t_now < t1_1 + t1_2)
//    {
//        track->pos_ref.X = 0.5 * acc * t1_1 * t1_1 + v_max * (t_now - t1_1);
//        track->pos_ref.Y = 0;
//        track->pos_ref.Q = 0;

//        track->velt_ref.fpLength = v_max;
//        track->velt_ref.fpVx = v_max;
//        track->velt_ref.fpVy = 0;
//        track->curvature.fpLength = 100000000;
//        track->curvature.fpVy = 100000000;
//        track->curvature.fpVx = 0;
//    }
//    if (t_now < t1_1 + t1_2 + t2)
//    {
//        theta = (t_now - t1_1 - t1_2) / t2;
//        track->pos_ref.X = s_test + r * sinf(theta);
//        track->pos_ref.Y = r - r * cosf(theta);
//        track->pos_ref.Q = 0;
//        track->velt_ref.fpLength = v_max;
//        track->velt_ref.fpVx = v_max * cosf(theta);
//        track->velt_ref.fpVx = v_max * sinf(theta);
//        track->curvature.fpLength = r;
//        track->curvature.fpVx = s_test;
//        track->curvature.fpVy = r;
//    }
//    if (t_now < t1_1 + t1_2 + t2 + t3_1)
//    {
//        track->pos_ref.X = s_test + v_max * (t_now - (t1_1 + t1_2 + t2));
//        track->pos_ref.Y = 0;
//        track->pos_ref.Q = 0;
//        track->velt_ref.fpLength = v_max;
//        track->velt_ref.fpVx = v_max;
//        track->velt_ref.fpVy = 0;
//        track->curvature.fpLength = 10000000;
//        track->curvature.fpVy = 10000000;
//        track->curvature.fpVx = 0;
//    }
//    if (t_now < t1_1 + t1_2 + t2 + t3_2)
//    {
//        track->pos_ref.X = s_test + s_uni + v_max * (t_now - (t1_1 + t1_2 + t2 + t3_1)) + 0.5 * acc * (t_now
//        - (t1_1 + t1_2 + t2 + t3_1)) * (t_now - (t1_1 + t1_2 + t2 + t3_1)); track->pos_ref.Y = 0;
//        track->pos_ref.Q = 0;
//        track->velt_ref.fpLength = v_max - acc * (t_now - (t1_1 + t1_2 + t2 + t3_1));
//        track->velt_ref.fpVx = v_max - acc * (t_now - (t1_1 + t1_2 + t2 + t3_1));
//        track->velt_ref.fpVy = 0;
//        track->curvature.fpLength = 10000000;
//        track->curvature.fpVy = 10000000;
//        track->curvature.fpVx = 0;
//    }
//    if (t_now < t1_1 + t1_2 + t2 + t3_2 + 0.5)
//    {
//        track->pos_ref.X = 2 * s_test;
//        track->pos_ref.Y = 0;
//        track->pos_ref.Q = 0;
//        track->velt_ref.fpLength = v_max - acc * (t_now - (t1_1 + t1_2 + t2 + t3_1));
//        track->velt_ref.fpVx = v_max - acc * (t_now - (t1_1 + t1_2 + t2 + t3_1));
//        track->velt_ref.fpVy = 0;
//        track->curvature.fpLength = 10000000;
//        track->curvature.fpVy = 10000000;
//        track->curvature.fpVx = 0;
//    }
//}

// float DELTA_X_1, DELTA_Y_1, DELTA_X_2, DELTA_Y_2;
// static void path_40(C_NAV *p_nav) //直线圆弧斜线
//{
//     static fp32 R;
//     static fp32 S1, S2;
//     static fp32 A_up_1, A_down_1, A_up_2, A_down_2;
//     //  static fp32 S_x,S_y;
//     static fp32 V_max_1, V_max_2, V_r, V_1, V_2;
//     static fp32 sina, cosa;
//     if (flag_record) //初始化加速度，运动时间，位移etc
//     {
//         DELTA_X = 5320;
//         DELTA_Y = 3220;

//        A_up_1 = 5000;
//        A_down_1 = 5000;
//        V_max_1 = 5000;
//        DELTA_X_1 = 4000;
//        DELTA_Y_1 = 0;
//        S1 = Geometric_mean(DELTA_X_1, DELTA_Y_1);

//        V_r = 2000;
//        R = 1000;

//        A_up_2 = 5000;
//        A_down_2 = 5000;
//        V_max_2 = 5000;
//        DELTA_X_2 = DELTA_X - R - DELTA_X_1;
//        DELTA_Y_2 = DELTA_Y - R - DELTA_Y_1;
//        S2 = Geometric_mean(DELTA_X_2, DELTA_Y_2);
//        cosa = DELTA_X_2 / S2;
//        sina = DELTA_Y_2 / S2;

//        StartX = p_nav->auto_path.pos_pid.x.fpFB;
//        StartY = p_nav->auto_path.pos_pid.y.fpFB;
//        StartQ = p_nav->auto_path.pos_pid.w.fpFB;

//        //目前用的加减速段加速度大小相同的模式,且V>V_r
//        float acct_1 = (V_max_1) / A_up_1;
//        float accd_1 = 0.5f * V_max_1 * acct_1;
//        float dcct_1 = (V_max_1 - V_r) / A_up_1;
//        float dccd_1 = 0.5f * (V_max_1 + V_r) * dcct_1;

//        float acct_2 = (V_max_2 - V_r) / A_up_2;
//        float accd_2 = 0.5f * (V_max_2 + V_r) * acct_2;
//        float dcct_2 = (V_max_2) / A_up_2;
//        float dccd_2 = 0.5f * V_max_2 * dcct_2;

//        if (S1 < accd_1 + dccd_1)
//        {
//            V_1 = sqrt(A_up_1 * S1 + 0.5f * V_r * V_r);
//            V_max_1 = V_1;
//            t1 = V_1 / A_up_1;
//            t2 = 0;
//            t3 = (V_1 - V_r) / A_up_1;
//        }
//        else
//        {
//            t1 = acct_1;
//            t2 = (S1 - accd_1 - dccd_1) / V_max_1;
//            t3 = dcct_1;
//        }

//        t4 = PI / 2 * R / V_r;

//        if (S2 < accd_2 + dccd_2)
//        {
//            V_2 = sqrt(A_up_2 * S2 + 0.5f * V_r * V_r);
//            V_max_2 = V_2;
//            t5 = (V_2 - V_r) / A_up_2;
//            t6 = 0;
//            t7 = V_2 / A_up_2;
//        }
//        else
//        {
//            t5 = acct_2;
//            t6 = (S2 - accd_2 - dccd_2) / V_max_2;
//            t7 = dcct_2;
//        }

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//        flag_record = 0;
//    }

//    t_run = Ts * p_nav->auto_path.run_time;
//    if (t_run < t1)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_up_1 * t_run * t_run);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = A_up_1 * t_run;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//        // p_nav->auto_path.acceleration = A_up;
//    }
//    else if (t_run < t1 + t2)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_up_1 * t1 * t1) + V_max_1 * (t_run - t1);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = V_max_1;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_up_1 * t1 * t1) + V_max_1 * t2 + V_max_1 *
//        (t_run - t1 - t2) - 0.5f * A_up_1 * (t_run - t1 - t2) * (t_run - t1 - t2);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = V_max_1 - A_up_1 * (t_run - t1 - t2);
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R * sinf(V_r * (t_run - t1 - t2 - t3) / R);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R - R * cosf(V_r * (t_run - t1 - t2 - t3) /
//        R); p_nav->auto_path.basic_velt.fpVx = V_r * cosf(V_r * (t_run - t1 - t2 - t3) / R);
//        p_nav->auto_path.basic_velt.fpVy = V_r * sinf(V_r * (t_run - t1 - t2 - t3) / R);
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4 + t5)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R + (V_r * (t_run - t1 - t2 - t3 - t4) +
//        0.5f * A_up_2 * (t_run - t1 - t2 - t3 - t4) * (t_run - t1 - t2 - t3 - t4)) * cosa;
//        p_nav->auto_path.pos_pid.y.fpDes = StartX + DELTA_Y_1 + R + (V_r * (t_run - t1 - t2 - t3 - t4) +
//        0.5f * A_up_2 * (t_run - t1 - t2 - t3 - t4) * (t_run - t1 - t2 - t3 - t4)) * sina;
//        p_nav->auto_path.basic_velt.fpVx = (V_r + A_up_2 * (t_run - t1 - t2 - t3 - t4)) * cosa;
//        p_nav->auto_path.basic_velt.fpVy = (V_r + A_up_2 * (t_run - t1 - t2 - t3 - t4)) * sina;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4 + t5 + t6)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R + (V_r * t5 + 0.5f * A_up_2 * t5 * t5 +
//        V_max_2 * (t_run - t1 - t2 - t3 - t4 - t5)) * cosa; p_nav->auto_path.pos_pid.y.fpDes = StartX +
//        DELTA_Y_1 + R + (V_r * t5 + 0.5f * A_up_2 * t5 * t5 + V_max_2 * (t_run - t1 - t2 - t3 - t4 - t5)) *
//        sina; p_nav->auto_path.basic_velt.fpVx = V_max_2 * cosa; p_nav->auto_path.basic_velt.fpVy = V_max_2
//        * sina;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4 + t5 + t6 + t7)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R + (V_r * t5 + 0.5f * A_up_2 * t5 * t5 +
//        V_max_2 * t6 + V_max_2 * (t_run - t1 - t2 - t3 - t4 - t5 - t6) - 0.5f * A_up_2 * (t_run - t1 - t2 -
//        t3 - t4 - t5 - t6) * (t_run - t1 - t2 - t3 - t4 - t5 - t6)) * cosa; p_nav->auto_path.pos_pid.y.fpDes
//        = StartX + DELTA_Y_1 + R + (V_r * t5 + 0.5f * A_up_2 * t5 * t5 + V_max_2 * t6 + V_max_2 * (t_run -
//        t1 - t2 - t3 - t4 - t5 - t6) - 0.5f * A_up_2 * (t_run - t1 - t2 - t3 - t4 - t5 - t6) * (t_run - t1 -
//        t2 - t3 - t4 - t5 - t6)) * sina; p_nav->auto_path.basic_velt.fpVx = (V_max_2 - A_up_2 * (t_run - t1
//        - t2 - t3 - t4 - t5 - t6)) * cosa; p_nav->auto_path.basic_velt.fpVy = (V_max_2 - A_up_2 * (t_run -
//        t1 - t2 - t3 - t4 - t5 - t6)) * sina;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4 + t5 + t6 + t7 + 0.1f)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X;
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y;
//        p_nav->auto_path.basic_velt.fpVx = 0;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else
//    {
//        p_nav->auto_path.run_time = 0;
//        p_nav->state = NAV_STOP;

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//    }
//}

///*原理说明
//可控变量：距离：直线段位移，曲线段x位移，曲线段y位移，x总位移，y总位移
//         速度：直线段最大速度，曲线段初速度
//         加速度：直线1段加速度，斜线2段加速度
//路径过程：直线：先加后减、先加后匀速、一直加速
//         曲线：x、y分别加减速，互不干扰
//         斜线：匀减速
//*/
// float DELTA_X_CURVE, DELTA_Y_CURVE;
// float alpha;

// static void path_41(C_NAV *p_nav) //直线曲线斜线(一直减)
//{
//     static fp32 R;
//     static fp32 S1, S2;
//     static fp32 A_up_1, A_down_1, A_up_2, A_down_2;
//     static float A_1, A_2, A_3, A_4, A_X_CURVE, A_Y_CURVE, A_CURVE, A_4_X, A_4_Y;
//     static fp32 V_max_1, V_max_2, V_r, V_1, V_2, V_Y_r, V_X_r, V_S_r;
//     static fp32 sina, cosa;

//    if (flag_record) //初始化加速度，运动时间，位移etc
//    {
//        DELTA_X = 5220;
//        //		DELTA_Y = 3220;
//        DELTA_Y = 3230 + times_path * 300;
//        times_path++;

//        A_1 = 3000; //可更改
//        A_2 = 3000; //可更改

//        V_max_1 = 3500; //可更改
//        V_r = 2000;     //可更改
//        V_max_2 = 2500; //可更改

//        DELTA_X_1 = 4000;                                //可更改
//        DELTA_X_CURVE = 1000;                            //可更改
//        DELTA_Y_CURVE = 1000;                            //可更改
//        DELTA_X_2 = DELTA_X - DELTA_X_1 - DELTA_X_CURVE; //斜线段x位移
//        DELTA_Y_2 = DELTA_Y - DELTA_Y_CURVE;             //斜线段y位移

//        alpha = atanf(DELTA_Y_2 / DELTA_X_2);
//        S2 = Geometric_mean(DELTA_X_2, DELTA_Y_2);

//        StartX = p_nav->auto_path.pos_pid.x.fpFB;
//        StartY = p_nav->auto_path.pos_pid.y.fpFB;
//        StartQ = p_nav->auto_path.pos_pid.w.fpFB;
//        if ((fabs(StartX) < 250) && (fabs(StartY) < 250) && (fabs(StartQ) < 10))
//        {
//            StartX = 0;
//            StartY = 0;
//            StartQ = 0;
//        }

//        V_X_r = V_r / ((DELTA_X_CURVE * tanf(alpha) / DELTA_Y_CURVE) - 1);
//        //由x方向初速度和距离算出曲线段x末速度 V_Y_r = V_X_r * tanf(alpha); V_S_r = Geometric_mean(V_X_r,
//        V_Y_r);

//        t1 = V_max_1 / A_1;                                        //直线加速
//        t2 = 2 * (DELTA_X_1 - V_max_1 * t1 / 2) / (V_r + V_max_1); //直线——圆弧衔接
//        t3 = 2 * DELTA_Y_CURVE / (V_X_r * tanf(alpha));
//        t4 = 2 * DELTA_Y_2 / V_Y_r;

//        A_3 = (V_r - V_max_1) / t2;
//        A_4_X = -V_X_r / t4; //斜线段x加速度
//        A_4_Y = -V_Y_r / t4; //斜线段y加速度
//        A_X_CURVE = (V_X_r - V_r) / t3;
//        A_Y_CURVE = V_Y_r / t3;

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//        flag_record = 0;
//    }

//    t_run = Ts * p_nav->auto_path.run_time;
//    if (t_run < t1)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_1 * t_run * t_run);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = A_1 * t_run;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//        // p_nav->auto_path.acceleration = A_up;
//    }
//    else if (t_run < t1 + t2)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_1 * t1 * t1) + V_max_1 * (t_run - t1) + 0.5f *
//        A_3 * (t_run - t1) * (t_run - t1); p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = V_max_1 + A_3 * (t_run - t1);
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + V_r * (t_run - t1 - t2) + 0.5f * A_X_CURVE *
//        (t_run - t1 - t2) * (t_run - t1 - t2); p_nav->auto_path.pos_pid.y.fpDes = StartY + 0.5f * A_Y_CURVE
//        * (t_run - t1 - t2) * (t_run - t1 - t2); p_nav->auto_path.basic_velt.fpVx = V_r + A_X_CURVE * (t_run
//        - t1 - t2); p_nav->auto_path.basic_velt.fpVy = A_Y_CURVE * (t_run - t1 - t2);
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + DELTA_X_CURVE + (V_X_r * (t_run - t1 - t2 -
//        t3) + 0.5f * A_4_X * (t_run - t1 - t2 - t3) * (t_run - t1 - t2 - t3));
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_CURVE + (V_Y_r * (t_run - t1 - t2 - t3) + 0.5f *
//        A_4_Y * (t_run - t1 - t2 - t3) * (t_run - t1 - t2 - t3)); p_nav->auto_path.basic_velt.fpVx = (V_X_r
//        + A_4_X * (t_run - t1 - t2 - t3)); p_nav->auto_path.basic_velt.fpVy = (V_Y_r + A_4_Y * (t_run - t1 -
//        t2 - t3));
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }

//    else if (t_run < t1 + t2 + t3 + t4 + 0.1f)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X;
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y;
//        p_nav->auto_path.basic_velt.fpVx = 0;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else
//    {
//        p_nav->auto_path.run_time = 0;
//        p_nav->state = NAV_STOP;

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//    }
//}

// static void path_42(C_NAV *p_nav) //直线曲线斜线（先加后减）
//{
//     static fp32 R;
//     static fp32 S1, S2;
//     static fp32 A_up_1, A_down_1, A_up_2, A_down_2;
//     static float A_1, A_2, A_3, A_4, A_X_CURVE, A_Y_CURVE, A_CURVE, A_4_X, A_4_Y;
//     static fp32 V_max_1, V_max_2, V_r, V_1, V_2, V_Y_r, V_X_r, V_S_r;
//     static fp32 sina, cosa;

//    if (flag_record) //初始化加速度，运动时间，位移etc
//    {
//        //		END_X = FIRST_BALL_X;
//        //		END_Y = FIRST_BALL_Y + times_path * 300;

//        StartX = p_nav->auto_path.pos_pid.x.fpFB;
//        StartY = p_nav->auto_path.pos_pid.y.fpFB;
//        StartQ = p_nav->auto_path.pos_pid.w.fpFB;

//        if (fabs(StartQ) < 10)
//        {
//            StartQ = 0;
//        }

//        if ((fabs(StartX) < 250) && (fabs(StartY) < 250))
//        {
//            StartX = 0;
//            StartY = 0;
//        }

//        DELTA_X = END_X - StartX;
//        DELTA_Y = END_Y - StartY;
//        times_path++;

//        //		A_1 = 3000;//可更改
//        //		A_2 = -3000;//可更改
//        A_1 = 2000;  //可更改
//        A_2 = -2000; //可更改

//        if (StartX < 1500)
//        {
//            //			V_max_1 = 3000;//可更改
//            //			V_r = 2000;//可更改
//            //			V_max_2 = 2500;//可更改
//            V_max_1 = 2000; //可更改
//            V_r = 1500;     //可更改
//            V_max_2 = 1500; //可更改
//        }
//        else if (StartX < 3500)
//        {
//            V_max_1 = 1000; //可更改
//            V_r = 1000;     //可更改
//            V_max_2 = 2000; //可更改
//        }
//        else if (StartX <= 3900)
//        {
//            V_max_1 = 500;  //可更改
//            V_r = 500;      //可更改
//            V_max_2 = 2000; //可更改
//        }

//        DELTA_X_1 = -StartX + 4000;                      //可更改
//        DELTA_X_CURVE = 1000;                            //可更改
//        DELTA_Y_CURVE = 1000;                            //可更改
//        DELTA_X_2 = DELTA_X - DELTA_X_1 - DELTA_X_CURVE; //斜线段x位移
//        DELTA_Y_2 = DELTA_Y - DELTA_Y_CURVE;             //斜线段y位移

//        alpha = atanf(DELTA_Y_2 / DELTA_X_2);
//        S2 = Geometric_mean(DELTA_X_2, DELTA_Y_2);

//        V_X_r = V_r / ((DELTA_X_CURVE * tanf(alpha) / DELTA_Y_CURVE) - 1);
//        //由x方向初速度和距离算出曲线段x末速度 V_Y_r = V_X_r * tanf(alpha); V_S_r = Geometric_mean(V_X_r,
//        V_Y_r);

//        t1 = V_max_1 / A_1;                                        //直线加速
//        t2 = 2 * (DELTA_X_1 - V_max_1 * t1 / 2) / (V_r + V_max_1); //直线——圆弧衔接
//        t3 = 2 * DELTA_Y_CURVE / (V_X_r * tanf(alpha));
//        t5 = -V_max_2 / A_2;
//        t4 = 2 * (DELTA_Y_2 + 0.5f * V_max_2 * V_max_2 / A_2 * sinf(alpha)) / (V_Y_r + V_max_2 *
//        sinf(alpha));

//        A_3 = (V_r - V_max_1) / t2;
//        A_4_X = (V_max_2 * cosf(alpha) - V_X_r) / t4; //斜线段x加速度
//        A_4_Y = (V_max_2 * sinf(alpha) - V_Y_r) / t4; //斜线段y加速度
//        A_X_CURVE = (V_X_r - V_r) / t3;
//        A_Y_CURVE = V_Y_r / t3;
//        A_4 = Geometric_mean(A_4_X, A_4_Y);
//        A_CURVE = Geometric_mean(A_X_CURVE, A_Y_CURVE);

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//        flag_record = 0;
//    }

//    t_run = Ts * p_nav->auto_path.run_time;
//    if (t_run < t1)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_1 * t_run * t_run);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = A_1 * t_run;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//        // p_nav->auto_path.acceleration = A_up;
//    }
//    else if (t_run < t1 + t2)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_1 * t1 * t1) + V_max_1 * (t_run - t1) + 0.5f *
//        A_3 * (t_run - t1) * (t_run - t1); p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = V_max_1 + A_3 * (t_run - t1);
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + V_r * (t_run - t1 - t2) + 0.5f * A_X_CURVE *
//        (t_run - t1 - t2) * (t_run - t1 - t2); p_nav->auto_path.pos_pid.y.fpDes = StartY + 0.5f * A_Y_CURVE
//        * (t_run - t1 - t2) * (t_run - t1 - t2); p_nav->auto_path.basic_velt.fpVx = V_r + A_X_CURVE * (t_run
//        - t1 - t2); p_nav->auto_path.basic_velt.fpVy = A_Y_CURVE * (t_run - t1 - t2);
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + DELTA_X_CURVE + (V_X_r * (t_run - t1 - t2 -
//        t3) + 0.5f * A_4_X * (t_run - t1 - t2 - t3) * (t_run - t1 - t2 - t3));
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_CURVE + (V_Y_r * (t_run - t1 - t2 - t3) + 0.5f *
//        A_4_Y * (t_run - t1 - t2 - t3) * (t_run - t1 - t2 - t3)); p_nav->auto_path.basic_velt.fpVx = (V_X_r
//        + A_4_X * (t_run - t1 - t2 - t3)); p_nav->auto_path.basic_velt.fpVy = (V_Y_r + A_4_Y * (t_run - t1 -
//        t2 - t3));
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }

//    else if (t_run < t1 + t2 + t3 + t4 + t5)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + DELTA_X_CURVE + 0.5f * (V_max_2 *
//        cosf(alpha) * V_max_2 * cosf(alpha) - V_X_r * V_X_r) / A_4_X + (V_max_2 * (t_run - t1 - t2 - t3 -
//        t4) + 0.5f * A_2 * (t_run - t1 - t2 - t3 - t4) * (t_run - t1 - t2 - t3 - t4)) * cosf(alpha);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_CURVE + 0.5f * (V_max_2 * sinf(alpha) * V_max_2
//        * sinf(alpha) - V_Y_r * V_Y_r) / A_4_Y + (V_max_2 * (t_run - t1 - t2 - t3 - t4) + 0.5f * A_2 *
//        (t_run - t1 - t2 - t3 - t4) * (t_run - t1 - t2 - t3 - t4)) * sinf(alpha);
//        p_nav->auto_path.basic_velt.fpVx = (V_max_2 + A_2 * (t_run - t1 - t2 - t3 - t4)) * cosf(alpha);
//        p_nav->auto_path.basic_velt.fpVy = (V_max_2 + A_2 * (t_run - t1 - t2 - t3 - t4)) * sinf(alpha);
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }

//    else if (t_run < t1 + t2 + t3 + t4 + t5 + 0.1f)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X;
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y;
//        p_nav->auto_path.basic_velt.fpVx = 0;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else
//    {
//        p_nav->auto_path.run_time = 0;
//        p_nav->state = NAV_STOP;

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;

//        //		flag_path_end = 1;
//        Auto_Key = 3;
//    }
//}

// float t5_y, t6_y, t7_y, t5_x, t6_x, t7_x, t_2_y, t_2_x_up, t_2_x_down;
// float delta_x_path, delta_y_path;
// int flag_change = 0;
// static void path_50(C_NAV *p_nav) //直线圆弧曲线
//{
//     static fp32 R;
//     static fp32 S1, S2;
//     static fp32 A_up_1, A_down_1, A_up_2_x, A_down_2_x, A_up_2_y, A_down_2_y;
//     //  static fp32 S_x,S_y;
//     static fp32 V_max_1, V_max_2_y, V_max_2_x, V_r, V_1, V_2;
//     static fp32 Kp_A_2_x;
//     if (flag_record) //初始化加速度，运动时间，位移etc
//     {
//         //如果要改变路径距离，flag_change要置1，且delta_x_path满足要求
//         if ((delta_x_path >= 3000 && delta_x_path <= 8000) && flag_change)
//         {
//             DELTA_X = delta_x_path;
//             DELTA_Y = 3230 + times_path * 300;
//             times_path++;

//            DELTA_X_1 = 4000 + delta_x_path - 5220;
//            DELTA_Y_1 = 0;
//        }
//        else
//        {
//            DELTA_X = 5220;
//            DELTA_Y = 3230 + times_path * 300;
//            times_path++;
//            //		  DELTA_Y = 3230;

//            DELTA_X_1 = 4000;
//            DELTA_Y_1 = 0;
//        }

//        A_up_1 = 3000;
//        A_down_1 = 3000;
//        V_max_1 = 2000;
//        S1 = Geometric_mean(DELTA_X_1, DELTA_Y_1);

//        V_r = 2000;
//        R = 1000;

//        Kp_A_2_x = 0.1f;
//        A_up_2_y = 3000;
//        A_down_2_y = 3000;
//        V_max_2_y = 2000;
//        DELTA_X_2 = DELTA_X - R - DELTA_X_1;
//        DELTA_Y_2 = DELTA_Y - R - DELTA_Y_1;
//        StartX = p_nav->auto_path.pos_pid.x.fpFB;
//        StartY = p_nav->auto_path.pos_pid.y.fpFB;
//        StartQ = p_nav->auto_path.pos_pid.w.fpFB;

//        if ((fabs(StartX - 5350 + DELTA_X) < 250) && (fabs(StartY) < 250) && (fabs(StartQ) < 10))
//        {
//            StartX = 5350 - DELTA_X;
//            StartY = 0;
//            StartQ = 0;
//        }

//        //目前用的加减速段加速度大小相同的模式,且V>V_r
//        float acct_1 = (V_max_1) / A_up_1;
//        float accd_1 = 0.5f * V_max_1 * acct_1;
//        float dcct_1 = (V_max_1 - V_r) / A_up_1;
//        float dccd_1 = 0.5f * (V_max_1 + V_r) * dcct_1;

//        float acct_2_y = (V_max_2_y - V_r) / A_up_2_y;
//        float accd_2_y = 0.5f * (V_max_2_y + V_r) * acct_2_y;
//        float dcct_2_y = (V_max_2_y) / A_up_2_y;
//        float dccd_2_y = 0.5f * V_max_2_y * dcct_2_y;

//        if (S1 < accd_1 + dccd_1)
//        {
//            V_1 = sqrt(A_up_1 * S1 + 0.5f * V_r * V_r);
//            V_max_1 = V_1;
//            t1 = V_1 / A_up_1;
//            t2 = 0;
//            t3 = (V_1 - V_r) / A_up_1;
//        }
//        else
//        {
//            t1 = acct_1;
//            t2 = (S1 - accd_1 - dccd_1) / V_max_1;
//            t3 = dcct_1;
//        }

//        t4 = PI / 2 * R / V_r;

//        if (DELTA_Y_2 < accd_2_y + dccd_2_y)
//        {
//            V_2 = sqrt(A_up_2_y * DELTA_Y_2 + 0.5f * V_r * V_r);
//            V_max_2_y = V_2;
//            t5_y = (V_2 - V_r) / A_up_2_y;
//            t6_y = 0;
//            t7_y = V_2 / A_up_2_y;
//        }
//        else
//        {
//            t5_y = acct_2_y;
//            t6_y = (DELTA_Y_2 - accd_2_y - dccd_2_y) / V_max_2_y;
//            t7_y = dcct_2_y;
//        }

//        t_2_y = t5_y + t6_y + t7_y;

//        V_max_2_x = 2 * DELTA_X_2 / t_2_y;
//        t_2_x_up = t_2_y / (1 + Kp_A_2_x);
//        t_2_x_down = Kp_A_2_x * t_2_y / (1 + Kp_A_2_x);
//        A_up_2_x = V_max_2_x / t_2_x_up;
//        A_down_2_x = A_up_2_x / Kp_A_2_x;

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//        flag_record = 0;
//    }

//    t_run = Ts * p_nav->auto_path.run_time;
//    if (t_run < t1)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_up_1 * t_run * t_run);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = A_up_1 * t_run;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//        // p_nav->auto_path.acceleration = A_up;
//    }
//    else if (t_run < t1 + t2)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_up_1 * t1 * t1) + V_max_1 * (t_run - t1);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = V_max_1;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        // p_nav->auto_path.acceleration  = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_up_1 * t1 * t1) + V_max_1 * t2 + V_max_1 *
//        (t_run - t1 - t2) - 0.5f * A_up_1 * (t_run - t1 - t2) * (t_run - t1 - t2);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = V_max_1 - A_up_1 * (t_run - t1 - t2);
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R * sinf(V_r * (t_run - t1 - t2 - t3) / R);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R - R * cosf(V_r * (t_run - t1 - t2 - t3) /
//        R); p_nav->auto_path.basic_velt.fpVx = V_r * cosf(V_r * (t_run - t1 - t2 - t3) / R);
//        p_nav->auto_path.basic_velt.fpVy = V_r * sinf(V_r * (t_run - t1 - t2 - t3) / R);
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4 + t_2_y)
//    {
//        if (t_run < t1 + t2 + t3 + t4 + t5_y)
//        {
//            p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R + V_r * (t_run - t1 - t2 - t3 - t4) +
//            0.5f * A_up_2_y * (t_run - t1 - t2 - t3 - t4) * (t_run - t1 - t2 - t3 - t4);
//            p_nav->auto_path.basic_velt.fpVy = V_r + A_up_2_y * (t_run - t1 - t2 - t3 - t4);
//        }
//        else if (t_run < t1 + t2 + t3 + t4 + t5_y + t6_y)
//        {
//            p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R + V_r * t5_y + 0.5f * A_up_2_y * t5_y
//            * t5_y + V_max_2_y * (t_run - t1 - t2 - t3 - t4 - t5_y); p_nav->auto_path.basic_velt.fpVy =
//            V_max_2_y;
//        }
//        else if (t_run < t1 + t2 + t3 + t4 + t5_y + t6_y + t7_y)
//        {
//            p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R + V_r * t5_y + 0.5f * A_up_2_y * t5_y
//            * t5_y + V_max_2_y * t6_y + V_max_2_y * (t_run - t1 - t2 - t3 - t4 - t5_y - t6_y) - 0.5f *
//            A_up_2_y * (t_run - t1 - t2 - t3 - t4 - t5_y - t6_y) * (t_run - t1 - t2 - t3 - t4 - t5_y -
//            t6_y); p_nav->auto_path.basic_velt.fpVy = V_max_2_y - A_up_2_y * (t_run - t1 - t2 - t3 - t4 -
//            t5_y - t6_y);
//        }

//        if (t_run < t1 + t2 + t3 + t4 + t_2_x_up)
//        {
//            p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R + 0.5f * A_up_2_x * (t_run - t1 - t2 -
//            t3 - t4) * (t_run - t1 - t2 - t3 - t4); p_nav->auto_path.basic_velt.fpVx = A_up_2_x * (t_run -
//            t1 - t2 - t3 - t4);
//        }
//        else if (t_run < t1 + t2 + t3 + t4 + t_2_x_up + t_2_x_down)
//        {
//            p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R + 0.5f * A_up_2_x * t_2_x_up *
//            t_2_x_up + V_max_2_x * (t_run - t1 - t2 - t3 - t4 - t_2_x_up) - 0.5f * A_down_2_x * (t_run - t1
//            - t2 - t3 - t4 - t_2_x_up) * (t_run - t1 - t2 - t3 - t4 - t_2_x_up);
//            p_nav->auto_path.basic_velt.fpVx = V_max_2_x - A_down_2_x * (t_run - t1 - t2 - t3 - t4 -
//            t_2_x_up);
//        }
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4 + t_2_y + 0.1f)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X;
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y;
//        p_nav->auto_path.basic_velt.fpVx = 0;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else
//    {
//        p_nav->auto_path.run_time = 0;
//        p_nav->state = NAV_STOP;

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//    }
//}

// float t_run_op;
////对称路径简化方法
////时间颠倒（位置颠倒），速度取反
// static void path_51(C_NAV *p_nav) //直线圆弧曲线
//{
//     static fp32 R;
//     static fp32 S1, S2;
//     static fp32 A_up_1, A_down_1, A_up_2_x, A_down_2_x, A_up_2_y, A_down_2_y;
//     //  static fp32 S_x,S_y;
//     static fp32 V_max_1, V_max_2_y, V_max_2_x, V_r, V_1, V_2;
//     static fp32 Kp_A_2_x;
//     if (flag_record) //初始化加速度，运动时间，位移etc
//     {
//         //		DELTA_X = 5320;
//         //		DELTA_Y = 2930 + times_path * 300;
//         //		DELTA_Y = 3230;

//        StartX = p_nav->auto_path.pos_pid.x.fpFB;
//        StartY = p_nav->auto_path.pos_pid.y.fpFB;
//        StartQ = p_nav->auto_path.pos_pid.w.fpFB;

//        if ((fabs(StartX - FIRST_BALL_X) < 250) && (fabs(StartY - FIRST_BALL_Y - (times_path - 1) * 300) <
//        250) && (fabs(StartQ) < 10))
//        {
//            StartX = FIRST_BALL_X;
//            StartY = FIRST_BALL_Y + (times_path - 1) * 300;
//            StartQ = 0;
//        }

//        DELTA_X = -(END_X - StartX);
//        DELTA_Y = -(END_Y - StartY);

//        //		A_up_1 = 3000;
//        //		A_down_1 = 3000;
//        A_up_1 = 2000;
//        A_down_1 = 2000;

//        if (StartY > 2000)
//        {
//            //			V_max_1 = 3000;//可更改
//            //			V_r = 2000;//可更改
//            //			V_max_2_y = 2500;
//            V_max_1 = 2000; //可更改
//            V_r = 1500;     //可更改
//            V_max_2_y = 1500;
//        }

//        DELTA_X_1 = -END_X + 4000;
//        DELTA_Y_1 = 0;
//        S1 = Geometric_mean(DELTA_X_1, DELTA_Y_1);

//        R = 1000;

//        Kp_A_2_x = 0.1f;
//        A_up_2_y = 3000;
//        A_down_2_y = 3000;
//        DELTA_X_2 = DELTA_X - R - DELTA_X_1;
//        DELTA_Y_2 = DELTA_Y - R - DELTA_Y_1;

//        StartX -= DELTA_X;
//        StartY -= DELTA_Y;

//        //目前用的加减速段加速度大小相同的模式,且V>V_r
//        float acct_1 = (V_max_1) / A_up_1;
//        float accd_1 = 0.5f * V_max_1 * acct_1;
//        float dcct_1 = (V_max_1 - V_r) / A_up_1;
//        float dccd_1 = 0.5f * (V_max_1 + V_r) * dcct_1;

//        float acct_2_y = (V_max_2_y - V_r) / A_up_2_y;
//        float accd_2_y = 0.5f * (V_max_2_y + V_r) * acct_2_y;
//        float dcct_2_y = (V_max_2_y) / A_up_2_y;
//        float dccd_2_y = 0.5f * V_max_2_y * dcct_2_y;

//        if (S1 < accd_1 + dccd_1)
//        {
//            V_1 = sqrt(A_up_1 * S1 + 0.5f * V_r * V_r);
//            V_max_1 = V_1;
//            t1 = V_1 / A_up_1;
//            t2 = 0;
//            t3 = (V_1 - V_r) / A_up_1;
//        }
//        else
//        {
//            t1 = acct_1;
//            t2 = (S1 - accd_1 - dccd_1) / V_max_1;
//            t3 = dcct_1;
//        }

//        t4 = PI / 2 * R / V_r;

//        if (DELTA_Y_2 < accd_2_y + dccd_2_y)
//        {
//            V_2 = sqrt(A_up_2_y * DELTA_Y_2 + 0.5f * V_r * V_r);
//            V_max_2_y = V_2;
//            t5_y = (V_2 - V_r) / A_up_2_y;
//            t6_y = 0;
//            t7_y = V_2 / A_up_2_y;
//        }
//        else
//        {
//            t5_y = acct_2_y;
//            t6_y = (DELTA_Y_2 - accd_2_y - dccd_2_y) / V_max_2_y;
//            t7_y = dcct_2_y;
//        }

//        t_2_y = t5_y + t6_y + t7_y;

//        V_max_2_x = 2 * DELTA_X_2 / t_2_y;
//        t_2_x_up = t_2_y / (1 + Kp_A_2_x);
//        t_2_x_down = Kp_A_2_x * t_2_y / (1 + Kp_A_2_x);
//        A_up_2_x = V_max_2_x / t_2_x_up;
//        A_down_2_x = A_up_2_x / Kp_A_2_x;

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//        flag_record = 0;
//    }

//    t_run = Ts * p_nav->auto_path.run_time;
//    t_run_op = t1 + t2 + t3 + t4 + t_2_y - t_run;

//    if (t_run_op < -0.1f)
//    {
//        p_nav->auto_path.run_time = 0;
//        p_nav->state = NAV_STOP;

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//    }
//    else if (t_run_op < 0)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX;
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = 0;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        p_nav->auto_path.basic_velt.fpVx = -p_nav->auto_path.basic_velt.fpVx;
//        p_nav->auto_path.basic_velt.fpVy = -p_nav->auto_path.basic_velt.fpVy;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run_op < t1)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_up_1 * t_run_op * t_run_op);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = A_up_1 * t_run_op;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        p_nav->auto_path.basic_velt.fpVx = -p_nav->auto_path.basic_velt.fpVx;
//        p_nav->auto_path.basic_velt.fpVy = -p_nav->auto_path.basic_velt.fpVy;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//        // p_nav->auto_path.acceleration = A_up;
//    }
//    else if (t_run_op < t1 + t2)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_up_1 * t1 * t1) + V_max_1 * (t_run_op - t1);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = V_max_1;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        p_nav->auto_path.basic_velt.fpVx = -p_nav->auto_path.basic_velt.fpVx;
//        p_nav->auto_path.basic_velt.fpVy = -p_nav->auto_path.basic_velt.fpVy;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run_op < t1 + t2 + t3)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_up_1 * t1 * t1) + V_max_1 * t2 + V_max_1 *
//        (t_run_op - t1 - t2) - 0.5f * A_up_1 * (t_run_op - t1 - t2) * (t_run_op - t1 - t2);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = V_max_1 - A_up_1 * (t_run_op - t1 - t2);
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        p_nav->auto_path.basic_velt.fpVx = -p_nav->auto_path.basic_velt.fpVx;
//        p_nav->auto_path.basic_velt.fpVy = -p_nav->auto_path.basic_velt.fpVy;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run_op < t1 + t2 + t3 + t4)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R * sinf(V_r * (t_run_op - t1 - t2 - t3) /
//        R); p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R - R * cosf(V_r * (t_run_op - t1 - t2 -
//        t3) / R); p_nav->auto_path.basic_velt.fpVx = V_r * cosf(V_r * (t_run_op - t1 - t2 - t3) / R);
//        p_nav->auto_path.basic_velt.fpVy = V_r * sinf(V_r * (t_run_op - t1 - t2 - t3) / R);
//        p_nav->auto_path.basic_velt.fpVx = -p_nav->auto_path.basic_velt.fpVx;
//        p_nav->auto_path.basic_velt.fpVy = -p_nav->auto_path.basic_velt.fpVy;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run_op < t1 + t2 + t3 + t4 + t_2_y)
//    {
//        if (t_run_op < t1 + t2 + t3 + t4 + t5_y)
//        {
//            p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R + V_r * (t_run_op - t1 - t2 - t3 - t4)
//            + 0.5f * A_up_2_y * (t_run_op - t1 - t2 - t3 - t4) * (t_run_op - t1 - t2 - t3 - t4);
//            p_nav->auto_path.basic_velt.fpVy = V_r + A_up_2_y * (t_run_op - t1 - t2 - t3 - t4);
//            p_nav->auto_path.basic_velt.fpVy = -p_nav->auto_path.basic_velt.fpVy;
//        }
//        else if (t_run_op < t1 + t2 + t3 + t4 + t5_y + t6_y)
//        {
//            p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R + V_r * t5_y + 0.5f * A_up_2_y * t5_y
//            * t5_y + V_max_2_y * (t_run_op - t1 - t2 - t3 - t4 - t5_y); p_nav->auto_path.basic_velt.fpVy =
//            V_max_2_y; p_nav->auto_path.basic_velt.fpVy = -p_nav->auto_path.basic_velt.fpVy;
//        }
//        else if (t_run_op < t1 + t2 + t3 + t4 + t5_y + t6_y + t7_y)
//        {
//            p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R + V_r * t5_y + 0.5f * A_up_2_y * t5_y
//            * t5_y + V_max_2_y * t6_y + V_max_2_y * (t_run_op - t1 - t2 - t3 - t4 - t5_y - t6_y) - 0.5f *
//            A_up_2_y * (t_run_op - t1 - t2 - t3 - t4 - t5_y - t6_y) * (t_run_op - t1 - t2 - t3 - t4 - t5_y -
//            t6_y); p_nav->auto_path.basic_velt.fpVy = V_max_2_y - A_up_2_y * (t_run_op - t1 - t2 - t3 - t4 -
//            t5_y - t6_y); p_nav->auto_path.basic_velt.fpVy = -p_nav->auto_path.basic_velt.fpVy;
//        }

//        if (t_run_op < t1 + t2 + t3 + t4 + t_2_x_up)
//        {
//            p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R + 0.5f * A_up_2_x * (t_run_op - t1 -
//            t2 - t3 - t4) * (t_run_op - t1 - t2 - t3 - t4); p_nav->auto_path.basic_velt.fpVx = A_up_2_x *
//            (t_run_op - t1 - t2 - t3 - t4); p_nav->auto_path.basic_velt.fpVx =
//            -p_nav->auto_path.basic_velt.fpVx;
//        }
//        else if (t_run_op < t1 + t2 + t3 + t4 + t_2_x_up + t_2_x_down)
//        {
//            p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R + 0.5f * A_up_2_x * t_2_x_up *
//            t_2_x_up + V_max_2_x * (t_run_op - t1 - t2 - t3 - t4 - t_2_x_up) - 0.5f * A_down_2_x * (t_run_op
//            - t1 - t2 - t3 - t4 - t_2_x_up) * (t_run_op - t1 - t2 - t3 - t4 - t_2_x_up);
//            p_nav->auto_path.basic_velt.fpVx = V_max_2_x - A_down_2_x * (t_run_op - t1 - t2 - t3 - t4 -
//            t_2_x_up); p_nav->auto_path.basic_velt.fpVx = -p_nav->auto_path.basic_velt.fpVx;
//        }
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//}

// static void path_52(C_NAV *p_nav) //直线圆弧直线变速圆弧
//{
//     static fp32 R, r;
//     static fp32 S1, S2, S_r, sr;
//     static fp32 A_up_1, A_down_1, A_up_2, A_down_2, A_r;
//     //  static fp32 S_x,S_y;
//     static fp32 V_max_1, V_max_2, V_R, V_r0, V_r, V_1, V_2;
//     if (flag_record) //初始化加速度，运动时间，位移etc
//     {
//         DELTA_X = 5220;
//         DELTA_Y = 3230 + times_path * 300;
//         times_path++;

//        A_up_1 = 3000;
//        A_down_1 = 3000;
//        V_max_1 = 2000;
//        DELTA_X_1 = 4000;
//        DELTA_Y_1 = 0;
//        S1 = Geometric_mean(DELTA_X_1, DELTA_Y_1);

//        V_R = 2000;
//        R = 1000;

//        V_r0 = 1000;
//        r = DELTA_X - R - DELTA_X_1;
//        S_r = PI / 2 * r;
//        A_r = V_r0 * V_r0 / 2 / S_r;

//        A_up_2 = 3000;
//        A_down_2 = 3000;
//        V_max_2 = 2000;
//        DELTA_Y_2 = DELTA_Y - R - DELTA_Y_1 - r;
//        DELTA_X_2 = 0;

//        StartX = p_nav->auto_path.pos_pid.x.fpFB;
//        StartY = p_nav->auto_path.pos_pid.y.fpFB;
//        StartQ = p_nav->auto_path.pos_pid.w.fpFB;

//        //目前用的加减速段加速度大小相同的模式,且V>V_R
//        float acct_1 = (V_max_1) / A_up_1;
//        float accd_1 = 0.5f * V_max_1 * acct_1;
//        float dcct_1 = (V_max_1 - V_R) / A_up_1;
//        float dccd_1 = 0.5f * (V_max_1 + V_R) * dcct_1;

//        float acct_2 = (V_max_2 - V_R) / A_up_2;
//        float accd_2 = 0.5f * (V_max_2 + V_R) * acct_2;
//        float dcct_2 = (V_max_2 - V_r0) / A_up_2;
//        float dccd_2 = 0.5f * (V_max_2 + V_r0) * dcct_2;

//        if (S1 < accd_1 + dccd_1)
//        {
//            V_1 = sqrt(A_up_1 * S1 + 0.5f * V_R * V_R);
//            V_max_1 = V_1;
//            t1 = V_1 / A_up_1;
//            t2 = 0;
//            t3 = (V_1 - V_R) / A_up_1;
//        }
//        else
//        {
//            t1 = acct_1;
//            t2 = (S1 - accd_1 - dccd_1) / V_max_1;
//            t3 = dcct_1;
//        }

//        t4 = PI / 2 * R / V_R;

//        if (DELTA_Y_2 < accd_2 + dccd_2)
//        {
//            V_2 = sqrt(A_up_2 * DELTA_Y_2 + 0.5f * V_r0 * V_r0 + 0.5f * V_R * V_R);
//            V_max_2 = V_2;
//            t5 = (V_2 - V_R) / A_up_2;
//            t6 = 0;
//            t7 = (V_2 - V_r0) / A_up_2;
//        }
//        else
//        {
//            t5 = acct_2;
//            t6 = (DELTA_Y_2 - accd_2 - dccd_2) / V_max_2;
//            t7 = dcct_2;
//        }

//        t8 = V_r0 / A_r;

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//        flag_record = 0;
//    }

//    t_run = Ts * p_nav->auto_path.run_time;
//    if (t_run < t1)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_up_1 * t_run * t_run);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = A_up_1 * t_run;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//        // p_nav->auto_path.acceleration = A_up;
//    }
//    else if (t_run < t1 + t2)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_up_1 * t1 * t1) + V_max_1 * (t_run - t1);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = V_max_1;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + (0.5f * A_up_1 * t1 * t1) + V_max_1 * t2 + V_max_1 *
//        (t_run - t1 - t2) - 0.5f * A_up_1 * (t_run - t1 - t2) * (t_run - t1 - t2);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY;
//        p_nav->auto_path.basic_velt.fpVx = V_max_1 - A_up_1 * (t_run - t1 - t2);
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R * sinf(V_R * (t_run - t1 - t2 - t3) / R);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R - R * cosf(V_R * (t_run - t1 - t2 - t3) /
//        R); p_nav->auto_path.basic_velt.fpVx = V_R * cosf(V_R * (t_run - t1 - t2 - t3) / R);
//        p_nav->auto_path.basic_velt.fpVy = V_R * sinf(V_R * (t_run - t1 - t2 - t3) / R);
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4 + t5)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R;
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R + V_R * (t_run - t1 - t2 - t3 - t4) + 0.5f
//        * A_up_2 * (t_run - t1 - t2 - t3 - t4) * (t_run - t1 - t2 - t3 - t4);
//        p_nav->auto_path.basic_velt.fpVx = 0;
//        p_nav->auto_path.basic_velt.fpVy = V_R + A_up_2 * (t_run - t1 - t2 - t3 - t4);
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4 + t5 + t6)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R;
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R + V_R * t5 + 0.5f * A_up_2 * t5 * t5 +
//        V_max_2 * (t_run - t1 - t2 - t3 - t4 - t5); p_nav->auto_path.basic_velt.fpVx = 0;
//        p_nav->auto_path.basic_velt.fpVy = V_max_2;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4 + t5 + t6 + t7)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R;
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R + V_R * t5 + 0.5f * A_up_2 * t5 * t5 +
//        V_max_2 * t6 + V_r0 * (t_run - t1 - t2 - t3 - t4 - t5 - t6) - 0.5f * A_up_2 * (t_run - t1 - t2 - t3
//        - t4 - t5 - t6) * (t_run - t1 - t2 - t3 - t4 - t5 - t6); p_nav->auto_path.basic_velt.fpVx = 0;
//        p_nav->auto_path.basic_velt.fpVy = V_max_2 - A_up_2 * (t_run - t1 - t2 - t3 - t4 - t5 - t6);
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4 + t5 + t6 + t7 + t8)
//    {
//        V_r = V_r0 - A_r * (t_run - t1 - t2 - t3 - t4 - t5 - t6 - t7);
//        sr = 0.5f * (V_r + V_r0) * (t_run - t1 - t2 - t3 - t4 - t5 - t6 - t7);
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X_1 + R + DELTA_X_2 + r - r * cosf(sr / r);
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y_1 + R + DELTA_Y_2 + r * sinf(sr / r);
//        p_nav->auto_path.basic_velt.fpVx = V_r * sinf(sr / r);
//        p_nav->auto_path.basic_velt.fpVy = V_r * cosf(sr / r);
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else if (t_run < t1 + t2 + t3 + t4 + t5 + t6 + t7 + t8 + 0.1f)
//    {
//        p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X;
//        p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y;
//        p_nav->auto_path.basic_velt.fpVx = 0;
//        p_nav->auto_path.basic_velt.fpVy = 0;
//        // p_nav->auto_path.acceleration    = 0;
//        p_nav->auto_path.pos_pid.w.fpDes = StartQ;
//        p_nav->auto_path.basic_velt.fpW = 0;
//    }
//    else
//    {
//        p_nav->auto_path.run_time = 0;
//        p_nav->state = NAV_STOP;

//        p_nav->auto_path.pos_pid.x.fpSumE = 0;
//        p_nav->auto_path.pos_pid.y.fpSumE = 0;
//        p_nav->auto_path.pos_pid.w.fpSumE = 0;
//    }
//}