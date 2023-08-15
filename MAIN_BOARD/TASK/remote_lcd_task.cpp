#include "remote_lcd_task.h"
#include "action_task.h"
extern GET_NUM target_num;
extern float cal_distance_by_sensor;
extern DesSet DES;
extern float cal_sssssssss;

static void wifi_rxdata_update(void)
{
    uart6_tx_buf[0].float_num = Sys_Monitor.system_state;
    uart6_tx_buf[1].float_num = Sys_Monitor.system_error;

    uart6_tx_buf[2].float_num = keyboard_mode;
    uart6_tx_buf[3].float_num = nav.state;

    uart6_tx_buf[6].float_num = cRobot.stPot.fpPosX;
    uart6_tx_buf[7].float_num = cRobot.stPot.fpPosY;
    uart6_tx_buf[8].float_num = cRobot.stPot.fpPosQ;

    uart6_tx_buf[15].float_num = cRobot.cFollowoerWheel.m_CoderACur;
    uart6_tx_buf[16].float_num = cRobot.cFollowoerWheel.m_CoderBCur;
    uart6_tx_buf[17].float_num = table.slide_motor1.pos_pid.fpDes;

    uart6_tx_buf[18].float_num = delta_fb_aruco_w.delta_x;//delta_fb_aruco_w.delta_x/*delta_fb_des.delta_x/*dist_1*/;
    uart6_tx_buf[19].float_num = delta_fb_aruco_w.delta_y;//delta_fb_aruco_w.delta_y/*delta_fb_des.delta_y/*dist_2*/;
    uart6_tx_buf[20].float_num = aruco_fdb.thetaz;
    uart6_tx_buf[21].float_num = table.lift_motor1.pid_current;
}

void lcd_display(void)
{
    wifi_rxdata_update();
    memcpy(uart6_remote_ctrl_eft.num, uart6_tx_buf, 4 * 26 * sizeof(uint8_t)); // 要随着上面发送的变量数量改变
    USART6_DMA_Tx(REMOTE_CTRL);
}
