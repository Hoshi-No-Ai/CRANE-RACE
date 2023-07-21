#include "air_operated_board.h"

class C_AirOperation AirOperation(2);

/********************************************************************************************************
气动板有关函数
*********************************************************************************************************/
/**************************************************************
函数功能: 通过CAN发送气缸的变量
输入: pAir 要控制的气缸ID
输出：无
备注: 小气动板0-15，大气动板16-23，对应位至1为打开气缸
***************************************************************/
void C_AirOperation::SendAirMsgByCan(void) {
    static CanTxMsg TxMessage = {CAN_AIR_ID, 0x00, CAN_ID_STD, CAN_RTR_DATA, 4, 0, 0, 0, 0, 0, 0, 0, 0};
    *((uint32_t *)TxMessage.Data) = uiAirValve;  //达到类似memcpy的效果

    if (Can_x == 1) {
        CAN_Transmit(CAN1, &TxMessage);
    } else if (Can_x == 2) {
        CAN_Transmit(CAN2, &TxMessage);
    }
}

/**************************************************************
函数名:Send_LED_Mode(uint8_t mode)
函数功能: 全彩LED控制
输入: mode LED闪烁模式
输出：无
备注: mode :0-3    3:red  2:green 1:blue  0:默认呼吸灯
***************************************************************/
void C_AirOperation::Send_LED_Mode(uint8_t mode) {
    static CanTxMsg TxMessage = {0x50, 0x00, CAN_ID_STD, CAN_RTR_DATA, 1, 0, 0, 0, 0, 0, 0, 0, 0};
    TxMessage.Data[0] = mode;

    if (Can_x == 1) {
        CAN_Transmit(CAN1, &TxMessage);
    } else if (Can_x == 2) {
        CAN_Transmit(CAN2, &TxMessage);
    }
}

/**************************************************************
函数功能: 通过CAN发送舵机的变量
输入: chan 要控制的舵机ID
      value 舵机信号的数值
输出：无
备注:
***************************************************************/
void C_AirOperation::SendServoMsgByCan(uint8_t chan, uint8_t value) {
    static CanTxMsg TxMessage = {CAN_SERVO_ID, 0x00, CAN_ID_STD, CAN_RTR_DATA, 3, 0, 0, 0, 0, 0, 0, 0, 0};
    static uint8_t s_aucLastPwm[4] = {150, 150, 150, 150};
    if (s_aucLastPwm[chan] != value)  //值改变时才发送
    {
        s_aucLastPwm[chan] = value;
        TxMessage.Data[0] = 0;
        TxMessage.Data[1] = chan;
        TxMessage.Data[2] = value;

        if (Can_x == 1) {
            CAN_Transmit(CAN1, &TxMessage);
        } else if (Can_x == 2) {
            CAN_Transmit(CAN2, &TxMessage);
        }
    }
}

void C_AirOperation::SendServoMsgByCan_Plus(s16 value1, s16 value2, s16 value3, s16 value4) {
    static CanTxMsg TxMessage = {CAN_SERVO_ID, 0x00, CAN_ID_STD, CAN_RTR_DATA, 8, 0, 0, 0, 0, 0, 0, 0, 0};

    memcpy(TxMessage.Data, &value1, 2 * sizeof(uint8_t));
    memcpy(TxMessage.Data + 2, &value2, 2 * sizeof(uint8_t));
    memcpy(TxMessage.Data + 4, &value3, 2 * sizeof(uint8_t));
    memcpy(TxMessage.Data + 6, &value4, 2 * sizeof(uint8_t));

    if (Can_x == 1) {
        CAN_Transmit(CAN1, &TxMessage);
    } else if (Can_x == 2) {
        CAN_Transmit(CAN2, &TxMessage);
    }
}

void C_AirOperation::open_value(int channel) {
    uiAirValve |= (0x01 << (channel - 1));
    SendAirMsgByCan();
}

void C_AirOperation::close_value(int channel) {
    uiAirValve &= (~(0x01 << (channel - 1)));
    SendAirMsgByCan();
}

void C_AirOperation::change_value(int channel) {
    uiAirValve ^= ((uint32_t)(0x01 << (channel - 1)));
    SendAirMsgByCan();
}

void C_AirOperation::set_value_value(uint32_t value) {
    uiAirValve = value;
    SendAirMsgByCan();
}

void C_AirOperation::Update_Switch(CanRxMsg &RxMsg) {
    usSwitchPre = usSwitch;
    UpdateSwitchValue(RxMsg);
}
