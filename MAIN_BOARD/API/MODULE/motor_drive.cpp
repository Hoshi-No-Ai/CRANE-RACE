#include "motor_drive.h"

/*************************************************************************
函 数 名：GetEncoderNumber
函数功能：接收6623、C620/C610返回的机械角度值（绝对式编码器值）
备    注：机械角度值范围：0~8191（0x1FFF）
*************************************************************************/
int32_t C_Encoder::Get_Encoder_Number(CanRxMsg *rx_message)
{
    int32_t encoder_temp;
    encoder_temp = rx_message->Data[0] << 8 | rx_message->Data[1];
    return encoder_temp;
}

/************************************************************************
函 数 名：Get_Speed
函数功能：接收C620/C610返回的转速，单位：r/min
备    注：RM3508电机减速比为1：19；M2006电机减速比为1：36
*************************************************************************/
int32_t C_Encoder::Get_Speed(CanRxMsg *rx_message)
{
    int32_t speed_temp;
    if (rx_message->Data[2] & 0x01 << 7)
    {
        speed_temp = (0xFFFF << 16 | rx_message->Data[2] << 8 | rx_message->Data[3]);
    }
    else
        speed_temp = (rx_message->Data[2] << 8 | rx_message->Data[3]); // rpm
    return speed_temp;
}

/*************************************************************************
函 数 名：Encoder_Process
函数功能：有刷电机增量式编码器及RM3510电机绝对式编码器数据处理，得到转速
备    注：type:0--绝对式编码器；1--增量式编码器
*************************************************************************/
void C_Encoder::Encoder_Process(int32_t value, uint8_t type)
{
    static fp32 fpVeltCoff;
    siPreRawValue = siRawValue;
    siRawValue = value;
    siDiff = siRawValue - siPreRawValue;

    if (siDiff <
        -siNumber /
            2) // 两次编码器的反馈值差别太大,表示绝对式编码器圈数发生了改变或增量式编码器的定时器计数器向上溢出
    {
        (type == 0) ? (siDiff += (siNumber + 1))
                    : (siDiff += 65536); // 定时器16位计数器计数范围0-65535共65536个数
    }
    else if (
        siDiff >
        siNumber /
            2) // 两次编码器的反馈值差别太大,表示绝对式编码器圈数发生了改变或增量式编码器的定时器计数器向下溢出
    {
        (type == 0) ? (siDiff -= (siNumber + 1)) : (siDiff -= 65536);
    }

    if (type == 0)
    {
        fpVeltCoff = 60.0f / siGearRatio / siNumber / 0.001f; // 0.001是指两次采样间隔1ms
    }
    else
    {
        fpVeltCoff = 60.0f / siGearRatio / siNumber / 0.001f /
                     4.0f; // 0.001是指两次采样间隔1ms,4是指定时器编码器模式的4倍频
    }
    fpSpeed = fpVeltCoff * siDiff; // 单位：r/min
    siSumValue += siDiff;          // 记录编码器的总数，位置闭环用
}

/*************************************************************************
函 数 名：GetPosition
函数功能：获得大红的编码器累加值
备
注：此函数只计算了针对360绝对返回值的累加值，并没有返回转速等参数，因此仅适用于大红电机这种由电调返回转速值的情况
*************************************************************************/
// void C_Encoder::GetPosition(void) {
//     siDiff = siRawValue - siPreRawValue;
//     if (siDiff > 180) {
//         siDiff -= 360;
//     } else if (siDiff < -180) {
//         siDiff += 360;
//     }
//     siSumValue += siDiff;
//     siPreRawValue = siRawValue;
// }

void C_Motor::can_send_data(CAN_TypeDef *CANx, uint32_t StdID, int16_t ssMotor1, int16_t ssMotor2,
                            int16_t ssMotor3, int16_t ssMotor4)
{
    CanTxMsg tx_message;

    tx_message.StdId = StdID;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = ssMotor1 >> 8;
    tx_message.Data[1] = ssMotor1;
    tx_message.Data[2] = ssMotor2 >> 8;
    tx_message.Data[3] = ssMotor2;

    tx_message.Data[4] = ssMotor3 >> 8;
    tx_message.Data[5] = ssMotor3;
    tx_message.Data[6] = ssMotor4 >> 8;
    tx_message.Data[7] = ssMotor4;

    CAN_Transmit(CANx, &tx_message);
}
