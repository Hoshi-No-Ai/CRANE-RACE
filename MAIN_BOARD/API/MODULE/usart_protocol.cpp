#include "usart_protocol.h"

using _api_module_::calculate_flag;
using _api_module_::f_g_error;
using _api_module_::flag_tuoluo;

// 串口1：陀螺仪
double temp;

fp32 k_gyro = -0.0125;
double test_v;
void Comm1Rx_IRQ(void)
{
    int RATE;
    static int count;
    if (UA1RxMailbox[0] == 0xDD)
    {
        RATE = (UA1RxMailbox[2] << 24) | (UA1RxMailbox[1] << 8) | (UA1RxMailbox[3] << 16);
        RATE = RATE / 256;
        GyroData = ((double)RATE) *
                   2.980232238769531e-003; // GyroData = ((double )RATE)  * 2.5 / 8388608;陀螺仪解算
        gyroGz = GyroData - f_g_error;
        GyroLPF.input = gyroGz;
        Butterworth50HzLPF(&GyroLPF);
        if (calculate_flag)
        {
            if ((gyroGz > value_door) || (gyroGz < ((-1) * value_door)))
            {
                GyroTab[GyroCount] = Gyro_TableLookUp(gyroGz);
            }
            else
            {
                GyroTab[GyroCount] = 0;
            }
            Angel_v = GyroTab[GyroCount];
            GyroCount++;
            if (GyroCount == 0x03)
            {
                CalFlag = 0x01;
                GyroCount = 0x01;
            }

            if (CalFlag == 0x01)
            {
                Angel_d = 6.6666 * (GyroTab[0] + 4 * GyroTab[1] + GyroTab[2]) / 6.0 * 1e-2;
                if (abs(Angel_d) > 200)
                {
                    Angel_d = 0;
                }
                GyroTab[0] = GyroTab[2];
                RobAngel += Angel_d;
                CalFlag = 0x00;
            }
            if ((gyroGz > value_door) || (gyroGz < ((-1) * value_door)))
            {
                test_v = gyroGz * k_gyro;
                temp = 3.3333 * test_v * 1e-2;
                if (abs(temp) > 200)
                {
                    temp = 0;
                }
                TestAngle2 += temp;
            }
        }
        else
        {
            if (flag_tuoluo)
            {
                f_g_error += GyroData;
                count++;
            }
            if (count >= 3000)
            {
                f_g_error = f_g_error / 3000;
                calculate_flag = 1;
                count = 0;
            }
        }
    }
}

// SB
uart1_rx_protocol_t uart1_efr = {0x55, 0x52, 0x55, 0x53, UART1_RX_DATA_LEN1, UART1_RX_DATA_LEN2, {0}, {0}, 0x01, 0xAA};

uint8_t ucData1;
uint8_t uart1_data_buf1[UART1_RX_DATA_LEN1];
uint8_t uart1_data_buf2[UART1_RX_DATA_LEN2];
// void Comm1Rx_IRQ(void)
//{
//     static unsigned char Comm1_Rx_Status = RX_FREE; // 初始状态
//     static unsigned char ucPit = 0;                 // 数据字节计数
//     unsigned char i = 0;                            // 计数变量，遍历整个DMA数组

//   for (i = 0; i < UART1_RX_DATA_LEN1 + 2 + UART1_RX_DATA_LEN2 + 2;
//        ++i) // 遍历数组，数组大小i改变根据USART1_RXMB_LEN大小变化
//   {
//       ucData1 = UA1RxMailbox[i]; // 取出一个字节
//				// ucData1 = USART_ReceiveData(USART1);
//        /*********************************状态机解析数据包************************************/
//        switch (Comm1_Rx_Status)
//        {
//        case RX_FREE:
//            if (ucData1 == uart1_efr.start11)
//            {
//                Comm1_Rx_Status = RX_START_1; // 自由状态下接到0x55认为开始
//            }
//            break;

//        case RX_START_1:
//            //角速度
//            if (ucData1 == uart1_efr.start12)
//            {
//                Comm1_Rx_Status = RX_DATAS_1;
//            }
//            //角度
//            else if (ucData1 == uart1_efr.start22)
//            {
//                Comm1_Rx_Status = RX_DATAS_2;
//            }
//            else
//            {
//                Comm1_Rx_Status = RX_FREE;
//            }
//            break;

//        case RX_DATAS_1:
//            if (ucPit < uart1_efr.datanum1) // 如果没够数，存
//            {
//                *(uart1_data_buf1 + ucPit) = ucData1;
//                ucPit++;
//            }
//            if (ucPit == uart1_efr.datanum1) // 够数了判断0x00
//            {
//                ucPit = 0;

//                char test1 = 0x55 + 0x52 + uart1_data_buf1[2] + uart1_data_buf1[3] + uart1_data_buf1[4] +
//                                uart1_data_buf1[5];

//                if (test1 ==
//                    uart1_data_buf1[8])
//                {
//                    memcpy(uart1_efr.num1, uart1_data_buf1, UART1_RX_DATA_LEN1 * sizeof(uint8_t));
//                }
//                Comm1_Rx_Status = RX_FREE;
//            }
//            break;

//        case RX_DATAS_2:
//            if (ucPit < uart1_efr.datanum2) // 如果没够数，存
//            {
//                *(uart1_data_buf2 + ucPit) = ucData1;
//                ucPit++;
//            }
//            if(ucPit == uart1_efr.datanum2) // 够数了判断0x00
//            {
//                ucPit = 0;

//                char test2 = 0x55 + 0x53 + uart1_data_buf2[4] + uart1_data_buf2[5] + uart1_data_buf2[6] +
//                                uart1_data_buf2[7];

//                if (test2 ==
//                    uart1_data_buf2[8])
//                {
//                    memcpy(uart1_efr.num2, uart1_data_buf2, UART1_RX_DATA_LEN2 * sizeof(uint8_t));
//                }

//                Comm1_Rx_Status = RX_FREE;
//            }
//            break;

//        default:
//            break;
//        }
//   }
//}

/*
uart2:两主控间通讯
数据包格式：0x55 0x22 DATALEN 状态位[0] 指令位[1] 数据位[2-9] 0x01 0xAA
状态位：1正常，0出错
指令位：1捡箭，2捡箭复位，3递箭，4递箭复位，5干扰启动，6干扰停止
*/

uart3_tx_protocol_t uart3_eft = {0x55, 0x22, UART3_TX_DATA_LEN, {0}, 0x01, 0xAA};

uart3_rx_protocol_t uart3_efr = {0x55, 0x22, UART3_RX_DATA_LEN, {0}, 0x01, 0xAA};

// int count_test;

void USART3_DMA_Tx(void) // 串口2 DMA发送函数
{
    DMA_ClearITPendingBit(
        USART3_TX_STREAM,
        DMA_IT_TCIF3 /*DMA_IT_TCIF6*/); // 和stream对应
                                        ////开启DMA_Mode_Normal,即便没有使用完成中断也要软件清除，否则只发一次

    DMA_Cmd(USART3_TX_STREAM, DISABLE); // 设置当前计数值前先禁用DMA
    uart3_eft.tail1 = 0x01;
    uart3_eft.tail2 = 0xAA;
    USART3_TX_STREAM->M0AR = (uint32_t)&uart3_eft; // 设置当前待发数据基地址:Memory0 tARget
    USART3_TX_STREAM->NDTR =
        (uint32_t)sizeof(uart3_eft); // 设置当前待发的数据的数量:Number of Data units to be TRansferred
    DMA_Cmd(USART3_TX_STREAM, ENABLE);
    while (/*count_test=*/DMA_GetCurrDataCounter(USART3_TX_STREAM))
        ;
}

uint8_t ucData3;
/*串口2相关的缓存区*/
uint8_t uart3_data_buf[UART3_RX_DATA_LEN];
int dist_1, dist_2;
int dist_1_buff[5];
int dist_2_buff[5];

int temp_target_detect;
extern float pre_motor_sucker;
extern float velt_sucker;
extern float cal_distance_by_sensor;
extern int16_t dt35_1,dt35_2;
void Comm3Rx_IRQ(void) // 串口2电流DMA接收函数
{
    static unsigned char Comm3_Rx_Status = RX_FREE; // 初始状态
    static unsigned char ucPit = 0;                 // 数据字节计数
    unsigned char i = 0;                            // 计数变量，遍历整个DMA数组

    for (i = 0; i < UART3_RX_DATA_LEN + 5; ++i) // 遍历数组，数组大小i改变根据USART1_RXMB_LEN大小变化
    {
        ucData3 = UA3RxMailbox[i]; // 取出一个字节
        /*********************************状态机解析数据包************************************/
        switch (Comm3_Rx_Status)
        {
        case RX_FREE:
            if (ucData3 == uart3_efr.start1)
            {
                Comm3_Rx_Status = RX_START_1; // 自由状态下接到0x55认为开始
            }
            break;

        case RX_START_1:
            if (ucData3 == uart3_efr.start2)
            {
                Comm3_Rx_Status = RX_START_2;
            }
            else
            {
                Comm3_Rx_Status = RX_FREE;
            }
            break;

        case RX_START_2:
            if (ucData3 == uart3_efr.datanum)
            {
                Comm3_Rx_Status = RX_DATAS;
            }
            else
            {
                Comm3_Rx_Status = RX_FREE;
            }
            break;

        case RX_DATAS:
            if (ucPit < uart3_efr.datanum) // 如果没够数，存
            {
                *(uart3_data_buf + ucPit) = ucData3;
                ucPit++;
            }
            else // 够数了判断0x00
            {
                ucPit = 0;
                if (ucData3 == uart3_efr.tail1)
                {
                    Comm3_Rx_Status = RX_TAIL_1;
                    pre_motor_sucker = sucker.lift_motor.pos_pid.fpFB;
                    memcpy(&sucker.slide_motor.pos_pid.fpFB, uart3_efr.num, 4);
                    memcpy(&sucker.lift_motor.pos_pid.fpFB, &uart3_efr.num[4], 4);
                    velt_sucker = (sucker.lift_motor.pos_pid.fpFB - pre_motor_sucker) / 0.001;

//                    memcpy(&dist_1, &uart3_efr.num[8], 4);

//                    memcpy(&dist_2, &uart3_efr.num[12], 4);
////                    dist_1 += 128;
//                    dist_2 += 42;
										dist_1 = dt35_1;
									dist_2 =dt35_2;
                    if (dist_1 > 250 || dist_2 > 250)
                    {
                        temp_target_detect = 2;
                    }
                    else
                    {
                        temp_target_detect = 1;
                        cal_distance_by_sensor = 0.1047 * (dist_1 + dist_2) / 2 - 20.33;
                        if (cal_distance_by_sensor > 0)
                            cal_distance_by_sensor = 0;
                        if (cal_distance_by_sensor < -10)
                            cal_distance_by_sensor = -12;
                    }
                }
                else
                {
                    Comm3_Rx_Status = RX_FREE;
                }
            }
            break;

        case RX_TAIL_1:
        {
            if (ucData3 == uart3_efr.tail2) // 如果接到了0xAA，数据有效
            {
                memcpy(uart3_efr.num, uart3_data_buf, UART3_RX_DATA_LEN);
            }
            Comm3_Rx_Status = RX_FREE;
        }
        break;
        default:
            break;
        }
    }
}

uart6_tx_remote_ctrl_t uart6_remote_ctrl_eft = {0xAA, 0xFE, 0x55, 0x00, 0x55, 0x22, REMOTE_CTRL, {0}, 0xBB, 0xAA};

uart6_tx_robot_t uart6_robot_eft = {0xAA, 0xFE, 0x55, 0xFF, ROBOT, {0}, 0x01, 0x03};

rx_protocol_t uart6_efr = {0x55, 0x22, REMOTE_CTRL, USART6_RX_DATA_LEN, {0}, {0}, 0xBB, 0xAA};

uint8_t robot_socket_num = 0XFF, remote_ctrl_socket_num = 0xFF, temp_socket_num;
void USART6_DMA_Tx(wifi_rx_id_e device) // 串口2 DMA发送函数
{
    DMA_ClearITPendingBit(
        USART6_TX_STREAM,
        DMA_IT_TCIF6);                  // 开启DMA_Mode_Normal,即便没有使用完成中断也要软件清除，否则只发一次
    DMA_Cmd(USART6_TX_STREAM, DISABLE); // 设置当前计数值前先禁用DMA
    if (device == REMOTE_CTRL)
    {
        uart6_remote_ctrl_eft.ID = REMOTE_CTRL;
        uart6_remote_ctrl_eft.wifi_start4 = remote_ctrl_socket_num;
        uart6_remote_ctrl_eft.tail1 = 0xBB;
        uart6_remote_ctrl_eft.tail2 = 0xAA;
        USART6_TX_STREAM->M0AR = (uint32_t)&uart6_remote_ctrl_eft;
        USART6_TX_STREAM->NDTR = (uint32_t)sizeof(uart6_remote_ctrl_eft);
    }
    else if (device == ROBOT)
    {
        uart6_robot_eft.ID = ROBOT;
        uart6_robot_eft.wifi_start4 = robot_socket_num;
        uart6_robot_eft.tail2 = 0x03;
        USART6_TX_STREAM->M0AR = (uint32_t)&uart6_robot_eft;
        USART6_TX_STREAM->NDTR = (uint32_t)sizeof(uart6_robot_eft);
    }
    DMA_Cmd(USART6_TX_STREAM, ENABLE);
    while (DMA_GetCurrDataCounter(USART6_TX_STREAM))
        ;
}

uint8_t ucData6;
/*串口4相关的缓存区*/
uint8_t rx_rate;
uint8_t real_rx_rate;
u8 RX_COUNT = 0;
uint8_t uart6_data_buf[USART6_RX_DATA_LEN];
void Comm6Rx_IRQ(void) // 串口2电流DMA接收函数
{
    static unsigned char Comm6_Rx_Status = RX_FREE; // 初始状态
                                                    // 计数变量，遍历整个DMA数组
    ucData6 = USART_ReceiveData(USART6);            // 取出一个字节

    /*********************************状态机解析数据包************************************/
    switch (Comm6_Rx_Status)
    {
    case RX_FREE:
        if (ucData6 == uart6_efr.start1)
        {
            Comm6_Rx_Status = RX_START_1; // 自由状态下接到0x55认为开始
        }
        break;

    case RX_START_1:
        if (ucData6 == uart6_efr.start2)
        {
            Comm6_Rx_Status = RX_START_2;
        }
        else
        {
            Comm6_Rx_Status = RX_FREE;
        }
        break;

    case RX_START_2:
        if (ucData6 == uart6_efr.ID)
        {
            Comm6_Rx_Status = RX_DATAS_1;
        }
        else
        {
            Comm6_Rx_Status = RX_FREE;
        }
        break;

    case RX_DATAS_1:
        if (ucData6 == uart6_efr.datanum)
        {
            Comm6_Rx_Status = RX_DATAS;
        }
        break;

    case RX_DATAS:
        if (ucData6 != uart6_efr.tail1)
        {
            uart6_data_buf[RX_COUNT] = ucData6;
            RX_COUNT++;
            if (RX_COUNT > USART_REC_LEN)
            {
                Comm6_Rx_Status = RX_FREE;
                RX_COUNT = 0;
            }
        }
        else
        {
            Comm6_Rx_Status = RX_TAIL_1;
        }
        break;

    case RX_TAIL_1:
    {
        if (ucData6 == uart6_efr.tail2) // 如果接到了0xAA，数据有效
        {
            memcpy(uart6_efr.num, uart6_data_buf, RX_COUNT);
        }
        RX_COUNT = 0;
        Comm6_Rx_Status = RX_FREE;
    }
    break;
    default:
        break;
    }
}

uint16_t USART_Receive(USART_RX_TypeDef *USARTx)
{
    USARTx->rxConter = USARTx->DMALen - DMA_GetCurrDataCounter(USARTx->DMAy_Streamx);

    USARTx->rxBufferPtr += USARTx->rxSize;

    if (USARTx->rxBufferPtr >= USARTx->DMALen)
    {
        USARTx->rxBufferPtr %= USARTx->DMALen;
    }

    if (USARTx->rxBufferPtr < USARTx->rxConter)
    {
        USARTx->rxSize = USARTx->rxConter - USARTx->rxBufferPtr;
        if (USARTx->rxSize <= USARTx->MbLen)
        {
            for (uint16_t i = 0; i < USARTx->rxSize; ++i)
            {
                *(USARTx->pMailbox + i) = *(USARTx->pDMAbuf + USARTx->rxBufferPtr + i);
            }
        }
    }
    else
    {
        USARTx->rxSize = USARTx->rxConter + USARTx->DMALen - USARTx->rxBufferPtr;
        if (USARTx->rxSize <= USARTx->MbLen)
        {
            for (uint16_t i = 0; i < USARTx->rxSize - USARTx->rxConter; ++i)
            {
                *(USARTx->pMailbox + i) = *(USARTx->pDMAbuf + USARTx->rxBufferPtr + i);
            }
            for (uint16_t i = 0; i < USARTx->rxConter; ++i)
            {
                *(USARTx->pMailbox + USARTx->rxSize - USARTx->rxConter + i) = *(USARTx->pDMAbuf + i);
            }
        }
    }
    return USARTx->rxSize;
}

uart2_tx_protocol_t uart2_eft = {0x55,
                                 0x00,
                                 0x09,
                                 UART2_TX_DATA_LEN, // 现在是1
                                 {0},
                                 0x00,
                                 0xAA};

u8 data_len_float = COM_LENGTH;
uart2_rx_protocol_t uart2_efr = {0x55, 0x00, 0x04, data_len_float, UART2_RX_DATA_LEN, {0}, 0x00, 0xAA};

// uart2_rx_protocol_t uart2_efr_1 = {0x55, 0x00, 0x11, data_len_float, UART2_RX_DATA_LEN, {0}, 0x00, 0xAA};

aruco aruco_fdb;

u8 usart2_cnt;
uint8_t ucData2;
uint8_t uart2_data_buf[UART2_RX_DATA_LEN];
uint8_t dt35_data_buf[DT35_NUM];

void UART2_DMA_Tx(void)
{
    DMA_ClearITPendingBit(
        USART2_TX_STREAM,
        DMA_IT_TCIF6); // 开启DMA_Mode_Normal,即便没有使用完成中断也要软件清除，否则只发一次

    DMA_Cmd(USART2_TX_STREAM, DISABLE); // 设置当前计数值前先禁用DMA
    uart2_eft.tail1 = 0x00;
    uart2_eft.tail2 = 0xAA;
    USART2_TX_STREAM->M0AR = (uint32_t)&uart2_eft; // 设置当前待发数据基地址:Memory0 tARget
    USART2_TX_STREAM->NDTR =
        (uint32_t)sizeof(uart2_eft); // 设置当前待发的数据的数量:Number of Data units to be TRansferred
    DMA_Cmd(USART2_TX_STREAM, ENABLE);
    while (DMA_GetCurrDataCounter(USART2_TX_STREAM))
        ;
}

void Comm2Rx_IRQ(void) // 串口6 DMA接收函数
{
    static unsigned char Comm2_Rx_Status = RX_FREE; // 初始状态
    static unsigned char ucPit = 0;                 // 数据字节计数
    static unsigned char ucdt35 = 0;
    unsigned char i = 0; // 计数变量，遍历整个DMA数组
    usart2_cnt++;
    for (i = 0; i < UART2_RX_DATA_LEN + 6; ++i) // 遍历数组，数组大小i改变根据USART1_RXMB_LEN大小变化
    {
        ucData2 = UA2RxMailbox[i]; // 取出一个字节
        /*********************************状态机解析数据包************************************/
        switch (Comm2_Rx_Status)
        {
        case RX_FREE:
            if (ucData2 == uart2_efr.start1)
            {
                Comm2_Rx_Status = RX_START_1; // 自由状态下接到0x55认为开始
            }
            break;

        case RX_START_1:
            if (ucData2 == uart2_efr.start2)
            {
                Comm2_Rx_Status = RX_START_2;
            }
            else
            {
                Comm2_Rx_Status = RX_FREE;
            }
            break;

        case RX_START_2:
            if (ucData2 == uart2_efr.ID)
            {
                Comm2_Rx_Status = RX_START_3;
            }
            else
            {
                Comm2_Rx_Status = RX_FREE;
            }
            break;
        case RX_START_3:
            if (ucData2 == uart2_efr.data_len_float)
            {
                Comm2_Rx_Status = RX_DATAS;
            }
            else
            {
                Comm2_Rx_Status = RX_FREE;
            }
            break;
        case RX_DATAS:
            if (ucPit < uart2_efr.datanum) // 如果没够数，存
            {
                *(uart2_data_buf + ucPit) = ucData2;
                ucPit++;
            }
            else // 够数了判断0x00
            {
                ucPit = 0;
                if (ucData2 == uart2_efr.tail1)
                {
                    Comm2_Rx_Status = RX_TAIL_1;
                }
                else
                {
                    Comm2_Rx_Status = RX_FREE;
                }
            }
            break;

        case RX_TAIL_1:
            if (ucData2 == uart2_efr.tail2) // 如果接到了0xAA，数据有效
            {
                memcpy(uart2_efr.num, uart2_data_buf, UART2_RX_DATA_LEN);
                memcpy(&aruco_fdb, uart2_efr.num, 4 * 7);
								aruco_fdb.x=-aruco_fdb.x;
								aruco_fdb.y=-aruco_fdb.y;
								aruco_fdb.thetaz=-aruco_fdb.thetaz;
            }
            Comm2_Rx_Status = RX_FREE;
            break;

        default:
            break;
        }
    }
}