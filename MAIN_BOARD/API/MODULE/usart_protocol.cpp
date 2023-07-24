#include "usart_protocol.h"

using _api_module_::calculate_flag;
using _api_module_::f_g_error;
using _api_module_::flag_tuoluo;

// ����1��������
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
                   2.980232238769531e-003; // GyroData = ((double )RATE)  * 2.5 / 8388608;�����ǽ���
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

// SBά��
uart1_rx_protocol_t uart1_efr = {0x55, 0x52, 0x55, 0x53, UART1_RX_DATA_LEN1, UART1_RX_DATA_LEN2, {0}, {0}, 0x01, 0xAA};

uint8_t ucData1;
uint8_t uart1_data_buf1[UART1_RX_DATA_LEN1];
uint8_t uart1_data_buf2[UART1_RX_DATA_LEN2];
// void Comm1Rx_IRQ(void)
//{
//     static unsigned char Comm1_Rx_Status = RX_FREE; // ��ʼ״̬
//     static unsigned char ucPit = 0;                 // �����ֽڼ���
//     unsigned char i = 0;                            // ������������������DMA����

//   for (i = 0; i < UART1_RX_DATA_LEN1 + 2 + UART1_RX_DATA_LEN2 + 2;
//        ++i) // �������飬�����Сi�ı����USART1_RXMB_LEN��С�仯
//   {
//       ucData1 = UA1RxMailbox[i]; // ȡ��һ���ֽ�
//				// ucData1 = USART_ReceiveData(USART1);
//        /*********************************״̬���������ݰ�************************************/
//        switch (Comm1_Rx_Status)
//        {
//        case RX_FREE:
//            if (ucData1 == uart1_efr.start11)
//            {
//                Comm1_Rx_Status = RX_START_1; // ����״̬�½ӵ�0x55��Ϊ��ʼ
//            }
//            break;

//        case RX_START_1:
//            //���ٶ�
//            if (ucData1 == uart1_efr.start12)
//            {
//                Comm1_Rx_Status = RX_DATAS_1;
//            }
//            //�Ƕ�
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
//            if (ucPit < uart1_efr.datanum1) // ���û��������
//            {
//                *(uart1_data_buf1 + ucPit) = ucData1;
//                ucPit++;
//            }
//            if (ucPit == uart1_efr.datanum1) // �������ж�0x00
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
//            if (ucPit < uart1_efr.datanum2) // ���û��������
//            {
//                *(uart1_data_buf2 + ucPit) = ucData1;
//                ucPit++;
//            }
//            if(ucPit == uart1_efr.datanum2) // �������ж�0x00
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
uart2:�����ؼ�ͨѶ
���ݰ���ʽ��0x55 0x22 DATALEN ״̬λ[0] ָ��λ[1] ����λ[2-9] 0x01 0xAA
״̬λ��1������0����
ָ��λ��1�����2�����λ��3�ݼ���4�ݼ���λ��5����������6����ֹͣ
*/

uart3_tx_protocol_t uart3_eft = {0x55, 0x22, UART3_TX_DATA_LEN, {0}, 0x01, 0xAA};

uart3_rx_protocol_t uart3_efr = {0x55, 0x22, UART3_RX_DATA_LEN, {0}, 0x01, 0xAA};

// int count_test;

void USART3_DMA_Tx(void) // ����2 DMA���ͺ���
{
    DMA_ClearITPendingBit(
        USART3_TX_STREAM,
        DMA_IT_TCIF3 /*DMA_IT_TCIF6*/); // ��stream��Ӧ
                                        ////����DMA_Mode_Normal,����û��ʹ������ж�ҲҪ������������ֻ��һ��

    DMA_Cmd(USART3_TX_STREAM, DISABLE); // ���õ�ǰ����ֵǰ�Ƚ���DMA
    uart3_eft.tail1 = 0x01;
    uart3_eft.tail2 = 0xAA;
    USART3_TX_STREAM->M0AR = (uint32_t)&uart3_eft; // ���õ�ǰ�������ݻ���ַ:Memory0 tARget
    USART3_TX_STREAM->NDTR =
        (uint32_t)sizeof(uart3_eft); // ���õ�ǰ���������ݵ�����:Number of Data units to be TRansferred
    DMA_Cmd(USART3_TX_STREAM, ENABLE);
    while (/*count_test=*/DMA_GetCurrDataCounter(USART3_TX_STREAM))
        ;
}

uint8_t ucData3;
/*����2��صĻ�����*/
uint8_t uart3_data_buf[UART3_RX_DATA_LEN];

void Comm3Rx_IRQ(void) // ����2����DMA���պ���
{
    static unsigned char Comm3_Rx_Status = RX_FREE; // ��ʼ״̬
    static unsigned char ucPit = 0;                 // �����ֽڼ���
    unsigned char i = 0;                            // ������������������DMA����

    for (i = 0; i < UART3_RX_DATA_LEN + 5; ++i) // �������飬�����Сi�ı����USART1_RXMB_LEN��С�仯
    {
        ucData3 = UA3RxMailbox[i]; // ȡ��һ���ֽ�
        /*********************************״̬���������ݰ�************************************/
        switch (Comm3_Rx_Status)
        {
        case RX_FREE:
            if (ucData3 == uart3_efr.start1)
            {
                Comm3_Rx_Status = RX_START_1; // ����״̬�½ӵ�0x55��Ϊ��ʼ
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
            if (ucPit < uart3_efr.datanum) // ���û��������
            {
                *(uart3_data_buf + ucPit) = ucData3;
                ucPit++;
            }
            else // �������ж�0x00
            {
                ucPit = 0;
                if (ucData3 == uart3_efr.tail1)
                {
                    Comm3_Rx_Status = RX_TAIL_1;
                }
                else
                {
                    Comm3_Rx_Status = RX_FREE;
                }
            }
            break;

        case RX_TAIL_1:
        {
            if (ucData3 == uart3_efr.tail2) // ����ӵ���0xAA��������Ч
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
void USART6_DMA_Tx(wifi_rx_id_e device) // ����2 DMA���ͺ���
{
    DMA_ClearITPendingBit(
        USART6_TX_STREAM,
        DMA_IT_TCIF6);                  // ����DMA_Mode_Normal,����û��ʹ������ж�ҲҪ������������ֻ��һ��
    DMA_Cmd(USART6_TX_STREAM, DISABLE); // ���õ�ǰ����ֵǰ�Ƚ���DMA
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
/*����4��صĻ�����*/
uint8_t rx_rate;
uint8_t real_rx_rate;
u8 RX_COUNT = 0;
uint8_t uart6_data_buf[USART6_RX_DATA_LEN];
void Comm6Rx_IRQ(void) // ����2����DMA���պ���
{
    static unsigned char Comm6_Rx_Status = RX_FREE; // ��ʼ״̬
                                                    // ������������������DMA����
    ucData6 = USART_ReceiveData(USART6);            // ȡ��һ���ֽ�

    /*********************************״̬���������ݰ�************************************/
    switch (Comm6_Rx_Status)
    {
    case RX_FREE:
        if (ucData6 == uart6_efr.start1)
        {
            Comm6_Rx_Status = RX_START_1; // ����״̬�½ӵ�0x55��Ϊ��ʼ
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
        if (ucData6 == uart6_efr.tail2) // ����ӵ���0xAA��������Ч
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
                                 UART2_TX_DATA_LEN, // ������1
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
        DMA_IT_TCIF6); // ����DMA_Mode_Normal,����û��ʹ������ж�ҲҪ������������ֻ��һ��

    DMA_Cmd(USART2_TX_STREAM, DISABLE); // ���õ�ǰ����ֵǰ�Ƚ���DMA
    uart2_eft.tail1 = 0x00;
    uart2_eft.tail2 = 0xAA;
    USART2_TX_STREAM->M0AR = (uint32_t)&uart2_eft; // ���õ�ǰ�������ݻ���ַ:Memory0 tARget
    USART2_TX_STREAM->NDTR =
        (uint32_t)sizeof(uart2_eft); // ���õ�ǰ���������ݵ�����:Number of Data units to be TRansferred
    DMA_Cmd(USART2_TX_STREAM, ENABLE);
    while (DMA_GetCurrDataCounter(USART2_TX_STREAM))
        ;
}

void Comm2Rx_IRQ(void) // ����6 DMA���պ���
{
    static unsigned char Comm2_Rx_Status = RX_FREE; // ��ʼ״̬
    static unsigned char ucPit = 0;                 // �����ֽڼ���
    static unsigned char ucdt35 = 0;
    unsigned char i = 0; // ������������������DMA����
    usart2_cnt++;
    for (i = 0; i < UART2_RX_DATA_LEN + 6; ++i) // �������飬�����Сi�ı����USART1_RXMB_LEN��С�仯
    {
        ucData2 = UA2RxMailbox[i]; // ȡ��һ���ֽ�
        /*********************************״̬���������ݰ�************************************/
        switch (Comm2_Rx_Status)
        {
        case RX_FREE:
            if (ucData2 == uart2_efr.start1)
            {
                Comm2_Rx_Status = RX_START_1; // ����״̬�½ӵ�0x55��Ϊ��ʼ
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
            if (ucPit < uart2_efr.datanum) // ���û��������
            {
                *(uart2_data_buf + ucPit) = ucData2;
                ucPit++;
            }
            else // �������ж�0x00
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
            if (ucData2 == uart2_efr.tail2) // ����ӵ���0xAA��������Ч
            {
                memcpy(uart2_efr.num, uart2_data_buf, UART2_RX_DATA_LEN);
                memcpy(&aruco_fdb, uart2_efr.num, 4*7);
            }
            Comm2_Rx_Status = RX_FREE;
            break;

        default:
            break;
        }
    }
}