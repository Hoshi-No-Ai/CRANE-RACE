#ifndef __USART_PROTOCOL_H__
#define __USART_PROTOCOL_H__

#include "Gyro.h"
#include "E103_W06.h"
#include "basic_type.h"
#include "flag_global.h"
#include "math.h"
#include "stm32f4xx.h"
#include "string.h"
#include "uart.h"

#include "sucker.h"

#define DT35_NUM 0x04

#define MAX_RX_DATA_LEN 114

#define USART6_RX_DATA_LEN 33  // 遥控器发来的数据为32字节，两车通讯的数据为114字节
#define USART6_TX_DATA_LEN 114

#define UART2_RX_DATA_LEN (COM_LENGTH * 4)
#define UART2_TX_DATA_LEN 8

#define UART1_RX_DATA_LEN1 9
#define UART1_RX_DATA_LEN2 9
#define UART1_TX_DATA_LEN 10

#define UART3_RX_DATA_LEN 16
#define UART3_TX_DATA_LEN 16

#define COM_LENGTH 7

// #define UART6_RX_DATA_LEN 	16
// #define UART6_TX_DATA_LEN   46

enum rx_protocol_e {
    RX_FREE,
    RX_START_1,
    RX_START_2,
    RX_START_3,
    RX_DATAS,
    RX_DATAS_1,
    RX_DATAS_2,
    RX_TAIL_1,
    RX_TAIL_2,
    RX_TAIL_3,
    RX_TAIL_4,
    RX_TOWER_START_3,
    RX_TOWER_DATAS,
};

struct aruco{
	fp32 x;
	fp32 y;
	fp32 z;
	fp32 thetax;
	fp32 thetay;
	fp32 thetaz;
  fp32 if_detect;
};



// class uart_protocol
//{
// public:
//	uint8_t start1;	//帧头1
//	uint8_t start2;	//帧头2
//	uint8_t datanum;
//	uint8_t tail1;	//帧尾1
//	uint8_t tail2;	//帧尾2
//
//	uart_protocol(uint8_t _start1 = 0x55, uint8_t _start2 = 0x22,
//													uint8_t _tail1 = 0x01, uint8_t _tail2 = 0xAA)
//	{
//		start1 = _start1;
//		start2 = _start2;
//		tail1 = _tail1;
//		tail2 = _tail2;
//	}
// };

// class cUart
//{
// private:
//	uart_protocol rx_protocol, tx_protocol;
//	uint8_t tx_data[MAX_TX_LEN];
// public:
//	uint8_t rx_data[MAX_RX_LEN];
//	cUart(){};
//	void send_data(uint8_t *data, uint8_t num);
//	uint8_t get_receive_data_num(void) {return rx_protocol.datanum;}
// };

struct uart1_rx_protocol_t {
    uint8_t start11;  // 帧头1
    uint8_t start12;  // 帧头2
    uint8_t start21;  // 帧头1
    uint8_t start22;  // 帧头2
    uint8_t datanum1;
    uint8_t datanum2;

    s8 num1[UART1_RX_DATA_LEN1];  // 待接收数据
    s8 num2[UART1_RX_DATA_LEN2];  // 待接收数据

    uint8_t tail1;  // 帧尾1
    uint8_t tail2;  // 帧尾2

};  // 接收信息报文结构体

struct uart3_tx_protocol_t {
    uint8_t start1;  // 帧头1
    uint8_t start2;  // 帧头2
    uint8_t datanum;

    s8 num[UART3_TX_DATA_LEN];  // 待发送数据

    uint8_t tail1;  // 帧尾1
    uint8_t tail2;  // 帧尾2

};  // 发送信息报文结构体

struct uart3_rx_protocol_t {
    uint8_t start1;  // 帧头1
    uint8_t start2;  // 帧头2
    uint8_t datanum;

    s8 num[UART3_RX_DATA_LEN];  // 待接收数据

    uint8_t tail1;  // 帧尾1
    uint8_t tail2;  // 帧尾2

};  // 接收信息报文结构体

enum wifi_rx_id_e { REMOTE_CTRL, ROBOT };

/*串口6用作和视觉通信*/
struct uart2_tx_protocol_t {
    uint8_t start1;  // 帧头1
    uint8_t start2;  // 帧头2
    uint8_t ID;
    uint8_t datanum;

    float2char num[UART2_TX_DATA_LEN];

    uint8_t tail1;  // 帧尾1
    uint8_t tail2;  // 帧尾2

};  // 发送信息报文结构体

struct uart2_rx_protocol_t {
    uint8_t start1;  // 帧头1
    uint8_t start2;  // 帧头2
    uint8_t ID;
    uint8_t data_len_float;
    uint8_t datanum;
    uint8_t num[UART2_RX_DATA_LEN];  // 待接收数据

    uint8_t tail1;  // 帧尾1
    uint8_t tail2;  // 帧尾2

};  // 接收信息报文结构体

// struct uart6_tx_robot_t
//{
//	uint8_t wifi_start1;	//帧头1
//	uint8_t wifi_start2;	//帧头2
//	uint8_t wifi_start3;
//	uint8_t wifi_start4;

//	wifi_rx_id_e ID;
//
//	uint8_t num[USART6_TX_DATA_LEN];		//待发送数据
//
//	uint8_t tail1;	//帧尾1
//	uint8_t tail2;	//帧尾2
//
//};//发送信息报文结构体

// struct rx_protocol_t
//{
//	uint8_t start1;  //帧头1
//	uint8_t start2;  //帧头2
//	uint8_t start3;
//	uint8_t start4;

//	uint8_t datanum;
//
//	uint8_t num[MAX_RX_DATA_LEN];	//待接收数据
//	uint8_t remote_ctrl_num[USART6_RX_DATA_LEN];

//	uint8_t tail1;   //帧尾1
//	uint8_t tail2;   //帧尾2
//	uint8_t tail3;
//	uint8_t tail4;
//
//};//接收信息报文结构体

struct uart6_tx_remote_ctrl_t {
    uint8_t wifi_start1;  // 帧头1
    uint8_t wifi_start2;  // 帧头2
    uint8_t wifi_start3;
    uint8_t wifi_start4;

    uint8_t start1;
    uint8_t start2;

    wifi_rx_id_e ID;

    uint8_t num[USART6_TX_DATA_LEN];  // 待发送数据

    uint8_t tail1;  // 帧尾1
    uint8_t tail2;  // 帧尾2

};  // 发送信息报文结构体

struct uart6_tx_robot_t {
    uint8_t wifi_start1;  // 帧头1
    uint8_t wifi_start2;  // 帧头2
    uint8_t wifi_start3;
    uint8_t wifi_start4;

    wifi_rx_id_e ID;

    uint8_t num[USART6_TX_DATA_LEN];  // 待发送数据

    uint8_t tail1;  // 帧尾1
    uint8_t tail2;  // 帧尾2

};  // 发送信息报文结构体

struct rx_protocol_t {
    uint8_t start1;  // 帧头1
    uint8_t start2;  // 帧头2

    wifi_rx_id_e ID;

    uint8_t datanum;

    uint8_t num[MAX_RX_DATA_LEN];  // 待接收数据
    uint8_t remote_ctrl_num[USART6_RX_DATA_LEN];

    uint8_t tail1;  // 帧尾1
    uint8_t tail2;  // 帧尾2
};                  // 接收信息报文结构体

extern uart1_rx_protocol_t uart1_efr;

extern uart2_tx_protocol_t uart2_eft;
extern uart2_rx_protocol_t uart2_efr;

extern uart3_tx_protocol_t uart3_eft;
extern uart3_rx_protocol_t uart3_efr;

extern uart6_tx_robot_t uart6_robot_eft;
extern uart6_tx_remote_ctrl_t uart6_remote_ctrl_eft;
extern rx_protocol_t uart6_efr;

// extern uart6_tx_protocol_t uart6_eft;
// extern uart6_rx_protocol_t uart6_efr;

extern aruco aruco_fdb;
extern int temp_target_detect;
extern cSucker sucker;
extern int dist_1, dist_2;

void USART3_DMA_Tx(void);
void Comm3Rx_IRQ(void);
void USART6_DMA_Tx(wifi_rx_id_e device);
void Comm6Rx_IRQ(void);
void UART2_DMA_Tx(void);
void Comm2Rx_IRQ(void);
uint16_t USART_Receive(USART_RX_TypeDef* USARTx);
void Comm1Rx_IRQ(void);

#endif
