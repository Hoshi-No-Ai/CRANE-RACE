#ifndef __UDP_DEMO_H
#define __UDP_DEMO_H

#include "lwip_comm.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//UDP 测试代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/8/15
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//*******************************************************************************
//修改信息
//无
////////////////////////////////////////////////////////////////////////////////// 	   
 
#define UDP_DEMO_RX_BUFSIZE		2000	//定义udp最大接收数据长度 
#define UDP_DEMO_PORT			8089	//定义udp连接的端口 


	__packed struct ETHERDATA
{
    __packed struct
    {
		u8 head[2];		//2
        u8 functype;		     //1
        u8 numlength;		     //1
	    int Order;               //4
		float Motor[12];         //48
		u8 tail[2];              //2      
    }Receive;                    //total:58
    
    __packed struct
    {
		u8 head[2];                 //2
		u8 functype;                //1
		u8 numlength;               //1
        int Order;	                //4       
		u32 MotorID;                //4
		u8  Data[8];                //8
		u8 tail[2];                 // 2
    }Send;                          //total:22
};
extern struct ETHERDATA MotorData;
extern u8 EtherInitFlag;
extern u8 tbuf[200];
extern u8 udp_demo_recvbuf[UDP_DEMO_RX_BUFSIZE];
void udp_demo_test(void);
void udp_demo_recv(void *arg,struct udp_pcb *upcb,struct pbuf *p,struct ip_addr *addr,u16_t port);
void udp_demo_senddata(struct udp_pcb *upcb);
void udp_demo_connection_close(struct udp_pcb *upcb);

extern struct udp_pcb *udppcb;

#endif

