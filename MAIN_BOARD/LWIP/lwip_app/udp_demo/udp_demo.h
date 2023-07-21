#ifndef __UDP_DEMO_H
#define __UDP_DEMO_H

#include "lwip_comm.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//UDP ���Դ���	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/8/15
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//*******************************************************************************
//�޸���Ϣ
//��
////////////////////////////////////////////////////////////////////////////////// 	   
 
#define UDP_DEMO_RX_BUFSIZE		2000	//����udp���������ݳ��� 
#define UDP_DEMO_PORT			8089	//����udp���ӵĶ˿� 


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

