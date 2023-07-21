
#include "stdio.h"
#include "string.h" 
#include "stm32f4xx_it.h"
#include "can.h"
#include "udp_demo.h" 
#include "debug_task.h"

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
//__packed struct ETHERDATA
//    {
//		u8 head[2]; 		     //2
//        u8 functype;		     //1
//        u8 numlength;		     //1
//		float Motor[10];         //40
//		u8 tail[2];              //2
//    }MotorData;                    //total:46

	


struct ETHERDATA MotorData = {0};


//UDP接收数据缓冲区
u8 udp_demo_recvbuf[UDP_DEMO_RX_BUFSIZE];	//UDP接收数据缓冲区 
//UDP发送数据内容
// u8 *tcp_demo_sendbuf = "Explorer STM32F407 UDP demo send data/r/n" ;
char tcp_demo_sendbuf[ARM_DEBUG_SIZE*2];
//UDP 测试全局状态标记变量
//bit7:没有用到
//bit6:0,没有收到数据;1,收到数据了.
//bit5:0,没有连接上;1,连接上了.
//bit4~0:保留
u8 udp_demo_flag;

//设置远端IP地址
void udp_demo_set_remoteip(void)
{
	u8 *tbuf;

	lwipdev.remoteip[0]=255;
	lwipdev.remoteip[1]=255;
	lwipdev.remoteip[2]=255; 
	lwipdev.remoteip[3]=255;

} 

//UDP测试
int a=0;
u8 res=0;
u8 EtherInitFlag = 0;
s16 Current = 0;
u8 tbuf[200];

struct udp_pcb *udppcb;
  	//定义一个TCP服务器控制块
void udp_demo_test(void)
{
 	err_t err;
	struct ip_addr rmtipaddr;  	//远端ip地址
	udp_demo_set_remoteip();//先选择IP
	udppcb=udp_new();
	if(udppcb)//创建成功
	{ 
		IP4_ADDR(&rmtipaddr,lwipdev.remoteip[0],lwipdev.remoteip[1],lwipdev.remoteip[2],lwipdev.remoteip[3]);
		err=udp_connect(udppcb,&rmtipaddr,8089);//UDP客户端连接到指定IP地址和端口号的服务器
		if(err==ERR_OK)
		{
			err=udp_bind(udppcb,IP_ADDR_ANY,8089);//绑定本地IP地址与端口号
			if(err==ERR_OK)	//绑定完成
			{
				udp_recv(udppcb,udp_demo_recv,NULL);//注册接收回调函数 
				//标记连接上了(UDP是非可靠连接,这里仅仅表示本地UDP已经准备好)

				udp_demo_flag |= 1<<5;			//标记已经连接上
				//提示消息	
					
				
			}else EtherInitFlag=1;
		}else EtherInitFlag=1;		
	}else EtherInitFlag=1;
	
} 

//UDP回调函数
void udp_demo_recv(void *arg,struct udp_pcb *upcb,struct pbuf *p,struct ip_addr *addr,u16_t port)
{
	u32 data_len = 0;
	struct pbuf *q;
	if(p!=NULL)	//接收到不为空的数据时
	{
		memset(udp_demo_recvbuf,0,UDP_DEMO_RX_BUFSIZE);  //数据接收缓冲区清零
		for(q=p;q!=NULL;q=q->next)  //遍历完整个pbuf链表
		{
			//判断要拷贝到UDP_DEMO_RX_BUFSIZE中的数据是否大于UDP_DEMO_RX_BUFSIZE的剩余空间，如果大于
			//的话就只拷贝UDP_DEMO_RX_BUFSIZE中剩余长度的数据，否则的话就拷贝所有的数据
			if(q->len > (UDP_DEMO_RX_BUFSIZE-data_len)) memcpy(udp_demo_recvbuf+data_len,q->payload,(UDP_DEMO_RX_BUFSIZE-data_len));//拷贝数据
			else memcpy(udp_demo_recvbuf+data_len,q->payload,q->len);
			
			data_len += q->len;  	
			if(data_len > UDP_DEMO_RX_BUFSIZE) break; //超出TCP客户端接收数组,跳出	
		}
		upcb->remote_ip=*addr; 				//记录远程主机的IP地址
		upcb->remote_port=port;  			//记录远程主机的端口号
		lwipdev.remoteip[0]=upcb->remote_ip.addr&0xff; 		//IADDR4
		lwipdev.remoteip[1]=(upcb->remote_ip.addr>>8)&0xff; //IADDR3
		lwipdev.remoteip[2]=(upcb->remote_ip.addr>>16)&0xff;//IADDR2
		lwipdev.remoteip[3]=(upcb->remote_ip.addr>>24)&0xff;//IADDR1 
		udp_demo_flag|=1<<6;	//标记接收到数据了
		pbuf_free(p);//释放内存
	}else
	{
		udp_disconnect(upcb); 
		udp_demo_flag &= ~(1<<5);	//标记连接断开
	} 
} 
//UDP服务器发送数据
void udp_demo_senddata(struct udp_pcb *upcb)
{
	struct pbuf *ptr;
	ptr=pbuf_alloc(PBUF_TRANSPORT,ARM_DEBUG_SIZE*4+4,PBUF_POOL); //申请内存
	if(ptr)
	{
		pbuf_take(ptr,(char*)tcp_demo_sendbuf,ARM_DEBUG_SIZE*4+4); //将tcp_demo_sendbuf中的数据打包进pbuf结构中
		udp_send(upcb,ptr);	//udp发送数据 
		pbuf_free(ptr);//释放内存
	} 
} 
//关闭UDP连接
void udp_demo_connection_close(struct udp_pcb *upcb)
{
	udp_disconnect(upcb); 
	udp_remove(upcb);			//断开UDP连接 
	udp_demo_flag &= ~(1<<5);	//标记连接断开

}

























