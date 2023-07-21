#ifndef __AIR_OPERATED_BOARD_H__
#define __AIR_OPERATED_BOARD_H__

#include "stm32f4xx.h"

//气动板气缸通道对应定义
#define DOOR_LEFT_CH   0x01
#define DOOR_RIGHT_CH  0x02
#define SEND_LEFT_CH   0x03
#define SEND_RIGHT_CH  0x04

//气动板有关宏定义

#define  CAN_SWITCH_ID   	0x20
#define  CAN_SWITCH_ID_Plus   	0x21
#define  CAN_AIR_ID      	0x30
#define  CAN_AIR_ID_Plus  0x31
#define  CAN_AIR_ID_reply 0x32
#define  CAN_AIR_ID_Plus_reply 0x33
#define  CAN_SERVO_ID    	0x40
#define  CAN_SERVO_ID_Plus 0x41
#define  CAN_SERVO_ID_Plus_reply 0x42

//PWM通道控制涵道或舵机的标志
#define	  Servo_Ctrl_Flag     0x01
#define   DutedFan_Ctrl_Flag  0x02

//气缸有关宏定义 (1-8)
//#define OPEN_VALVE(channel)   if(1){( g_uiAirValve |= (0x01 << (channel-1)) ) ; SendAirMsgByCan2(&g_uiAirValve);}
//#define CLOSE_VALVE(channel)  if(1){( g_uiAirValve &= ( ~(0x01 << (channel-1) ) )) ;	SendAirMsgByCan2(&g_uiAirValve);}
#define OPEN_VALVE(channel)   if(1){( g_uiAirValve |= (0x01 << (channel-1)) ) ; SendAirMsgByCan1(&g_uiAirValve);}
#define CLOSE_VALVE(channel)  if(1){( g_uiAirValve &= ( ~(0x01 << (channel-1) ) )) ;	SendAirMsgByCan1(&g_uiAirValve);}
#define IS_VALVE_OPEN(channel)    (g_uiAirValve &(0x01<<(channel-1)))
#define IS_VALVE_CLOSE(channel)   (!(g_uiAirValve &(0x01<<(channel-1))))
#define GET_VALVE_VALUE()             (g_uiAirValve)
#define SET_VALVE_VALUE(value)    (g_uiAirValve = value);SendAirMsgByCan2(&g_uiAirValve)
#define CHANGE_VALVE(channel)        (g_uiAirValve ^= (((uint32_t)(0x01<<(channel-1)))));SendAirMsgByCan2(&g_uiAirValve)


//气缸有关宏定义 (1-8)  第二个舵机板
#define OPEN_VALVE_Plus(channel)   ( g_uiAirValve_Plus |= (0x01 << (channel-1)) ) ; SendAirMsgByCan2_Plus(&g_uiAirValve_Plus)
#define CLOSE_VALVE_Plus(channel)  ( g_uiAirValve_Plus &= ( ~(0x01 << (channel-1) ) )) ;	SendAirMsgByCan2_Plus(&g_uiAirValve_Plus)
#define IS_VALVE_OPEN_Plus(channel)    (g_uiAirValve_Plus &(0x01<<(channel-1)))
#define IS_VALVE_CLOSE_Plus(channel)   (!(g_uiAirValve_Plus &(0x01<<(channel-1))))
#define GET_VALVE_VALUE_Plus()             (g_uiAirValve_Plus)
#define SET_VALVE_VALUE_Plus(value)    (g_uiAirValve_Plus = value);SendAirMsgByCan2_Plus(&g_uiAirValve_Plus)
#define CHANGE_VALVE_Plus(channel)        (g_uiAirValve_Plus ^= (((uint32_t)(0x01<<(channel-1)))));SendAirMsgByCan2_Plus(&g_uiAirValve_Plus)


//开关有关宏定义
#define UPDATE_SWITCH(pRxMsg)   g_usSwitchPre = g_usSwitch;\
								(UpdateSwitchValue(&g_usSwitch, pRxMsg))								
#define IS_SWITCH_ON(channel)   (g_usSwitch&(0x0001 << channel))
#define IS_SWITCH_OFF(channel)  (!(g_usSwitch&(0x0001 << channel)))
#define GET_SWITCH_VALUE()        (g_usSwitch)
#define SET_SWITCH_VALUE(value)   (g_usSwitch = value)
#define IS_SWITCH_CHANGE(channel) ((g_usSwitch&(1<<channel)) != (g_usSwitchPre&(1<<channel)))

//开关有关宏定义1
#define UPDATE_SWITCH1(pRxMsg)   g_usSwitchPre1 = g_usSwitch1;\
								(UpdateSwitchValue(&g_usSwitch1, pRxMsg))								
#define IS_SWITCH_ON1(channel)   (g_usSwitch1&(0x0001 << channel))
#define IS_SWITCH_OFF1(channel)  (!(g_usSwitch1&(0x0001 << channel)))
#define GET_SWITCH_VALUE1()        (g_usSwitch1)
#define SET_SWITCH_VALUE1(value)   (g_usSwitch1 = value)
#define IS_SWITCH_CHANGE1(channel) ((g_usSwitch1&(1<<channel)) != (g_usSwitchPre1&(1<<channel)))

extern uint32_t g_uiAirValve;	   //气缸有关全局变量
extern uint32_t g_uiAirValvePre;  //气缸上一次状态
extern uint16_t g_usSwitch;     //开关有关全局变量
extern uint16_t g_usSwitchPre;//保存上一时刻开关值

extern uint16_t g_usSwitch1;     //开关有关全局变量1
extern uint16_t g_usSwitchPre1;//保存上一时刻开关值1


extern void SendAirMsgByCan2(uint32_t* pAir);
extern void SendAirMsgByCan1(uint32_t* pAir);
extern void SendAirMsgByCan2_Plus(uint32_t* pAir);
extern void UpdateSwitchValue(uint16_t*pusSwitch,CanRxMsg * pRxMsg);
extern void Send_LED_Mode(uint8_t mode);

extern void SendServoMsgByCan2(uint8_t chan,uint8_t value);//通过CAN2发送舵机的占空比
extern void SendServoMsgByCan2_Plus(uint8_t chan,uint8_t value);//通过CAN2发送舵机的占空比


#endif
