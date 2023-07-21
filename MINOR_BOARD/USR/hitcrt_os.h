/**********************************************************************************************************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：HITCRT_OS.h
最近修改日期：2016.10.21
版本：1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
模块描述：
函数列表： 
----------------------------------------------------------------------------------------------------------------------------------------------------------
修订记录：
	 作者        	时间            版本     	说明
  pyx & Chris    2016.10.21     	1.0      划分建立此模块，重新命名文件
**********************************************************************************************************************************************************/

#ifndef _HITCRT_OS_H
#define _HITCRT_OS_H

#include "os.h"

/*定义主任务的堆栈长度*/
#define	init_task_SIZE			    	  	1024	
#define system_monitor_task_SIZE			1024
#define action_task_SIZE					    1024
#define transmit_task_SIZE            1024

/*定义主任务的优先级*/
#define	init_task_PRIO					      9//8
#define motor_control_task_PRIO				7//6
#define system_monitor_task_PRIO			4
#define catch_task_PRIO						  	8//7 
#define action_task_PRIO				    	6//5
#define transmit_task_PRIO            5//9

/****************************************************************************************************
宏函数名称：DECLARE_OS_TASK(NAME)
函数功能：申明操作系统任务堆栈以及操作系统的任务
入口参数：NAME，任务名称
****************************************************************************************************/
#define DECLARE_OS_TASK(NAME)\
OS_TCB NAME##TCB;\
__align(8) CPU_STK NAME##Stk[NAME##_SIZE];\
void NAME(void *p)

//声明创建的任务
#define DECLARE_HITCRT_OS_TASK()\
DECLARE_OS_TASK(init_task);\
DECLARE_OS_TASK(system_monitor_task);\
DECLARE_OS_TASK(action_task);\
DECLARE_OS_TASK(transmit_task);

/****************************************************************************************************
宏函数名称：CREATE_OS_TASK(NAME)
函数功能：根据任务名称创建操作系统任务，根据名称分配堆栈以及任务优先级
入口参数：NAME，任务名称
****************************************************************************************************/

#define CREATE_OS_TASK(NAME)\
OSTaskCreate((OS_TCB*	  )&NAME##TCB,\
			 (CPU_CHAR	* ) #NAME,\
             (OS_TASK_PTR )NAME,\
             (void* 	  )0,\
             (OS_PRIO	  )NAME##_PRIO,\
             (CPU_STK   * )&NAME##Stk[0],\
             (CPU_STK_SIZE)NAME##_SIZE/10,\
             (CPU_STK_SIZE)NAME##_SIZE,\
             (OS_MSG_QTY  )0,\
             (OS_TICK	  )0,\
             (void   	* )0,\
             (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,\
             (OS_ERR 	* )&err)	

/****************************************************************************************************
宏函数名称：OSTimeDly_ms(NUM)
函数功能：统一进行毫秒级的延时
入口参数：NUM，延时毫秒数，milliseconds (0...4294967295) 
****************************************************************************************************/
#define OSTimeDly_ms(NUM) OSTimeDlyHMSM(0,0,0,NUM,OS_OPT_TIME_HMSM_NON_STRICT,&err)

/****************************************定时器相关***********************************************/	
             
/*
使用步骤：
1.调用DECLARE_OS_TMR，申明创建定时器的必要变量
2.调用CREATE_OS_TMR，创建定时器
3.编写定时器回调函数，void NAME_CallBack(OS_TMR *p_tmr, void *p_arg)，不需要在.h文件中申明
4.在需要的地方调用OSTmrQuickStart，使能定时器，定时器开始计时
5.如果需要删除，调用OSTmrQuickDel，即可删除定时器

相关说明：
1.选择单次定时（OS_OPT_TMR_ONE_SHOT），只有DLY有效，其他可以随便赋值
2.选择周期定时（OS_OPT_TMR_PERIODIC），若DLY==0，则周期定时，周期为PERIOD；
  若DLY！=0，则第一次定时时间为DLY，后面周期定时，周期为DLY
3.不能在临界区内开始定时器
*/

////声明定时器
//#define DECLARE_RC_OS_TMR() \
//DECLARE_OS_TMR(JUDGE_ON);\
//DECLARE_OS_TMR(NUC_ON);\
//DECLARE_OS_TMR(PITCH_ON);\
//DECLARE_OS_TMR(YAW_ON);\
//DECLARE_OS_TMR(SHOOTER_ON);\
//DECLARE_OS_TMR(MPU6050_ON);\
//DECLARE_OS_TMR(GYRO_ON)

////创建定时器
//#define CREATE_RM_OS_TMR() \
//CREATE_OS_TMR(JUDGE_ON, 5, 0, OS_OPT_TMR_ONE_SHOT);\
//CREATE_OS_TMR(NUC_ON, 5, 0, OS_OPT_TMR_ONE_SHOT);\
//CREATE_OS_TMR(PITCH_ON, 5, 0, OS_OPT_TMR_ONE_SHOT);\
//CREATE_OS_TMR(YAW_ON, 5, 0, OS_OPT_TMR_ONE_SHOT);\
//CREATE_OS_TMR(SHOOTER_ON, 5, 0, OS_OPT_TMR_ONE_SHOT);\
//CREATE_OS_TMR(MPU6050_ON, 5, 0, OS_OPT_TMR_ONE_SHOT);\
//CREATE_OS_TMR(GYRO_ON, 5, 0, OS_OPT_TMR_ONE_SHOT)

/*---------------------------------------------------------------------------------------------------
宏函数名称：DECLARE_OS_TMR(NAME)
函数功能：申明创建定时器的必要变量
入口参数：@NAME      定时器名称
---------------------------------------------------------------------------------------------------*/
#define DECLARE_OS_TMR(NAME)\
OS_TMR NAME##TMR;\
OS_ERR NAME##ERROR;\
void NAME##_CallBack(OS_TMR *p_tmr, void *p_arg)

/*---------------------------------------------------------------------------------------------------
宏函数名称：CREATE_OS_TMR(NAME)
函数功能：创建定时器
入口参数：
        @NAME       定时器名称
        @DLY        延时时间
        @PERIOD     定时周期时间
        @OPT        选择是单次定时（OS_OPT_TMR_ONE_SHOT）还是周期定时（OS_OPT_TMR_PERIODIC）
---------------------------------------------------------------------------------------------------*/
#define CREATE_OS_TMR(NAME, DLY, PERIOD, OPT)\
OSTmrCreate((OS_TMR*	  )&NAME##TMR,\
			 (CPU_CHAR	* ) #NAME,\
			 (OS_TICK	  )DLY,\
             (OS_TICK	  )PERIOD,\
             (OS_OPT      )OPT,\
			 (OS_TMR_CALLBACK_PTR )(OS_TMR_CALLBACK_PTR )&NAME##_CallBack,\
			 (void   	* ) 0,\
             (OS_ERR 	* )&NAME##ERROR)	

/*---------------------------------------------------------------------------------------------------
宏函数名称：OSTmrQuickStart(NAME)
函数功能：省去定义错误变量
入口参数：NAME，任务名称
---------------------------------------------------------------------------------------------------*/
#define OSTmrQuickStart(NAME) do{\
OSTmrStart((OS_TMR *)&NAME##TMR,(OS_ERR*)0);\
}while(0)


/*---------------------------------------------------------------------------------------------------
宏函数名称：OSOutsideDlyDel(OEDER)
函数功能：删除已经定义的外部定时器
入口参数：ORDER（定时器序号）
---------------------------------------------------------------------------------------------------*/
#define OSTmrQuickDel(NAME) do{\
OSTmrDel((OS_TMR    *)&NAME##TMR,(OS_ERR 	*)0);\
}while(0)


/****************************************************************************************************/			 
/*********************************************事件标志组*********************************************/		
/****************************************************************************************************/	
			 
/*---------------------------------------------------------------------------------------------------
宏函数名称：DECLARE_OS_FLAG(NAME)
函数功能：定义事件标志组
入口参数：NAME，定时器名称
---------------------------------------------------------------------------------------------------*/
#define DECLARE_OS_FLAG(NAME) \
OS_FLAG_GRP NAME##FLAG

//声明创建的事件标志组			 
#define DECLARE_RC_OS_FLAG() \
DECLARE_OS_FLAG(CAN)

/*---------------------------------------------------------------------------------------------------
宏函数名称：CREATE_OS_FLAG(NAME)
函数功能：创建事件标志组
入口参数：NAME
---------------------------------------------------------------------------------------------------*/
#define CREATE_OS_FLAG(NAME) \
OSFlagCreate((OS_FLAG_GRP*	  )&NAME##FLAG,\
			 (CPU_CHAR	* )"NAME##_FLAG",\
			 (OS_FLAGS	  )0x00,\
             (OS_ERR 	* )&err)

/*---------------------------------------------------------------------------------------------------
宏函数名称：OSFlagConsumePend(NAME)
函数功能：创建一个等待四个事件标志或运算的事件标志组
/形参说明：(OS_FLAGS)0x0F：等待多少个事件标志，就将多少位置1.
			(OS_OPT)OS_OPT_PEND_FLAG_SET_ANY;任意位置1结束等待
					OS_OPT_PEND_FLAG_SET_ALL:全部置1结束等待
入口参数：NAME，任务名称
---------------------------------------------------------------------------------------------------*/
#define OSFlagConsumePend(NAME) \
OSFlagPend((OS_FLAG_GRP*)&NAME##FLAG,\
			(OS_FLAGS)0x0F,\
			(OS_TICK)0,\
			(OS_OPT)OS_OPT_PEND_FLAG_SET_ANY+OS_OPT_PEND_FLAG_CONSUME,\
			(CPU_TS*)0,\
			(OS_ERR*)&err)
			
/*---------------------------------------------------------------------------------------------------
宏函数名称：OSFlagNonConsumePend(NAME)
函数功能：创建一个等待四个事件标志或运算的事件标志组
/形参说明：(OS_FLAGS)0x0F：等待多少个事件标志，就将多少位置1.
			(OS_OPT)OS_OPT_PEND_FLAG_SET_ANY;任意位置1结束等待
					OS_OPT_PEND_FLAG_SET_ALL:全部置1结束等待
入口参数：NAME，任务名称
---------------------------------------------------------------------------------------------------*/
#define OSFlagNonConsumePend(NAME) \
OSFlagPend((OS_FLAG_GRP*)&NAME##FLAG,\
			(OS_FLAGS)0x0F,\
			(OS_TICK)0,\
			(OS_OPT)OS_OPT_PEND_FLAG_SET_ANY,\
			(CPU_TS*)0,\
			(OS_ERR*)&err)
			

/*---------------------------------------------------------------------------------------------------
宏函数名称：OSFlagQuickPost(NAME,ORDER)
函数功能：快速发布事件标志
入口参数：NAME，ORDER，标志位
---------------------------------------------------------------------------------------------------*/
#define OSFlagQuickPost(NAME,ORDER) do{\
extern OS_FLAG_GRP NAME##FLAG;\
OSFlagPost((OS_FLAG_GRP*)&NAME##FLAG,(OS_FLAGS)ORDER,(OS_OPT)OS_OPT_POST_FLAG_SET,(OS_ERR *)0);\
}while(0)

#endif
