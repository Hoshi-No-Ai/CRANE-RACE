/**********************************************************************************************************************************************************
��Ȩ������HITCRT(�����󾺼�������Э��)
�ļ�����HITCRT_OS.h
����޸����ڣ�2016.10.21
�汾��1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
ģ��������
�����б� 
----------------------------------------------------------------------------------------------------------------------------------------------------------
�޶���¼��
	 ����        	ʱ��            �汾     	˵��
  pyx & Chris    2016.10.21     	1.0      ���ֽ�����ģ�飬���������ļ�
**********************************************************************************************************************************************************/

#ifndef _HITCRT_OS_H
#define _HITCRT_OS_H

#include "os.h"

/*����������Ķ�ջ����*/
#define	init_task_SIZE			    	  	1024	
#define system_monitor_task_SIZE			1024
#define action_task_SIZE					    1024
#define transmit_task_SIZE            1024

/*��������������ȼ�*/
#define	init_task_PRIO					      9//8
#define motor_control_task_PRIO				7//6
#define system_monitor_task_PRIO			4
#define catch_task_PRIO						  	8//7 
#define action_task_PRIO				    	6//5
#define transmit_task_PRIO            5//9

/****************************************************************************************************
�꺯�����ƣ�DECLARE_OS_TASK(NAME)
�������ܣ���������ϵͳ�����ջ�Լ�����ϵͳ������
��ڲ�����NAME����������
****************************************************************************************************/
#define DECLARE_OS_TASK(NAME)\
OS_TCB NAME##TCB;\
__align(8) CPU_STK NAME##Stk[NAME##_SIZE];\
void NAME(void *p)

//��������������
#define DECLARE_HITCRT_OS_TASK()\
DECLARE_OS_TASK(init_task);\
DECLARE_OS_TASK(system_monitor_task);\
DECLARE_OS_TASK(action_task);\
DECLARE_OS_TASK(transmit_task);

/****************************************************************************************************
�꺯�����ƣ�CREATE_OS_TASK(NAME)
�������ܣ������������ƴ�������ϵͳ���񣬸������Ʒ����ջ�Լ��������ȼ�
��ڲ�����NAME����������
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
�꺯�����ƣ�OSTimeDly_ms(NUM)
�������ܣ�ͳһ���к��뼶����ʱ
��ڲ�����NUM����ʱ��������milliseconds (0...4294967295) 
****************************************************************************************************/
#define OSTimeDly_ms(NUM) OSTimeDlyHMSM(0,0,0,NUM,OS_OPT_TIME_HMSM_NON_STRICT,&err)

/****************************************��ʱ�����***********************************************/	
             
/*
ʹ�ò��裺
1.����DECLARE_OS_TMR������������ʱ���ı�Ҫ����
2.����CREATE_OS_TMR��������ʱ��
3.��д��ʱ���ص�������void NAME_CallBack(OS_TMR *p_tmr, void *p_arg)������Ҫ��.h�ļ�������
4.����Ҫ�ĵط�����OSTmrQuickStart��ʹ�ܶ�ʱ������ʱ����ʼ��ʱ
5.�����Ҫɾ��������OSTmrQuickDel������ɾ����ʱ��

���˵����
1.ѡ�񵥴ζ�ʱ��OS_OPT_TMR_ONE_SHOT����ֻ��DLY��Ч������������㸳ֵ
2.ѡ�����ڶ�ʱ��OS_OPT_TMR_PERIODIC������DLY==0�������ڶ�ʱ������ΪPERIOD��
  ��DLY��=0�����һ�ζ�ʱʱ��ΪDLY���������ڶ�ʱ������ΪDLY
3.�������ٽ����ڿ�ʼ��ʱ��
*/

////������ʱ��
//#define DECLARE_RC_OS_TMR() \
//DECLARE_OS_TMR(JUDGE_ON);\
//DECLARE_OS_TMR(NUC_ON);\
//DECLARE_OS_TMR(PITCH_ON);\
//DECLARE_OS_TMR(YAW_ON);\
//DECLARE_OS_TMR(SHOOTER_ON);\
//DECLARE_OS_TMR(MPU6050_ON);\
//DECLARE_OS_TMR(GYRO_ON)

////������ʱ��
//#define CREATE_RM_OS_TMR() \
//CREATE_OS_TMR(JUDGE_ON, 5, 0, OS_OPT_TMR_ONE_SHOT);\
//CREATE_OS_TMR(NUC_ON, 5, 0, OS_OPT_TMR_ONE_SHOT);\
//CREATE_OS_TMR(PITCH_ON, 5, 0, OS_OPT_TMR_ONE_SHOT);\
//CREATE_OS_TMR(YAW_ON, 5, 0, OS_OPT_TMR_ONE_SHOT);\
//CREATE_OS_TMR(SHOOTER_ON, 5, 0, OS_OPT_TMR_ONE_SHOT);\
//CREATE_OS_TMR(MPU6050_ON, 5, 0, OS_OPT_TMR_ONE_SHOT);\
//CREATE_OS_TMR(GYRO_ON, 5, 0, OS_OPT_TMR_ONE_SHOT)

/*---------------------------------------------------------------------------------------------------
�꺯�����ƣ�DECLARE_OS_TMR(NAME)
�������ܣ�����������ʱ���ı�Ҫ����
��ڲ�����@NAME      ��ʱ������
---------------------------------------------------------------------------------------------------*/
#define DECLARE_OS_TMR(NAME)\
OS_TMR NAME##TMR;\
OS_ERR NAME##ERROR;\
void NAME##_CallBack(OS_TMR *p_tmr, void *p_arg)

/*---------------------------------------------------------------------------------------------------
�꺯�����ƣ�CREATE_OS_TMR(NAME)
�������ܣ�������ʱ��
��ڲ�����
        @NAME       ��ʱ������
        @DLY        ��ʱʱ��
        @PERIOD     ��ʱ����ʱ��
        @OPT        ѡ���ǵ��ζ�ʱ��OS_OPT_TMR_ONE_SHOT���������ڶ�ʱ��OS_OPT_TMR_PERIODIC��
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
�꺯�����ƣ�OSTmrQuickStart(NAME)
�������ܣ�ʡȥ����������
��ڲ�����NAME����������
---------------------------------------------------------------------------------------------------*/
#define OSTmrQuickStart(NAME) do{\
OSTmrStart((OS_TMR *)&NAME##TMR,(OS_ERR*)0);\
}while(0)


/*---------------------------------------------------------------------------------------------------
�꺯�����ƣ�OSOutsideDlyDel(OEDER)
�������ܣ�ɾ���Ѿ�������ⲿ��ʱ��
��ڲ�����ORDER����ʱ����ţ�
---------------------------------------------------------------------------------------------------*/
#define OSTmrQuickDel(NAME) do{\
OSTmrDel((OS_TMR    *)&NAME##TMR,(OS_ERR 	*)0);\
}while(0)


/****************************************************************************************************/			 
/*********************************************�¼���־��*********************************************/		
/****************************************************************************************************/	
			 
/*---------------------------------------------------------------------------------------------------
�꺯�����ƣ�DECLARE_OS_FLAG(NAME)
�������ܣ������¼���־��
��ڲ�����NAME����ʱ������
---------------------------------------------------------------------------------------------------*/
#define DECLARE_OS_FLAG(NAME) \
OS_FLAG_GRP NAME##FLAG

//�����������¼���־��			 
#define DECLARE_RC_OS_FLAG() \
DECLARE_OS_FLAG(CAN)

/*---------------------------------------------------------------------------------------------------
�꺯�����ƣ�CREATE_OS_FLAG(NAME)
�������ܣ������¼���־��
��ڲ�����NAME
---------------------------------------------------------------------------------------------------*/
#define CREATE_OS_FLAG(NAME) \
OSFlagCreate((OS_FLAG_GRP*	  )&NAME##FLAG,\
			 (CPU_CHAR	* )"NAME##_FLAG",\
			 (OS_FLAGS	  )0x00,\
             (OS_ERR 	* )&err)

/*---------------------------------------------------------------------------------------------------
�꺯�����ƣ�OSFlagConsumePend(NAME)
�������ܣ�����һ���ȴ��ĸ��¼���־��������¼���־��
/�β�˵����(OS_FLAGS)0x0F���ȴ����ٸ��¼���־���ͽ�����λ��1.
			(OS_OPT)OS_OPT_PEND_FLAG_SET_ANY;����λ��1�����ȴ�
					OS_OPT_PEND_FLAG_SET_ALL:ȫ����1�����ȴ�
��ڲ�����NAME����������
---------------------------------------------------------------------------------------------------*/
#define OSFlagConsumePend(NAME) \
OSFlagPend((OS_FLAG_GRP*)&NAME##FLAG,\
			(OS_FLAGS)0x0F,\
			(OS_TICK)0,\
			(OS_OPT)OS_OPT_PEND_FLAG_SET_ANY+OS_OPT_PEND_FLAG_CONSUME,\
			(CPU_TS*)0,\
			(OS_ERR*)&err)
			
/*---------------------------------------------------------------------------------------------------
�꺯�����ƣ�OSFlagNonConsumePend(NAME)
�������ܣ�����һ���ȴ��ĸ��¼���־��������¼���־��
/�β�˵����(OS_FLAGS)0x0F���ȴ����ٸ��¼���־���ͽ�����λ��1.
			(OS_OPT)OS_OPT_PEND_FLAG_SET_ANY;����λ��1�����ȴ�
					OS_OPT_PEND_FLAG_SET_ALL:ȫ����1�����ȴ�
��ڲ�����NAME����������
---------------------------------------------------------------------------------------------------*/
#define OSFlagNonConsumePend(NAME) \
OSFlagPend((OS_FLAG_GRP*)&NAME##FLAG,\
			(OS_FLAGS)0x0F,\
			(OS_TICK)0,\
			(OS_OPT)OS_OPT_PEND_FLAG_SET_ANY,\
			(CPU_TS*)0,\
			(OS_ERR*)&err)
			

/*---------------------------------------------------------------------------------------------------
�꺯�����ƣ�OSFlagQuickPost(NAME,ORDER)
�������ܣ����ٷ����¼���־
��ڲ�����NAME��ORDER����־λ
---------------------------------------------------------------------------------------------------*/
#define OSFlagQuickPost(NAME,ORDER) do{\
extern OS_FLAG_GRP NAME##FLAG;\
OSFlagPost((OS_FLAG_GRP*)&NAME##FLAG,(OS_FLAGS)ORDER,(OS_OPT)OS_OPT_POST_FLAG_SET,(OS_ERR *)0);\
}while(0)

#endif
