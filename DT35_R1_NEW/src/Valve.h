#ifndef _VALVE_H
#define _VALVE_H

void ValveCtrl(void);
void ValveAllStart(void);
void ValveAllClose(void);

/*气动端口宏定义 */
#define VALVE1 	0x0001
#define VALVE2 	0x0002
#define VALVE3 	0x0004 
#define VALVE4 	0x0008 
#define VALVE5 	0x0010
#define VALVE6 	0x0020
#define VALVE7 	0x0040
#define VALVE8 	0x0080
#define VALVE9 	0x0100
#define VALVE10 0x0200
#define VALVE11 0x0400
#define VALVE12 0x0800
#define VALVE13 0x1000
#define VALVE14 0x2000
#define VALVE15 0x4000
#define VALVE16 0x8000

/*气阀控制，通道数1-16*/
#define VALVE_1_OPEN   	GPIOA->BRR = GPIO_Pin_11 
#define VALVE_1_CLOSE 	GPIOA->BSRR = GPIO_Pin_11

#define VALVE_2_OPEN  	GPIOA->BRR = GPIO_Pin_10
#define VALVE_2_CLOSE 	GPIOA->BSRR = GPIO_Pin_10

#define VALVE_3_OPEN  	GPIOA->BRR = GPIO_Pin_9
#define VALVE_3_CLOSE		GPIOA->BSRR = GPIO_Pin_9

#define VALVE_4_OPEN  	GPIOA->BRR = GPIO_Pin_8
#define VALVE_4_CLOSE		GPIOA->BSRR = GPIO_Pin_8

#define VALVE_5_OPEN   	GPIOC->BRR = GPIO_Pin_9
#define VALVE_5_CLOSE	 	GPIOC->BSRR = GPIO_Pin_9

#define VALVE_6_OPEN   	GPIOC->BRR = GPIO_Pin_8
#define VALVE_6_CLOSE	 	GPIOC->BSRR = GPIO_Pin_8

#define VALVE_7_OPEN   	GPIOC->BRR = GPIO_Pin_7
#define VALVE_7_CLOSE	 	GPIOC->BSRR = GPIO_Pin_7

#define VALVE_8_OPEN   	GPIOC->BRR = GPIO_Pin_6
#define VALVE_8_CLOSE		GPIOC->BSRR = GPIO_Pin_6

#define VALVE_9_OPEN   	GPIOB->BRR = GPIO_Pin_15
#define VALVE_9_CLOSE		GPIOB->BSRR = GPIO_Pin_15

#define VALVE_10_OPEN   GPIOB->BRR = GPIO_Pin_14
#define VALVE_10_CLOSE	GPIOB->BSRR = GPIO_Pin_14

#define VALVE_11_OPEN   GPIOB->BRR = GPIO_Pin_13
#define VALVE_11_CLOSE	GPIOB->BSRR = GPIO_Pin_13

#define VALVE_12_OPEN   GPIOB->BRR = GPIO_Pin_12
#define VALVE_12_CLOSE	GPIOB->BSRR = GPIO_Pin_12

#define VALVE_13_OPEN  	GPIOB->BRR = GPIO_Pin_11
#define VALVE_13_CLOSE	GPIOB->BSRR = GPIO_Pin_11

#define VALVE_14_OPEN   GPIOB->BRR = GPIO_Pin_10
#define VALVE_14_CLOSE	GPIOB->BSRR = GPIO_Pin_10

#define VALVE_15_OPEN   GPIOA->BRR = GPIO_Pin_5
#define VALVE_15_CLOSE	GPIOA->BSRR = GPIO_Pin_5

#define VALVE_16_OPEN  	GPIOA->BRR = GPIO_Pin_4
#define VALVE_16_CLOSE	GPIOA->BSRR = GPIO_Pin_4


#endif
