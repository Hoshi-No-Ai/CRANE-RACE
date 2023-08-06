#ifndef _POSITIONSWITCH_H
#define _POSITIONSWITCH_H

//#define ScanKey(n)  GET_KEY##n
#define GET_KEY1	  ((GPIOC->IDR & GPIO_Pin_10) == GPIO_Pin_10)
#define GET_KEY2    ((GPIOC->IDR & GPIO_Pin_11) == GPIO_Pin_11)
#define GET_KEY3    ((GPIOC->IDR & GPIO_Pin_12) == GPIO_Pin_12)
#define GET_KEY4    ((GPIOD->IDR & GPIO_Pin_2) == GPIO_Pin_2)

#define GET_KEY5    ((GPIOB->IDR & GPIO_Pin_3) == GPIO_Pin_3)
#define GET_KEY6    ((GPIOB->IDR & GPIO_Pin_4) == GPIO_Pin_4)
#define GET_KEY7    ((GPIOB->IDR & GPIO_Pin_5) == GPIO_Pin_5)
#define GET_KEY8    ((GPIOB->IDR & GPIO_Pin_6) == GPIO_Pin_6)
#define GET_KEY9    ((GPIOC->IDR & GPIO_Pin_13) == GPIO_Pin_13)
#define GET_KEY10   ((GPIOC->IDR & GPIO_Pin_15) == GPIO_Pin_15)
#define GET_KEY11   ((GPIOC->IDR & GPIO_Pin_14) == GPIO_Pin_14)
#define GET_KEY12   ((GPIOC->IDR & GPIO_Pin_0) == GPIO_Pin_0)

#define KEY1	(1<<10)
#define KEY2	(1<<11)
#define KEY3	(1<<12)			  
#define KEY4	(1<<2)
#define KEY5	(1<<3)
#define KEY6	(1<<4)
#define KEY7	(1<<5)
#define KEY8  (1<<6)
#define KEY9  (1<<13)
#define KEY10	(1<<15)
#define KEY11	(1<<14)
#define KEY12	(1<<0)



/*读到的数为1时，行程开关接GND，表示按下*/
//#define GET_KEY1	((GPIOB->IDR & GPIO_Pin_12) == GPIO_Pin_12)
//#define GET_KEY2    ((GPIOB->IDR & GPIO_Pin_13) == GPIO_Pin_13)
//#define GET_KEY3    ((GPIOB->IDR & GPIO_Pin_14) == GPIO_Pin_14)
//#define GET_KEY4    ((GPIOB->IDR & GPIO_Pin_15) == GPIO_Pin_15)
//#define GET_KEY5    ((GPIOC->IDR & GPIO_Pin_6) == GPIO_Pin_6)
//#define GET_KEY6    ((GPIOC->IDR & GPIO_Pin_7) == GPIO_Pin_7)
//#define GET_KEY7    ((GPIOC->IDR & GPIO_Pin_8) == GPIO_Pin_8)
//#define GET_KEY8    ((GPIOC->IDR & GPIO_Pin_9) == GPIO_Pin_9)
//#define GET_KEY9    ((GPIOA->IDR & GPIO_Pin_8) == GPIO_Pin_8)
//#define GET_KEY10   ((GPIOA->IDR & GPIO_Pin_9) == GPIO_Pin_9)
//#define GET_KEY11   ((GPIOA->IDR & GPIO_Pin_10) == GPIO_Pin_10)
//#define GET_KEY12   ((GPIOA->IDR & GPIO_Pin_11) == GPIO_Pin_11)
//#define GET_KEY13   ((GPIOC->IDR & GPIO_Pin_12) == GPIO_Pin_12)
//#define GET_KEY14   ((GPIOC->IDR & GPIO_Pin_13) == GPIO_Pin_13)
//#define GET_KEY15   ((GPIOC->IDR & GPIO_Pin_14) == GPIO_Pin_14)
//#define GET_KEY16   ((GPIOC->IDR & GPIO_Pin_15) == GPIO_Pin_15)

//#define KEY1	(1<<12)
//#define KEY2	(1<<13)
//#define KEY3	(1<<14)			  
//#define KEY4	(1<<15)
//#define KEY5	(1<<6)
//#define KEY6	(1<<7)
//#define KEY7	(1<<8)
//#define KEY8   (1<<9)
//#define KEY9   (1<<8)
//#define KEY10	(1<<9)
//#define KEY11	(1<<10)
//#define KEY12	(1<<11)
//#define KEY13	(1<<12)
//#define KEY14	(1<<13)
//#define KEY15	(1<<14)
//#define KEY16	(1<<15)


//#define KEY_GP1  ((GPIOB->IDR&((KEY1)|(KEY2)|(KEY3)|(KEY4)))>>12)
//#define KEY_GP2  ((GPIOC->IDR&((KEY5)|(KEY6)|(KEY7)|(KEY8)))>>2)
//#define KEY_GP3  (GPIOA->IDR&((KEY9)|(KEY10)|(KEY11)|(KEY12)))
//#define KEY_GP4  (GPIOC->IDR&((KEY13)|(KEY14)|(KEY15)|(KEY16)))


//#define ScanKeyAll()  (KEY_GP1|KEY_GP2|KEY_GP3|KEY_GP4)

u8 ScanKey(u8 KeyNum);

#endif
