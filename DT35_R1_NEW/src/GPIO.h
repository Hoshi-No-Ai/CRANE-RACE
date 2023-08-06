#ifndef _GPIO_H
#define _GPIO_H

void GPIO_Configuration(void);			   //GPIO

void GPIO_JTAG_Configuration(void);

/*RUNÖ¸Ê¾µÆ¿ØÖÆ*/
#define LED_ON  		GPIOC->BRR = GPIO_Pin_2
#define LED_OFF  		GPIOC->BSRR = GPIO_Pin_2
#define LED_ROLLING 	GPIOC->ODR ^= GPIO_Pin_2



#endif

