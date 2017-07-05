#ifndef __LED_H__
#define __LED_H__

void Led_Configuration(void);

#define    LED1(x)  x ? GPIO_SetBits(GPIOC, GPIO_Pin_6): GPIO_ResetBits(GPIOC, GPIO_Pin_6)

#endif
