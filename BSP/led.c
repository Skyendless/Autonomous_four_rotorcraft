/***********************************************************
* Copyright (c) 2015,湖南农业大学机器人小组
* All rights reserved.
*
* 文件名称：led.c
* 文件标识：无
* 摘    要：
*
* 当前版本：1.0
* 作    者: Huws
* 完成日期：2015年11月08日
*
* 平    台：STM32_F103    
*************************************************************/


#include "main.h"

void Led_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
    
}
