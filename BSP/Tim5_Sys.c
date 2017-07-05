/***********************************************************
* Copyright (c) 2015,����ũҵ��ѧ������С��
* All rights reserved.
*
* �ļ����ƣ�Tim5_Sys.c
* �ļ���ʶ����
* ժ    Ҫ��
*
* ��ǰ�汾��1.0
* ��    ��: Huws
* ������ڣ�2015��11��08��
*
* ƽ    ̨��STM32_F103    
*************************************************************/

#include "main.h"
#include "Mpu6050.h"

void TIM5_Init(unsigned short Period)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef        NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
		
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = Period;
	TIM_TimeBaseStructure.TIM_Prescaler = 71;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM5, DISABLE);
}

void TIM5_Control(unsigned char Sta)
{
	if(Sta==0)
		TIM_Cmd(TIM5,DISABLE);
	if(Sta==1)
		TIM_Cmd(TIM5,ENABLE);
}


void TIM5_IRQ(void)     
{
	static unsigned short int TIM5_Cnt = 0;
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
    {
			TIM_ClearITPendingBit(TIM5, TIM_FLAG_Update);
			TIM5_Cnt++;
			TIM5_Cnt = TIM5_Cnt % 1000;  //1S
				
				if(TIM5_Cnt % 2 == 0)//2ms
			{
				MPU6050Read();
				ATT_FLAG = 1;
				Att_Angle_Count();
			}	
		}
}









