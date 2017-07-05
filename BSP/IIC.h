/***********************************************************
* Copyright (c) 2015,����ũҵ��ѧ������С��
* All rights reserved.
*
* �ļ����ƣ�IIC.h
* �ļ���ʶ����
* ժ    Ҫ��
*
* ��ǰ�汾��1.0
* ��    ��: Huws
* ������ڣ�2015��11��10��
*
* ƽ    ̨��STM32_F103    
*************************************************************/


#ifndef _IIC_H_
#define _IIC_H_


#include "main.h"

#define IIC_SDA 		GPIO_Pin_11	 
#define IIC_SCL 		GPIO_Pin_10	
#define IIC_PORT        GPIOB
#define RCC_IIC_PORT 	RCC_APB2Periph_GPIOB		

//
#define IIC_SCL_L 		GPIO_ResetBits(IIC_PORT, IIC_SCL)
#define IIC_SCL_H 		GPIO_SetBits(IIC_PORT, IIC_SCL)
#define IIC_SDA_L 		GPIO_ResetBits(IIC_PORT, IIC_SDA)
#define IIC_SDA_H   	GPIO_SetBits(IIC_PORT, IIC_SDA)

#define IIC_SDA_READ()  GPIO_ReadInputDataBit(IIC_PORT, IIC_SDA)	/* ��SDA����״̬ */

#define IIC_WR	0		
#define IIC_RD	1		
 

static void IIC_delay(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_SendByte(unsigned char Data);
unsigned char IIC_ReceiveByte(unsigned char ack);
unsigned char IIC_WaitAck(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Init(void);
unsigned char IIC_CheckDevice(unsigned char Addr);
unsigned char IIC_Read_nByte(uint8_t slaveaddr, uint8_t reg_addr, uint8_t len, uint8_t *buf);
unsigned char IIC_Write_1Byte(uint8_t slaveaddr, uint8_t reg, uint8_t reg_data);
uint8_t IIC_Read_1Byte(uint8_t slaveaddr, uint8_t reg, uint8_t *reg_data);




#endif



























