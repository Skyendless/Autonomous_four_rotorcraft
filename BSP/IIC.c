/***********************************************************
* Copyright (c) 2015,湖南农业大学机器人小组
* All rights reserved.
*
* 文件名称：IIC.c
* 文件标识：无
* 摘    要：
*
* 当前版本：1.0
* 作    者: Huws
* 完成日期：2015年11月10日
*
* 平    台：STM32_F103    
*************************************************************/

#include "IIC.h"


/* A simple Delay */
static void IIC_delay(void)
{
    unsigned char i;
	for(i = 0; i < 10; i++);
}
	


/* IIC Start */
void IIC_Start(void)
{
    IIC_SDA_H;
	IIC_SCL_H;
	IIC_delay();
	IIC_SDA_L;
	IIC_delay();
	IIC_SCL_L;
	IIC_delay();
}


/* IIC Stop */
void IIC_Stop(void)
{
    IIC_SDA_L;
	IIC_SCL_H;
	IIC_delay();
	IIC_SDA_H;
}


/* IIC Send_Byte */
void IIC_SendByte(unsigned char Data)
{
    unsigned char i;
		
	for(i = 0; i < 8; i++)
	{
		if(Data & 0x80)
		{
			IIC_SDA_H;
		}
		else
		{
			IIC_SDA_L;
		}
		IIC_delay();
		IIC_SCL_H;
		IIC_delay();
		IIC_SCL_L;
		if(i == 7)
		{
			IIC_SDA_H;
		}
		Data <<= 1;
		IIC_delay();
	}
}


/* IIC ReadData */
unsigned char IIC_ReceiveByte(unsigned char ack)
{
    unsigned char i;
	unsigned char value;
		
	value = 0;
	for(i = 0; i < 8; i++)
	{
		value <<= 1;
		IIC_SCL_H;
		IIC_delay();
		if(IIC_SDA_READ())
		{
			value++;
		}
		IIC_SCL_L;
		IIC_delay();		
	}
	if(ack == 0)
	{
		IIC_NAck();
	}
	else
	{
		IIC_Ack();
	}
	return value;
}


/* IIC Wait a Ack */
unsigned char IIC_WaitAck(void)
{
    unsigned char re;
		
	IIC_SDA_H;
	IIC_delay();
	IIC_SCL_H;
	IIC_delay();
		
	if(IIC_SDA_READ())
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	IIC_SCL_L;
	IIC_delay();
	return re;
}



/* IIC Send Ack */
void IIC_Ack(void)
{
    IIC_SDA_L;
	IIC_delay();
	IIC_SCL_H;
	IIC_delay();
	IIC_SCL_L;
	IIC_delay();
	IIC_SDA_H;
}


/* IIC Send NAck */
void IIC_NAck(void)
{
    IIC_SDA_H;
	IIC_delay();
	IIC_SCL_H;
	IIC_delay();
	IIC_SCL_L;
	IIC_delay();
}


/* IIC Init */
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_IIC_PORT, ENABLE);	

    GPIO_InitStructure.GPIO_Pin = IIC_SCL | IIC_SDA;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 
    GPIO_Init(IIC_PORT, &GPIO_InitStructure);

    IIC_Stop();
}

/* IIC Cheack */
unsigned char IIC_CheckDevice(unsigned char Addr)
{
    unsigned char Ack;
	
	IIC_Start();
	IIC_SendByte(Addr | IIC_WR);
	Ack = IIC_WaitAck();
	IIC_Stop();
	return Ack;
}


unsigned char IIC_Write_1Byte(uint8_t slaveaddr, uint8_t reg, uint8_t reg_data)
{
    IIC_Start();
	IIC_SendByte(slaveaddr << 1);
	if(IIC_WaitAck() == 0)
	{
	    IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg);
	IIC_WaitAck();
	IIC_SendByte(reg_data);
	IIC_WaitAck();
	IIC_Stop();
	
	return 0;
}

uint8_t IIC_Read_1Byte(uint8_t slaveaddr, uint8_t reg, uint8_t *reg_data)
{
    IIC_Start();
	IIC_SendByte(slaveaddr << 1);
	if(  IIC_WaitAck() == 0 )
	{
	    IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg);
	IIC_WaitAck();
	IIC_Start();
	IIC_SendByte(slaveaddr << 1 | 0x01);
	IIC_WaitAck();
	*reg_data = IIC_ReceiveByte(0);
	IIC_Stop();
	
	return 0;
}

unsigned char IIC_Read_nByte(uint8_t slaveaddr, uint8_t reg_addr, uint8_t len, uint8_t *buf)
{

    IIC_Start();
	IIC_SendByte(slaveaddr << 1);
    IIC_WaitAck();
	IIC_SendByte(reg_addr);
	IIC_WaitAck();
	
	IIC_Start();
	IIC_SendByte(slaveaddr << 1 | 0x01);
	IIC_WaitAck();
	while(len)
	{
	    if(len == 1)
		{
		    *buf = IIC_ReceiveByte(0);
		}
		else
		{
		    *buf = IIC_ReceiveByte(1);
		}
		buf ++;
		len --;
	}
    IIC_Stop();
	return 0;
}







