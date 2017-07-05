/***********************************************************
* Copyright (c) 2015,湖南农业大学机器人小组
* All rights reserved.
*
* 文件名称：Mpu6050.c
* 文件标识：无
* 摘    要：
*
* 当前版本：1.0
* 作    者: Huws
* 完成日期：2015年11月10日
*
* 平    台：STM32_F103    
*************************************************************/

#include "Mpu6050.h"

//static 
unsigned char MPU6050_Buffer[14];
/* Save Data drift */ 
Int16_xyz GYRO_OFFSET;
Int16_xyz ACC_OFFSET;

Int16_xyz MPU6050_DataAcc;
Int16_xyz MPU6050_DataGyr;

/* Data drift Flag */
static unsigned char GYRO_OFFSET_OK = 0;
static unsigned char ACC_OFFSET_OK = 0;


#define 	MPU6050_MAX		32767
#define		MPU6050_MIN		-32768


/* delay_ms function */
void Delay_ms_mpu(unsigned short int nms)
{
    unsigned short int i;
	unsigned short int j;
	for(i = 0; i < nms; i++)
	{
		for(j = 0; j < 8500; j++);
	}
}

/* MPU6050 Init */
void MPU6050_Init(void)
{
	Delay_ms_mpu(50);	
	MPU6050_Sleep(0);
	Delay_ms_mpu(200);
	MPU6050_Set_Clock(MPU6050_CLOCK_PLL_XGYRO);
	Delay_ms_mpu(50);
	MPU6050_Set_GyroRange(MPU6050_GYRO_FS_2000);
	Delay_ms_mpu(50);
	MPU6050_Set_AccelRange(MPU6050_ACCEL_FS_4);
	Delay_ms_mpu(50);
	MPU6050_Set_DLPF(MPU6050_DLPF_BW_42);
	Delay_ms_mpu(50);
	/* 关闭MPU6050的IIC主机模式 */
    MPU6050_Set_IICMasterMode(0);
	Delay_ms_mpu(50);
	MPU6050_Set_IICBypassEnabled(1);
	Delay_ms_mpu(50);
}




/* Get MPU6050 Data */
void MPU6050_DataAnl(Int16_xyz *data_TempAcc, Int16_xyz *data_TempGyr)
{
    int acc_x; 
    int acc_y;
    int acc_z;
    int gyr_x;
    int gyr_y;
    int gyr_z;

    acc_x =((((short int)MPU6050_Buffer[0])	<< 8) | MPU6050_Buffer[1]) - ACC_OFFSET.X;
    acc_y =((((short int)MPU6050_Buffer[2])	<< 8) | MPU6050_Buffer[3]) - ACC_OFFSET.Y;
	acc_z =((((short int)MPU6050_Buffer[4])	<< 8) | MPU6050_Buffer[5]) - ACC_OFFSET.Z;
		
		/* Give up Temp */
	gyr_x =((((short int)MPU6050_Buffer[8])	 << 8) | MPU6050_Buffer[9]) - GYRO_OFFSET.X;
    gyr_y =((((short int)MPU6050_Buffer[10]) << 8) | MPU6050_Buffer[11]) - GYRO_OFFSET.Y;
	gyr_z =((((short int)MPU6050_Buffer[12]) << 8) | MPU6050_Buffer[13]) - GYRO_OFFSET.Z;
		
		
	if(gyr_z >= -5 && gyr_z <= 5)
	{
		gyr_z = 0;
	}
		
    acc_x > MPU6050_MAX ? MPU6050_MAX : acc_x;
    acc_y > MPU6050_MAX ? MPU6050_MAX : acc_y;
    acc_z > MPU6050_MAX ? MPU6050_MAX : acc_z;
    acc_x < MPU6050_MIN ? MPU6050_MIN : acc_x;
    acc_y < MPU6050_MIN ? MPU6050_MIN : acc_y;
    acc_z < MPU6050_MIN ? MPU6050_MIN : acc_z;

    gyr_x >	MPU6050_MAX ?	MPU6050_MAX : gyr_x;
    gyr_y >	MPU6050_MAX ?	MPU6050_MAX : gyr_y;
    gyr_z >	MPU6050_MAX ?	MPU6050_MAX : gyr_z;
    gyr_x <	MPU6050_MIN ?	MPU6050_MIN : gyr_x;
    gyr_y <	MPU6050_MIN ?	MPU6050_MIN : gyr_y;
    gyr_x <	MPU6050_MIN ?	MPU6050_MIN : gyr_z;
		
	data_TempAcc->X = acc_x;
	data_TempAcc->Y = acc_y;
	data_TempAcc->Z = acc_z;
	data_TempGyr->X = gyr_x;
	data_TempGyr->Y = gyr_y;
	data_TempGyr->Z = gyr_z;
		
	if(GYRO_OFFSET_OK == 1)
	{
		static int TempGx = 0;
		static int TempGy = 0;
		static int TempGz = 0;
		static unsigned char Cnt_g = 0;
				
		if(Cnt_g == 0)
		{
			TempGx = 0;
			TempGy = 0;
			TempGz = 0;
			GYRO_OFFSET.X = 0;
			GYRO_OFFSET.Y = 0;
			GYRO_OFFSET.Z = 0;
			Cnt_g = 1;
			return;
		}
		TempGx += data_TempGyr->X;
		TempGy += data_TempGyr->Y;
		TempGz += data_TempGyr->Z;
		if(Cnt_g == 200)
		{
			GYRO_OFFSET.X = TempGx / Cnt_g;
			GYRO_OFFSET.Y = TempGy / Cnt_g;
			GYRO_OFFSET.Z = TempGz / Cnt_g;
//			EE_Save_MPU6050_Gyro_Offset();
			Cnt_g  = 0;
			GYRO_OFFSET_OK = 0;
			return;
	    }
		Cnt_g++;
	}
	//水平校准
	if(ACC_OFFSET_OK == 1)
	{
		static int TempAx = 0;
		static int TempAy = 0;
		static int TempAz = 0;
		static unsigned char Cnt_a = 0;
				
		if(Cnt_a == 0)
		{
			TempAx = 0;
			TempAy = 0;
			TempAz = 0;
			ACC_OFFSET.X = 0;
			ACC_OFFSET.Y = 0;
			ACC_OFFSET.Z = 0;
			Cnt_a = 1;
			return;
		}
		TempAx += data_TempAcc->X;
		TempAy += data_TempAcc->Y;
		if(Cnt_a == 200)
		{
			ACC_OFFSET.X = TempAx / Cnt_a;
			ACC_OFFSET.Y = TempAy / Cnt_a;
			ACC_OFFSET.Z = TempAz / Cnt_a;
//			EE_Save_MPU6050_Acc_Offset();
			Cnt_a = 1;
			ACC_OFFSET_OK = 0;
			return;
		}
	Cnt_a++;				
	}
}


void MPU6050_Offset_Set(void)
{
    unsigned char Times;
	int GYR_X = 0;
	int GYR_Y = 0;
	int GYR_Z = 0;
		
	for(Times = 0; Times < 100; Times++)
	{
		MPU6050Read();
		MPU6050_DataAnl(&MPU6050_DataAcc, &MPU6050_DataGyr);
		GYR_X += MPU6050_DataGyr.X;
		GYR_Y += MPU6050_DataGyr.Y;
		GYR_Z += MPU6050_DataGyr.Z;
		Delay_ms_mpu(10);
	}
	GYRO_OFFSET.X = GYR_X / 100;
	GYRO_OFFSET.Y = GYR_Y / 100;
	GYRO_OFFSET.Z = GYR_Z / 100;
//	EE_Save_MPU6050_Gyro_Offset();
//	EE_Save_MPU6050_Acc_Offset();
}


void MPU6050_WriteReg(unsigned char Reg_Add, unsigned char Data)
{
    IIC_Start();
	IIC_SendByte(MPU6050_SLAVE_ADDRESS);
	IIC_WaitAck();
	IIC_SendByte(Reg_Add);
	IIC_WaitAck();
	IIC_SendByte(Data);
	IIC_WaitAck();
	IIC_Stop();
}

void MPU6050_ReadData(unsigned char Reg_Add, unsigned char *Read, unsigned char num)
{
    unsigned char i;
		
	IIC_Start();
	IIC_SendByte(MPU6050_SLAVE_ADDRESS);
	IIC_WaitAck();
	IIC_SendByte(Reg_Add);
	IIC_WaitAck();
		
	IIC_Start();
	IIC_SendByte(MPU6050_SLAVE_ADDRESS + 1);
	IIC_WaitAck();
		
	for(i = 0; i < (num -1); i++)
	{
		*Read = IIC_ReceiveByte(1);
	    Read++;
	}
	*Read = IIC_ReceiveByte(0);
	IIC_Stop();		
}

void IIC_WriteBit(unsigned char Reg, unsigned char BitNum, unsigned char data)
{
    unsigned char b;
	MPU6050_ReadData(Reg, &b, 1);
	b = (data != 0) ? (b | (1 << BitNum)) : (b & ~(1 << BitNum));
	MPU6050_WriteReg(Reg, b);
}

void IIC_WriteBits(unsigned char Reg, unsigned char BitStart, unsigned char Length, unsigned char Data)
{
  unsigned char b;
	unsigned char mask;
	MPU6050_ReadData(Reg, &b, 1);
	mask = (0xFF << (BitStart + 1)) | 0xFF >> ((8 - BitStart) + Length - 1);
	Data <<= (8 - Length);
	Data >>= (7 - BitStart);
	b &= mask;
	b |= Data;
	MPU6050_WriteReg(Reg, b);
}


void MPU6050Read(void)
{
    MPU6050_ReadData(MPU6050_ACC_OUT, MPU6050_Buffer, 14);
}


void MPU6050_Sleep(unsigned char Enabled)
{
    IIC_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, Enabled);
}



void MPU6050_Set_IICMasterMode(unsigned char Enabled)
{
    IIC_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, Enabled);
}


void MPU6050_Set_IICBypassEnabled(unsigned char Enabled)
{
    IIC_WriteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, Enabled);
}



void MPU6050_Set_DLPF(unsigned char Mode)
{
    IIC_WriteBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, Mode);
}


void MPU6050_Set_GyroRange(unsigned char Range)
{
    IIC_WriteBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, Range);
}


void MPU6050_Set_AccelRange(unsigned char Range)
{
    IIC_WriteBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, Range);
}


void MPU6050_Set_Clock(unsigned char Source)
{
    IIC_WriteBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, Source);
}


void MPU6050_CalOff_Acc(void)
{
    ACC_OFFSET_OK = 1;
}


void MPU6050_CalOff_Gyr(void)
{
    GYRO_OFFSET_OK = 1;
}

















