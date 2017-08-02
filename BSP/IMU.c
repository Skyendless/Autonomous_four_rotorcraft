/***********************************************************
* Copyright (c) 2015,湖南农业大学机器人小组
* All rights reserved.
*
* 文件名称：IMU.c
* 文件标识：无
* 摘    要：
*
* 当前版本：1.0
* 作    者: Huws
* 完成日期：2015年11月10日
*
* 平    台：STM32_F103    
*************************************************************/


#include "MPU6050.h"
#include "math.h"



#define RtA       57.324841f   //弧度到角度
#define AtR       0.0174533f   //度到弧度
#define ACC_G     0.0011963f   //加速度到G
#define Gyro_G    0.0609756f   //角速度到度   1度 = 0.0174533 弧度
#define Gyro_Gr   0.0010642f   //角速度变弧度
#define FILTER_NUM  20


float AngleOffset_Rol = 0;
float AngleOffset_Pit = 0;
unsigned char AngleOffset_FLAG = 1;
unsigned char ATT_FLAG = 1;


Int16_xyz MPU6050_DataAcc1;
Int16_xyz MPU6050_DataGyr1;
Int16_xyz MPU6050_DataAcc2;
Int16_xyz MPU6050_DataGyr2;
Int16_xyz MPU6050_DataAcc3;
Int16_xyz MPU6050_DataGyr3;
Int16_xyz Acc;
Int16_xyz Gyr;
Int16_xyz Acc_AVG;
float_Angle Att_Angle;


void Prepare_Data(Int16_xyz *acc_in, Int16_xyz *acc_out);
void IMU_Update(Int16_xyz *Gyr, Int16_xyz *Acc, float_Angle *angle);



float Num_To_Dps(short int Num)
{
    float Temp;
	Temp = (float)Num * Gyro_G; 
	return Temp;
}


void Prepare_Data(Int16_xyz *acc_in, Int16_xyz *acc_out)  //窗口滑动滤波
{
    /* Slide Window Filter */
	static unsigned char Filter_Cnt = 0;
	static short int ACC_X_BUF[FILTER_NUM];
	static short int ACC_Y_BUF[FILTER_NUM];
	static short int ACC_Z_BUF[FILTER_NUM];
	
	int temp1 = 0;
	int temp2 = 0;
	int temp3 = 0;
	unsigned char i;
		
	ACC_X_BUF[Filter_Cnt] = acc_in->X;
	ACC_Y_BUF[Filter_Cnt] = acc_in->Y;
	ACC_Z_BUF[Filter_Cnt] = acc_in->Z;
		
	for(i = 0; i < FILTER_NUM; i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
		
	acc_out->X = temp1 / FILTER_NUM;
	acc_out->Y = temp2 / FILTER_NUM;
	acc_out->Z = temp3 / FILTER_NUM;
	Filter_Cnt++;
	if(Filter_Cnt == FILTER_NUM)
	{
		Filter_Cnt = 0;
	}		
}



#define Kp     2.0f
#define Ki     0.001f
#define halfT  0.001f

/* 参数整定找最佳  从小到大顺序查 */
/* 先是比例后积分  最后再把微分加 */
/* 曲线震荡很频繁  比例度盘要放大 */
/* 曲线漂浮绕大弯  比例度盘往小扳 */
/* 曲线偏离回复慢  积分时间往下降 */
/* 曲线波动周期长  积分时间再加长 */
/* 曲线震荡频率快  先把微分降下来 */
/* 动差大来波动慢  微分时间应加长 */
/* 理想曲线两个波  前高后低四比一 */
/* 一看二调多分析  调节质量不会低 */


float q0 = 1; 
float q1 = 0;
float q2 = 0;
float q3 = 0;
float exInt = 0;
float eyInt = 0;
float ezInt = 0;


void IMU_Update(Int16_xyz *Gyr, Int16_xyz *Acc, float_Angle *angle)		//四元数 解算姿态
{
	float ax = Acc->X;
	float ay = Acc->Y;
	float az = Acc->Z;
	float gx = Gyr->X;
	float gy = Gyr->Y;
	float gz = Gyr->Z;
	float norm;
	float vx;
	float vy;
	float vz;
	float ex;
	float ey;
	float ez;
	static unsigned char i = 0;
	static float Angle_Pit_Sum = 0;
	static float Angle_Rol_Sum = 0;
		
	if(ax*ay*az==0)   
	{
	return;	
	}			 
		
	/* degree to Radian */
	gx *= Gyro_Gr;
	gy *= Gyro_Gr;
	gz *= Gyro_Gr;
		
	/* Accelerate change to unit vector */
	norm = sqrt(ax*ax + ay*ay + az*az);
    	
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
		
        vx = 2 * (q1*q3 - q0*q2);
	vy = 2 * (q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	/* error is the across product */
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
		
	exInt = exInt + ex * Ki;
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;

	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	q0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;
	
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
		
	angle->yaw += Gyr->Z*Gyro_G*0.002f;
	angle->pit = asin(-2* q1*q3 + 2*q0*q2) * 57.3 - AngleOffset_Rol;
	angle->rol = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3 - AngleOffset_Pit;	
    
	if(AngleOffset_FLAG)
    {
		i++;
		Angle_Rol_Sum += angle->rol;
		Angle_Pit_Sum += angle->pit;
		if(i == 30)
		{
			i = 0;
			AngleOffset_FLAG = 0;
			AngleOffset_Rol = Angle_Rol_Sum / 30;
			AngleOffset_Pit = Angle_Pit_Sum / 30;
		}
	}
}




/* count the attitude data */
void Att_Angle_Count(void) 
{
    static unsigned char att_cnt = 0;
	if(ATT_FLAG == 1)
	{
	    ATT_FLAG = 0;
		att_cnt++;

		if(att_cnt == 1)
		{
				MPU6050_DataAnl(&MPU6050_DataAcc1,&MPU6050_DataGyr1);
		}
				
		else
		{
			att_cnt = 0;
			MPU6050_DataAnl(&MPU6050_DataAcc2,&MPU6050_DataGyr2);								
			Acc.X = (MPU6050_DataAcc1.X + MPU6050_DataAcc2.X) / 2;
			Acc.Y = (MPU6050_DataAcc1.Y + MPU6050_DataAcc2.Y) / 2;
			Acc.Z = (MPU6050_DataAcc1.Z + MPU6050_DataAcc2.Z) / 2;
			Gyr.X = (MPU6050_DataGyr1.X + MPU6050_DataGyr2.X) / 2;
			Gyr.Y = (MPU6050_DataGyr1.Y + MPU6050_DataGyr2.Y) / 2;
			Gyr.Z = (MPU6050_DataGyr1.Z + MPU6050_DataGyr2.Z) / 2;
			Prepare_Data(&Acc, &Acc_AVG);
			IMU_Update(&Gyr, &Acc_AVG, &Att_Angle);						
		}
	}		
}








