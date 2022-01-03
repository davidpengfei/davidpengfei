/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "stdlib.h"
#include "ALL_DATA.h"
#include "mpu6050.h"
#include "I2C.h"
#include <string.h>
#include "LED.h"
#include "kalman.h"
#include "delay.h"
#include "AT24C02.h"
#include "math.h"
#include "myMath.h"
#include "attitude_process.h"
#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1


/*******************************************************************************************************/
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIGL			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_ADDRESS	0x3B
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define GYRO_ADDRESS  0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		  0x75	//IIC地址寄存器(默认数值0x68，只读)

//-------------------------------6050系列---------------------------------
#define MPU6050_PRODUCT_ID 0x68
#define MPU6050A_PRODUCT_ID 0x98
#define MPU6052C_PRODUCT_ID 0x72

#define MPU6050_ADDRESS 0xD0//0x68
/*******************************************************************************************************/

struct {
	int16_t X;
	int16_t Y;
	int16_t Z;	
}Mpu_GyroOffset;


static volatile int16_t *pMpu = (int16_t *)&Device.MPU6050;


/****************************************************************************************
MPU6050初始化
*@brief   
*@brief  
*@param[in]
*****************************************************************************************/
int8_t MpuInit(void) //初始化
{
	uint8_t check = SUCCESS;//检查，I2C返回值为FAILED的时候则读取错误
	
		check += 		IIC_Write_One_Byte(0xD0,PWR_MGMT_1  , 0x00);//关闭所有中断,解除休眠
    check +=     IIC_Write_One_Byte(0xD0,SMPLRT_DIV  , 0x00); // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz
    check +=    IIC_Write_One_Byte(0xD0,CONFIGL  , 0x02); //内部低通滤波频率，98hz
        //IIC_Write_One_Byte(0xD0,MPU_CONFIG  , 0x03); //内部低通滤波频率，44hz
    check +=    IIC_Write_One_Byte(0xD0,GYRO_CONFIG , 0x10);//1000deg/s
    check +=    IIC_Write_One_Byte(0xD0,ACCEL_CONFIG, 0x10);// Accel scale 8g (4096 LSB/g)	
	if(check == FAILED)  //如果陀螺仪不正常
	{
		return FAILED;
	}
	
	IIC_Read_One_Byte(MPU6050_ADDRESS, 0x75,&check);  //判断MPU6050地址
	
	if(check!= MPU6050_PRODUCT_ID && check!= MPU6052C_PRODUCT_ID  && check!= MPU6050A_PRODUCT_ID) //如果地址正确	
		return FAILED;
	
	//---------------------------------------------------------------------------校准	
 //六面校准

		if(read_AT24C02(MPU6050_SIX_B_OFFSET,0,0)==0xaa) //读取的eeprom，如果已经六面校准了，
		{
				read_AT24C02(MPU6050_SIX_B_OFFSET,B,12);	//如果已经校准过了 六面校准B值
				read_AT24C02(MPU6050_SIX_K_OFFSET,K,12);	//如果已经校准过了 六面校准K值
		}	
		if(read_AT24C02(GRYO_OFFSET,0,0)==0xaa) //如果校准了角速度，	则读取eeprom
			read_AT24C02(GRYO_OFFSET,&Mpu_GyroOffset,6);	//如果已经校准过了 六面校准K值

	return SUCCESS;

}
/****************************************************************************************
MPU6050坐标系
                     机头向前
                   ∧Y
									 |				
									 |
									 |
								   |
			Z					 	 |
				\				   |										 
					\	   		 |										 
						\  	   |										 
						 	\		 |							 
								\ |
	                 O--------------------->X
*****************************************************************************************/
/****************************************************************************************
获取MPU6050数据
*@brief    
*@brief   
*@param[in]
*****************************************************************************************/


#define  Acc_Read() IIC_read_Bytes(MPU6050_ADDRESS, 0X3B,6,buffer) //读取加速度
#define  Gyro_Read() IIC_read_Bytes(MPU6050_ADDRESS, 0x43,6,&buffer[6])  //  读取角速度
int8_t MpuGetData(void) //读取陀螺仪数据
{
	  uint8_t i;
    uint8_t buffer[12];
	
		I2C_FastMode = 0; //快速模式
		Gyro_Read();//读取角速度
	  Acc_Read();//读取加速度
	
		for(i=0;i<6;i++)
		{
			pMpu[i] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1]);		//数据整合
		}	
		Device.MPU6050.gyroX-=Mpu_GyroOffset.X;//角速度减去静止时的偏差
		Device.MPU6050.gyroY-=Mpu_GyroOffset.Y;
		Device.MPU6050.gyroZ-=Mpu_GyroOffset.Z;

		return SUCCESS;
}


/****************************************************************************************
*@brief   角速度静态校准，请置于完全静止,在上位机点击角速度校准。校准完成后将会保存数据到AT24C02
*@brief   
*@brief   initial and cmd call this
*@param[in]
*****************************************************************************************/
void Gyro_Calibration(void)
{
	u8 i;
	int32_t g_Gyro_xoffset = 0,g_Gyro_yoffset = 0,g_Gyro_zoffset = 0;
	uint8_t k=30;
	int16_t LastGyro[3] = {0};
	int16_t ErrorGyro[3];		
	const int8_t MAX_GYRO_QUIET = 5;
	const int8_t MIN_GYRO_QUIET = -5;	
	
	if(Command.FlightMode != LOCK) // 只有在锁定状态才能校准，解锁后不能校准。
	{
		return;
	}		
	if(!Command.GyroOffset)//如果不处于校准状态则立刻返回
	{
		return;
	}
	LED.color = YELLOW;//校准指示灯为黄色
	Mpu_GyroOffset.X = Mpu_GyroOffset.Y = Mpu_GyroOffset.Z = 0;//清0
//-------------------------------------------------	
 //判断飞控是否处于静止状态	
	while(k--) 
	{
		do
		{
			delay_ms(10);
			MpuGetData();
			for(i=0;i<3;i++)
			{
				ErrorGyro[i] = pMpu[i+3] - LastGyro[i];
				LastGyro[i] = pMpu[i+3];	
			}			
		}while ((ErrorGyro[0] >  MAX_GYRO_QUIET )|| (ErrorGyro[0] < MIN_GYRO_QUIET)
					||(ErrorGyro[1] > MAX_GYRO_QUIET )|| (ErrorGyro[1] < MIN_GYRO_QUIET)
					||(ErrorGyro[2] > MAX_GYRO_QUIET )|| (ErrorGyro[2] < MIN_GYRO_QUIET)
						);
	}	
//-------------------------------------------------		
	for (i = 0; i < 100; i++)			//连续采样100次，一共耗时100*3=300ms
	{
		MpuGetData();						//读取MPU6050的值
		g_Gyro_xoffset +=Device.MPU6050.gyroX;
		g_Gyro_yoffset +=Device.MPU6050.gyroY;
		g_Gyro_zoffset +=Device.MPU6050.gyroZ;
	}
	Mpu_GyroOffset.X = g_Gyro_xoffset/100;//得到静态偏移值//采集了100组做平均
	Mpu_GyroOffset.Y =	g_Gyro_yoffset/100;
	Mpu_GyroOffset.Z =	g_Gyro_zoffset/100;
	Command.GyroOffset = 0;//校准完成
	write_AT24C02(GRYO_OFFSET,&Mpu_GyroOffset,6);//保存校准数据
}

/****************************************************************************************
*@brief  在上位机连续点击校准，根据上位机提示进行操作。校准完成后将会保存数据到AT24C02
*@brief   
*@brief   
*@param[in]
*****************************************************************************************/
/***************加速度计6面矫正，参考APM代码，配合上位机进行现场矫正**************************/
#define AcceMax_1G      4096
#define GRAVITY_MSS     9.80665f
#define ACCEL_TO_1G     GRAVITY_MSS/AcceMax_1G
#define One_G_TO_Accel  AcceMax_1G/GRAVITY_MSS
typedef struct
{
 float x;
 float y;
 float z;
}Acce_Unit;
//-------------------------最小二乘法，摘自APM，比较难理解，可按照套路照搬----------------
void Calibrate_Reset_Matrices(float dS[6], float JS[6][6])
{
    int16_t j,k;
    for( j=0; j<6; j++ )
    {
        dS[j] = 0.0f;
        for( k=0; k<6; k++ )
        {
            JS[j][k] = 0.0f;
        }
    }
}

void Calibrate_Find_Delta(float dS[6], float JS[6][6], float delta[6])
{
    //Solve 6-d matrix equation JS*x = dS
    //first put in upper triangular form
    int16_t i,j,k;
    float mu;
    //make upper triangular
    for( i=0; i<6; i++ ) {
        //eliminate all nonzero entries below JS[i][i]
        for( j=i+1; j<6; j++ ) {
            mu = JS[i][j]/JS[i][i];
            if( mu != 0.0f ) {
                dS[j] -= mu*dS[i];
                for( k=j; k<6; k++ ) {
                    JS[k][j] -= mu*JS[k][i];
                }
            }
        }
    }
    //back-substitute
    for( i=5; i>=0; i-- ) {
        dS[i] /= JS[i][i];
        JS[i][i] = 1.0f;

        for( j=0; j<i; j++ ) {
            mu = JS[i][j];
            dS[j] -= mu*dS[i];
            JS[i][j] = 0.0f;
        }
    }
    for( i=0; i<6; i++ ) {
        delta[i] = dS[i];
    }
}

void Calibrate_Update_Matrices(float dS[6],
                               float JS[6][6],
                               float beta[6],
                               float data[3])
{
    int16_t j, k;
    float dx, b;
    float residual = 1.0;
    float jacobian[6];
    for(j=0;j<3;j++)
    {
        b = beta[3+j];
        dx = (float)data[j] - beta[j];
        residual -= b*b*dx*dx;
        jacobian[j] = 2.0f*b*b*dx;
        jacobian[3+j] = -2.0f*b*dx*dx;
    }

    for(j=0;j<6;j++)
    {
        dS[j]+=jacobian[j]*residual;
        for(k=0;k<6;k++)
        {
            JS[j][k]+=jacobian[j]*jacobian[k];
        }
    }
}

uint8 Calibrate_accel(Acce_Unit accel_sample[6],
                      Acce_Unit *accel_offsets,
                      Acce_Unit *accel_scale)
{
    int16_t i;
    int16_t num_iterations = 0;
    float eps = 0.000000001;
    float change = 100.0;
    float data[3]={0};
    float beta[6]={0};
    float delta[6]={0};
    float ds[6]={0};
    float JS[6][6]={0};
    unsigned char temp = SUCCESS;
    // reset
    beta[0] = beta[1] = beta[2] = 0;
    beta[3] = beta[4] = beta[5] = 1.0f/GRAVITY_MSS;
    while( num_iterations < 20 && change > eps ) {
        num_iterations++;
        Calibrate_Reset_Matrices(ds, JS);

        for( i=0; i<6; i++ ) {
            data[0] = accel_sample[i].x;
            data[1] = accel_sample[i].y;
            data[2] = accel_sample[i].z;
            Calibrate_Update_Matrices(ds, JS, beta, data);

        }
        Calibrate_Find_Delta(ds, JS, delta);
        change =    delta[0]*delta[0] +
                    delta[0]*delta[0] +
                    delta[1]*delta[1] +
                    delta[2]*delta[2] +
                    delta[3]*delta[3] / (beta[3]*beta[3]) +
                    delta[4]*delta[4] / (beta[4]*beta[4]) +
                    delta[5]*delta[5] / (beta[5]*beta[5]);
        for( i=0; i<6; i++ ) {
            beta[i] -= delta[i];
        }
    }
    // copy results out
    accel_scale->x = beta[3] * GRAVITY_MSS;
    accel_scale->y = beta[4] * GRAVITY_MSS;
    accel_scale->z = beta[5] * GRAVITY_MSS;
    accel_offsets->x = beta[0] * accel_scale->x;
    accel_offsets->y = beta[1] * accel_scale->y;
    accel_offsets->z = beta[2] * accel_scale->z;

    // sanity check scale
    if(fabsf(accel_scale->x-1.0f) > 0.2f
         || fabsf(accel_scale->y-1.0f) > 0.2f
           || fabsf(accel_scale->z-1.0f) > 0.2f )
    {
        temp = FAILED;
    }
    // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
    if(fabsf(accel_offsets->x) > 3.5f
         || fabsf(accel_offsets->y) > 3.5f
           || fabsf(accel_offsets->z) > 3.5f )
    {
        temp = FAILED;
    }
    // return success or failure
    return temp;
}
//------------------最小二乘法-------------------------------------结束
//下面开始采集校准数据
/*第一面飞控平放，Z轴正向朝着正上方（正面垂直朝上），Z axis is about 1g,X、Y is about 0g*/
/*第二面飞控平放，X轴正向朝着正上方（机头垂直朝上），X axis is about 1g,Y、Z is about 0g*/
/*第三面飞控平放，X轴正向朝着正下方（左侧垂直朝上），X axis is about -1g,Y、Z is about 0g*/
/*第四面飞控平放，Y轴正向朝着正下方（右侧垂直朝上），Y axis is about -1g,X、Z is about 0g*/
/*第五面飞控平放，Y轴正向朝着正上方（机尾垂直朝上），Y axis is about 1g,X、Z is about 0g*/
/*第六面飞控平放，Z轴正向朝着正下方（背面垂直朝上），Z axis is about -1g,X、Y is about 0g*/
Acce_Unit acce_sample[6]={0};//三行6列，保存6面待矫正数据
float acce_sample_sum[3]={0,0,0};//加速度和数据
uint16_t  sample_num;// 采样次数

void Accel_six_Calibartion(void)	
{
	static uint8_t status = 0;
	
	if(Command.FlightMode != LOCK) // 只有在锁定状态才能校准，解锁后不能校准。
	{
		status = 255;
	}	
	if(Command.six_acc_offset == 0x20) // 上位机发送过来的退出六面校准。
	{
		status = 255;
	}		
	
	//角速度模长
	float Yaw_Gyro = Device.MPU6050.gyroZ * Gyro_G;
	float Pitch_Gyro = Device.MPU6050.gyroY * Gyro_G;
	float Roll_Gyro = Device.MPU6050.gyroX * Gyro_G;
	float Gyro_Length=sqrt(Yaw_Gyro*Yaw_Gyro
												 +Pitch_Gyro*Pitch_Gyro
																 +Roll_Gyro*Roll_Gyro);	
  switch(status) //注意：每一部都要做到垂直静止，直到闪烁的灯变为青色。黄色处于正在校准，青色为正常解锁前的状态
	{
		case 0:
					status = 1;			
			break;
			//-------------------------------------------------
		case 1:
				if(Command.six_acc_offset == 0x21)//进入六面校准模式 第一步
				{
					uint8_t i;
					for(i=0;i<6;i++)
					{
				//		Accel_Calibration_Finished[i]=0;//对应面标志位清零
						acce_sample[i].x=0; //清空对应面的加速度计量
						acce_sample[i].y=0; //清空对应面的加速度计量
						acce_sample[i].z=0; //清空对应面的加速度计量
					}
					sample_num = 0;	
					acce_sample_sum[0] =acce_sample_sum[1] = acce_sample_sum[2] = 0;	 //对应面数据清0
					status=2;
				}
			break;
		case 2:		//第一次点校准，正面（有元器件的）垂直朝上
				LED.color = YELLOW;//黄色闪烁为六面校准
				Command.six_acc_offset = 1;
				if(Gyro_Length>20.0f) //步长太大，认为没静止，则不校准
					break;
				else
				{
					if(sample_num<200) //取200组
					{
						 sample_num++;	
						 acce_sample_sum[0]+=Attitude.accX_correct*ACCEL_TO_1G;//加速度计转化为1g量程下
						 acce_sample_sum[1]+=Attitude.accY_correct*ACCEL_TO_1G;//加速度计转化为1g量程
						 acce_sample_sum[2]+=Attitude.accZ_correct*ACCEL_TO_1G;//加速度计转化为1g量程
					}
					else
					{
						acce_sample[0].x = acce_sample_sum[0]/200;//取平均值
						acce_sample[0].y = acce_sample_sum[1]/200;
						acce_sample[0].z = acce_sample_sum[2]/200;
						LED.color = CYAN;	
						status = 3;
						Command.six_acc_offset = 0; //单面校准完毕
					}
				}	
			break;
		case 3:				
				if(Command.six_acc_offset == 0x22)//进入六面校准模式 第二步
				{
					sample_num = 0;	
					acce_sample_sum[0] =acce_sample_sum[1] = acce_sample_sum[2] = 0;	
					status = 4;
				}
			break;
		case 4: //第二次点校准，有排针的一边垂直朝上
				LED.color = YELLOW;
				Command.six_acc_offset = 1;
				if(Gyro_Length>20.0f)
					break;
				else
				{
					if(sample_num<200)
					{
						 sample_num++;	
						 acce_sample_sum[0]+=Attitude.accX_correct*ACCEL_TO_1G;//加速度计转化为1g量程下
						 acce_sample_sum[1]+=Attitude.accY_correct*ACCEL_TO_1G;//加速度计转化为1g量程
						 acce_sample_sum[2]+=Attitude.accZ_correct*ACCEL_TO_1G;//加速度计转化为1g量程
					}
					else
					{
						acce_sample[1].x = acce_sample_sum[0]/200;
						acce_sample[1].y = acce_sample_sum[1]/200;
						acce_sample[1].z = acce_sample_sum[2]/200;
						LED.color = CYAN;	
						Command.six_acc_offset = 0;
						status = 5;
					}
				}									
			break;	
		case 5:
				if(Command.six_acc_offset == 0x23)//进入六面校准模式 第三步
				{
					sample_num = 0;	
					acce_sample_sum[0] =acce_sample_sum[1] = acce_sample_sum[2] = 0;						
					status = 6;
				}
			break;
		case 6:		//第三次点校准，与有排针对立的那边没排针的垂直朝上
				LED.color = YELLOW;
				Command.six_acc_offset = 1;
				if(Gyro_Length>20.0f)
					break;
				else
				{
					if(sample_num<200)
					{
						 sample_num++;	
						 acce_sample_sum[0]+=Attitude.accX_correct*ACCEL_TO_1G;//加速度计转化为1g量程下
						 acce_sample_sum[1]+=Attitude.accY_correct*ACCEL_TO_1G;//加速度计转化为1g量程
						 acce_sample_sum[2]+=Attitude.accZ_correct*ACCEL_TO_1G;//加速度计转化为1g量程
					}
					else
					{
						acce_sample[2].x = acce_sample_sum[0]/200;
						acce_sample[2].y = acce_sample_sum[1]/200;
						acce_sample[2].z = acce_sample_sum[2]/200;
						LED.color = CYAN;	
						Command.six_acc_offset = 0;
						status = 7;
					}
				}		
			break;
		case 7:
				if(Command.six_acc_offset == 0x24)//进入六面校准模式 第四步
				{
						sample_num = 0;	
					acce_sample_sum[0] =acce_sample_sum[1] = acce_sample_sum[2] = 0;					
					status = 8;
				}
			break;
		case 8:		//第四次点校准，机尾垂直朝上
				LED.color = YELLOW;
				Command.six_acc_offset = 1;
				if(Gyro_Length>20.0f)
					break;
				else
				{
					if(sample_num<200)
					{
						 sample_num++;	
						 acce_sample_sum[0]+=Attitude.accX_correct*ACCEL_TO_1G;//加速度计转化为1g量程下
						 acce_sample_sum[1]+=Attitude.accY_correct*ACCEL_TO_1G;//加速度计转化为1g量程
						 acce_sample_sum[2]+=Attitude.accZ_correct*ACCEL_TO_1G;//加速度计转化为1g量程
					}
					else
					{
						acce_sample[3].x = acce_sample_sum[0]/200;
						acce_sample[3].y = acce_sample_sum[1]/200;
						acce_sample[3].z = acce_sample_sum[2]/200;
						LED.color = CYAN;	
						Command.six_acc_offset = 0;
						status = 9;
					}
				}		
			break;
		case 9:
				if(Command.six_acc_offset == 0x25)//进入六面校准模式 第五步
				{
						sample_num = 0;	
					acce_sample_sum[0] =acce_sample_sum[1] = acce_sample_sum[2] = 0;							
					status = 10;
				}
			break;
		case 10:		//第五次点校准，背面没元器件的垂直朝上，
				LED.color = YELLOW;
		Command.six_acc_offset = 1;
				if(Gyro_Length>20.0f)
					break;
				else
				{
					if(sample_num<200)
					{
						 sample_num++;	
						 acce_sample_sum[0]+=Attitude.accX_correct*ACCEL_TO_1G;//加速度计转化为1g量程下
						 acce_sample_sum[1]+=Attitude.accY_correct*ACCEL_TO_1G;//加速度计转化为1g量程
						 acce_sample_sum[2]+=Attitude.accZ_correct*ACCEL_TO_1G;//加速度计转化为1g量程
					}
					else
					{
						acce_sample[4].x = acce_sample_sum[0]/200;
						acce_sample[4].y = acce_sample_sum[1]/200;
						acce_sample[4].z = acce_sample_sum[2]/200;
						LED.color = CYAN;	
						Command.six_acc_offset = 0;
						status = 11;
					}
				}		
			break;
		case 11:
				if(Command.six_acc_offset == 0x26)//进入六面校准模式 第六步
				{
					sample_num = 0;	
					acce_sample_sum[0] =acce_sample_sum[1] = acce_sample_sum[2] = 0;							
					status = 12;
				}
			break;
		case 12:		//第一次点校准，进入第一面校准
				LED.color = YELLOW;
		Command.six_acc_offset = 1;
				if(Gyro_Length>20.0f)
					break;
				else
				{
					if(sample_num<200)
					{
						 sample_num++;	
						 acce_sample_sum[0]+=Attitude.accX_correct*ACCEL_TO_1G;//加速度计转化为1g量程下
						 acce_sample_sum[1]+=Attitude.accY_correct*ACCEL_TO_1G;//加速度计转化为1g量程
						 acce_sample_sum[2]+=Attitude.accZ_correct*ACCEL_TO_1G;//加速度计转化为1g量程
					}
					else
					{
						acce_sample[5].x = acce_sample_sum[0]/200;
						acce_sample[5].y = acce_sample_sum[1]/200;
						acce_sample[5].z = acce_sample_sum[2]/200;
						LED.color = CYAN;	
						Command.six_acc_offset = 0;
						status = 254;
					}
				}	
			break;		
		case 254: //保存校准数据
			{
				  uint8_t Cal_Flag;
					Acce_Unit new_offset={
						0,0,0,
					};
					Acce_Unit new_scales={
						1.0,1.0,1.0,
					};
					Cal_Flag=Calibrate_accel(acce_sample,
                                &new_offset,
                                  &new_scales);//将所得6面数据  
						if(Cal_Flag == SUCCESS)
				   {
							B[0]=new_offset.x;//*One_G_TO_Accel;
							B[1]=new_offset.y;//*One_G_TO_Accel;
							B[2]=new_offset.z;//*One_G_TO_Accel;
							K[0]=new_scales.x;
							K[1]=new_scales.y;
							K[2]=new_scales.z;
						  write_AT24C02(MPU6050_SIX_B_OFFSET,B,12);//保存校准数据
						  delay_ms(10);
							write_AT24C02(MPU6050_SIX_K_OFFSET,K,12);//保存校准数据
					 }
			}
		case 255: 
			if(Command.FlightMode == LOCK) // 只有在锁定状态才能校准，解锁后不能校准。
			{
				status = 0;
			}	
			break;		
		default:
			break;	
	}
}



/**************************************END OF FILE*************************************/

