/*******************************************************************
 *MPU6050
 *@brief 
 *@brief 
 *@time  2016.1.8
 *@editor小南&zin
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 *非授权使用人员，禁止使用。禁止传阅，违者一经发现，侵权处理。
 ******************************************************************/
#include "AK8975.h"
#include <string.h>
#include "myMath.h"
#include "I2C.h"
#include "usart.h"
#include "delay.h"
#include "kalman.h"
#include "ALL_DATA.h"
#include "AT24C02.h"
#include "imu.h"
#include "attitude_process.h"
#include <stdio.h>


int16_t MagOffset[3];  

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1
//***********AK8963 磁力计寄存器******
#define MAG_ID				0x00
#define MAG_ID_BUMBER				0x48
#define MAG_Status1			0x02
#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08

#define MAG_Status2			0x09
#define MAG_Control			0x0A
#define MAG_Self_test		0x0C
#define MAG_Self_test		0x0C


#define	MAG_ADDR 0x18   //IIC写入时的地址字节数据，+1为读取



#define MAG_Restart() IIC_Write_One_Byte(MAG_ADDR, MAG_Control, 0x11)  //0x10 16位模式 | 0x01 单次测量模式  //开始转换，和读取数据之间必须间隔7ms以上的时间


void Get_Mag_OFFSET(void);



//磁力计做地理坐标系与飞控坐标系转换使用
/****************************************************************************************
                   机头
                    ∧
										 |
		         				 O--------------------->X  
										/|
									 / |
								  /  |
								 /	 |
								/		 |										 
						   /		 |										 
						  /		   |										 
						 /		   |										 
						Z				 |
		磁力计Z轴指向地	 Y					 
*****************************************************************************************/

/****************************************************************************************
*@brief   
*@brief   
*@param[in]
*****************************************************************************************/
int8_t AK8975_init(void)
{
	uint8_t address=0;

	if(IIC_Read_One_Byte(MAG_ADDR,MAG_ID,&address)  == FAILED) //读取磁力计ID
		return FAILED;//如果磁力计不存在或者I2C错误
	
	if(address	!= 0x48) //检查ID
		return FAILED;

	IIC_Write_One_Byte(MAG_ADDR, 0x0A, 0x11); //0x10 16位模式  0x 01 单次测量模式		//14位 - 0.6uT/LSB      16位 - 0.15uT/LSB

	delay_ms(10); //延时等待磁力计可用
	
  read_AT24C02(AK8975_OFFSET,MagOffset,6); //读取AK8975的eeprom，看是否校准过了

	return SUCCESS;
}	


/****************************************************************************************
*@brief   
*@brief   
*@param[in]
*****************************************************************************************/
int8_t AK8975_Updata(void)
{	
   uint8_t BUF[6];
	 I2C_FastMode = 0;
   IIC_Read_One_Byte (MAG_ADDR,MAG_XOUT_L,&BUF[0]);
   IIC_Read_One_Byte (MAG_ADDR,MAG_XOUT_H,&BUF[1]);
   Device.AK8975.magX =((BUF[1]<<8)|BUF[0]) - MagOffset[0];

   IIC_Read_One_Byte(MAG_ADDR,MAG_YOUT_L,&BUF[2]);
   IIC_Read_One_Byte(MAG_ADDR,MAG_YOUT_H,&BUF[3]);
   Device.AK8975.magY =	-((BUF[3]<<8)|BUF[2]) - MagOffset[1];  //-Y为重映射到陀螺仪坐标系下
   						   //读取计算Y轴数据
	 
   IIC_Read_One_Byte(MAG_ADDR,MAG_ZOUT_L,&BUF[4]);
   IIC_Read_One_Byte(MAG_ADDR,MAG_ZOUT_H,&BUF[5]);
   Device.AK8975.magZ =	-((BUF[5]<<8)|BUF[4]) - MagOffset[2];  //-Z为重映射到陀螺仪坐标系下

	
	
	IIC_Write_One_Byte(MAG_ADDR,0x0A,0x11); //启动下一次磁力计转换
	
	{ //此处进行磁力计简单滤波，磁力计滤波太多会导致偏航角度滞后
		static float MagTemp_x,MagTemp_y,MagTemp_z;
		static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.1},{0.02,0,0,0,0.001,0.1},{0.02,0,0,0,0.001,0.1}};
		//  ekf.Q = 0.001f;//0.001; 
		//  ekf.R = 0.15f;//0.020;	
		kalman_1(&ekf[0],Device.AK8975.magX);  //一维卡尔曼		
		MagTemp_x = ekf[0].out;	
		kalman_1(&ekf[1],Device.AK8975.magY);  //一维卡尔曼		
		MagTemp_y = ekf[1].out;			
		kalman_1(&ekf[2],Device.AK8975.magZ);  //一维卡尔曼		
		MagTemp_z = ekf[2].out;

		
				//Yaw_Lock(Device.AK8975.magX,Device.AK8975.magY,Device.AK8975.magZ); //磁力计补偿偏航角
	   /************磁力计倾角补偿*****************/
		float TempX;
    float TempY;			
		TempX = MagTemp_x * IMU.Cos_Roll+ MagTemp_z * IMU.Sin_Roll;
    TempY = MagTemp_x * IMU.Sin_Pitch*IMU.Sin_Roll
                    +MagTemp_y * IMU.Cos_Pitch
                    -MagTemp_z * IMU.Cos_Roll*IMU.Sin_Pitch;
   /***********反正切得到磁力计观测角度*********/
   IMU.yaw_mag = atan2(TempX,TempY)*57.296; //得到罗盘偏航角
	}
	return SUCCESS;
} 
/****************************************************************************************
*@brief   第一次在上位机点击磁力计校准，板子正面朝上，水平旋转400度直到粉红灯退出
          第二次在上位机点击磁力计校准，板子立起，机头朝向天，水平旋转400度直到粉红灯退出
				  校准完将自动保存校准值，直到不满意继续校准
*@brief   
*@param[in]
*****************************************************************************************/
//校准至yaw = 90度 上位机界面显示机头指向正东即可。可用手机进行对比，只要对比机头即可，其它不用看。
int16_t x_max ;
int16_t x_min ;
int16_t y_max ;
int16_t y_min ;
int16_t z_max ;
int16_t z_min ;

 float Y_up_angle; //Y轴朝上的角度
 float Z_up_angle; //Z轴朝上的角度

void Mag_Calibartion(void)	
{
	static uint8_t status = 0;
	
	if(Command.FlightMode != LOCK) // 只有在锁定状态才能校准，解锁后不能校准。
	{
		status = 255;
	}	
	
  switch(status)
	{
		case 0:
					status = 1;			
			break;
			//-------------------------------------------------
		case 1:
				if(Command.MagOffset)//进入磁力计校准模式
				{
						MagOffset[0] = MagOffset[1] = MagOffset[2] = 0;//校准值清0
						 x_max = -32767;
						 x_min = 32767;
						 y_max = -32767;
						 y_min = 32767;
						 z_max = -32767;
						 z_min = 32767;
						LED.color = PINK	;
						status = 2;	
						Z_up_angle = 0;
				}
			break;
		case 2:		//第一次点校准，进入第一面校准
				LED.color = PINK	;
				Command.MagOffset = 1;
				if(Attitude.accZ_origion>=2500)//Z轴基本竖直
				{
						Z_up_angle += Attitude.gyroZ_IMU*0.005F* Gyro_G;

							if(Device.AK8975.magX >= x_max)   x_max = (int16_t)(Device.AK8975.magX);
							if(Device.AK8975.magX <  x_min)   x_min = (int16_t)(Device.AK8975.magX);
							if(Device.AK8975.magY >= y_max)   y_max = (int16_t)(Device.AK8975.magY);
							if(Device.AK8975.magY <  y_min)   y_min = (int16_t)(Device.AK8975.magY);
							if(Device.AK8975.magZ >= z_max)   z_max = (int16_t)(Device.AK8975.magZ);
							if(Device.AK8975.magZ <  z_min)   z_min = (int16_t)(Device.AK8975.magZ);
						
							if(Z_up_angle>450 || Z_up_angle<-450)
						{
								Command.MagOffset = 0; //清除上位机发来的校准标志位  //进入下一面校准
								status = 3;
						}	
				}
			break;
		case 3:				
		 			if(Command.MagOffset)//再点一次磁力计校准，进入第二面校准
					{
							LED.color = PINK	;
							status = 4;	
							Y_up_angle = 0;
					}
			break;
		case 4:
				LED.color = PINK	; //粉色
				LED.FlashTime  = 50; //第二次校准快闪
				Command.MagOffset = 1;//保持正在校准状态
		
				if(Attitude.accY_origion>=2500)//Z轴基本竖直
				{
						Y_up_angle += Attitude.gyroY_IMU*0.005F* Gyro_G;
					
						
							if(Device.AK8975.magX >= x_max)   x_max = (int16_t)(Device.AK8975.magX);
							if(Device.AK8975.magX <  x_min)   x_min = (int16_t)(Device.AK8975.magX);
							if(Device.AK8975.magY >= y_max)   y_max = (int16_t)(Device.AK8975.magY);
							if(Device.AK8975.magY <  y_min)   y_min = (int16_t)(Device.AK8975.magY);
							if(Device.AK8975.magZ >= z_max)   z_max = (int16_t)(Device.AK8975.magZ);
							if(Device.AK8975.magZ <  z_min)   z_min = (int16_t)(Device.AK8975.magZ);
						
						if(Y_up_angle>600 || Y_up_angle<-600)
						{
								Command.MagOffset = 0; //清除上位机发来的校准标志位  //进入下一面校准
								LED.FlashTime = 300;//正常闪烁
								status = 254;
						}	
				}											
			break;				
		case 254:
						MagOffset[0] =(x_min+x_max)/2.0;
						MagOffset[1] =(y_min+y_max)/2.0;
						MagOffset[2] =(z_min+z_max)/2.0;	
						write_AT24C02(AK8975_OFFSET,MagOffset,6);//保存校准数据
		case 255: //全都校准完成
					status = 0;
			break;		
		default:
			break;	
	}
}





/**********************END OF FILE *******************************************************************/





