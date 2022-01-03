/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "imu.h"
#include "myMath.h"
#include <math.h>
#include "mpu6050.h"
#include "attitude_process.h"

typedef volatile struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;
 Quaternion NumQ = {1.0f, 0.0f, 0.0f, 0.0f};



float yaw_control;

_st_IMU IMU;


/******************************************************************************************
函数名:	void AHRSUpdate_GraDes(float gx, float gy, float gz, float ax, float ay, float az)
说明:	陀螺仪+加速度计梯度下降姿态融合算法
入口:	float gx	陀螺仪x分量
		float gy	陀螺仪y分量
		float gz	陀螺仪z分量
		float ax	加速度计x分量
		float ay	加速度计y分量
		float az	加速度计z分量
出口:	无
******************************************************************************************/
float Yaw_Correct=0;

static const uint16_t Quad_Delay=10;//5
const uint8_t Quad_Num = Quad_Delay;
	
static float Quad_Buf[Quad_Num+1][4]={0}; //延时取第10组以后的四元数

void AHRSUpdate_GraDes_Delay_Corretion(float gx, float gy, float gz, float ax, float ay, float az)
{
	float CNTLCYCLE=0.00525f;
	
	    {
			  static	float last_time;
				float now_time;
				now_time = micros();
				CNTLCYCLE = now_time - last_time;
				CNTLCYCLE /=1000;
				last_time = now_time;
			
			}	
	
	
	float recipNorm;					// 平方根
	float s0, s1, s2, s3;					// 梯度下降算子求出来的姿态
	float qDot1, qDot2, qDot3, qDot4;			// 四元数微分方程求得的姿态
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	float delta;

        

        for(uint8_t i=Quad_Num;i>0;i--)//将四元数历史值保存起来
        {
          Quad_Buf[i][0]=Quad_Buf[i-1][0];
          Quad_Buf[i][1]=Quad_Buf[i-1][1];
          Quad_Buf[i][2]=Quad_Buf[i-1][2];
          Quad_Buf[i][3]=Quad_Buf[i-1][3];
        }
          Quad_Buf[0][0]=NumQ.q0;
          Quad_Buf[0][1]=NumQ.q1;
          Quad_Buf[0][2]=NumQ.q2;
          Quad_Buf[0][3]=NumQ.q3;

	/* 转换为弧度制，用于姿态更新*/
	float tempx,tempy,tempz;
	tempx = gx * Gyro_Gr;
	tempy = gy * Gyro_Gr;
	tempz = gz * Gyro_Gr;
				
	/* 四元数微分方程计算本次待矫正四元数 */
	qDot1 = 0.5f * (-Quad_Buf[Quad_Delay][1] * tempx - Quad_Buf[Quad_Delay][2] * tempy - Quad_Buf[Quad_Delay][3] * tempz);
	qDot2 = 0.5f * (Quad_Buf[Quad_Delay][0] * tempx + Quad_Buf[Quad_Delay][2] * tempz - Quad_Buf[Quad_Delay][3] * tempy);
	qDot3 = 0.5f * (Quad_Buf[Quad_Delay][0] * tempy - Quad_Buf[Quad_Delay][1] * tempz + Quad_Buf[Quad_Delay][3] * tempx);
	qDot4 = 0.5f * (Quad_Buf[Quad_Delay][0] * tempz + Quad_Buf[Quad_Delay][1] * tempy - Quad_Buf[Quad_Delay][2] * tempx);
	/* 加速度计输出有效时,利用加速度计补偿陀螺仪 */
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
                recipNorm=Q_rsqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
		/* 避免重复运算 */
		_2q0 = 2.0f * NumQ.q0;
		_2q1 = 2.0f * NumQ.q1;
		_2q2 = 2.0f * NumQ.q2;
		_2q3 = 2.0f * NumQ.q3;
		_4q0 = 4.0f * NumQ.q0;
		_4q1 = 4.0f * NumQ.q1;
		_4q2 = 4.0f * NumQ.q2;
		_8q1 = 8.0f * NumQ.q1;
		_8q2 = 8.0f * NumQ.q2;
		q0q0 = NumQ.q0 * NumQ.q0;
		q1q1 = NumQ.q1 * NumQ.q1;
		q2q2 = NumQ.q2 * NumQ.q2;
		q3q3 = NumQ.q3 * NumQ.q3;

		/* 梯度下降算法,计算误差函数的梯度 */
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * NumQ.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * NumQ.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * NumQ.q3 - _2q1 * ax + 4.0f * q2q2 * NumQ.q3 - _2q2 * ay;

		/* 梯度归一化 */
		recipNorm=Q_rsqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		{	//快速补偿姿态，根据飞行器变化飞行角度的幅度
								const float Beta_Adjust[5]={0.03,0.03,0.025,0.02,0.01};//{0.04,0.03,0.025,0.02,0.01};//根据加速度模长的变化调整
								float BETADEF=0.04f;
								float Tmep_Acce_Length;
                Tmep_Acce_Length=LIMIT(Attitude.Acceleration_Length,0,1000);//正常悬停在500以内 //限幅
                BETADEF=Beta_Adjust[0]-0.01f*Tmep_Acce_Length/1000.0f;//动态步长
                qDot1 -= BETADEF * s0;
                qDot2 -= BETADEF * s1;
                qDot3 -= BETADEF * s2;
                qDot4 -= BETADEF * s3;
		}
	}
        /* 补偿由四元数微分方程引入的姿态误差 */
	/* 将四元数姿态导数积分,得到当前四元数姿态 */
	/* 二阶毕卡求解微分方程 */
	delta = (CNTLCYCLE * tempx) * (CNTLCYCLE * tempx) + (CNTLCYCLE * tempy) * (CNTLCYCLE * tempy) + (CNTLCYCLE * tempz) * (CNTLCYCLE * tempz);
	NumQ.q0 = (1.0f - delta / 8.0f) * NumQ.q0 + qDot1 * CNTLCYCLE;
	NumQ.q1 = (1.0f - delta / 8.0f) * NumQ.q1 + qDot2 * CNTLCYCLE;
	NumQ.q2 = (1.0f - delta / 8.0f) * NumQ.q2 + qDot3 * CNTLCYCLE;
	NumQ.q3 = (1.0f - delta / 8.0f) * NumQ.q3 + qDot4 * CNTLCYCLE;
	/* 单位化四元数 */
	recipNorm=Q_rsqrt(NumQ.q0 * NumQ.q0 + NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3);
	NumQ.q0 *= recipNorm;
	NumQ.q1 *= recipNorm;
	NumQ.q2 *= recipNorm;
	NumQ.q3 *= recipNorm;
	/* 四元数到欧拉角转换,转换顺序为Z-Y-X,参见<Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors>.pdf一文,P24 */
	IMU.pitch= atan2(2.0f * NumQ.q2 * NumQ.q3 + 2.0f * NumQ.q0 * NumQ.q1, -2.0f * NumQ.q1 * NumQ.q1 - 2.0f * NumQ.q2* NumQ.q2 + 1.0f) * RtA;// Pitch
	IMU.roll= asin(2.0f * NumQ.q0* NumQ.q2-2.0f * NumQ.q1 * NumQ.q3) * RtA;									// Roll
	//NumQ.angle[YAW] = atan2(2.0f * NumQ.q[1] * NumQ.q[2] + 2.0f * NumQ.q[0] * NumQ.q[3], -2.0f * NumQ.q[3] * NumQ.q[3] - 2.0f * NumQ.q[2] * NumQ.q[2] + 1.0f) * RAD2DEG;// Yaw
  
          /*偏航角一阶互补*/       
				float yaw_G = gz * Gyro_G;
				if((yaw_G > 2.0f) || (yaw_G < -2.0f)) //数据太小可以认为是干扰，不是偏航动作
				{
					IMU.yaw +=gz * Gyro_G*CNTLCYCLE;
				}
       if((IMU.yaw_mag>90 && IMU.yaw<-90)
             || (IMU.yaw_mag<-90 && IMU.yaw>90))
				IMU.yaw = -IMU.yaw * 0.98f + IMU.yaw_mag * 0.02f;
			else IMU.yaw = IMU.yaw * 0.98f + IMU.yaw_mag * 0.02f;				
			 
			if(IMU.yaw<0)   yaw_control = IMU.yaw + 360; //如果小于0，则+360		
			else yaw_control = IMU.yaw;
 
		//倾斜角度的三角函数值
		IMU.Cos_Pitch = cos(IMU.pitch*AtR);
		IMU.Sin_Pitch = sin(IMU.pitch*AtR);
		IMU.Cos_Roll = cos(IMU.roll*AtR);
		IMU.Sin_Roll = sin(IMU.roll*AtR);
		IMU.Cos_Yaw = cos(IMU.yaw*AtR);
		IMU.Sin_Yaw = sin(IMU.yaw*AtR);			
}




/***************************************************END OF FILE***************************************************/




