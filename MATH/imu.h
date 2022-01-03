#ifndef __IMU_H
#define	__IMU_H


#include "ALL_DATA.h"

typedef struct {
	float yaw;
	float pitch;
	float roll;
	float yaw_mag; //单独由磁力计的出来的角度
	float Cos_Roll;
	float Sin_Roll;
	float Cos_Pitch;
	float Sin_Pitch;
	float Cos_Yaw;
	float Sin_Yaw;
}_st_IMU;

extern float yaw_control;
extern float Yaw_Correct;
extern _st_IMU IMU;

void AHRSUpdate_GraDes_Delay_Corretion(float gx, float gy, float gz, float ax, float ay, float az);//四元数梯度下降法
#endif
