#ifndef __MPU6050_H
#define __MPU6050_H



#include "sys.h"
#include "I2C.h"




extern int8_t MpuInit(void);
extern void Gyro_Calibration(void);
extern int8_t mpu6050_rest(void);
extern int8_t MpuGetData(void); //读取陀螺仪数据加滤波
extern void Accel_six_Calibartion(void);	
#endif // __MPU6050_H__








