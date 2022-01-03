#ifndef _MAGNETIC_H
#define _MAGNETIC_H 


#include "mpu6050.h"





extern int8_t AK8975_init(void);
extern int8_t AK8975_Updata(void);
extern void Mag_Calibartion(void);	


#endif




