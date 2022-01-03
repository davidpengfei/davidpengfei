#include "ALL_DATA.h"
#include "spl06.h"
#include "i2c.h"
#include "kalman.h"




#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

#define PRESSURE_REG 0X00
#define TEMP_REG 0X03
#define PRS_CFG 0x06
#define TMP_CFG 0x07
#define MEAS_CFG 0x08

#define SPL06_REST_VALUE 0x09
#define SPL06_REST_REG 0x0C
#define SPL06_PRODUCT_ID_ADDR 0x0D
#define SPL06_PRODUCT_ID 0X10

#define uint32 unsigned int

static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;

void spl0601_write(uint8_t hwadr, uint8_t regadr, uint8_t val);
uint8 spl0601_read(uint8 hwadr, uint8 regadr);
void spl0601_get_calib_param(void);


/*****************************************************************************
 函 数 名  : spl0601_write
 功能描述  : I2C 寄存器写入子函数
 输入参数  : uint8 hwadr   硬件地址
             uint8 regadr  寄存器地址
             uint8 val     值
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_write(unsigned char hwadr, unsigned char regadr, unsigned char val)
{
//	hwI2C0_Device_Addr = hwadr;
//	bI2C0_TxM_Data[0] = regadr;
//	bI2C0_TxM_Data[1] = val;
//
//	I2C0_Engine(2,0,0);
	IIC_Write_One_Byte(hwadr,regadr,val);
}

/*****************************************************************************
 函 数 名  : spl0601_read
 功能描述  : I2C 寄存器读取子函数
 输入参数  : uint8 hwadr   硬件地址
             uint8 regadr  寄存器地址
 输出参数  :
 返 回 值  : uint8 读出值
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
uint8 spl0601_read(unsigned char hwadr, unsigned char regadr)
{
	uint8 reg_data;

//	hwI2C0_Device_Addr = hwadr;
//	bI2C0_TxM_Data[0] = regadr; //
//	I2C0_Engine(1,1,1);

//	reg_data = bI2C0_RxM_Data[0];
	IIC_Read_One_Byte(hwadr,regadr,&reg_data);
	return reg_data;
}

/*****************************************************************************
 函 数 名  : spl0601_init
 功能描述  : SPL06-01 初始化函数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
uint8 spl0601_init(void)
{
	  uint8 data;
    p_spl0601 = &spl0601; /* read Chip Id */
    p_spl0601->i32rawPressure = 0;
    p_spl0601->i32rawTemperature = 0;
  //  p_spl0601->chip_id = 0x34;
	
  	IIC_Read_One_Byte (HW_ADR,SPL06_PRODUCT_ID_ADDR,&data);
	  if(SPL06_PRODUCT_ID != data)
	  {
			return FAILED;
		}
    spl0601_get_calib_param();
// sampling rate = 32Hz; Pressure oversample = 8;
// spl0601_rateset(PRESSURE_SENSOR,32, 8);
// sampling rate = 32Hz; Temperature oversample = 8;
// spl0601_rateset(TEMPERATURE_SENSOR,32, 8);
    spl0601_rateset(PRESSURE_SENSOR,128, 32);
// sampling rate = 1Hz; Temperature oversample = 1;
    spl0601_rateset(TEMPERATURE_SENSOR,32, 8);
    //Start background measurement
    spl0601_start_continuous(CONTINUOUS_P_AND_T);
    return  SUCCESS;
}




/*****************************************************************************
 函 数 名  : spl0601_rateset
 功能描述  :  设置温度传感器的每秒采样次数以及过采样率
 输入参数  : uint8 u8OverSmpl  过采样率         Maximal = 128
             uint8 u8SmplRate  每秒采样次数(Hz) Maximal = 128
             uint8 iSensor     0: Pressure; 1: Temperature
 输出参数  : 无
 返 回 值  : 无
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月24日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_rateset(uint8 iSensor, uint8 u8SmplRate, uint8 u8OverSmpl)
{
    uint8 reg = 0;
    int32 i32kPkT = 0;
    switch(u8SmplRate)
    {
        case 2:
            reg |= (1<<5);
            break;
        case 4:
            reg |= (2<<5);
            break;
        case 8:
            reg |= (3<<5);
            break;
        case 16:
            reg |= (4<<5);
            break;
        case 32:
            reg |= (5<<5);
            break;
        case 64:
            reg |= (6<<5);
            break;
        case 128:
            reg |= (7<<5);
            break;
        case 1:
        default:
            break;
    }
    switch(u8OverSmpl)
    {
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if(iSensor == 0)
    {
        p_spl0601->i32kP = i32kPkT;
        spl0601_write(HW_ADR, 0x06, reg);
        if(u8OverSmpl > 8)
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg | 0x04);
        }
    }
    if(iSensor == 1)
    {
        p_spl0601->i32kT = i32kPkT;
        spl0601_write(HW_ADR, 0x07, reg|0x80);  //Using mems temperature
        if(u8OverSmpl > 8)
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg | 0x08);
        }
    }
}

/*****************************************************************************
 函 数 名  : spl0601_get_calib_param
 功能描述  : 获取校准参数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_get_calib_param(void)
{
    uint32 h;
    uint32 m;
    uint32 l;
    h =  spl0601_read(HW_ADR, 0x10);
    l  =  spl0601_read(HW_ADR, 0x11);
    p_spl0601->calib_param.c0 = (int16)h<<4 | l>>4;
    p_spl0601->calib_param.c0 = (p_spl0601->calib_param.c0&0x0800)?(0xF000|p_spl0601->calib_param.c0):p_spl0601->calib_param.c0;
    h =  spl0601_read(HW_ADR, 0x11);
    l  =  spl0601_read(HW_ADR, 0x12);
    p_spl0601->calib_param.c1 = (int16)(h&0x0F)<<8 | l;
    p_spl0601->calib_param.c1 = (p_spl0601->calib_param.c1&0x0800)?(0xF000|p_spl0601->calib_param.c1):p_spl0601->calib_param.c1;
    h =  spl0601_read(HW_ADR, 0x13);
    m =  spl0601_read(HW_ADR, 0x14);
    l =  spl0601_read(HW_ADR, 0x15);
    p_spl0601->calib_param.c00 = (int32)h<<12 | (int32)m<<4 | (int32)l>>4;
    p_spl0601->calib_param.c00 = (p_spl0601->calib_param.c00&0x080000)?(0xFFF00000|p_spl0601->calib_param.c00):p_spl0601->calib_param.c00;
    h =  spl0601_read(HW_ADR, 0x15);
    m =  spl0601_read(HW_ADR, 0x16);
    l =  spl0601_read(HW_ADR, 0x17);
    p_spl0601->calib_param.c10 = (int32)h<<16 | (int32)m<<8 | l;
    p_spl0601->calib_param.c10 = (p_spl0601->calib_param.c10&0x080000)?(0xFFF00000|p_spl0601->calib_param.c10):p_spl0601->calib_param.c10;
    h =  spl0601_read(HW_ADR, 0x18);
    l  =  spl0601_read(HW_ADR, 0x19);
    p_spl0601->calib_param.c01 = (int16)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x1A);
    l  =  spl0601_read(HW_ADR, 0x1B);
    p_spl0601->calib_param.c11 = (int16)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x1C);
    l  =  spl0601_read(HW_ADR, 0x1D);
    p_spl0601->calib_param.c20 = (int16)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x1E);
    l  =  spl0601_read(HW_ADR, 0x1F);
    p_spl0601->calib_param.c21 = (int16)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x20);
    l  =  spl0601_read(HW_ADR, 0x21);
    p_spl0601->calib_param.c30 = (int16)h<<8 | l;
}


/*****************************************************************************
 函 数 名  : spl0601_start_temperature
 功能描述  : 发起一次温度测量
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_temperature(void)
{
    spl0601_write(HW_ADR, 0x08, 0x02);
}

/*****************************************************************************
 函 数 名  : spl0601_start_pressure
 功能描述  : 发起一次压力值测量
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_pressure(void)
{
    spl0601_write(HW_ADR, 0x08, 0x01);
}

/*****************************************************************************
 函 数 名  : spl0601_start_continuous
 功能描述  : Select node for the continuously measurement
 输入参数  : uint8 mode  1: pressure; 2: temperature; 3: pressure and temperature
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月25日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_continuous(uint8 mode)
{
    spl0601_write(HW_ADR, 0x08, mode+4);
}


/*****************************************************************************
 函 数 名  : spl0601_get_raw_temp
 功能描述  : 获取温度的原始值，并转换成32Bits整数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_get_raw_temp(void)
{
    uint8 h[3] = {0};
    h[0] = spl0601_read(HW_ADR, 0x03);
    h[1] = spl0601_read(HW_ADR, 0x04);
    h[2] = spl0601_read(HW_ADR, 0x05);
    p_spl0601->i32rawTemperature = (int32)h[0]<<16 | (int32)h[1]<<8 | (int32)h[2];
    p_spl0601->i32rawTemperature= (p_spl0601->i32rawTemperature&0x800000) ? (0xFF000000|p_spl0601->i32rawTemperature) : p_spl0601->i32rawTemperature;
}

/*****************************************************************************
 函 数 名  : spl0601_get_raw_pressure
 功能描述  : 获取压力原始值，并转换成32bits整数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_get_raw_pressure(void)
{
    uint8 h[3];
    h[0] = spl0601_read(HW_ADR, 0x00);
    h[1] = spl0601_read(HW_ADR, 0x01);
    h[2] = spl0601_read(HW_ADR, 0x02);
    p_spl0601->i32rawPressure = (int32)h[0]<<16 | (int32)h[1]<<8 | (int32)h[2];
    p_spl0601->i32rawPressure= (p_spl0601->i32rawPressure&0x800000) ? (0xFF000000|p_spl0601->i32rawPressure) : p_spl0601->i32rawPressure;
}


/*****************************************************************************
 函 数 名  : spl0601_get_temperature
 功能描述  : 在获取原始值的基础上，返回浮点校准后的温度值
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
float spl0601_get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
    return fTCompensate;
}

/*****************************************************************************
 函 数 名  : spl0601_get_pressure
 功能描述  : 在获取原始值的基础上，返回浮点校准后的压力值
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
float spl0601_get_pressure(int32_t P,int32_t T)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = T / (float)p_spl0601->i32kT;
    fPsc = P / (float)p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * (p_spl0601->calib_param.c20 + fPsc* p_spl0601->calib_param.c30);
    qua3 = fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);
		//qua3 = 0.9f *fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);
	
    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
		//fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}
	/**************************************************************
 *  data process
 * @param[in] 
 * @param[out] NONE
 * @return   NONE  
 ***************************************************************/

//static const uint8_t D_AVERAGE_NUM = 20;  //原始数据防脉冲干扰的均值滤波   D1  D2
//static int32_t average[2][D_AVERAGE_NUM]={0};

void spl06_001_HeightHighProcess(void)  
{

		static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};	
			
		float f32rawTemperature;	
		float f32rawPressure;	
		float presure;	
			
		kalman_1(&ekf[0],p_spl0601->i32rawPressure);  //一维卡尔曼
		f32rawPressure = 	ekf[0].out;//原始气压数据滤波
			
		kalman_1(&ekf[1],p_spl0601->i32rawTemperature);  //一维卡尔曼		
		f32rawTemperature = 	ekf[1].out;	//原始温度数据滤波
			
		presure = spl0601_get_pressure(f32rawPressure, f32rawTemperature);	
		FlightData.High.bara_height =  (float)((102000.0f	- presure) * 7.8740f); 	//获取高度数据，单位cm
			
		kalman_1(&ekf[2],FlightData.High.bara_height);  //一维卡尔曼
		FlightData.High.bara_height = 	ekf[2].out;		//解算后的高度值滤波	
}	



void user_spl0601_get(void)
{
	
		spl0601_get_raw_temp();
///		temperature = spl0601_get_temperature();
	
		spl0601_get_raw_pressure();
//		pressure = spl0601_get_pressure();
		
}	
