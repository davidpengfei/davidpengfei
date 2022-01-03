/**************************************************************
 * @brief
   ZIN-A 大四轴主控板
	 飞控爱好群551883670
 * @attention
		请购买者为ZIN小店的代码提供保护。
		您可以移植和其它方式使用，但请不要放到网上。谢谢大家，祝大家学习愉快。
 * @brief	
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "GPS.h"
#include <string.h>
#include "ALL_DATA.h"


#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "delay.h"
#include "USART.h" 
#include "sys.h" 
#include "math.h" 


#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1
_st_GPS GPS;



volatile unsigned char GPS_buffer[256];
volatile unsigned char GPS_wr_index;
volatile unsigned char Frame_End,GPS_Data_Ready=0;
//----------------------------------
volatile unsigned char GPS_Satel[4]; //卫星数量原始数据
volatile unsigned char GPS_DOP[6]; //经度质量
volatile unsigned char GPS_Latitude[11],Lat;  //经度信息和W,E字符
volatile unsigned char GPS_Longitude[13],Lon;
volatile unsigned char GPS_Speed[7];  //GPS速度
volatile unsigned char GPS_Course[7];  //GPS航向角
volatile unsigned char GPS_Height[9];   //GPS海拔高度
volatile unsigned char GPS_Time[8];  //UTC时间
volatile unsigned char GPS_Date[8];
//----------------------------------

float GPS_Altitude = 0,
	  Latitude_GPS[2],
	  Longitude_GPS[2], 
	  Speed_GPS = 0;
//---------家点信息---------------------
float Home_Latitude[2],Home_Longitude[2],Home_Altitude = 0;

void GPS_Distance(float * lat1,float * lon1,float * lat2,float * lon2);
float Get_LatLon_Error(float * temp1,float * temp2);
float Get_LatLon_Sum(float * temp1,float * temp2);	


/**************************实现函数********************************************
*函数原型:		void UBLOX_GPS_initial(void)
*功　　能:		初始化 UBLOX GPS 模块，以去掉一些没必要的数据，节省CPU的开支 
*******************************************************************************/ 
 const u8 set_baurd_115200[] = {0xB5 ,0x62 ,0x06 ,0x00 ,0x14 ,0x00 ,0x01 ,0x00 ,0x00 ,0x00 ,0xD0 ,0x08 ,0x00 ,0x00 ,0x00 ,0xC2 ,0x01 ,0x00 ,0x07 ,0x00 ,0x03 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0xC0 ,0x7E};
 const u8 set_rate[] = {0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12 };
 

const u8 disable_GxGLL[] ={0xB5 ,0x62 ,0x06 ,0x01 ,0x08 ,0x00 ,0xF0 ,0x01 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x2A};
const u8 disable_GxGSA[] ={0xB5 ,0x62 ,0x06 ,0x01 ,0x08 ,0x00 ,0xF0 ,0x02 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x01 ,0x31};
const u8 disable_GxGSV[] ={0xB5 ,0x62 ,0x06 ,0x01 ,0x08 ,0x00 ,0xF0 ,0x03 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x02 ,0x38};
const u8 disable_GxVTG[] ={0xB5 ,0x62 ,0x06 ,0x01 ,0x08 ,0x00 ,0xF0 ,0x05 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x04 ,0x46};
const u8 disable_GxGRS[] ={0xB5 ,0x62 ,0x06 ,0x01 ,0x08 ,0x00 ,0xF0 ,0x06 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x05 ,0x4D};	
const u8 disable_GxGST[] ={0xB5 ,0x62 ,0x06 ,0x01 ,0x08 ,0x00 ,0xF0 ,0x07 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x06 ,0x54};
const u8 disable_GxZDA[] ={0xB5 ,0x62 ,0x06 ,0x01 ,0x08 ,0x00 ,0xF0 ,0x08 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x07 ,0x5B};
const u8 disable_GxGBS[] ={0xB5 ,0x62 ,0x06 ,0x01 ,0x08 ,0x00 ,0xF0 ,0x09 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x08 ,0x62};
const u8 disable_GxDTM[] ={0xB5 ,0x62 ,0x06 ,0x01 ,0x08 ,0x00 ,0xF0 ,0x0A ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x09 ,0x69};
const u8 disable_GxGNS[] ={0xB5 ,0x62 ,0x06 ,0x01 ,0x08 ,0x00 ,0xF0 ,0x0D ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x0C ,0x7E};
const u8 disable_GxVLW[] ={0xB5 ,0x62 ,0x06 ,0x01 ,0x08 ,0x00 ,0xF0 ,0x0F ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x0E ,0x8C};

 
void GPS_Init(void)
{
	u16 i;
//	USART4_Send(set_baurd_115200,sizeof(set_baurd_115200));//设置波特率115200
//	for(i=0;i<100;i++)//等待1秒
//		delay_ms(10);   //延时等待GPS保存数据完成
	USART4_Send(set_baurd_115200,sizeof(set_baurd_115200));//设置波特率115200
	for(i=0;i<100;i++)//等待1秒
		delay_ms(10);   //延时等待GPS保存数据完成
	UART4_init(115200); //重新初始化波特率
	for(i=0;i<300;i++)//等待3秒
		delay_ms(10);   //延时等待GPS保存数据完成
	
//	USART4_Send(set_rate,sizeof(set_rate));//设置10HZ
//		delay_ms(10);   //延时等待GPS保存数据完成	
	
	USART4_Send(set_rate,sizeof(set_rate));//设置10HZ
	for(i=0;i<100;i++)
		delay_ms(10);   //延时等待GPS保存数据完成
//	USART4_Send(set_rate,sizeof(set_rate));//设置10HZ
//		delay_ms(10);   //延时等待GPS保存数据完成
	
	USART4_Send(disable_GxGLL,sizeof(disable_GxGLL));//设置GLL
		delay_ms(10);   //延时等待GPS保存数据完成
	USART4_Send(disable_GxGSA,sizeof(disable_GxGSA));//设置GSA
	  delay_ms(10);   //延时等待GPS保存数据完成
	USART4_Send(disable_GxGSV,sizeof(disable_GxGSV));//设置GSV
		delay_ms(10);   //延时等待GPS保存数据完成
	USART4_Send(disable_GxVTG,sizeof(disable_GxVTG));//设置VTG
		delay_ms(10);   //延时等待GPS保存数据完成
	USART4_Send(disable_GxGRS,sizeof(disable_GxGRS));//设置GRS
		delay_ms(10);   //延时等待GPS保存数据完成
	USART4_Send(disable_GxGST,sizeof(disable_GxGST));//设置GST
		delay_ms(10);   //延时等待GPS保存数据完成
	USART4_Send(disable_GxZDA,sizeof(disable_GxZDA));//设置ZDA
		delay_ms(10);   //延时等待GPS保存数据完成	
	USART4_Send(disable_GxGBS,sizeof(disable_GxGBS));//设置GBS
		delay_ms(10);   //延时等待GPS保存数据完成
	USART4_Send(disable_GxDTM,sizeof(disable_GxDTM));//设置DTM
		delay_ms(10);   //延时等待GPS保存数据完成
	USART4_Send(disable_GxGNS,sizeof(disable_GxGNS));//设置GNS
		delay_ms(10);   //延时等待GPS保存数据完成
	USART4_Send(disable_GxVLW,sizeof(disable_GxVLW));//设置VLM
		delay_ms(10);   //延时等待GPS保存数据完成
}
/**************************实现函数********************************************
*函数原型:		float Asc_to_f(volatile unsigned char *str)
*功　　能:		提取字符串中的 有效数字
输入参数：
		unsigned char *str    字符串数组
		返回数组表示的值。  比如字符串 "1230.01"  经过这个程序后，返回浮点的值为1230.01
*******************************************************************************/
float Asc_to_f(volatile unsigned char *str)
{
	signed char temp,flag1,flag2; 
	float value,count;
	flag1 = 1;
	flag2 = 0;
	value = 0;
	count = 1;
	temp = *str;
	while(((*str>='0')&&(*str<='9'))||(*str=='-')||(*str=='.')) //数字或者是符号
	{ 
	temp=*str++;
	if(temp=='-'){ 
	if(flag1)
	   	flag1=-1;
	  else
	   return(0x00); //出现两次'-' 结果无效
	}
	else if(temp=='.'){ 
		 flag2=1;	  
	     }
		 else{ 
		   value=value*10+(temp&0x0f);
	       if(flag2)
		    	count*=0.1f;
		 }
	}
	value*=count*flag1; //符号位
	return(value);
}



/**********************************************************
GPS数据更新

*************************************************************/
//----------------------------------

float Last_Longitude[2],Last_Latitude[2],GPS_Period;
float GPS_SPEED[2]={0,0},now_speed[2]={0,0},speed_old[2]={0,0};
//-----------GPS模块 的线程，需要定时调用-----------
void GPS_Run(void)
{
	float temp;

	if(GPS.updata_ok == 0)//有数据需要处理吗？ 该标志在GPS数据接收中断程序中置位
		return; //没有就退出吧。

	GPS.updata_ok = 0;	// 清标志
	 
	if(GPS.status == 'V')	//定位状态，A=有效定位，V=无效定位
	{ 
		return;
	}

	//开始提取有效的位置信息
	GPS.satellite_num = Asc_to_f(GPS_Satel); //使用卫星数量，从00到12
	memset((unsigned char *)GPS_Satel,0,4); //清0，防止数据没更新而出错
//	GPS.Altitude = Asc_to_f(GPS_Height); //天线离海平面的高度，-9999.9到9999.9米
	GPS.Quality = Asc_to_f(GPS_DOP); //精度因子
	memset((unsigned char *)GPS_DOP,0,6);
	if(GPS.satellite_num  < 7 && GPS.Quality >3)	 //锁定超过7个卫星，并且精度因子小于3。
	{ 
		return;
		
	}
	
	//纬度 [2446.5241]
	Latitude_GPS[0] =  (float)(GPS_Latitude[0]&0x0F)*10.000000f+(float)(GPS_Latitude[1]&0x0F);  //整数部分
	Latitude_GPS[1] =  (float)(GPS_Latitude[2]&0x0F)*10.0f+(float)(GPS_Latitude[3]&0x0F)+
					   (float)(GPS_Latitude[5]&0x0F)*0.1f+(float)(GPS_Latitude[6]&0x0F)*0.01f+	
					   (float)(GPS_Latitude[7]&0x0F)*0.001f+(float)(GPS_Latitude[8]&0x0F)*0.0001f+
					   (float)(GPS_Latitude[9]&0x0F)*0.00001f;
	Latitude_GPS[1] /= 60.000000f;  //转成度，小数部分
	if(Lat == 'S')	//S（南半球）
	{
		Latitude_GPS[0] = -Latitude_GPS[0];
		Latitude_GPS[1] = -Latitude_GPS[1];
	}
	//经度 [12100.1536]
	Longitude_GPS[0] = (float)(GPS_Longitude[0]&0x0F)*100.0f+(float)(GPS_Longitude[1]&0x0F)*10.0f+(float)(GPS_Longitude[2]&0x0F);//整数部分
	Longitude_GPS[1] = (float)(GPS_Longitude[3]&0x0F)*10.0f+(float)(GPS_Longitude[4]&0x0F)+
					   (float)(GPS_Longitude[6]&0x0F)*0.1f+(float)(GPS_Longitude[7]&0x0F)*0.01f+	
					   (float)(GPS_Longitude[8]&0x0F)*0.001f+(float)(GPS_Longitude[9]&0x0F)*0.0001f+
					   (float)(GPS_Longitude[10]&0x0F)*0.00001f;
	Longitude_GPS[1] /= 60.000000f;  //转成度，小数部分
	if(Lon == 'W')	 //W（西经）
	{
		Longitude_GPS[0] = -Longitude_GPS[0];
		Longitude_GPS[1] = -Longitude_GPS[1];
	}
	//速度
	temp = Asc_to_f(GPS_Speed); //地面速率（000.0~999.9节
		GPS.Speed = temp * 0.51444f; //1节＝1海里/小时＝1.852公里/小时 = 0.514米每秒
	//航向
	GPS.Angle = Asc_to_f(GPS_Course);

	if(GPS.Home_Ready  == 1) //如果home点没设置就不计算了
	{
		static unsigned char init_delay = 0;
		//计算经纬度位移差，
		GPS_Distance(Latitude_GPS,Longitude_GPS,Home_Latitude,Home_Longitude);
		
		//计算前厚两次的GPS更新，为计算GPS速度做准备
		static float last_time=0, now_time=0; // 采样周期计数 单位 us
		now_time = micros();
		GPS_Period = now_time - last_time;
		GPS_Period /= 1000.0f; //转成秒为单位，为计算速度做准备。
		last_time = now_time;

		if (init_delay) 	//上电首次计算延迟一次//因为要计算前后两组经纬度数据来得到速度
		{	  
			#define _ROLL 0
			#define _PITCH 1
			//计算速度
			now_speed[_ROLL] = Get_LatLon_Error(Longitude_GPS,Last_Longitude) * cos((Get_LatLon_Sum(Latitude_GPS,Last_Latitude)/2)* 0.0174532925f) / GPS_Period * 11131800.0f; //cm
			now_speed[_PITCH] = Get_LatLon_Error(Latitude_GPS,Last_Latitude) / GPS_Period * 11094600.0f;  //cm
				
			//过滤有害数据
//			if(now_speed[1]>250.0f || now_speed[1]<-250.0f)
//			{
//				Latitude_GPS[0] = Last_Latitude[0];
//				Latitude_GPS[1] = Last_Latitude[1];
//				now_speed[1] =	speed_old[1];
//			}

//			if(now_speed[0]>250.0f || now_speed[0]<-250.0f)
//			{
//				Longitude_GPS[0] = Last_Longitude[0];
//				Longitude_GPS[1] = Last_Longitude[1];
//				now_speed[0] = speed_old[0];
//			}
		  
			now_speed[0] = (now_speed[0] + speed_old[0]) / 2;//前后两次取平均
			now_speed[1] = (now_speed[1] + speed_old[1]) / 2;
		  
			speed_old[0] = now_speed[0];
			speed_old[1] = now_speed[1];

				GPS.Lattitude_N_S_speed = now_speed[1];   //纬度上的速度数据
				GPS.Longitude_W_E_speed = now_speed[0];  //经度上的速度数据
		}
		init_delay=1;

		Last_Longitude[0] = Longitude_GPS[0];
		Last_Longitude[1] = Longitude_GPS[1];
		Last_Latitude[0] = Latitude_GPS[0];
		Last_Latitude[1] = Latitude_GPS[1];
	}
	else if(GPS.Home_Ready == 0) //HOME点未设置 则先设置home点
	{
		if(GPS.satellite_num  > 9 && GPS.Quality <3)	 //锁定超过7个卫星。
		{ 
  
					Home_Latitude[0] = Latitude_GPS[0];
					Home_Latitude[1] = Latitude_GPS[1];
					Home_Longitude[0] = Longitude_GPS[0];
					Home_Longitude[1] = Longitude_GPS[1];
					Home_Altitude = GPS_Altitude;
					GPS.Home_Ready  = 1;
		}	
	
	}
	GPS.Analy_ok = 1;
}

/**************************实现函数********************************************
*函数原型:		void GPS_Decode(unsigned char len)
*功　　能:		将刚刚接收到的帧数据必要的信息提取出来。
输入参数：
		unsigned char len   接收到的字节数
*******************************************************************************/
void GPS_Decode(unsigned char len)
{
	unsigned char i , data ,j = 0 ,k = 0;
	unsigned char temp,temp1,temp2;

	temp = GPS_buffer[0];
	for(i=0;i<GPS_wr_index-4;i++)	  //异或校验
	{
		temp ^= GPS_buffer[i+1];	
	}
	temp1 =	temp/16+0x30;
	temp2 = temp&0x0f;
	if(temp2>0x09) temp2 = 0x40 + temp2 - 0x09;
	else temp2 = temp2 + 0x30;
	if(!(temp1==GPS_buffer[GPS_wr_index-2] && temp2==GPS_buffer[GPS_wr_index-1]))	//校验不通过 返回
	{
		return;
	}
	// $GPGGA 全球定位数据	
	if((GPS_buffer[0]=='G')&&(GPS_buffer[1]=='N')&&(GPS_buffer[2]=='G')&&(GPS_buffer[3]=='G')&&(GPS_buffer[4]=='A'))
	{
		j = 0; 
		for(i = 4; i < len; i++ )
		{
			data = GPS_buffer[i]; //取数组中的数据
		if(data == ',')
		{
			j++; //下一个字段
			k = 0;
		}
		else //非逗号
		{ 
			switch( j )
			{  
						case 7: if(k<3) //使用卫星数量，从00到12（前导位数不足则补0）
							{
								GPS_Satel[k++] = data; 
								} 
						break;
						case 8:

								GPS_DOP[k++] = data;

							break;
						case 9: if(k<8)	 //天线离海平面的高度，-9999.9到9999.9米
						{ 
								GPS_Height[k++] = data; 
											GPS_Height[k] = 0;
							} 
							break;
				case 10 :return ; //后面的数据我们不关心，return 退出   
						default:break;
					}	//switch 结束
			}	
		}	
	}
	else if((GPS_buffer[0]=='G')&&(GPS_buffer[1]=='N')&&(GPS_buffer[2]=='R')&&(GPS_buffer[3]=='M')&&(GPS_buffer[4]=='C'))  //精简定位数据
	{
		// $GPRMC 运输定位数据
		j = 0;
		for(i = 4; i < len; i++ )
		{
			data = GPS_buffer[i]; //取数组中的数据
			if(data == ',')
			{
					j++; //下一个字段
				k = 0;
			}
			else
			{ //非逗号
			switch( j )
			{ 
				case 1: if(k < 6) //UTC 时间，hhmmss（时分秒）格式
									GPS_Time[k++] = data;
								break;
				case 2: GPS.status=data; //定位状态，A=有效定位，V=无效定位
								break;
				case 3: if(k < 10) //Latitude，纬度ddmm.mmmm（度分）格式
									GPS_Latitude[k++] = data;
								break;
				case 4: Lat = data; //纬度半球N（北半球）或S（南半球）
								break;		
				case 5: if(k < 11) // Longitude，经度dddmm.mmmm（度分）格式
									GPS_Longitude[k++] = data; 
								break;
				case 6: Lon = data;//经度半球E（东经）或W（西经）
								break;            
				case 7: if(k < 6) //地面速率（000.0~999.9节)
								{ 
									GPS_Speed[k++] = data; 
									GPS_Speed[k] = 0;
								} 
								break; 
				case 8: if(k < 6) //地面航向（000.0~359.9度，以真北为参考基准)
								{
									GPS_Course[k++] = data; 
									GPS_Course[k] = 0;
								} 
								break;  
				case 9: if(k<6) //UTC日期，ddmmyy（日月年）格式
								{ 
									GPS_Date[k++] = data; 
									GPS_Date[k] = 0;
								} 
								break;      
				default:break;
			}
				}
		}
		GPS.updata_ok = 1;//数据准备好了，提示主程序可以进行提取转换了。
	} 
}


float Get_LatLon_Error(float * temp1,float * temp2)
{
	float temp;
	temp = temp1[0] - temp2[0] + temp1[1] - temp2[1];
	return temp;	
}

float Get_LatLon_Sum(float * temp1,float * temp2)
{
	float temp;
	temp = temp1[0] + temp2[0] + temp1[1] + temp2[1];
	return temp;	
}


/*
计算两个点的距离。
lat1 lon1  点1的经纬度  单位度
lat2 lon2  点2的经纬度 
返回计算出来的距离   单位 米
*/
void GPS_Distance(float * lat1,float * lon1,float * lat2,float * lon2)
{
	GPS.Lattitude_N_S_Position = Get_LatLon_Error(lat1,lat2)*110946.0f;	
	GPS.Longitude_W_E_Position = Get_LatLon_Error(lon1,lon2)* cos((Get_LatLon_Sum(lat1,lat2)/2)* 0.0174532925f)*111318.0f ;
}

/*
计算两个点的连线的 航向角， 以正北为0 。
lat1 lon1  点1的经纬度  单位度
lat2 lon2  点2的经纬度 
返回 的航向角，单位度。

float GPS_Heading(float lat1,float lon1,float lat2,float lon2)
{
	float temp;
	float mLat = lat2 - lat1;
	float mLon = (lon2 - lon1)* cos(((lat2 + lat1)/2)* 0.0174532925f);
	temp = 90.0f + atan2(-mLat, mLon) * 57.2957795f;

	if(temp < 0)temp += 360.0f;
	return temp;                 
}
*/




void UART4_IRQHandler(void)                	//串口4中断服务程序
{
	unsigned char indata;
	//开启CR3,bit0的EIE: Error interrupt enable, 处理USART_IT_ERR,USART_IT_ORE_ER,USART_IT_NE,USART_IT_FE   错误

	if (UART4->SR & USART_FLAG_ORE)	
	{
		indata = UART4->DR;
    UART4->SR = (uint16_t)~USART_FLAG_ORE;	
	
	}
	if (UART4->SR & USART_IT_RXNE)
	{
		indata = USART_ReceiveData(UART4);	//取接收到的数据
		
		indata = UART4 ->DR;
		if(indata == 0x24)	    //帧命令起始位 NMEA-0183 '$'
		{	 
			GPS_wr_index = 0;
			Frame_End = 0;
		}
		else if(indata == 0x0D) //CR回车键
		{ 
			Frame_End = 0xff;
		}
		else if(indata == 0x0A) //LF换行键代表上一帧数据读取完毕
		{ 
			
			if(Frame_End != 0x00)  //上个字节是0x0D
			{	
				GPS_Decode(GPS_wr_index);  //解出必要的数据
				Frame_End = 0;
			}
		}
		else //非关键字
		{ 
			GPS_buffer[GPS_wr_index++] = indata;  //存入缓冲区
			Frame_End = 0;
		}
		if(GPS_wr_index == 0xff)
		{
	    GPS_wr_index--;
		}	 
		UART4->SR = (uint16_t)~((uint16_t)0x01 << (uint16_t)(USART_IT_RXNE >> 0x08));		
  } 
}

