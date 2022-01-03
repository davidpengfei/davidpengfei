#ifndef ___GPS___H_
#define ___GPS___H_


typedef struct{
  float Lattitude_N_S_Position;	//相对于HOME点的位置
  float Longitude_W_E_Position;  //相对于HOME点的位置
	float Lattitude_N_S_speed;	
	float Longitude_W_E_speed;	
	float Altitude;
	float Quality;//水平经度衰减因子	
	float Angle;//航向角
	float Speed;//对地航速
	unsigned char satellite_num;//卫星数量
	unsigned char status;//定位状态  //A无效 //V有效
	unsigned char updata_ok;
	unsigned char Analy_ok;
	unsigned char Read_ok;	
	unsigned char Home_Ready;
}_st_GPS; //GPS原始数据


extern _st_GPS GPS;
void GPS_Init(void);

void GPS_Run(void);

#endif
