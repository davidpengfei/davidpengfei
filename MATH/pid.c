#include "ALL_DEFINE.h"
#include "PID.h"





//V1.7
/*
1偏差限幅标志；  2积分限幅标志；3积分分离标志；   4期望；
5反馈            6偏差；        7上次偏差；       8偏差限幅值；
9积分分离偏差值；10积分值       11积分限幅值；    12控制参数Kp；
13控制参数Ki；   14控制参数Kd； 15控制器总输出；  16上次控制器总输出
17总输出限幅度
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,35  ,0  ,0 , 20,  1.2    ,0.000    ,0.00     ,0  ,0 , 250},//Pitch_Angle;偏航角度
 {0  ,1 ,0 ,0 ,0 ,0 , 0 ,500 ,0  ,0 ,150,  1.0     ,0.006     ,0.5     ,0  ,0 ,500},//Pitch_Gyro;偏航角速度*/
const float Control_Unit[15][17]=
{
/*                                         Kp        Ki        Kd            */
 /*1  2  3  4  5  6   7  8   9   10  11    12        13        14  15  16  17*/
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,35  ,0  ,0 , 20,   5.60   ,0.0000  ,0.00 ,0  ,0 , 250},//Pitch_Angle;俯仰角度
 {0  ,1 ,0 ,0 ,0 ,0 , 0 ,500 ,0  ,0 , 200,  0.45   ,0.0055  ,3.0  ,0  ,0 ,500},//Pitch_Gyro;俯仰角速度
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,35  ,0  ,0 , 20,   5.60   ,0.0000  ,0.00 ,0  ,0 , 250},//Roll_Angle;横滚角
 {0  ,1 ,0 ,0 ,0 ,0 , 0 ,500 ,0  ,0 , 200,  0.45   ,0.0055  ,3.0  ,0  ,0 ,500},//Roll_Gyro;横滚角速度
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,35  ,0  ,0 , 0 ,   6.50   ,0       ,0.00  ,0  ,0 ,100},//Yaw_Angle;偏航角
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100 ,0  ,0 , 30 ,  2.50   ,0.002   ,4.5   ,0  ,0 ,120},//Yaw_Gyro;偏航角速度
 //定高参数
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,200 ,0  ,0 ,100 ,  1.5     ,0.000   ,0    ,0  ,0 ,400},//High_Position;垂直位置
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,400 ,0  ,0 ,500 ,  2.5     ,0.000   ,0    ,0  ,0 ,700},//High_Speed;垂直速度

 //定点参数
/*                                       Kp        Ki        Kd            */
 /*1  2  3  4  5  6   7  8   9   10  11    12        13        14  15  16  17*/
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,0 ,0 ,8,   0.10    ,0.000    ,0    ,0    ,0 ,150},//Longitude_Position;水平经度位置
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100 ,0  ,0 ,15,  0.20    ,0.0001   ,0    ,0    ,0 ,25},//Longitude_Speed;水平经度速度
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100 ,0  ,0 ,8,   0.10    ,0.000    ,0    ,0    ,0 ,150},//Latitude_Position;水平纬度位置
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100 ,0  ,0 ,15,  0.20    ,0.0001   ,0    ,0    ,0 ,25},//Latitude_Speed;水平纬度速度
 //加速度控制器

 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,500  ,0  ,0 ,300,   0.30     ,0.0035     ,0 ,0   ,0 ,500},//垂直加速度控制器 
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,0  ,0 ,3,     0.32    ,0.0001    ,0    ,0   ,0 ,150},//水平经度方向加速度控制器
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,0  ,0 ,15,    0.45    ,0.0015    ,0.3  ,0   ,0 ,25},//水平维度方向加速度控制器
};


_st_pidData pidData;//系统总控制器

typedef enum
{
     Pitch_Angle_Controler=0,
     Pitch_Gyro_Controler=1,
     Roll_Angle_Controler=2,
     Roll_Gyro_Controler=3,
     Yaw_Angle_Controler=4,
     Yaw_Gyro_Controler=5,
     High_Position_Controler=6,
     High_Speed_Controler=7,
     Longitude_Position_Controler=8,
     Longitude_Speed_Controler=9,
     Latitude_Position_Controler=10,
     Latitude_Speed_Controler=11,
     High_Acce_Controler=12,
     Longitude_Acce_Controler=13,
     Latitude_Acce_Controler=14

}Controler_Label;

void PID_Init(PidObject *Controler,Controler_Label Label)
{
  Controler->Err_Limit_Flag=(uint8)(Control_Unit[Label][0]);//1偏差限幅标志
  Controler->Integrate_Limit_Flag=(uint8)(Control_Unit[Label][1]);//2积分限幅标志
  Controler->Integrate_Separation_Flag=(uint8)(Control_Unit[Label][2]);//3积分分离标志
  Controler->Expect=Control_Unit[Label][3];//4期望
  Controler->FeedBack=Control_Unit[Label][4];//5反馈值
  Controler->Err=Control_Unit[Label][5];//6偏差
  Controler->Last_Err=Control_Unit[Label][6];//7上次偏差
  Controler->Err_Max=Control_Unit[Label][7];//8偏差限幅值
  Controler->Integrate_Separation_Err=Control_Unit[Label][8];//9积分分离偏差值
  Controler->Integrate=Control_Unit[Label][9];//10积分值
  Controler->Integrate_Max=Control_Unit[Label][10];//11积分限幅值
  Controler->Kp=Control_Unit[Label][11];//12控制参数Kp
  Controler->Ki=Control_Unit[Label][12];//13控制参数Ki
  Controler->Kd=Control_Unit[Label][13];//14控制参数Ki
  Controler->Control_OutPut=Control_Unit[Label][14];//15控制器总输出
  Controler->Last_Control_OutPut=Control_Unit[Label][15];//16上次控制器总输出
  Controler->Control_OutPut_Limit=Control_Unit[Label][16];//16上次控制器总输出
}





void Total_PID_Init(void)
{
 PID_Init(&pidData.Pitch,Pitch_Angle_Controler);
 PID_Init(&pidData.RateX,Pitch_Gyro_Controler);
 PID_Init(&pidData.Roll,Roll_Angle_Controler);
 PID_Init(&pidData.RateY,Roll_Gyro_Controler);
 PID_Init(&pidData.Yaw,Yaw_Angle_Controler);
 PID_Init(&pidData.RateZ,Yaw_Gyro_Controler);
 PID_Init(&pidData.HeightHigh,High_Position_Controler);
 PID_Init(&pidData.HeightRate,High_Speed_Controler);
 PID_Init(&pidData.GPS_Longitude_position,Longitude_Position_Controler);
 PID_Init(&pidData.GPS_Longitude_rate,Longitude_Speed_Controler);
 PID_Init(&pidData.GPS_Latitude_position,Latitude_Position_Controler);
 PID_Init(&pidData.GPS_Latitude_rate,Latitude_Speed_Controler);

 PID_Init(&pidData.HeightAccel,High_Acce_Controler);
 PID_Init(&pidData.GPS_Longitude_Acce,Longitude_Acce_Controler);
 PID_Init(&pidData.GPS_Latitude_Acce,Latitude_Acce_Controler);
}

float PID_Control(PidObject *Controler)
{
/*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect  + Controler->Offset -Controler->FeedBack;//期望减去反馈得到偏差
  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
                         +Controler->Integrate//积分
                         +Controler->Kd*(Controler->Err-Controler->Last_Err);//微分
/*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******返回总输出*********************/
  return Controler->Control_OutPut;
}

float PID_Control_Yaw(PidObject *Controler)
{
/*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect  + Controler->Offset -Controler->FeedBack;//期望减去反馈得到偏差
/***********************偏航角偏差超过+-180处理*****************************/
  if(Controler->Err<-180)  Controler->Err=Controler->Err+360;
  if(Controler->Err>180)  Controler->Err=Controler->Err-360;

  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
                         +Controler->Integrate//积分
                         +Controler->Kd*(Controler->Err-Controler->Last_Err);//微分
/*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******返回总输出*********************/
  return Controler->Control_OutPut;
}



float PID_Control_High(PidObject *Controler)
{
/*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect  + Controler->Offset -Controler->FeedBack;//期望减去反馈得到偏差
  //Controler->Err=LPButter_Vel_Error(Controler->Err);
  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
                         +Controler->Integrate//积分
                         +Controler->Kd*(Controler->Err-Controler->Last_Err);//微分
/*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******返回总输出*********************/
  return Controler->Control_OutPut;
}

float Control_Device_LPF(float curr_inputer,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
        /* 加速度计Butterworth滤波 */
	/* 获取最新x(n) */
        Buffer->Input_Butter[2]=curr_inputer;
	/* Butterworth滤波 */
        Buffer->Output_Butter[2]=
         Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
	+Parameter->b[2] * Buffer->Input_Butter[0]
        -Parameter->a[1] * Buffer->Output_Butter[1]
        -Parameter->a[2] * Buffer->Output_Butter[0];
	/* x(n) 序列保存 */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) 序列保存 */
        Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
        Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
        return (Buffer->Output_Butter[2]);
}



Butter_Parameter Control_Device_Div_LPF_Parameter={
 //200---20hz
  1,    -1.14298050254,   0.4128015980962,
  0.06745527388907,   0.1349105477781,  0.06745527388907
};

Butter_Parameter Control_Device_Err_LPF_Parameter={
  //200hz---2hz
  1,   -1.911197067426,   0.9149758348014,
  0.0009446918438402,  0.00188938368768,0.0009446918438402
};


float PID_Control_Div_LPF(PidObject *Controler)
{
  int16  i=0;
/*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect + Controler->Offset - Controler->FeedBack;//期望减去反馈得到偏差
  Controler->Dis_Err=Controler->Err-Controler->Last_Err;//原始微分
  for(i=4;i>0;i--)//数字低通后微分项保存
  {
  Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Div_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz

  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
                         +Controler->Integrate//积分
                         //+Controler->Kd*Controler->Dis_Err;//微分
                         +Controler->Kd*Controler->Dis_Error_History[0];//微分项来源于巴特沃斯低通滤波器
/*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******返回总输出*********************/
  return Controler->Control_OutPut;
}


float PID_Control_Err_LPF(PidObject *Controler)
{
/*******偏差计算*********************/
  Controler->Last_Err=Controler->Err;//保存上次偏差
  Controler->Err=Controler->Expect + Controler->Offset - Controler->FeedBack;//期望减去反馈得到偏差
  Controler->Dis_Err=Controler->Err-Controler->Last_Err;//原始微分

  Controler->Last_Err_LPF=Controler->Err_LPF;
  Controler->Err_LPF=Control_Device_LPF(Controler->Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Err_LPF_Parameter);//巴特沃斯低通后得到的微分项,20hz

  Controler->Dis_Err_LPF=Controler->Err_LPF-Controler->Last_Err_LPF;//偏差经过低通后的微分量

  if(Controler->Err_Limit_Flag==1)//偏差限幅度标志位
  {
  if(Controler->Err_LPF>=Controler->Err_Max)   Controler->Err_LPF= Controler->Err_Max;
  if(Controler->Err_LPF<=-Controler->Err_Max)  Controler->Err_LPF=-Controler->Err_Max;
  }
/*******积分计算*********************/
  if(Controler->Integrate_Separation_Flag==1)//积分分离标志位
  {
    if(ABS(Controler->Err_LPF)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err_LPF;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err_LPF;
  }
/*******积分限幅*********************/
 if(Controler->Integrate_Limit_Flag==1)//积分限制幅度标志
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******总输出计算*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//输出值递推
  Controler->Control_OutPut=Controler->Kp*Controler->Err_LPF//比例
                         +Controler->Integrate//积分
                          +Controler->Kd*Controler->Dis_Err_LPF;//已对偏差低通，此处不再对微分项单独低通
/*******总输出限幅*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******返回总输出*********************/
  return Controler->Control_OutPut;
}


void pidRest(PidObject *Controler,uint8_t len)
{
	while(len)
	{
		Controler-> Expect = 0;
		Controler-> Offset = 0;
		Controler-> FeedBack = 0;
		Controler-> Err = 0;
		Controler-> Last_Err = 0;
		Controler-> Integrate = 0;
		Controler-> Control_OutPut = 0;
		Controler-> Last_Control_OutPut = 0;
		Controler-> Last_FeedBack = 0;
		Controler-> Dis_Err = 0;
		Controler-> Err_LPF = 0;
		Controler-> Last_Err_LPF = 0;
		Controler-> Dis_Err_LPF = 0;
		Controler-> Dis_Error_History[0] = 0;
		Controler-> Dis_Error_History[1] = 0;
		Controler-> Dis_Error_History[2] = 0;
		Controler-> Dis_Error_History[3] = 0;	
		Controler-> Dis_Error_History[4] = 0;	
		len--;
		Controler++;
	}

}


