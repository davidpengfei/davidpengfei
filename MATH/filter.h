#ifndef _FILTER_H_
#define _FILTER_H_



typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
 const float a[3];
 const float b[3];
}Butter_Parameter;


float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);
#endif


