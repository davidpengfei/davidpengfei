/*******************************************************************
 *@title LED system
 *@brief flight light
 *@brief 
 *@time  2016.10.19
 *@editor小南&zin
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 ******************************************************************/
#include "stm32f4xx.h"
#include "LED.h"
#include "ALL_DEFINE.h"


//led指示灯信息
 _st_LED LED;
//红色			 
#define RED_H PEout(2) = 1                 //暗
#define RED_L  PEout(2) = 0                //亮
#define RED_Toggle  PEout(2) ^= 1          //闪	
//绿色
#define GREE_H PEout(1) = 1                //暗
#define GREE_L  PEout(1) = 0               //亮
#define GREE_Toggle  PEout(1) ^= 1         //闪	
//蓝色
#define BLUE_H PEout(0) = 1                //暗
#define BLUE_L  PEout(0) = 0               //亮
#define BLUE_Toggle  PEout(0) ^= 1         //闪	
//青色
#define CYAN_H  GPIOE->ODR |= 0x03         //暗
#define CYAN_L  GPIOE->ODR &= ~0x03		     //亮							 
#define CYAN_Toggle GPIOE->ODR ^= 0x03     //闪	
//粉色										
#define PINK_H  GPIOE->ODR |= 0x05         //暗
#define PINK_L  GPIOE->ODR &= ~0x05		     //亮								
#define PINK_Toggle GPIOE->ODR ^= 0x05	   //闪	
//黄色										
#define YELLOW_H GPIOE->ODR |= 0x06	       //暗
#define YELLOW_L GPIOE->ODR &= ~0x06	     //亮
#define YELLOW_Toggle GPIOE->ODR ^= 0x06	 //闪																	
//白色										
#define WHITE_H  GPIOE->ODR |= 0x07        //暗
#define WHITE_L  GPIOE->ODR &= ~0x07		   //亮							
#define WHITE_Toggle GPIOE->ODR ^= 0x07	   //闪																	
/*******************************************************
 *  LED Init
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void LEDInit(void)	
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	
  GPIO_StructInit(&GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2| GPIO_Pin_1| GPIO_Pin_0;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOE, GPIO_Pin_2);		
	GPIO_SetBits(GPIOE, GPIO_Pin_1);		
	GPIO_SetBits(GPIOE, GPIO_Pin_0);			
	
	LED.FlashTime = 150; //10MS 更新一次

	LED.status = AlwaysOff;
	
}
/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void LED_display(void) //flash 300MS interval
{
	static uint8_t status = 0;
	static uint8_t last_LEDstatus = AlwaysOff;
	static uint8_t last_LEDcorlor;
	
	if(LED.status != last_LEDstatus)      //状态切换
	{
		WHITE_H;
		status = 0;
		last_LEDstatus = LED.status;
	}
	else if(LED.color != last_LEDcorlor)  //颜色切换
	{
		WHITE_H;
		last_LEDcorlor = LED.color;
	}
	
	switch(status)
	{
//------------------------------------------------------------------				
		case 0: //判断闪烁状态
										switch(LED.status)
										{
												case AlwaysOff: 
															status = 1;
													break;						
												case Flash:
															status = 3;
													break;	
												case AlwaysOn: 
															status = 4;
													break;
												default:
													status = 3;
													break;
										}
			break;
//------------------------------------------------------------------												
		case 1:	//常暗	
				WHITE_H; //共阴极，高则暗
			break;
//------------------------------------------------------------------				
//------------------------------------------------------------------												
		case 3: //闪烁
										{
													static uint32_t FlashTime = 0;
											
													if(SysTick_count-FlashTime < LED.FlashTime)
													{
																			return;
													}	
													FlashTime = SysTick_count;			
										}
										switch(LED.color)
										{
												case RED:  //红闪
													RED_Toggle;
													break;		
												case YELLOW:  //黄闪
													YELLOW_Toggle;			
													break;	
												case GREE:  //绿闪
													GREE_Toggle;
													break;	
												case BLUE:  //蓝闪
													BLUE_Toggle;
													break;	
												case CYAN:  //青闪
													CYAN_Toggle;
													break;	
												case PINK:  //粉闪
													PINK_Toggle;
													break;	
												case WHITE:  //白闪
													WHITE_Toggle;
													break;
												case RANDOM:	
													{
															static uint8_t random; 
															random += TIM2->CNT;
															random++;
															PEout(0) ^= random;
															PEout(1) ^= random>>1;
															PEout(2) ^= random>>2;													
													}	
													break;
												default:
														LED.color = RED;
												break;						
										}	
				break;							
//------------------------------------------------------------------												
		case 4:		//常亮					
												switch(LED.color)
												{
														case RED:  //红亮
															RED_L;
															break;		
														case YELLOW:  //黄亮
															YELLOW_L;			
															break;	
														case GREE:  //绿亮
															GREE_L;
															break;	
														case BLUE:  //蓝亮
															BLUE_L;
															break;	
														case CYAN:  //青亮
															CYAN_L;
															break;	
														case PINK:  //粉亮
															PINK_L;
															break;	
														case WHITE:  //白亮
															WHITE_L;
															break;	
														default:
																LED.color = RED;
														break;						
												}	
												status = 	1;	
			break;
//------------------------------------------------------------------											
		default:
				status = 0;
			break;		
	}
}

/**************************END OF FILE*********************************/



