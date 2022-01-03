/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
 
#include "kalman.h"
#include <math.h>
/*****************************************************************************************
 * 卡尔曼
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/
void kalman_1(struct _1_ekf_filter *ekf,float input)  //一维卡尔曼
{
	ekf->Now_P = ekf->LastP + ekf->Q;
	ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);
	ekf->out = ekf->out + ekf->Kg * (input - ekf->out);
	ekf->LastP = (1-ekf->Kg) * ekf->Now_P ;
}

