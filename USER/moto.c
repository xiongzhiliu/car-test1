#include "moto.h"
int moto_dead_zone=300;
void MOTO_init()
{
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_GPIO_WritePin(moto1_1_GPIO_Port,moto1_1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(moto1_2_GPIO_Port,moto1_2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(moto2_1_GPIO_Port,moto2_1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(moto2_2_GPIO_Port,moto2_2_Pin,GPIO_PIN_RESET);
}	
/**
 * @brief 设置电机PWM
 * @param lf 左电机PWM
 * @param rg 右电机PWM
 * @return 
 */
void Moto_SetPwm(int lf,int rg)
{
	lf = Threshold_int(lf,7199);
	rg = Threshold_int(rg,7199);
//	printf("%d,%d\r\n",lf,rg);
//	lf = lf*100 /9999 ;
//	rg = rg*100 /9999 ;
    if(rg>0)
    {
        rg+= moto_dead_zone;
        HAL_GPIO_WritePin(moto1_1_GPIO_Port,moto1_1_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(moto1_2_GPIO_Port,moto1_2_Pin,GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,rg);
				
    }else if (rg<0)
    {
        rg = (- rg);
        rg+= moto_dead_zone;
        HAL_GPIO_WritePin(moto1_1_GPIO_Port,moto1_1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(moto1_2_GPIO_Port,moto1_2_Pin,GPIO_PIN_RESET);
        __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,rg);
    }
    else
    {
        HAL_GPIO_WritePin(moto1_1_GPIO_Port,moto1_1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(moto1_2_GPIO_Port,moto1_2_Pin,GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);
    }

    if(lf>0)
    {
        lf+= moto_dead_zone;
        HAL_GPIO_WritePin(moto2_1_GPIO_Port,moto2_1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(moto2_2_GPIO_Port,moto2_2_Pin,GPIO_PIN_RESET);
        __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,lf);
    }else if (lf<0)
    {
        lf = (- lf);
        lf+= moto_dead_zone;
        HAL_GPIO_WritePin(moto2_1_GPIO_Port,moto2_1_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(moto2_2_GPIO_Port,moto2_2_Pin,GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,lf);
    }
    else
    {
        HAL_GPIO_WritePin(moto2_1_GPIO_Port,moto2_1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(moto2_2_GPIO_Port,moto2_2_Pin,GPIO_PIN_SET);
        __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0);
    }	
}


// void Moto_Breaking(void)
// {
//     DL_GPIO_setPins(Moto_LA_PORT, Moto_LA_PIN);
//     DL_GPIO_setPins(Moto_LB_PORT, Moto_LB_PIN);
//     DL_GPIO_setPins(Moto_RA_PORT, Moto_RA_PIN);
//     DL_GPIO_setPins(Moto_RB_PORT, Moto_RB_PIN);
//     DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
//     DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
// }

int Xianfu_pwm(int t,int thre)
{	
	if(t>thre)
		t = thre;
	else if(t < -thre)
		t = -thre;
	 
	return t;
}
