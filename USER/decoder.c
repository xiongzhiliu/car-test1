#include "decoder.h"

struct key k1={0,0,0};
struct moto_decoer left={0,0,0}, right={0,0,0};

//��ʱ��������ģʽ����ж�
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)  //�ж����
	{
			int num = __HAL_TIM_GetAutoreload(&htim2);
			if(num >=65535 ) //����
					left.full_t++;
			else
					left.full_t--;
			__HAL_TIM_SetAutoreload(&htim2,65535);
	}
	
	if(htim->Instance == TIM3)  //�ж����
	{
			int num = __HAL_TIM_GetAutoreload(&htim3);
			if(num >= 65535)
					right.full_t++;
			else
					right.full_t--;
			__HAL_TIM_SetAutoreload(&htim3,65535);
	}
	
	if(htim->Instance == TIM1)
	{
		
		switch(k1.state)
		{
			case 0:
				if(HAL_GPIO_ReadPin(key_GPIO_Port,key_Pin) == GPIO_PIN_RESET)
				{
					k1.state = 1;
				}
				break;
				
			case 1:
				if(HAL_GPIO_ReadPin(key_GPIO_Port,key_Pin) == GPIO_PIN_RESET)
				{
					k1.state = 0;
					k1.is_pull = 1;
				}else
				{
					k1.state = 0;
				}
				break;
				
			default:
				k1.state=0;
				break;
		}
		__HAL_TIM_CLEAR_IT(&htim1,TIM_IT_UPDATE);
	}
}

int Read_Velocity_L()
{
	left.en_pul = -(short)__HAL_TIM_GetCounter(&htim3);  //��ȡ������������
	//left.spd_pul = left.en_pul - left.en_pul_last;  //�����ٶ�������
	left.en_pul_last = left.en_pul;  //�ϴζ�ȡ��������
	left.sum_pul += left.en_pul + 20000*left.full_t;  //�ۼ�������
	__HAL_TIM_SetCounter(&htim3,0);  //���������
	return left.en_pul;  //�����ٶ�������
}

int Read_Velocity_R()
{
	right.en_pul = (short)__HAL_TIM_GetCounter(&htim2);  //��ȡ������������
	right.spd_pul = right.en_pul - right.en_pul_last;  //�����ٶ�������
	right.en_pul_last = right.en_pul;  //�ϴζ�ȡ��������
	right.sum_pul += right.en_pul + 20000*right.full_t;  //�ۼ�������
	__HAL_TIM_SetCounter(&htim2,0);  //���������
	return right.en_pul;  //�����ٶ�������
}