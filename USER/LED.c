#include "led.h" 
#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h" 

//LEDӲ����ʼ����������
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //����һ�����ų�ʼ���Ľṹ��
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��GPIOAʱ�ӣ�GPIOA������APB2ʱ���£���STM32��ʹ��IO��ǰ��Ҫʹ�ܶ�Ӧʱ��
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4; //����4
	GPIO_InitStructure.Pin=GPIO_PIN_4;
	GPIO_InitStructure.Mode=GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	//GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP; //�����������ģʽΪ�������ģʽ
	//GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //��������ٶ�Ϊ50MHZ
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); //�����������úõ�GPIO_InitStructure��������ʼ������GPIOA_PIN4
	//HAL_GPIO_SetBits(GPIOA, GPIO_Pin_4); //��ʼ����������GPIOA4Ϊ�ߵ�ƽ
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
}
