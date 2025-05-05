#include "led.h" 
#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h" 

//LED硬件初始化函数定义
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能GPIOA时钟，GPIOA挂载在APB2时钟下，在STM32中使用IO口前都要使能对应时钟
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4; //引脚4
	GPIO_InitStructure.Pin=GPIO_PIN_4;
	GPIO_InitStructure.Mode=GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;
	//GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP; //引脚输入输出模式为推挽输出模式
	//GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //引脚输出速度为50MHZ
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); //根据上面设置好的GPIO_InitStructure参数，初始化引脚GPIOA_PIN4
	//HAL_GPIO_SetBits(GPIOA, GPIO_Pin_4); //初始化设置引脚GPIOA4为高电平
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
}
