#include "delay.h"

static u8  fac_us=0;//us延时倍乘数			   
static u16 fac_ms=0;//ms延时倍乘数,在ucos下,代表每个节拍的ms数

void delay_init(u8 SYSCLK)
{
	fac_us=SYSCLK/8;
	fac_ms=(u16)fac_us*1000;
}

void delay_us(volatile u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}
void delay_ms(volatile u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL=0x01 ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	    
} 


//void delay_us(uint32_t udelay)
//{
//  uint32_t startval,tickn,delays,wait;
// 
//  startval = SysTick->VAL;
//  tickn = HAL_GetTick();
//  //sysc = 72000;  //SystemCoreClock / (1000U / uwTickFreq);
//  delays =udelay * 72; //sysc / 1000 * udelay;
//  if(delays > startval)
//    {
//      while(HAL_GetTick() == tickn)
//        {
// 
//        }
//      wait = 72000 + startval - delays;
//      while(wait < SysTick->VAL)
//        {
// 
//        }
//    }
//  else
//    {
//      wait = startval - delays;
//      while(wait < SysTick->VAL && HAL_GetTick() == tickn)
//        {
// 
//        }
//    }
//}

//volatile unsigned long time_delay; // 延时时间，注意定义为全局变量
////延时n_ms
//void delay_ms(volatile unsigned long nms)
//{
//    //SYSTICK分频--1ms的系统时钟中断
//    if (SysTick_Config(SystemFrequency/1000))
//    {
//   
//        while (1);
//    }
//    time_delay=nms;//读取定时时间
//    while(time_delay);
//    SysTick->CTRL=0x00; //关闭计数器
//    SysTick->VAL =0X00; //清空计数器
//}
////延时nus
//void delay_us(volatile unsigned long nus)
//{
// //SYSTICK分频--1us的系统时钟中断
//    if (SysTick_Config(SystemFrequency/1000000))
//    {
//   
//        while (1);
//    }
//    time_delay=nus;//读取定时时间
//    while(time_delay);
//    SysTick->CTRL=0x00; //关闭计数器
//    SysTick->VAL =0X00; //清空计数器
//}
