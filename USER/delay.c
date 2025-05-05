#include "delay.h"

static u8  fac_us=0;//us��ʱ������			   
static u16 fac_ms=0;//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��

void delay_init(u8 SYSCLK)
{
	fac_us=SYSCLK/8;
	fac_ms=(u16)fac_us*1000;
}

void delay_us(volatile u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL=0x01 ;      //��ʼ���� 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}
void delay_ms(volatile u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL=0x01 ;          //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  	    
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

//volatile unsigned long time_delay; // ��ʱʱ�䣬ע�ⶨ��Ϊȫ�ֱ���
////��ʱn_ms
//void delay_ms(volatile unsigned long nms)
//{
//    //SYSTICK��Ƶ--1ms��ϵͳʱ���ж�
//    if (SysTick_Config(SystemFrequency/1000))
//    {
//   
//        while (1);
//    }
//    time_delay=nms;//��ȡ��ʱʱ��
//    while(time_delay);
//    SysTick->CTRL=0x00; //�رռ�����
//    SysTick->VAL =0X00; //��ռ�����
//}
////��ʱnus
//void delay_us(volatile unsigned long nus)
//{
// //SYSTICK��Ƶ--1us��ϵͳʱ���ж�
//    if (SysTick_Config(SystemFrequency/1000000))
//    {
//   
//        while (1);
//    }
//    time_delay=nus;//��ȡ��ʱʱ��
//    while(time_delay);
//    SysTick->CTRL=0x00; //�رռ�����
//    SysTick->VAL =0X00; //��ռ�����
//}
