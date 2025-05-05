#ifndef __DELAY_H
#define __DELAY_H
#include "headfiles.h"
#include "stm32f1xx_hal.h"

#define SystemFrequency 72000000
void delay_init(u8 SYSCLK);
//void delay_us(volatile unsigned long nus);
//void delay_ms(volatile unsigned long nms);
void delay_us(volatile u32 nus);
void delay_ms(volatile u16 nms);
extern volatile unsigned long time_delay; // 延时时间，注意定义为全局变量

#endif