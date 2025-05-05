#ifndef __headfiles_h
#define __headfiles_h

#define u8 uint8_t
#define u32 uint32_t
#define u16 uint16_t
#define uint unsigned int
#define uchar unsigned char 
#define PI 3.14159265
#define ZHONGZHI -6;     //小车机械中值

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

#include "stdio.h"
#include "stdint.h"
#include "string.h"


#include "main.h"
#include "gpio.h"
#include "adc.h"
//#include "i2c.h"
#include "tim.h"
#include "usart.h"


#include "mpu6050.h"
#include "dmpKey.h"
#include "inv_mpu.h"
#include "dmpmap.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "IOI2C.h"

#include "oled.h"
#include "myadc.h"
#include "LED.h"
#include "moto.h"
#include "interrupt.h"
#include "pid.h"
#include "decoder.h"
#include "fun.h"
#include "delay.h"
#include "filter.h"
#include "pid.h"



extern float pitch,roll,yaw; 		    //欧拉角
extern short aacx,aacy,aacz;			//加速度传感器原始数据
extern short gyrox,gyroy,gyroz;		//陀螺仪原始数据
extern float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
//extern u8 Way_Angle; 
extern int moto_pwm_l,moto_pwm_r; //左右电机pwm
extern float Acceleration_Z,Acceleration_X; 
extern pids bal;
extern int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
extern float ZhongZhi;
extern float Gyro_Pitch;  //角速度积分计算出来的角度
extern float angle,angle_dot;  //互补滤波和卡尔曼滤波输出的角度
extern int vl,vr;
extern int encoder_speed;
extern float Accel_Y,Accel_Angle,Accel_Z,Gyro_Y,Gyro_Z,Accel_X;

#endif 

