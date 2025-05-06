#ifndef __headfiles_h
#define __headfiles_h

#define u8 uint8_t
#define u32 uint32_t
#define u16 uint16_t
#define uint unsigned int
#define uchar unsigned char 
#define PI 3.14159265 

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
#include "gray.h"
#include "proc.h"

extern float turn_kp, turn_ki, turn_kd;   // Turn PID parameters
extern float velo_kp, velo_ki, velo_kd;   // Velocity PID parameters
extern float bal_kp, bal_ki, bal_kd;     // Balance PID parameters
extern pids velo, bal;                    // Velocity and balance PID structures
extern float pitch, roll, yaw;           // Pitch, roll, and yaw angles
extern short aacx, aacy, aacz;           // Accelerometer data
extern short gyrox, gyroy, gyroz;        // Gyroscope data
extern float Angle_Balance, Gyro_Balance, Gyro_Turn; // Balance angle, balance gyro, and turn gyro
//extern u8 Way_Angle; 
extern int moto_pwm_l, moto_pwm_r;       // Left and right motor PWM
extern float Acceleration_Z, Acceleration_X; 
extern int Balance_Pwm, Velocity_Pwm, Turn_Pwm;
extern float ZhongZhi;
extern float Gyro_Pitch;                 // Gyro pitch calculated value
extern float angle, angle_dot;           // Output of complementary filter: angle and angular velocity
extern int vl, vr;
extern int encoder_speed;
extern float Accel_Y, Accel_Angle, Accel_Z, Gyro_Y, Gyro_Z, Accel_X;
extern int moto_dead_zone; // Dead zone for motor control
extern u8 moto_lock_flag;
/*外部标志位*/
extern u8 Qina_flag, Hou_flag,Flag_sudu; // Flags for forward and backward movement, and speed control(1 is high speed,2 is low speed)
extern u8 turn_mode;   // Flag for turn control
extern int I0ntegral;  // Integral value for speed control,use to clear the integral value void overflow and restart the speed control
extern float Movement; // Movement speed value for speed control
#endif 

