#ifndef __pid_h
#define __pid_h

typedef struct 
{
	int Kp;
	float Ki;
	float Kd;
	float error;
	float prev_error;
	float prev_error_twice;
	float integral;
	float derivative;
	int dead_zone;
	float intergral_start_error;//积分起始值
	float intergral_max_val;    //积分上限
} pids;
#include "headfiles.h"

/** 
 * @brief     PID结构体定义
 * @param1    共11个参数
 * @param2 
 * @retval    
 */ 


void pid_init(pids *pid, float Kp, float Ki, float Kd);
void clear_pid(pids *pid);
int cf_Motor_PID(pids *Motor_velocity, int Target, short read);
int cf_PID(pids *pid, float tar, float read);
int cf_pid_ddz(pids *pid, int tar , int read);
int balance(float Angle,float Gyro);
int velocity(int encoder_left,int encoder_right);
int turn(int encoder_left,int encoder_right,float gyro);
int velocitydir2(int encoder_left,int encoder_right);
int turn_pwm(int error,float gyro);
int TurnAgle(int error,float Gyro_z);
void stop_move(int itgr);
#endif 