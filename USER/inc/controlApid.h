#ifndef __controlApid_h
#define __controlApid_h

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

int TurnAgle(int error,float Gyro_z);

//控制转弯的四个函数
void turnInit(int vel);  				 //初始化转向参数
void changeTurnAgle(float degree);   		//改变转向角度
int turnWithStage(int errorr,float gyro);   //判断情况执行性转向控制
int turn_pwm(int error,float gyro); 		//实际转向控制
void start_move(int vel);
void stop_move(int itgr);

void Locationhold(void);
void Location(void);

void set_corner_dir(int dir);//设置绝对方向
int get_dir(int ab_dir);
void clear_corner_dir(void);
int get_corner_dir(int dir);

extern int LocX,LocY,last_locx,last_locy,RE_LocX,RE_LocY;    //绝对坐标 起始方向为绝对坐标x正向；暂存的绝对坐标
extern u8 ab_x,ab_fx,ab_y,ab_fy; //当前节点的绝对方向	
extern u8 direct;
#endif 