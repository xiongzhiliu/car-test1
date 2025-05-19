#include "fun.h"

int ABS_int(int t)
{
	if(t<0)
		return -t;
	return t;
}


float ABS_float(float t)
{
	if(t<0)
		return -t;
	return t;
}

int Threshold_int(int num,int th)
{
	if(num > th)
		return th;
	else if(num < -th)
		return -th;
	else 
		return num;
}



void Test_Balance_Car(void)
{
    // 初始化变量
    int forward_pwm = 0; // 前进的PWM值
    int duration = 3000; // 前进的持续时间（单位：ms）
    int balance_pwm = 0, velocity_pwm = 0;

    // 第一步：平衡初始化
    printf("Initializing balance...\r\n");
	if(interrupt_allow_flag == 1);
	else return;
    delay_ms(1000);    // 等待1秒

    // 第二步：进入平衡状态

    // 第三步：向前运动
	start_move(30);

	delay_ms(1000);
    // 第四步：停车并保持平衡
    stop_move(-1500);

    delay_ms(5000);
}