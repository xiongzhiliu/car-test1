#include "pid.h"
pids st_pid={0,0,0,0,0,0,0,0},sp_pid={0,0,0,0,0,0,0,0};
pids bal,velo;
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send; 							//延时和调参等变量
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; 						//蓝牙遥控相关的变量
float Movement=0,Movementset=40;																			//速度控制与设置
float Balance_Kp=300,Balance_Kd=1.0,Velocity_Kp=120,Velocity_Ki=0.4;//PID参数

/**
 * @brief   PID初始化函数 
 * @param1 	pid结构体
 * @param2 
 * @retval    
 */ 
void pid_init(pids *pid, float Kp, float Ki, float Kd)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->integral = 0;
	pid->derivative = 0;
	pid->error = 0;
	pid->prev_error = 0;
	pid->prev_error_twice = 0;
}

/**
 * @brief    清除对于PID结构体的积分差分累计值
 * @param1 
 * @param2 
 * @retval    
 */ 
void clear_pid(pids *pid)
{
	pid->integral = 0;
	pid->derivative = 0;
	pid->error = 0;
	pid->prev_error = 0;
	pid->prev_error_twice = 0;
 }

/**
 * @brief  速度环
 * @param1 
 * @param2 
 * @retval    
 */ 
int cf_Motor_PID(pids *Motor_velocity, int Target, short read)
{
	int pwm;
	Motor_velocity->error = (float)(Target - (int)read);
	Motor_velocity->integral += Motor_velocity->error;
	if (Motor_velocity->integral > Motor_velocity->intergral_max_val)
		Motor_velocity->integral = Motor_velocity->intergral_max_val;
	else if (Motor_velocity->integral < -Motor_velocity->intergral_max_val)
		Motor_velocity->integral = -Motor_velocity->intergral_max_val;

	Motor_velocity->derivative = Motor_velocity->error - Motor_velocity->prev_error;
	Motor_velocity->prev_error = Motor_velocity->error;

	pwm = Motor_velocity->Kp * Motor_velocity->error +
		    Motor_velocity->Ki * Motor_velocity->integral +
		    Motor_velocity->Kd * Motor_velocity->derivative;
	return pwm;
}


/**
 * @brief      常规带积分上限的PID
 * @param1 
 * @param2 
 * @retval    
 */ 
int cf_PID(pids *pid, float tar, float read)
{
	int pwm;
	pid->error = (float)(tar - read);
	pid->integral += pid->error;
	if (pid->integral > pid->intergral_max_val)
		pid->integral = pid->intergral_max_val;
	else if (pid->integral < -pid->intergral_max_val)
		pid->integral = -pid->intergral_max_val;

	pid->derivative = pid->error - pid->prev_error;
	pid->prev_error = pid->error;

	pwm = pid->Kp * pid->error +
				pid->Ki * pid->integral +
				pid->Kd * pid->derivative;
	return pwm;
}


/**
 * @brief      带死区pid，常用于差速调节
 * @param1 
 * @param2 
 * @retval    返回一个差值，用于对称调速
 */ 
int cf_pid_ddz(pids *pid, int tar , int read)
{
		int pwm;
	pid->error = tar-read;
	pid->integral += pid->error;
	pid->derivative = pid->error - pid->prev_error;
	pid->prev_error = pid->error;
	
	pwm = pid->Kp * pid->error +
				pid->Ki * pid->integral +
				pid->Kd * pid->derivative;
	return pwm;
}


/**
 * @brief      直立环PD控制
 * @param1 
 * @param2 
 * @retval    
 */ 
int balance(float Angle,float Gyro){  
	 int balance;
	 bal.error=Angle-ZhongZhi;                       //===求出平衡的角度中值 和机械相关
	 balance=bal.Kp*bal.error+Gyro*bal.Kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数   加速度和速度
	 return (int)balance;
}
/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity，比如，改成60就比较慢了
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
int velocity(int encoder_left,int encoder_right){  
     static float Velocity,Encoder_Least,Encoder;
	  static float Encoder_Integral,Target_Velocity;
	  //=============遥控前进后退部分=======================// 
	  if(Bi_zhang==1&&Flag_sudu==1)  Target_Velocity=55;                 //如果进入避障模式,自动进入低速模式
    else 	                         Target_Velocity=110;                 
		if(1==Flag_Qian)    	Movement=-Target_Velocity/Flag_sudu;	         //===前进标志位置1 
		else if(1==Flag_Hou)	Movement=Target_Velocity/Flag_sudu;         //===后退标志位置1
	  else  Movement=0;	
//	  if(Bi_zhang==1&&Distance<500&&Flag_Left!=1&&Flag_Right!=1)        //避障标志位置1且非遥控转弯的时候，进入避障模式
//	  Movement=Target_Velocity/Flag_sudu;
   //=============速度PI控制器=======================//	
		Encoder_Least =(encoder_left+encoder_right)-Velocity;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.8;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===积分限幅
		if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===积分限幅	
		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===速度控制	
//		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   Encoder_Integral=0;      //===电机关闭后清除积分
	  return Velocity;
}
/**************************************************************************
函数功能：直接速度PI控制 修改前进后退速度
入口参数：左轮编码器、右轮编码器、速度参数
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
//int velocitydir(int encoder_left,int encoder_right,int speed){  
//     static float Velocity,Encoder_Least,Encoder;
//	  static float Encoder_Integral;
//		Movement=speed;   																										//速度设定
//	  
//   //=============速度PI控制器=======================//	
//		Encoder_Least =(encoder_left+encoder_right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
//		Encoder *= 0.8;		                                                //===一阶低通滤波器       
//		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
//		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
//		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
//		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===积分限幅
//		if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===积分限幅	
//		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===速度控制	
////		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   Encoder_Integral=0;      //===电机关闭后清除积分
//	  return Velocity;
//}
/**************************************************************************
函数功能：全局变量速度PI控制 修改前进后退速度
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
//int velocitydir2(int encoder_left,int encoder_right){  
//     static float Velocity,Encoder_Least,Encoder;
//   //=============速度PI控制器=======================//	
//	#define jifenxianfu 30000
//		Encoder_Least =(encoder_left+encoder_right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
//		Encoder *= 0.8;		                                                //===一阶低通滤波器       
//		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
//		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
//		Encoder_Integral=Encoder_Integral-Movement;                       
//		if(Encoder_Integral>jifenxianfu)  	Encoder_Integral=jifenxianfu;             //===积分限幅
//		if(Encoder_Integral<-jifenxianfu)	Encoder_Integral=-jifenxianfu;              //===积分限幅	
//		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===速度控制	
//		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   Encoder_Integral=0;      //===电机关闭后清除积分
//	  return Velocity;
//}
/**************************************************************************
函数功能：转向控制  修改转向速度，请修改Turn_Amplitude即可
入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
返回  值：转向控制PWM
作    者：平衡小车之家
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro){//转向控制
	 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude=88/Flag_sudu,Kp=42,Kd=0;     
	  //=============遥控左右旋转部分=======================//
  	if(1==Flag_Left||1==Flag_Right)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
		{
			if(++Turn_Count==1)
			Encoder_temp=ABS_int(encoder_left+encoder_right);
			Turn_Convert=50/Encoder_temp;
			if(Turn_Convert<0.6)Turn_Convert=0.6;
			if(Turn_Convert>3)Turn_Convert=3;
		}	
	  else
		{
			Turn_Convert=0.9;
			Turn_Count=0;
			Encoder_temp=0;
		}			
		if(1==Flag_Left)	           Turn_Target-=Turn_Convert;
		else if(1==Flag_Right)	     Turn_Target+=Turn_Convert; 
		else Turn_Target=0;
	
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向速度限幅
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5;        
		else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
  	//=============转向PD控制器=======================//
		Turn=-Turn_Target*Kp-gyro*Kd;                 //===结合Z轴陀螺仪进行PD控制
	  return Turn;
}

int encoder_speed;

int velocitydir2(int encoder_left,int encoder_right){  
     static int Velocity,Encoder_Least,Encoder,Integral;
   //=============速度PI控制器=======================//	
		#define jifenxianfu 30000
		Encoder_Least = (encoder_left+encoder_right)-encoder_speed;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.8;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
		Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Integral=Integral-Movement;                       
		if(Integral>jifenxianfu)  	Integral=jifenxianfu;             //===积分限幅
		if(Integral<-jifenxianfu)	Integral=-jifenxianfu;              //===积分限幅	
		Velocity=Encoder*velo.Kp+Integral*velo.Ki;        //===速度控制	
		//if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   Encoder_Integral=0;      //===电机关闭后清除积分
	  return (int)Velocity;
}


//#define SPE_DEAD_ZONE 2				// 死区
//#define SPE_INTEGRAL_START_ERR 1000 // 积分分离
//#define SPE_INTEGRAL_MAX_VAL 20000	// 积分限幅
//int Motor_PID(pids *Motor_velocity, int Target, int read)
//{
//	int pwm;
//	Motor_velocity->error = (float)(Target - (int)read);
//	Motor_velocity->integral += Motor_velocity->error;
//	if (Motor_velocity->integral > SPE_INTEGRAL_MAX_VAL)
//		Motor_velocity->integral = SPE_INTEGRAL_MAX_VAL;
//	else if (Motor_velocity->integral < -SPE_INTEGRAL_MAX_VAL)
//		Motor_velocity->integral = -SPE_INTEGRAL_MAX_VAL;

//	Motor_velocity->derivative = Motor_velocity->error - Motor_velocity->prev_error;
//	Motor_velocity->prev_error = Motor_velocity->error;

//	pwm = Motor_velocity->Kp * Motor_velocity->error +
//				Motor_velocity->Ki * Motor_velocity->integral +
//				Motor_velocity->Kd * Motor_velocity->derivative;

//	return pwm;
//}

