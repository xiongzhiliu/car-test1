#include "pid.h"
pids st_pid={0,0,0,0,0,0,0,0},sp_pid={0,0,0,0,0,0,0,0};
pids bal,velo;
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send;                           //计时和触发等变量
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2;                         //遥控相关的变量
float Movement=0,Movementset=40;                                                 //速度控制参数
float Balance_Kp=300,Balance_Kd=1.0,Velocity_Kp=120,Velocity_Ki=0.4;            //PID参数
int encoder_speed;
u8 Qina_flag=0,Hou_flag=0;
int Integral=0; //全局变量速度PI控制的积分值,积分值可以用于控制速度

/**
 * @brief   PID初始化函数
 * @param1  pid结构体
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
 * @brief    重置传入PID结构体的基础部分累计值
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
 * @brief      直立环PD控制
 * @param1 
 * @param2 
 * @retval    
 */ 
int balance(float Angle,float Gyro){  
	 int balance;
	 bal.error=Angle-ZhongZhi;                       //===求平衡的角度差值 和机械中值的差
	 balance=bal.Kp*bal.error+Gyro*bal.Kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return (int)balance;
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修改Target_Velocity，比如，改成60就比较慢了
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
int velocity(int encoder_left,int encoder_right){  
	 static float Velocity,Encoder_Least,Encoder;
	  static float Encoder_Integral,Target_Velocity;
	  //=============遥控前进后退部分=======================// 
	  if(Bi_zhang==1&&Flag_sudu==1)  Target_Velocity=55;                 //避障模式中，自动设定行走模式
	else 	                         Target_Velocity=110;                 
		if(1==Flag_Qian)    	Movement=-Target_Velocity/Flag_sudu;	         //===前进标志位为1 
		else if(1==Flag_Hou)	Movement=Target_Velocity/Flag_sudu;         //===后退标志位为1
	  else  Movement=0;	
   //=============速度PI控制器=======================//	
		Encoder_Least =(encoder_left+encoder_right)-Velocity;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.8;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===积分限幅
		if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===积分限幅	
		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===速度控制	
	  return Velocity;
}

int velocitydir2(int encoder_left,int encoder_right){  
	 static int Velocity,Encoder_Least,Encoder;
   //=============速度PI控制器=======================//	
		#define jifenxianfu 20000
		Encoder_Least = (encoder_left+encoder_right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.8;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
		Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Integral=Integral-Movement;                       
		if(Integral>jifenxianfu)  	Integral=jifenxianfu;             //===积分限幅
		if(Integral<-jifenxianfu)	Integral=-jifenxianfu;              //===积分限幅	
		Velocity=Encoder*velo.Kp+Integral*velo.Ki;        //===速度控制	
	  return (int)Velocity;
}


int turn_encode(int encoder_left,int encoder_right,float gyro)//转向控制
{
	static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;	//
	float Turn_Amplitude=88/Flag_sudu,Kp=42,Kd=0;     

	if(1==Flag_Left||1==Flag_Right)                     
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
	if(1==Flag_Left)	 Turn_Target-=Turn_Convert;
	else if(1==Flag_Right)	    Turn_Target+=Turn_Convert; 
	else Turn_Target=0;
	
	if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向速度限幅
	if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
	if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5;        
		else Kd=0;  

		Turn=-Turn_Target*Kp-gyro*Kd;                 
	  return Turn;
}

/**************************************************************************
函数功能：任意角任意速度转向控制 
入口参数：无
返回  值：转向控制PWM
作    者：cyh
**************************************************************************/
int TurnAgle(int error,float Gyro_z)
{//转向控制 degree>0右转 <0左转
	int turn;
	// 根据灰度传感器值计算偏差
	// 调用新的转向控制函数
	turn = turn_pwm(error, Gyro_z);
	return turn;
}


// int turn_bal(int en_lf,int en_rg,float gyro)
// {
// 	float Turn, Kp=1 ,Bias;
// 	Bias  = gyro


// }


/**
 * @brief  转向PID
 * @param1  灰度传参
 * @param2  陀螺仪
 * @retval  转向PWM
 **/
int turn_pwm(int error,float gyro)
{
	int Turn;
	float Bias;
	Bias= gyro - 0;
	Turn = error*turn_kp-gyro*turn_kd;
	return (int)Turn;
}

//全局刹车函数
void stop_move(int itgr){
	Movement=0; //速度清零
	Integral = itgr; 
}

/****************************以下代码仅做备份*******************************/
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
 * @brief      带积分限幅的PID
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
 * @brief      标准的pid函数，用于不想调用
 * @param1 
 * @param2 
 * @retval    返回一个整值，用于对称调用
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
// 以下是被注释的函数

/**************************************************************************
函数功能：直立速度PI控制 修改前进后退速度
入口参数：左轮编码器、右轮编码器、速度设定值
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
//
//	Motor_velocity->derivative = Motor_velocity->error - Motor_velocity->prev_error;
//	Motor_velocity->prev_error = Motor_velocity->error;
//
//	pwm = Motor_velocity->Kp * Motor_velocity->error +
//				Motor_velocity->Ki * Motor_velocity->integral +
//				Motor_velocity->Kd * Motor_velocity->derivative;
//
//	return pwm;
//}

// int turni=0,turnj=0,turnislow,turnjslow;							//快慢转累计控制变量
// int Tmax,rmis,lmis;																		//快转上限与偏移
// int Tmaxslow,rmisslow,lmisslow;												//慢转上限与偏移
// int TurnAgle(void){//转向控制 degree>0右转 <0左转
// 	int turn;
//   //***具体判别***//	
// 	if(turni<Tmax&&turnj==Tmax/*&&turnislow==Tmaxslow&&turnjslow==Tmaxslow*/)               //0  180 ,55 90
// 				turn=turnccd(rmis,Gyro_Turn);                                //===ccd转向环PID控制 
// 	else if(turni==Tmax&&turnj<Tmax/*&&turnislow==Tmaxslow&&turnjslow==Tmaxslow*/)	
// 				turn=turnccd(lmis,Gyro_Turn);
// 		else if(turni==Tmax&&turnj==Tmax&&turnislow<Tmaxslow&&turnjslow==Tmaxslow)
// 				turn=turnccd(rmisslow,Gyro_Turn);  
// 		else if(turni==Tmax&&turnj==Tmax&&turnislow==Tmaxslow&&turnjslow<Tmaxslow)
// 				turn=turnccd(lmisslow,Gyro_Turn);                            //===ccd转向环PID控制 
// 		else if(turni==Tmax&&turnj==Tmax/*&&turnislow==Tmaxslow&&turnjslow==Tmaxslow*/)
// 				turn=turnccd(CCD_Zhongzhi,Gyro_Turn);                                  //===不用ccd时调试用 			
// 		else //可能
// 		{
// 				Movement=0;
// 				turn=turnccd(64,Gyro_Turn);
// 		}
// 		if(!Flag_Stop&&turni!=Tmax) turni++;
// 		if(!Flag_Stop&&turnj!=Tmax) turnj++;
// 		if(!Flag_Stop&&turnislow!=Tmaxslow) turnislow++;
// 		if(!Flag_Stop&&turnjslow!=Tmaxslow) turnjslow++;
// 		return turn;
// }
