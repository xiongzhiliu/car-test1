#include "pid.h"
pids st_pid={0,0,0,0,0,0,0,0},sp_pid={0,0,0,0,0,0,0,0};
pids bal,velo;
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send; 							//��ʱ�͵��εȱ���
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; 						//����ң����صı���
float Movement=0,Movementset=40;																			//�ٶȿ���������
float Balance_Kp=300,Balance_Kd=1.0,Velocity_Kp=120,Velocity_Ki=0.4;//PID����

/**
 * @brief   PID��ʼ������ 
 * @param1 	pid�ṹ��
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
 * @brief    �������PID�ṹ��Ļ��ֲ���ۼ�ֵ
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
 * @brief  �ٶȻ�
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
 * @brief      ������������޵�PID
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
 * @brief      ������pid�������ڲ��ٵ���
 * @param1 
 * @param2 
 * @retval    ����һ����ֵ�����ڶԳƵ���
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
 * @brief      ֱ����PD����
 * @param1 
 * @param2 
 * @retval    
 */ 
int balance(float Angle,float Gyro){  
	 int balance;
	 bal.error=Angle-ZhongZhi;                       //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 balance=bal.Kp*bal.error+Gyro*bal.Kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ��   ���ٶȺ��ٶ�
	 return (int)balance;
}
/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶȣ�����Target_Velocity�����磬�ĳ�60�ͱȽ�����
��ڲ��������ֱ����������ֱ�����
����  ֵ���ٶȿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int velocity(int encoder_left,int encoder_right){  
     static float Velocity,Encoder_Least,Encoder;
	  static float Encoder_Integral,Target_Velocity;
	  //=============ң��ǰ�����˲���=======================// 
	  if(Bi_zhang==1&&Flag_sudu==1)  Target_Velocity=55;                 //����������ģʽ,�Զ��������ģʽ
    else 	                         Target_Velocity=110;                 
		if(1==Flag_Qian)    	Movement=-Target_Velocity/Flag_sudu;	         //===ǰ����־λ��1 
		else if(1==Flag_Hou)	Movement=Target_Velocity/Flag_sudu;         //===���˱�־λ��1
	  else  Movement=0;	
//	  if(Bi_zhang==1&&Distance<500&&Flag_Left!=1&&Flag_Right!=1)        //���ϱ�־λ��1�ҷ�ң��ת���ʱ�򣬽������ģʽ
//	  Movement=Target_Velocity/Flag_sudu;
   //=============�ٶ�PI������=======================//	
		Encoder_Least =(encoder_left+encoder_right)-Velocity;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
		Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===�����޷�
		if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===�����޷�	
		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===�ٶȿ���	
//		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   Encoder_Integral=0;      //===����رպ��������
	  return Velocity;
}
/**************************************************************************
�������ܣ�ֱ���ٶ�PI���� �޸�ǰ�������ٶ�
��ڲ��������ֱ����������ֱ��������ٶȲ���
����  ֵ���ٶȿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
//int velocitydir(int encoder_left,int encoder_right,int speed){  
//     static float Velocity,Encoder_Least,Encoder;
//	  static float Encoder_Integral;
//		Movement=speed;   																										//�ٶ��趨
//	  
//   //=============�ٶ�PI������=======================//	
//		Encoder_Least =(encoder_left+encoder_right)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
//		Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
//		Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
//		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
//		Encoder_Integral=Encoder_Integral-Movement;                       //===����ң�������ݣ�����ǰ������
//		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===�����޷�
//		if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===�����޷�	
//		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===�ٶȿ���	
////		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   Encoder_Integral=0;      //===����رպ��������
//	  return Velocity;
//}
/**************************************************************************
�������ܣ�ȫ�ֱ����ٶ�PI���� �޸�ǰ�������ٶ�
��ڲ��������ֱ����������ֱ�����
����  ֵ���ٶȿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
//int velocitydir2(int encoder_left,int encoder_right){  
//     static float Velocity,Encoder_Least,Encoder;
//   //=============�ٶ�PI������=======================//	
//	#define jifenxianfu 30000
//		Encoder_Least =(encoder_left+encoder_right)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
//		Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
//		Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
//		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
//		Encoder_Integral=Encoder_Integral-Movement;                       
//		if(Encoder_Integral>jifenxianfu)  	Encoder_Integral=jifenxianfu;             //===�����޷�
//		if(Encoder_Integral<-jifenxianfu)	Encoder_Integral=-jifenxianfu;              //===�����޷�	
//		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===�ٶȿ���	
//		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   Encoder_Integral=0;      //===����رպ��������
//	  return Velocity;
//}
/**************************************************************************
�������ܣ�ת�����  �޸�ת���ٶȣ����޸�Turn_Amplitude����
��ڲ��������ֱ����������ֱ�������Z��������
����  ֵ��ת�����PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro){//ת�����
	 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude=88/Flag_sudu,Kp=42,Kd=0;     
	  //=============ң��������ת����=======================//
  	if(1==Flag_Left||1==Flag_Right)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
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
	
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5;        
		else Kd=0;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
  	//=============ת��PD������=======================//
		Turn=-Turn_Target*Kp-gyro*Kd;                 //===���Z�������ǽ���PD����
	  return Turn;
}

int encoder_speed;

int velocitydir2(int encoder_left,int encoder_right){  
     static int Velocity,Encoder_Least,Encoder,Integral;
   //=============�ٶ�PI������=======================//	
		#define jifenxianfu 30000
		Encoder_Least = (encoder_left+encoder_right)-encoder_speed;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
		Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
		Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		Integral=Integral-Movement;                       
		if(Integral>jifenxianfu)  	Integral=jifenxianfu;             //===�����޷�
		if(Integral<-jifenxianfu)	Integral=-jifenxianfu;              //===�����޷�	
		Velocity=Encoder*velo.Kp+Integral*velo.Ki;        //===�ٶȿ���	
		//if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   Encoder_Integral=0;      //===����رպ��������
	  return (int)Velocity;
}


//#define SPE_DEAD_ZONE 2				// ����
//#define SPE_INTEGRAL_START_ERR 1000 // ���ַ���
//#define SPE_INTEGRAL_MAX_VAL 20000	// �����޷�
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

