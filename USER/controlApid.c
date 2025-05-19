#include "controlApid.h"
pids st_pid={0,0,0,0,0,0,0,0},sp_pid={0,0,0,0,0,0,0,0};
pids bal,velo;
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send;                           //计时和触发等变量
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2;                         //遥控相关的变量
float Movement=0;
int encoder_speed;
u8 Qina_flag=0,Hou_flag=0;
int Integral=0; //全局变量速度PI控制的积分值,积分值可以用于控制速度
int xiuzheng=1;  //电机位置和灰度位置的差值
int LocX=0,LocY=0,last_locx=0,last_locy=0,RE_LocX=0,RE_LocY=0;      //绝对坐标 起始方向为绝对坐标x正向；暂存的绝对坐标
u8 ab_x=0,ab_fx=0,ab_y=0,ab_fy=0;				//绝对方向 起始方向为绝对方向x正向
u8 direct=130; 									//全局绝对默认方向为2（模4运算）
int Encoder_Left,Encoder_Right;                 //左右编码器的脉冲计数（会一直清零） 在PID里面调用计算得出
int Encoder_Sum=0,Encoder_Sum2=0;              	//左右编码器的累计  在Decoder里面累加
u8 turn_in_progress = 0; //正在转向标志
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
/**
 * @brief 直立环PD控制函数
 * @param Angle 当前角度值
 * @param Gyro 陀螺仪角速度值
 * @return 平衡控制PWM值
 * 1. 如果当前角度与机械中值的差值大于1度:
 *    - 计算误差 = 当前角度 - 机械中值 + 前倾角度补偿
 * 2. 否则误差就等于前倾角度补偿值
 */
int balance(float Angle,float Gyro){  
	 int balance;
	 bal.error=Angle-ZhongZhi + setAngleForward;                       //===求平衡的角度差值 和机械中值的差
	 balance=bal.Kp*bal.error+Gyro*bal.Kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return (int)balance;
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修改Target_Velocity，比如，改成60就比较慢了
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
// int velocity(int encoder_left,int encoder_right){  
// 	 static float Velocity,Encoder_Least,Encoder;
// 	  static float Encoder_Integral,Target_Velocity;
// 	  //=============遥控前进后退部分=======================// 
// 	  if(Bi_zhang==1&&Flag_sudu==1)  Target_Velocity=55;                 //避障模式中，自动设定行走模式
// 	else 	                         Target_Velocity=110;                 
// 		if(1==Flag_Qian)    	Movement=-Target_Velocity/Flag_sudu;	         //===前进标志位为1 
// 		else if(1==Flag_Hou)	Movement=Target_Velocity/Flag_sudu;         //===后退标志位为1
// 	  else  Movement=0;	
//    //=============速度PI控制器=======================//	
// 		Encoder_Least =(encoder_left+encoder_right)-Velocity;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
// 		Encoder *= 0.8;		                                                //===一阶低通滤波器       
// 		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
// 		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
// 		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
// 		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===积分限幅
// 		if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===积分限幅	
// 		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===速度控制	
// 	  return Velocity;
// }

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

/**************************************************************************
函数功能：转向控制
入口参数：左轮编码器、右轮编码器、陀螺仪
返回  值：转向控制PWM
作    者：lzx
**************************************************************************/
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

u8 Flag_Stop;
int turni, turnj;  			//转向阶段计数器
int turnislow, turnjslow;  //慢速模式下的计数器
int Tmax, rerror, lerror;   //转向时对应的固定偏移量
int Tmaxslow, rerrorslow, lerrorslow;  //慢速模式下的最大值和偏移量
uint8_t stable_count = 0;
//转弯角度通过时间来测试 即Tmax和Tmaxslow
void turnInit(int vel)  //vel确定了转弯的速度，不同的转弯速度转到对应角度的时间不同
{
	Tmax = (int)(45*40/vel);   //需要通过实验确定
	rerror =  + vel;
	lerror =  - vel;
	turni = Tmax;   //都为max时进入巡线程序
	turnj = Tmax;
	Tmaxslow = (int)(50*40*1.5/vel);
	rerrorslow = + (int)(vel/1.5);
	lerrorslow = - (int)(vel/1.5);
	turnislow = Tmaxslow;
	turnjslow = Tmaxslow;
}	

void changeTurnAgle(float degree)   //通过设置turni和turnj计数值来改变转向角度
{
	
	if(degree>0&&degree<=180)
		turni = Tmax - degree/180*Tmax;
	else if(degree>100)
		turnislow = Tmaxslow - degree/180*Tmaxslow;
	else if(degree<0&&degree>=-180)
		turnj = Tmax + degree/180*Tmax;
	else 
		turnjslow = Tmaxslow + degree/180*Tmaxslow;
	turn_in_progress = 1;
}

// 阶段性控制转向
int turnWithStage(int error,float gyro)
{
	int turn;
	if(turni<Tmax&&turnj==Tmax){
		turn_in_progress = 1;
		turn = turn_pwm(lerror, gyro);
	}
	else if(turni==Tmax&&turnj<Tmax){
		turn_in_progress = 1;
		turn = turn_pwm(rerror, gyro);
	}
	else if(turni==Tmax&&turnj==Tmax&&turnislow<Tmaxslow&&turnjslow==Tmaxslow){
		turn_in_progress = 1;
		turn = turn_pwm(lerrorslow, gyro);
	}
	else if(turni==Tmax&&turnj==Tmax&&turnislow==Tmaxslow&&turnjslow<Tmaxslow){
		turn_in_progress = 1;
		turn = turn_pwm(rerrorslow, gyro);
	}
	else if(turni==Tmax&&turnj==Tmax&& turnislow==Tmaxslow&&turnjslow==Tmaxslow)  //上面的if是转弯控制，下面的if是直线控制
	{
		// printf("test1/r/n");
		turn = turn_pwm(error, gyro);
		if(turn_in_progress == 1) {
			stable_count++;
			if(stable_count > 5) { // 稳定5个周期后才清除标志
				turn_in_progress = 0;
				stable_count = 0;
			} 
		}else{
			stable_count = 0;
		}
	}	
	else{
		turn = turn_pwm(0, gyro); //保持直走
		if(turn_in_progress == 1) {
			stable_count++;
			if(stable_count > 5) { // 稳定5个周期后才清除标志
				turn_in_progress = 0;
				stable_count = 0;
			} 
		}else{
			stable_count = 0;
		}
	} //理论上不进入
		
	
	if(!Flag_Stop&&turni!=Tmax) turni++;
	if(!Flag_Stop&&turnj!=Tmax) turnj++;
	if(!Flag_Stop&&turnislow!=Tmaxslow) turnislow++;
	if(!Flag_Stop&&turnjslow!=Tmaxslow) turnjslow++;
	return turn;
}

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


u8 startFlag = 0; //确保第一次能启动
u8 stopFlag = 0;
void start_move(int vel)
{
	if(startFlag) return; //确保这个函数不会一直被调用到值积分一直被清除
	Movement = vel;
	Integral = 0;  //速度积分置为0
	startFlag = 1;
	stopFlag = 0;
	// setAngleForward = (int)vel/6;
}
//全局刹车函数
void stop_move(int itgr){
	if(stopFlag) return;
	Movement=0; //速度清零
	setAngleForward = 0;
	Integral = itgr; 
	stopFlag = 1;
	startFlag = 0;
	// delay_ms(100);
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



void Locationhold(void){
	static int flag,Last_CALC_LEFT=0,Last_CALC_RIGHT=0;
		//direct = judge_dir();
		// OLED_ShowNum(72,0,direct%4,1,16,1);
		switch(direct%4){     //对RE_LocX修正
			case 2:
				 RE_LocX=LocX+xiuzheng;  //修正
				break;
			case 0:
				 RE_LocX=LocX-xiuzheng;
				break;	
			case 1:	
				 RE_LocY = LocY+xiuzheng;
				break;
			case 3:
				RE_LocY = LocY-xiuzheng;
				break;
			
			default:
				break;}
}
		
	// if(RE_LocX>=0){
	// 		OLED_ShowString(0,0,(uint8_t *)"+",8,1);
	// 		OLED_ShowNum(8,0,abs(RE_LocX),4,16,1);
	// }else{
	// 		OLED_ShowString(0,0,(uint8_t *)"-",8,1);
	// 		OLED_ShowNum(8,0,abs(RE_LocX),4,16,1);
	// }
	
	// if(RE_LocY>=0){
	// 		OLED_ShowString(0,16,(uint8_t *)"+",8,1);
	// 		OLED_ShowNum(8,16,abs(RE_LocY),4,16,1);
	// }else{
	// 		OLED_ShowString(0,16,(uint8_t *)"-",8,1);
	// 		OLED_ShowNum(8,16,abs(RE_LocY),4,16,1);
	// }	
	// OLED_Refresh();  //OLED不显示可能的原因


void set_corner_dir(int dir){

	if (dir <= 0){
		dir +=4;} //保证dir是正数 
	switch(dir%4)
	{
		case 0:ab_fx=1;  //绝对x方向可行
      		break;		
		case 1:	ab_fy=1;
			break;
		case 2:ab_x=1;  //x反方向
      		break;
		case 3:ab_y=1;
      		break;
		default:break;
	}
}

void Location(void){     //PID中断调用，计算当前坐标
	static int last_distance=0;
	int tempdis;
	tempdis=(left.sum_pul+right.sum_pul)/144;
	//judge_dir();
	switch(direct%4){  //根据形式方向来计算坐标
		case 2:  //
			LocX+=tempdis-last_distance;
      break;		
		case 1:	
			LocY+=tempdis-last_distance;	
      break;
		case 0:
			LocX-=tempdis-last_distance;	
      break;
		case 3:
			LocY-=tempdis-last_distance;	
      break;
		default:
			break;
	}
	last_distance = tempdis;
}


/**************************************************************************
函数功能：清除绝对方向标志
入口参数：无
返回  值：无
**************************************************************************/
void clear_corner_dir(void){
	ab_x=0;ab_fx=0;ab_y=0;ab_fy=0;  
}


/**************************************************************************
函数功能：根据路口绝对坐标获得转向信息
入口参数：无
返回  值：转向
备    注：direct 是小车当前的方向， ab_directions是小车需要转向的绝对方向，小车当前转向需要二者结合判断
**************************************************************************/
int get_dir(int ab_dir)
{ //该部分与迷宫程序结合 return想要转的方向
	//direct=judge_dir();
	switch(direct%4){
		case 0:{  //当前方向的fy
			if (ab_dir==2) {  //绝对方向向前
				clear_corner_dir();
			return turnBack;
			}
			else if (ab_dir==3) {
				clear_corner_dir();
				return turnLeft;
			}
			else if (ab_dir==1) {
				clear_corner_dir();
				return turnRight;
			}
			else if (ab_dir==4){
				clear_corner_dir();
				return turnUp;
			}else if(ab_dir == 0)
			{
				clear_corner_dir();
				return turnUp;
			}
//			else if(ab_direction==7){
//				clear_corner_dir();
//				return theend;
//			}
			else{
		    return wrong;
			}	
				break;}
		case 1:{ //当前的绝对方向向左	
			if (ab_dir==3) {
				clear_corner_dir();
			return turnBack;
			}
			else if (ab_dir==4) {
				clear_corner_dir();
				return turnLeft;
			}
			else if (ab_dir==2) {
				clear_corner_dir();
				return turnRight;
			}
			else if (ab_dir==1){
				clear_corner_dir();
				return turnUp;
			}else if(ab_dir == 0)
			{
				 clear_corner_dir();
				return turnLeft;
			}
//			else if(ab_direction==7){
//				clear_corner_dir();
//				return theend;
//			}
			else{
		    return wrong;
			} 
			break;}
			
		case 2:{ //绝对向前
			if (ab_dir==4) {
			clear_corner_dir();
			return turnBack;
			}
			else if (ab_dir==1) {
				clear_corner_dir();
				return turnLeft;
			}
			else if (ab_dir==3) {
				clear_corner_dir();
				return turnRight;
			}
			else if (ab_dir==2){
				clear_corner_dir();
				return turnUp;} 
			else if(ab_dir == 0){
				 clear_corner_dir();
				return turnBack;
			}
			else{
		    return wrong;
			}
			break;}
			
		case 3:{ //当前的绝对方向向右
			if (ab_dir==1) {
			clear_corner_dir();
			return turnBack;
			}
			else if (ab_dir==2) {
				clear_corner_dir();
				return turnLeft;
			}
			else if (ab_dir==4) {
				clear_corner_dir();
				return turnRight;
			}
			else if (ab_dir==3){
				clear_corner_dir();
				return turnUp;
			}
			else if(ab_dir == 0){
				 clear_corner_dir();
				return turnRight;
			}
//			else if(ab_direction==7){
//				clear_corner_dir();
//				return theend;
//			}
			else{
		    return wrong;
			}
			break;}
	}
	return wrong;
}


/**************************************************************************
函数功能：设定绝对坐标
入口参数：无
返回  值：无
**************************************************************************/
int get_corner_dir(int dir){
	switch(dir%4)
	{
		case 0:
      return ab_x;
	
		case 1:	
		  return ab_y;

		case 2:
		  return ab_fx;

		case 3:
		  return ab_fy;

	}
	return 0;
}




/**
 * @brief  检测平衡小车是否被拿起
 * @param  Acceleration_Z 当前 Z 轴加速度
 * @param  Angle 当前小车的倾角
 * @param  encoder_left 左轮编码器值
 * @param  encoder_right 右轮编码器值
 * @retval 1 表示小车被拿起，0 表示未被拿起
 */
int Detect_Pick_Up(float Acceleration_Z, float Angle, int encoder_left, int encoder_right) {
  static u16 flag = 0, count_static = 0, count_angle = 0, count_speed = 0;

  // 第一步：检测小车是否接近静止
  if (flag == 0) {
      if (ABS_int(encoder_left) + ABS_int(encoder_right) < 30) { // 静止条件
          count_static++;
      } else {
          count_static = 0;
      }
      if (count_static > 10) { // 静止超过 100ms
          flag = 1;
          count_static = 0;
      }
  }

  // 第二步：检测小车是否在接近水平的位置
  if (flag == 1) {
      if (++count_angle > 200) { // 超时 2000ms，重置状态
          count_angle = 0;
          flag = 0;
      }
      if (Acceleration_Z > 26000 && Angle > (-20 + ZhongZhi) && Angle < (20 + ZhongZhi)) { // 水平条件
          flag = 2;
          count_angle = 0;
      }
  }

  // 第三步：检测小车轮胎是否达到最大转速
  if (flag == 2) {
      if (++count_speed > 100) { // 超时 1000ms，重置状态
          count_speed = 0;
          flag = 0;
      }
      if (ABS_int(encoder_left + encoder_right) > 135) { // 转速条件
          flag = 0;
          return 1; // 检测到小车被拿起
      }
  }

  return 0; // 未检测到小车被拿起
}