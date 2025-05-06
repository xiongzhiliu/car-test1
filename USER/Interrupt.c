#include "interrupt.h"
#include "headfiles.h"
#include "string.h"
u8 INT_FLAG=0;
u8 turn_allow_flag=0;
u8 turn_mode = 1;  //0:不启动，1：直接叠加，2：编码器模式
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
int vl,vr;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_0) // Assuming GPIO_PIN_0 is mpu6050 Exit pin ,evety 5 msecs
    {
        int error;
        INT_FLAG = !INT_FLAG;
        //printf("X:%.1f  Y:%.1f  Z:%.1f  %d C\r\n",roll,pitch,yaw,temp/100)

        if(INT_FLAG) //10ms执行控制一次
        {
            Get_Angle(2);//卡尔曼滤波
            if(ABS_float(angle)>20) //判断是否需要转向
            {
                moto_lock_flag = 1;
            }
            if(!k1.is_pull)  //判断按键是否按下启动
							return;
            vl = Read_Velocity_L();
            vr = Read_Velocity_R();
					
            // int velocity_out = velocitydir2(vl, vr);
            // Balance_Pwm = balance(Angle_Balance + velocity_out, Gyro_Balance);

            // moto_pwm_l = Balance_Pwm;
            // moto_pwm_r = Balance_Pwm; 

						// //printf("vl:%d,vr:%d\r\n",vl,vr);
            Balance_Pwm =balance(Angle_Balance,Gyro_Balance);  
            Velocity_Pwm = velocitydir2(vl,vr);		//===平衡PID控制	
           
            moto_pwm_l = Balance_Pwm + Velocity_Pwm;
            moto_pwm_r = Balance_Pwm + Velocity_Pwm;

            if(turn_mode) //转向PID控制
           {
                error = gray_calc_error(); //计算灰度误差
                Turn_Pwm = turn_pwm(error,Gyro_Z);		//===转向PID控制
                //printf("Turn_Pwm:%d\r\n",Turn_Pwm);
                moto_pwm_l -= Turn_Pwm;
                moto_pwm_r += Turn_Pwm;
           }else{  //保持平衡时不读灰度误差只看角速度
                Turn_Pwm = turn_pwm(0,Gyro_Z);		//===转向PID控制
                moto_pwm_l -= Turn_Pwm;
                moto_pwm_r += Turn_Pwm;
           }
            // //printf("%d\r\n",Velocity_Pwm);

						// //printf("V:%d\r\n",Velocity_Pwm);
            // //printf("1:%d,%d\r\n",moto_pwm_l,moto_pwm_r);
            moto_pwm_l=Xianfu_pwm(moto_pwm_l,7199);
            moto_pwm_r=Xianfu_pwm(moto_pwm_r,7199);
						//printf("%d,%d\r\n",moto_pwm_l,moto_pwm_r);
        	  //printf("2:%d,%d\r\n",moto_pwm_l,moto_pwm_r);
            Moto_SetPwm(moto_pwm_l,moto_pwm_r);
        }
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    }
}

//uart1的数据缓冲区
char temp_rx_1[30]={0};
u8 rxdat_1;
volatile u8 rx1_pointer=0;

char temp_rx[30]={0};
u8 rxdat;
volatile u8 rx2_pointer=0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
    HAL_UART_Receive_IT(&huart2,&rxdat,1);
  	temp_rx[rx2_pointer++]=rxdat;
  }
	
  if(huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1,&rxdat_1,1);
    temp_rx_1[rx1_pointer++] = rxdat_1;
  }
}