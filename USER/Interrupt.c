#include "interrupt.h"
#include "headfiles.h"
#include "string.h"
u8 INT_FLAG=0;
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
int vl,vr;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_0) // Assuming GPIO_PIN_0 is mpu6050 Exit pin ,evety 5 msecs
    {
        INT_FLAG = !INT_FLAG;
				if(!k1.is_pull)
							return;
        //printf("X:%.1f  Y:%.1f  Z:%.1f  %d C\r\n",roll,pitch,yaw,temp/100)
        if(INT_FLAG) //10ms执行控制一次
        {
            vl = Read_Velocity_L();
            vr = Read_Velocity_R();
					  Get_Angle(2);//互补滤波
						//printf("vl:%d,vr:%d\r\n",vl,vr);
            Balance_Pwm =balance(Angle_Balance,Gyro_Balance);  
            Velocity_Pwm = velocitydir2(vl,vr);		//===平衡PID控制	
            //printf("%d\r\n",Velocity_Pwm);
            moto_pwm_l = Balance_Pwm + Velocity_Pwm;
            moto_pwm_r = Balance_Pwm + Velocity_Pwm;
						//printf("V:%d\r\n",Velocity_Pwm);
            //printf("1:%d,%d\r\n",moto_pwm_l,moto_pwm_r);
            moto_pwm_l=Xianfu_pwm(moto_pwm_l,6900);
            moto_pwm_r=Xianfu_pwm(moto_pwm_r,6900);
						//printf("%d,%d\r\n",moto_pwm_l,moto_pwm_r);
        	//printf("2:%d,%d\r\n",moto_pwm_l,moto_pwm_r);
            Moto_SetPwm(moto_pwm_l,moto_pwm_r);
        }
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    }
}

char temp_rx[30]={0};
u8 rxdat;
volatile u8 rx2_pointer=0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart2,&rxdat,1);
	temp_rx[rx2_pointer++]=rxdat;
}