#include "interrupt.h"
#include "headfiles.h"
#include "string.h"
bool gray_dir_allow = 0,gray_allow=1;    //灰度检查路口禁止标志；中断中灰度读取允许标志

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
            // if(ABS_float(angle)>40) //判断是否需要转向
            // {
            //     moto_lock_flag = 1;
            // }
            vl = Read_Velocity_L();
            vr = Read_Velocity_R();
            // printf("vl:%d,vr:%d\r\n",vl,vr);
            if(!interrupt_allow_flag)  //判断按键是否按下启动
							return;
           
            Location();   //更新坐标
            // int velocity_out = velocitydir2(vl, vr);
            // Balance_Pwm = balance(Angle_Balance + velocity_out, Gyro_Balance);

            // moto_pwm_l = Balance_Pwm;
            // moto_pwm_r = Balance_Pwm; 

						// //printf("vl:%d,vr:%d\r\n",vl,vr);
            Balance_Pwm =balance(Angle_Balance,Gyro_Balance);  
            Velocity_Pwm = velocitydir2(vl,vr);		//===平衡PID控制	
            if(gray_allow)
            {
              error = gray_calc_error(gray_dir_allow);  
            }else{
              error = 0;
            }
            Turn_Pwm = turnWithStage(error,Gyro_Z);

            moto_pwm_l = Balance_Pwm + Velocity_Pwm - Turn_Pwm;
            moto_pwm_r = Balance_Pwm + Velocity_Pwm + Turn_Pwm;

        
            moto_pwm_l=Xianfu_pwm(moto_pwm_l,7199);
            moto_pwm_r=Xianfu_pwm(moto_pwm_r,7199);
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
uint8_t maxi_down=0;
#define BLERX_LEN_MAX 200
u8 BLERX_LEN_maxi = 0;
unsigned char BLERX_BUFF_maix[BLERX_LEN_MAX];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
    HAL_UART_Receive_IT(&huart2,&rxdat,1);
  	temp_rx[rx2_pointer++]=rxdat;
  }
	
  if(huart->Instance == USART1)
  {
    // temp_rx_1[rx1_pointer++] = rxdat_1;
    if (BLERX_LEN_maxi < BLERX_LEN_MAX - 1) // 保留一个字符的空间用于'\0'
    {
      if(rxdat_1 == 0xff) // 检查是否是结束标志
      {
        maxi_down = 1;
        BLERX_BUFF_maix[BLERX_LEN_maxi] = '\0'; //确保字符串正确结束
        //process_receive();
        BLERX_LEN_maxi = 0; // 重置接收长度
      }else if(rxdat_1 >= 0 && rxdat_1 <= 3){ //检查是否是路口方向
        BLERX_BUFF_maix[BLERX_LEN_maxi++] = rxdat_1;
      }
    }
    HAL_UART_Receive_IT(&huart1,&rxdat_1,1);
  }
  if(maxi_down == 1){
    for(int i=0; BLERX_BUFF_maix[i] != '\0';i++){
        //printf("%c",BLERX_BUFF[i]);
        node2_list[i] = BLERX_BUFF_maix[i]; 
        buzzerTurnOnDelay(10);
        //printf("%d",node2_list[i]);
        // printf("%d",BLERX_BUFF_maix[i]);
    }
  }
}