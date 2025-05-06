#include "proc.h"


void rx2_proc(void) 
{
	if(rx2_pointer>0)
	{
		//printf("%s",temp_rx);
		HAL_UART_Transmit(&huart1,(u8 *)temp_rx,strlen(temp_rx),20);
		//printf("%d\r\n",strlen(temp_rx));
		rx2_pointer = 0;
		memset(temp_rx,0,30);
	}
}

void rx1_proc(void)
{
  if(rx1_pointer>0)
  {
    float kp,kd_ki;
    u8 rx_type=0;
    sscanf(temp_rx_1,"%hhu,%f,%f,%f",&rx_type,&kp,&kd_ki,&ZhongZhi);
    rx1_pointer=0;
    Moto_SetPwm(0,0);
    k1.is_pull = 0; 
    k1.is_pull_again = 0;
    if(rx_type==0) //balance PID
    {
      pid_init(&bal,kp,0,kd_ki);
      pid_init(&velo,velo_kp,velo_ki,velo_kd);
      printf("bal initialized with kp: %.4f, ki: %.4f, zz: %.2f\r\n", kp, kd_ki, ZhongZhi);
    }
    else if(rx_type==1) //velocity PID
    {
      pid_init(&bal,bal_kp,bal_ki,bal_kd);
      pid_init(&velo,kp,kd_ki,0);
      printf("velo initialized with kp: %.4f, ki: %.4f, zz: %.2f\r\n", kp, kd_ki, ZhongZhi);
    }
    else if(rx_type==2) //pwm测试
    {
      Moto_SetPwm((int)kp,(int)kd_ki);
      printf("motor set to %d,%d\r\n", (int)kp,(int)kd_ki);
    }
    else if(rx_type==3) //encoder speed
    {
      encoder_speed = (int)kp;
      printf("encoder_speed set to %d\r\n", encoder_speed);
    }
    else if(rx_type==4) //dead_zone
    {
      moto_dead_zone = (int)kp;
      printf("dead_zone set to %d\r\n", moto_dead_zone);
    }
    else if(rx_type==5) //motor设置
    {
      moto_pwm_l = (int)kp;
      moto_pwm_r = (int)kd_ki;
      Moto_SetPwm(moto_pwm_l,moto_pwm_r);
      printf("motor set to %d,%d\r\n", moto_pwm_l,moto_pwm_r);
    }
    else if(rx_type==6) //turn PID,PD控制，没有积分无需清除
    {
      turn_kp = kp;
      turn_kd = kd_ki;
      printf("turn initialized with kp: %.4f, kd: %.4f, zz: %.2f\r\n", turn_kp, turn_kd, ZhongZhi);
    }
    else{
      printf("Invalid rx_type: %d\r\n", rx_type);
    }
    memset(temp_rx_1,0,30);
  }
}

void key_proc(void)
{
  if(k1.is_pull_again)
  {
    k1.is_pull_again = 0;
    if(Qina_flag)
    {
      Qina_flag = 0;
      turn_mode = 0;  //需要单独设置一个可以允许转向的情况
      printf("Qina_flag cleared\r\n");
    }
    else
    {
      Qina_flag = 1;
      turn_mode = 1;
      printf("Qina_flag set\r\n");
    }
  }
}