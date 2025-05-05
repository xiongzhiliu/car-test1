#include "filter.h"
#include "math.h"
#include "headfiles.h"
#define INT_TIME  0.005
float K1 =0.02; //�����˲�ϵ��, d��Ԫ������Ϊ�������ٶȻ���Ϊ��
float angle, angle_dot; 	
float Q_angle=0.001;// ����������Э����
float Q_gyro=0.003;//0.003 ����������Э���� ����������Э����Ϊһ��һ�����о���
float R_angle=0.5;// ����������Э���� �Ȳ���ƫ��
float dt=0.005;//                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
float Gyro_Pitch = 0;

/**************************************************************************
�������ܣ����׿������˲�
��ڲ��������ٶȡ����ٶ�
����  ֵ����
**************************************************************************/
void Kalman_Filter(float Accel,float Gyro)		
{
	angle+=(Gyro - Q_bias) * dt; //�������
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	angle_dot   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
}

/**************************************************************************
�������ܣ�һ�׻����˲�
��ڲ��������ٶȡ����ٶ�
����  ֵ����
**************************************************************************/
void Yijielvbo(float angle_m, float gyro_m)
{
    angle  = K1 * angle_m+ (1-K1) * (angle + gyro_m * 0.005);
}

/**************************************************************************
�������ܣ���ȡ�Ƕ� �����㷨�������ǵĵ�У�����ǳ����� 
��ڲ�������ȡ�Ƕȵ��㷨 1��DMP(ֱ�Ӷ�ȡ)  2��������  3�������˲�
����  ֵ����
**************************************************************************/
float Accel_Y,Accel_Angle,Accel_Z,Gyro_Y,Gyro_Z,Accel_X;
void Get_Angle(u8 way){ 
	   	temp=Read_Temperature();      //===��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
	    if(way==1)                           //===DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
		{	
			Read_DMP();                      //===��ȡ���ٶȡ����ٶȡ����
			Angle_Balance=-Pitch;             //===����ƽ�����
			Gyro_Balance=-gyro[1];            //===����ƽ����ٶ�
			Gyro_Turn=gyro[2];               //===����ת����ٶ�
			Acceleration_Z=accel[2];         //===����Z����ٶȼ�
		}			
      else
      {
			Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)|I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡY��������
			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)|I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
		  Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)|I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
	  	Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)|I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
			if(Gyro_Y>32768)  Gyro_Y-=65536;                       //��������ת��  Ҳ��ͨ��shortǿ������ת��
			if(Gyro_Z>32768)  Gyro_Z-=65536;                       //��������ת��
	  	if(Accel_X>32768) Accel_X-=65536;                      //��������ת��
		  if(Accel_Z>32768) Accel_Z-=65536;                      //��������ת��
			Accel_X = -Accel_X;
			Accel_Angle=atan2(Accel_X,Accel_Z)*180/PI;              //�������	
				
			Gyro_Y = (Gyro_Y+15);
			//printf("Gyro_Y:%f\r\n",Gyro_Y);
			Gyro_Balance=Gyro_Y;
			Gyro_Y=Gyro_Y/16.4;                                    //����������ת��	
			//printf("Gyro_Y:%d\r\n",Gyro_Y);
			//printf("%f\r\n",Gyro_Balance);
			//Gyro_Pitch+=-(gyrox/16.4)*0.005;  //mpu6050���õ�f=100hz����0.01�����һ��
			if(way==2)
			{
				Kalman_Filter(Accel_Angle,Gyro_Y);//�������˲�	
				//Angle_Balance=angle_dot;    			   //����ƽ�����
				//printf("%.2f\r\n",angle);
			}
			else if(way==3) 
			{
														//�����˲���ʹ��arctan����ĽǶȣ����ڼ��ٶȣ����Լ������ǽǶȼ�����ĽǶ�
				Yijielvbo(Accel_Angle,Gyro_Y);;    			   //����ƽ�����
				//printf("angle:%f\r\n",angle);
			}	
			Angle_Balance = angle;
			printf("%f\r\n",angle);
			Gyro_Turn=Gyro_Z;                         //����ת����ٶ�
			Acceleration_Z=Accel_Z;                   //===����Z����ٶȼ�	
	//				Angle_Balance = pitch;																//ƽ��Ƕ�
		}
}


