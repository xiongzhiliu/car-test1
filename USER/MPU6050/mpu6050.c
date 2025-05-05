#include "mpu6050.h"
/*修正值*/
#define X_ACCEL_OFFSET -600 
#define Y_ACCEL_OFFSET -100 
#define Z_ACCEL_OFFSET 2900 
#define X_GYRO_OFFSET 32 
#define Y_GYRO_OFFSET -11 
#define Z_GYRO_OFFSET 1 

#define mpu hi2c2

/**
* @brief      使用到的HAL函数,上下四选2
 * @param1 
 * @param2 
 * @retval    
 */ 
//HAL_I2C_Master_Transmit()
//HAL_I2C_Master_Receive()
//HAL_I2C_Mem_Read()
//HAL_I2C_Mem_Write()


/**
 * @brief  	    读寄存器
 * @param1 
 * @param2 
 * @retval    
 */ 
u8 MPU_Read_Byte(u8 reg)
{
	u8 data;
//	HAL_I2C_Master_Transmit(&hi2c2,MPU6050_WRITE_ADDR,&reg,1,100);
//	HAL_I2C_Master_Receive(&hi2c2,MPU6050_READ_ADDR,&data,1,100);
	HAL_I2C_Mem_Read(&mpu,MPU6050_WRITE_ADDR,DEV_ADDR,1,&data,1,100);
	return data;
}


u8 MPU_Write_Byte(u8 reg,u8 data)
{
	return HAL_I2C_Mem_Write(&hi2c2,MPU6050_WRITE_ADDR,reg,1,&data,1,100);
}


/**
 * @brief      连续写
* @addr: 期间地址 	
* @reg:  寄存器地址
* @len:  写入长度
* @buf:  数据区
 * @retval    0：正常；其他值：错误代码
 */ 
u8 MPU_Write_Len(u8 addr, u8 reg , u8 len, u8 *buf)
{
	return HAL_I2C_Mem_Write(&mpu,MPU6050_WRITE_ADDR,reg,1,buf,len,100);
}


u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
  HAL_I2C_Mem_Read(&mpu, MPU6050_WRITE_ADDR, reg, 1, buf, len, 100);
  return 0;
}

//void I2C_Start()
//{
//		I2C_SDA_H;//把数据线拉高
//    I2C_SCL_H;//把时钟线拉高
//    delay_us(5);//延时5微秒,要求大于4.7微秒
//    I2C_SDA_L; //拉低，产生下降沿
//    delay_us(5);//这个过程大于4.7微秒
//	}

//void I2C_Stop(void)
//{
//   I2C_SCL_L;
//   I2C_SDA_L;
//   I2C_SCL_H;
//   delay_us(5);
//   I2C_SDA_H;
//   delay_us(5);
//}

//void MPU6050_Write_Reg(uint8_t regAddr, uint8_t regData)
//{
//   
//    I2C_Start();
//    
//    I2C_Write_Byte(DEV_ADDR);
//    if (I2C_Read_ACK())
//        goto stop;
//    
//   
//    I2C_Write_Byte(regAddr);
//    if (I2C_Read_ACK())
//        goto stop;
//    
//    
//    I2C_Write_Byte(regData);
//    if (I2C_Read_ACK())
//        goto stop;
//    
//stop:
//    I2C_Stop();
//}

//uint8_t MPU6050_Read_Reg(uint8_t regAddr)
//{
//    uint8_t regData;
//    
//    /* 发送起始信号 */
//    I2C_Start();
//    
//    /* 发送设备地址 */        
//    I2C_Write_Byte(DEV_ADDR);
//    if (I2C_Read_ACK())
//        goto stop;
//    
//    /* 发送寄存器地址 */
//    I2C_Write_Byte(regAddr);
//    if (I2C_Read_ACK())
//        goto stop;
//    
//    /* 发送重复起始信号 */
//    I2C_Start();
//    
//    /* 发送读模式设备地址 */     
//    I2C_Write_Byte(DEV_ADDR | 0x01);
//    if (I2C_Read_ACK())
//        goto stop;
//    
//    /* 读寄存器数据 */
//    regData = I2C_Read_Byte();
//    I2C_Write_ACK(1);  // 非应答信号     
//stop:
//    I2C_Stop();
//    
//    return regData;
//}

#define u8 uint8_t

void MPU6050_Write_Reg(u8 ADDR, u8 data)
{
		HAL_I2C_Master_Transmit(&hi2c2,ADDR, &data,1,100);
}

u8 MPU6050_Read_Reg(u8 ADDR)
{
	u8 data;
	HAL_I2C_Master_Receive(&hi2c2,ADDR,&data,1,100);
	return data;
}


void MPU6050_Init(void)
{
    MPU6050_Write_Reg(PWR_MGMT_1, 0x00);    //解除休眠状态     
    MPU6050_Write_Reg(SMPLRT_DIV, 0x07);    //陀螺仪采样率，典型值：0x07(125Hz)     
    MPU6050_Write_Reg(CONFIG, 0x06);        //低通滤波频率，典型值：0x06(5Hz)     
    MPU6050_Write_Reg(GYRO_CONFIG, 0x18);   //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)     
    MPU6050_Write_Reg(ACCEL_CONFIG, 0x01);  //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz) 
}

int16_t MPU6050_Get_Data(uint8_t regAddr)
{
    uint8_t Data_H, Data_L;
    uint16_t data;
    Data_H = MPU6050_Read_Reg(regAddr);
    Data_L = MPU6050_Read_Reg(regAddr + 1);
    data = (Data_H << 8) | Data_L;  // 合成数据 
    return data;
}

void MPU6050_Display(void)
{
    /* 打印 x, y, z 轴加速度 */
    printf("ACCEL_X: %d\t", MPU6050_Get_Data(ACCEL_XOUT_H));
    printf("ACCEL_Y: %d\t", MPU6050_Get_Data(ACCEL_YOUT_H));
    printf("ACCEL_Z: %d\t", MPU6050_Get_Data(ACCEL_ZOUT_H));
    
    /* 打印温度，需要根据手册的公式换算为摄氏度 */
    printf("TEMP: %0.2f\t", MPU6050_Get_Data(TEMP_OUT_H) / 340.0 + 36.53);
    
    /* 打印 x, y, z 轴角速度 */
    printf("GYRO_X: %d\t", MPU6050_Get_Data(GYRO_XOUT_H));
    printf("GYRO_Y: %d\t", MPU6050_Get_Data(GYRO_YOUT_H));
    printf("GYRO_Z: %d\t", MPU6050_Get_Data(GYRO_ZOUT_H));
    
    printf("\r\n");
}


void MPU6050_Get_Angle(MPU6050_Angle *data)
{   
    /* 计算x, y, z 轴倾角，返回弧度值*/
    data->X_Angle = acos((MPU6050_Get_Data(ACCEL_XOUT_H) + X_ACCEL_OFFSET) / 16384.0);
    data->Y_Angle = acos((MPU6050_Get_Data(ACCEL_YOUT_H) + Y_ACCEL_OFFSET) / 16384.0);
    data->Z_Angle = acos((MPU6050_Get_Data(ACCEL_ZOUT_H) + Z_ACCEL_OFFSET) / 16384.0);

    /* 弧度值转换为角度值 */
    data->X_Angle = data->X_Angle * 57.29577;
    data->Y_Angle = data->Y_Angle * 57.29577;
    data->Z_Angle = data->Z_Angle * 57.29577;
} 