#include "MYI2C.h"

void I2C_Start()
{
		I2C_SDA_H;//把数据线拉高
    I2C_SCL_H;//把时钟线拉高
    delay_us(5);//延时5微秒,要求大于4.7微秒
    I2C_SDA_L; //拉低，产生下降沿
    delay_us(5);//这个过程大于4.7微秒
	}

void I2C_Stop(void)
{
   I2C_SCL_L;
   I2C_SDA_L;
   I2C_SCL_H;
   delay_us(5);
   I2C_SDA_H;
   delay_us(5);
}


#define IIC_DELAY_TIME	 10
static void SendAck(uint8_t _type){
    I2C_SCL_L;
    delay_us(IIC_DELAY_TIME); 	    
    if(_type == 0)
       I2C_SDA_L;
    else
		I2C_SCL_H;
    delay_us(IIC_DELAY_TIME);
    I2C_SCL_H;
    delay_us(IIC_DELAY_TIME);
    I2C_SCL_L;
    delay_us(IIC_DELAY_TIME); 
	}

	static unsigned char IIC_Wait_Ack(void)
{
	unsigned char ack;
	I2C_SCL_L;	//时钟线置低
	 delay_us(1);
	I2C_SDA_H;	//信号线置高
	 delay_us(1);
	I2C_SCL_H;	//时钟线置高
	delay_us(1);
 
	if(I2C_SDA_IN)	//读取SDA的电平
		ack = IIC_NO_ACK;	//如果为1，则从机没有应答
	else
		ack = IIC_ACK;		//如果为0，则从机应答
 
	I2C_SCL_L;//时钟线置低
	 delay_us(1);
	return ack;	//返回读取到的应答信息
}