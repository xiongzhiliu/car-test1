#ifndef __MYI2C_H
#define __MYI2C_H

#include "stm32f1xx_hal.h"
#include "delay.h"

#define I2C_SDA_H  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET)
#define I2C_SDA_L  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET)
#define I2C_SCL_H  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET)
#define I2C_SCL_L  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET)
#define I2C_SDA_IN HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)

#define IIC_NO_ACK 0
#define IIC_ACK 1


void I2C_Start();
void I2C_Stop();
static void SendAck(uint8_t _type);
#endif 