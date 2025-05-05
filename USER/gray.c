#include "gray.h"
u8 read_infrared_sensor(void) 
{
    u8 levels = 0;
    u8 a=HAL_GPIO_ReadPin(infrared1_GPIO_Port, infrared1_Pin); // Value 16 (leftmost)
    u8 b=HAL_GPIO_ReadPin(infrared2_GPIO_Port, infrared2_Pin); // Value 1
    u8 c=HAL_GPIO_ReadPin(infrared3_GPIO_Port, infrared3_Pin); // Value 8
    u8 d=HAL_GPIO_ReadPin(infrared4_GPIO_Port, infrared4_Pin); // Value 2
    u8 e=HAL_GPIO_ReadPin(infrared5_GPIO_Port, infrared5_Pin); // Value 4 (rightmost)
    // Mapping sensor values to the specified weights
    levels = (a << 4) | (b << 0) | (c << 3) | (d << 1) | (e << 2);
    return levels;
}

void show_gray_value(void)
{
  u8 gray_value = read_infrared_sensor();
  printf("Gray value: %d\r\n", gray_value);
}