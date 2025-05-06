#include "gray.h"

u8 Stop_flag=0;


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

int gray_calc_error(void)
{
  int error = 0;
  u8 gray_value = read_infrared_sensor();
  switch(gray_value) {
		case 0b00001: error = -4; break; // 最左边检测到线
		case 0b00011: error = -3; break;
		case 0b00110: error = 2; break;  // 中间检测到线
		case 0b00100: error = 0; break;  // 中间检测到线
		case 0b01110: error = 0; break;  // 中间检测到线
		case 0b01100: error = 2; break;
		case 0b11000: error = 3; break;
		case 0b10000: error = 4; break; // 最右边检测到线
		// 其他情况可以根据实际需求添加

    case 0b11111: // 所有传感器都检测到线
      if (Stop_flag == 0) {
        stop_move(-1500);
        Stop_flag = 1; // 设置停止标志
        error = 0; // 停止时不需要调整
      } else {
        error = 0; // 如果已经停止，则不需要调整
      }
      break;
		default: error = 0; break;
	}
  return error;
}