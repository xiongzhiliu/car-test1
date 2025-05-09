#include "gray.h"

u8 STOP_FLAG=0,END_FLAG=0;
u8 last_level=0;
u8 white_counter=0;
u8 whole_black_flag = 0;   //全黑标志
u8 wbf_counter=0;     //全黑j计数
u8 filter_times=3;  
u8 l_counter=0,r_counter=0;  //路口判断计数器
int error;
u8 NODE_DETECT_FLAG, TURN_RIGHT_FLAG, TURN_LEFT_FLAG, BOTH_FLAG, TURN_UP_FLAG, TURN_BACK_FLAG; // 由灰度确定

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

int gray_calc_error(bool judge_flag)
{

  u8 gray_value = read_infrared_sensor();
  switch(gray_value) {
		case 0b00001: error = -6; break; // 最左边检测到线
		case 0b00011: error = -4; break;
		case 0b00110: error = 2; break;  // 中间检测到线
		case 0b00100: error = 0; break;  // 中间检测到线
		case 0b01110: error = 0; break;  // 中间检测到线
		case 0b01100: error = 2; break;
		case 0b11000: error = 4; break;
		case 0b10000: error = 6; break; // 最右边检测到线
		// 其他情况可以根据实际需求添加
    /**************以下为路口状态*******************/
    case 0b00000:
      if(judge_flag==0 && last_level == 0b00000) {
        white_counter++;
        NODE_DETECT_FLAG = 1;
        TURN_BACK_FLAG = 1;    
      }
      break;

    case 0b11111:
      if(judge_flag==0 && last_level == 0b11111){
        wbf_counter++;
        if (wbf_counter > 40 ){  //10次全黑判断为终点，待测试
            //buzzer_turn_on_delay(50);
            stop_move(-1500);
            error = 0;
            STOP_FLAG = 1; 
            NODE_DETECT_FLAG = 1; //终点节点		
            set_corner_dir(direct+2);} //来时路设置为通
          }
      else{
          wbf_counter =0;
      }				 	
      break;
        
    case 0b11100:
      if(judge_flag==0 && last_level == 0b11100)
      {
        l_counter ++;          
        if(l_counter >= filter_times)       //连续读到三次相同level便认为有路口
        {
          NODE_DETECT_FLAG = 1;
          TURN_LEFT_FLAG = 1;
        }
        else{
          l_counter = 0;
        }
      }
      break;

    case 0b11110:
      if(judge_flag==0 && last_level == 0b11110)
      {
        l_counter ++;
        if(l_counter >= filter_times)       //连续读到三次相同level便认为有路口
        {
          NODE_DETECT_FLAG = 1;
          TURN_LEFT_FLAG = 1;
        }
        else{
          l_counter = 0;
        }
      }
      break;

    case 0b00111:
      if(judge_flag==0 && last_level == 0b00111)
      {
        r_counter ++;
        if(r_counter >= filter_times)       //连续读到三次相同level便认为有路口
        {
          NODE_DETECT_FLAG = 1;
          TURN_RIGHT_FLAG = 1;
        }
        else{
          r_counter = 0;
        }
      }
      break;

    case 0b01111:
      if(judge_flag==0 && last_level == 0b01111)
      {
        r_counter ++;
        if(r_counter >= filter_times)       //连续读到三次相同level便认为有路口
        {
          NODE_DETECT_FLAG = 1;
          TURN_RIGHT_FLAG = 1;
        }
      }
       else{
          r_counter = 0;
        }
      break;
      

		default: error = 0; break;
	}

  if(judge_flag==0 && gray_value != 0b11111){
    if(last_level == 0b11111){
      if(gray_value == 0b00000){
        NODE_DETECT_FLAG = 1;
        BOTH_FLAG = 1;
      }
      else{
        NODE_DETECT_FLAG = 1;
        BOTH_FLAG = 1;
        TURN_UP_FLAG = 1;
      }
    }
  }


  last_level = gray_value;
  return error;
}

/**
 * @brief 清除转向标志
 * 
 */
void Clear_NODEflag(void)
{
    TURN_RIGHT_FLAG = 0;
    TURN_LEFT_FLAG = 0;
    BOTH_FLAG = 0;
		TURN_UP_FLAG = 0;
    TURN_BACK_FLAG = 0;
    whole_black_flag = 0;
		counter_clear();
}
void counter_clear(void)
{
	r_counter = 0;
	l_counter = 0;
	wbf_counter = 0;
	white_counter = 0;
}

//清除灰度值
void Clear_levels(void){
	last_level = 0;
}