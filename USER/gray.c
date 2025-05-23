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


u8 last_node_gray = 0; // 记录上一次节点检测时的灰度值
u8 nodeJudgePending = 0; // 标记等待下一次判断
/**
 * @brief 灰度计算误差
 * 
 * @param judge_flag  是否允许判断路口；1：允许 0：不允许
 * @return int 
 */
int gray_calc_error(bool judge_flag)  //1：允许 0：不允许
{

  u8 gray_value = read_infrared_sensor();
  switch(gray_value) {
		case 0b00001: error = -6; break; // 最左边检测到线
		case 0b00011: error = -4; break;
		case 0b00010: error = -3; break;  // 中间检测到线
		case 0b00110: error = -2; break;  // 中间检测到线
		case 0b00100: error = 0; break;  // 中间检测到线
		case 0b01110: error = 0; break;  // 中间检测到线
		case 0b01100: error = 2; break;
		case 0b01000: error = 3; break;
		case 0b11000: error = 4; break;
		case 0b10000: error = 6; break; // 最右边检测到线
		// 其他情况可以根据实际需求添加
    /**************以下为路口状态*******************/
    case 0b00000:
      if(judge_flag==1 && last_level == 0b00000) {
        wbf_counter =0;
        white_counter++;
        if (white_counter > 10) {  // 10次全白判断为路口，待测试
            //buzzer_turn_on_delay(50);
            NODE_DETECT_FLAG = 1;
            nodeJudgePending = 0;
        }
      }else{
        white_counter = 0;	
      }
      break;

    case 0b11111:
      if(judge_flag==1 && last_level == 0b11111){
        white_counter=0;
        wbf_counter++;
        if (wbf_counter > 30 ){  //20次全黑判断为终点，待测试
            //buzzer_turn_on_delay(50);
            error = 0;
            STOP_FLAG = 1; 
            NODE_DETECT_FLAG = 1;
            BOTH_FLAG = 0;
            nodeJudgePending = 0;
            // nodeJudgePending = 1; //终点节点		
        } //来时路设置为通
        else if(wbf_counter >= filter_times){    //连续读到三次相同level便认为有路口
          nodeJudgePending = 1;
          BOTH_FLAG = 1;
        }
      }else{
          wbf_counter =0;
      }				 	
      break;
        
    case 0b11100:
      if(judge_flag==1 && last_level == 0b11100)
      {
        white_counter=0;
        wbf_counter =0;
        l_counter ++;          
        if(l_counter >= filter_times)       //连续读到三次相同level便认为有路口
        {
          nodeJudgePending = 1;
          TURN_LEFT_FLAG = 1;
        }
      }else{
          l_counter = 0;
        }
      break;

    case 0b11110:
      if(judge_flag==1 && last_level == 0b11110)
      {
        white_counter=0;
        wbf_counter =0;
        l_counter ++;
        if(l_counter >= filter_times)       //连续读到三次相同level便认为有路口
        {
          nodeJudgePending = 1;
          TURN_LEFT_FLAG = 1;
        }
      }else{
          l_counter = 0;
      }
      break;

    case 0b00111:
      if(judge_flag==1 && last_level == 0b00111)
      {
        white_counter=0;
        wbf_counter =0;
        r_counter ++;
        if(r_counter >= filter_times)       //连续读到三次相同level便认为有路口
        {
          nodeJudgePending = 1;
          TURN_RIGHT_FLAG = 1;
        }
      }else{
          r_counter = 0;
      }
      break;

    case 0b01111:
      if(judge_flag==1 && last_level == 0b01111)
      {
        white_counter=0;
        wbf_counter =0;
        r_counter ++;
        if(r_counter >= filter_times)       //连续读到三次相同level便认为有路口
        {
          nodeJudgePending = 1;
          TURN_RIGHT_FLAG = 1;
        }
      }
      else{
          r_counter = 0;
      }
      break;

		default: error = 0; break;
	}

  // if(judge_flag==1 && gray_value != 0b11111){
  //   if(last_level == 0b11111){
  //     if(gray_value == 0b00000){
  //       nodeJudgePending = 1;
  //       BOTH_FLAG = 1;
  //     }
  //     else{
  //       // NODE_DETECT_FLAG = 1;
  //       nodeJudgePending = 1;
  //       BOTH_FLAG = 1;
  //       // TURN_UP_FLAG = 1;
  //     }
  //   }
  // }
 
  if(judge_flag== 1 && nodeJudgePending == 1 && last_level != gray_value){  //识别到路口状态挂起且这次灰度不等于上一次，就说明是路口后的变化
    if(gray_value == 0b00000){
      NODE_DETECT_FLAG = 1;
      nodeJudgePending = 0;
    }else if ((gray_value & 0b01110)&&!(gray_value & 0b10001)) //左右两边没有检测到线，说明是直道
    {
      NODE_DETECT_FLAG = 1;
      TURN_UP_FLAG = 1;
      nodeJudgePending = 0;
    }else{
    ;
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