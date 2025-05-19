#include "decoder.h"

struct key k1={0,0,0,0};
struct moto_decoer left={0,0,0}, right={0,0,0};
u8 counter_pull=0;
int CALC_LEFT = 0, CALC_RIGHT = 0; //里程计
u8 oledUpdateFlag=0;
u8 oledCount =0 ;
u32 delay_count_10ms=0;
//定时器溢出模式下中断

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	
	static u8 key_pressed = 0;     // 按键按下标志
    static u16 long_press_count = 0; // 长按计数器

	if(htim->Instance == TIM1)  //10ms定时中断
    {
		oledCount++;
        delay_count_10ms++;
		if (oledCount>=50)
		{
			oledCount = 0;
			oledUpdateFlag = 1;
		}
		
        switch(k1.state)
        {
            case 0: // 初始状态：检测按键按下
                if(HAL_GPIO_ReadPin(key_GPIO_Port, key_Pin) == GPIO_PIN_RESET)
                {
                    counter_pull++;
                    if(counter_pull >= KEY_DEBOUNCE_COUNT) // 消抖
                    {
                        k1.state = 1;       // 切换到按下状态
                        counter_pull = 0;    // 清零计数器
                        key_pressed = 1;     // 设置按键按下标志
                        long_press_count = 0; // 清零长按计数器
                    }
                }
                else
                {
                    counter_pull = 0;  // 未持续按下，清零计数器
                }
                break;
                
            case 1: // 按下状态：检测长按或释放
                if(HAL_GPIO_ReadPin(key_GPIO_Port, key_Pin) == GPIO_PIN_RESET)
                {
                    long_press_count++;  // 按键持续按下，增加长按计数
                    
                    if(long_press_count >= KEY_LONG_PRESS_COUNT) // 达到长按阈值
                    {
                        k1.is_pull = 1;      // 触发按键事件
                        k1.press_type = KEY_LONG_PRESS; // 标记为长按
                        k1.state = 2;        // 切换到等待释放状态
                        printf("Key long press detected\r\n");
                    }
                }
                else // 按键释放
                {
                    if(key_pressed) // 确保之前按键确实被按下
                    {
                        if(long_press_count < KEY_LONG_PRESS_COUNT) // 短按释放
                        {
                            k1.is_pull = 1;      // 触发按键事件
                            k1.press_type = KEY_SHORT_PRESS; // 标记为短按
                            printf("Key short press detected\r\n");
                        }
                        key_pressed = 0;    // 清除按下标志
                    }
                    k1.state = 0;       // 返回初始状态
                    counter_pull = 0;    // 清零计数器
                    long_press_count = 0; // 清零长按计数器
                }
                break;
                
            case 2: // 长按后等待释放状态
                if(HAL_GPIO_ReadPin(key_GPIO_Port, key_Pin) != GPIO_PIN_RESET) // 按键释放
                {
                    k1.state = 0;       // 返回初始状态
                    key_pressed = 0;    // 清除按下标志
                    counter_pull = 0;    // 清零计数器
                    long_press_count = 0; // 清零长按计数器
                    printf("Key released after long press\r\n");
                }
                break;
                
            default:
                k1.state = 0;
                counter_pull = 0;
                long_press_count = 0;
                break;
        }
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    }

}

int Read_Velocity_L()
{
	left.en_pul = -(short)__HAL_TIM_GetCounter(&htim3);  //获取编码器计数值
	//left.spd_pul = left.en_pul - left.en_pul_last;  //计算速度增量
	left.en_pul_last = left.en_pul;  //上次读取的计数值
	left.sum_pul += left.en_pul + 20000*left.full_t;  //累计计数值
	__HAL_TIM_SetCounter(&htim3,0);  //计数器清零
	return left.en_pul;  //返回速度增量
}

int Read_Velocity_R()
{
	right.en_pul = (short)__HAL_TIM_GetCounter(&htim2);  //获取编码器计数值
	right.spd_pul = right.en_pul - right.en_pul_last;  //计算速度增量
	right.en_pul_last = right.en_pul;  //上次读取的计数值
	right.sum_pul += right.en_pul + 20000*right.full_t;  //累计计数值
	__HAL_TIM_SetCounter(&htim2,0);  //计数器清零
	return right.en_pul;  //返回速度增量
}
uint8_t lock_flag=0;
void lock_Loc(void)
{
	static int calc_l,calc_r;
	if(!lock_flag){
			calc_l = left.sum_pul;
			calc_r = right.sum_pul;
		lock_flag=1;
	}else if(lock_flag){
			left.sum_pul = calc_l;
			right.sum_pul = calc_r;
		lock_flag=0;
	}
}