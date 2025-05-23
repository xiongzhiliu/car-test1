#ifndef __decoder_h
#define __decoder_h

#define KEY_DEBOUNCE_COUNT 3      // 消抖计数阈值
#define KEY_LONG_PRESS_COUNT 100   // 长按计数阈值
#define KEY_SHORT_PRESS 1         // 短按标志
#define KEY_LONG_PRESS 2          // 长按标志
#include "headfiles.h"

struct moto_decoer
{
	int en_pul;
	int en_pul_last; //上次读取的脉冲数
	int sum_pul; //累加脉冲数
	int spd_pul; //速度脉冲数
	int full_t;  //溢出次数
};

struct key
{
	u8 state;
	u8 level;
	u8 is_pull;
	u8 is_pull_again;
	u8 press_type;
};

extern struct key k1;
extern struct moto_decoer left,right;//里程计
extern u8 oledUpdateFlag;
extern u32 delay_count_10ms;
int Read_Velocity_L();
int Read_Velocity_R();
void lock_Loc(void);
#endif 