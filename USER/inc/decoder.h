#ifndef __decoder_h
#define __decoder_h

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
};

extern struct key k1;
extern struct moto_decoer left,right;//里程计
int Read_Velocity_L();
int Read_Velocity_R();
void lock_Loc(void);
#endif 