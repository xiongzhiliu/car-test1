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
};

extern struct key k1;

int Read_Velocity_L();
int Read_Velocity_R();
#endif 