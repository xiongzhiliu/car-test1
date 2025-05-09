#ifndef __GRAY_H
#define __GRAY_H
#include "headfiles.h"

u8 read_infrared_sensor(void);
void show_gray_value(void);
int gray_calc_error(bool judge_flag);
void counter_clear(void);
void Clear_levels(void);
void Clear_NODEflag(void);

extern u8 STOP_FLAG,END_FLAG;
extern int error;
extern uint8_t NODE_DETECT_FLAG, TURN_RIGHT_FLAG, TURN_LEFT_FLAG, BOTH_FLAG, TURN_UP_FLAG, TURN_BACK_FLAG;
#endif