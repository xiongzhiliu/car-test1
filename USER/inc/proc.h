#ifndef __PROC_H
#define __PROC_H
#include "headfiles.h"

void rx2_proc(void);
void rx1_proc(void);
void key_proc(void);
int Detect_Pick_Up(float Acceleration_Z, float Angle, int encoder_left, int encoder_right);

#endif