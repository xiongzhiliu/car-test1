#ifndef __filter_h
#define __filter_h

#include "headfiles.h"
extern float angle, angle_dot; 	
void Kalman_Filter(float Accel,float Gyro);		
void Yijielvbo(float angle_m, float gyro_m);
extern int temp;


void Get_Angle(u8 way);
#endif
