#include "fun.h"

int ABS_int(int t)
{
	if(t<0)
		return -t;
	return t;
}


float ABS_float(float t)
{
	if(t<0)
		return -t;
	return t;
}

int Threshold_int(int num,int th)
{
	if(num > th)
		return th;
	else if(num < -th)
		return -th;
	else 
		return num;
}


