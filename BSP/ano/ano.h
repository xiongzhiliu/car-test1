#ifndef __ANO_H
#define __ANO_H

/* DriverLib Includes */
#include "board.h"
#include "uart.h"

//强制指针类型转换完成字节拆分
#define BYTE0(dwTemp)			(*(char *)(&dwTemp))
#define BYTE1(dwTemp)			(*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)			(*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)			(*((char *)(&dwTemp) + 3))

void ANO_SendF1(short a,short b,short c,unsigned char d);
void ANO_SendF2(short a, short b,short c,short d);
void ANO_Direct(short d);
void ANO_Direct_distance(uint8_t dir,uint8_t num);
#endif
