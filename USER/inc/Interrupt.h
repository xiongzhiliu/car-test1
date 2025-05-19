#ifndef __interrupt_h
#define __interrupt_h

#include "headfiles.h"

extern char temp_rx[30],temp_rx_1[30];
extern volatile u8 rx2_pointer, rx1_pointer;
extern u8 rxdat, rxdat_1;
extern bool gray_dir_allow,gray_allow;
#endif