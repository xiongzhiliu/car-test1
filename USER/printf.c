#include "printf.h"
#define u8 uint8_t
#if 1
__asm(".global __use_no_semihosting");

struct _FILE
{
	int handle;
};

FILE __stdout;
	
void _sys_exit(int x)
{
	x= x;
}

//int fputc(int ch, FILE *f)
//{
//	while((USART1->SR & 0x40) == 0);
//	USART1->DR = (u8)ch;
//	return ch;
//}
#endif