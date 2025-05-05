#include "MYI2C.h"


double getADC_value(ADC_HandleTypeDef* pin)
{
	int dat;
	dat = HAL_ADC_GetValue(pin);
	return dat*3.3/4096;	
}
