#include "buzzer.h"

void buzzerTurnOn(void)
{
    HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_SET);   
}

void buzzerTurnOff(void)
{
    HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_RESET);
}

void buzzerTurnOnDelay(int t)
{
    buzzerTurnOn();
    delay_ms(t);
    buzzerTurnOff();
}

void buzzerTurnAndClose(void)
{
    HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_RESET);
}

void buzzerToggle(void)
{
    HAL_GPIO_TogglePin(buzzer_GPIO_Port,buzzer_Pin);
}