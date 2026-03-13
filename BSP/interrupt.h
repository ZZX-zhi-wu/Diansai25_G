#ifndef __INTERRUPT_H
#define __INTERRUPT_H

#include "main.h"
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
struct keys
{
	char key_sta;
	char key_judge;
	char single_flag;
};

#endif

