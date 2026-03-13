#include "myusart.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "usart.h"


void UART_Printf(const char *format, ...)
{
	char tmp[128];
	
	va_list argptr;
	va_start(argptr, format);
	vsprintf((char* )tmp, format, argptr);
	va_end(argptr);
	
	HAL_UART_Transmit(&huart3, (const uint8_t *)&tmp, strlen(tmp), 50);
}


