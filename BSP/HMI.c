#include "HMI.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "TypeDef.h"
#include "usart.h"
#include "myusart.h"


void HMI_send_string(char* name, char* showdata)
{
    // printf("t0.txt=\"%d\"\xff\xff\xff", num);
    UART_Printf("%s=\"%s\"\xff\xff\xff", name, showdata);
}
void HMI_send_number(char* name, int num)
{
    // printf("t0.txt=\"%d\"\xff\xff\xff", num);
    UART_Printf("%s=%d\xff\xff\xff", name, num);
}
void HMI_send_float(char* name, float num)
{
    // printf("t0.txt=\"%d\"\xff\xff\xff", num);
    UART_Printf("%s=%d\xff\xff\xff", name, (int)(num * 100));
}






