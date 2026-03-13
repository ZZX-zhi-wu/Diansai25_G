#ifndef __HMI_H
#define __HMI_H

#include "main.h"

void HMI_send_string(char* name, char* showdata);
void HMI_send_number(char* name, int num);
void HMI_send_float(char* name, float num);


#endif

