#ifndef _ad9833_H_
#define _ad9833_H_

#include "main.h"

#define CS_9833_0() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define CS_9833_1() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)

void AD9833_GPIOinit(void);
void AD9833_Write(unsigned short TxData);
void AD9833_CtrlSet(unsigned char Reset,unsigned char SleeppMode,unsigned char optionbit,unsigned char modebit);
void AD9833_FreqSet(double Freq);
void AD9833_AmpSet(uint8_t amp);

#endif
