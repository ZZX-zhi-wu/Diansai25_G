#include "interrupt.h"
#include "usart.h"
#include "ad9833.h"
#include "adc.h"
#include "myusart.h"
struct keys key[2]={0,0,0};
extern char flag_SP;
extern double frq_fakui_1;
extern uint16_t adc_buf[]; 

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	if(htim->Instance==TIM6)//10ms
	{
		key[0].key_sta=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4);
		key[1].key_sta=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3);
		
		for(int i=0;i<2;i++)
	   {
		switch(key[i].key_judge)
		{
			case 0:
			{
				if(key[i].key_sta==0)
				{
					key[i].key_judge=1;
				}
			}
			break;
			case 1:
			{
				if(key[i].key_sta==0)
				{
					key[i].key_judge=2;
				}
				else key[i].key_judge=0;
			}
			break;
			case 2:
			{
				if(key[i].key_sta==1)
				{
					key[i].key_judge=0;
					key[i].single_flag=1;
				}
			}
			break;
		    }
		}
	}
	else if(htim->Instance==TIM2)
	{
		if(flag_SP==1)
		{
			
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 5000);
			
		}
	}

}

uint8_t rx=0,pointer=0;
char data_rx[30]={0};
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	data_rx[pointer++]=rx;
	HAL_UART_Receive_IT(&huart3,&rx,1);
}


