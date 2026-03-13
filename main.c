/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad9833.h"
#include "interrupt.h"
#include "math.h"
#include "myusart.h"
#include "string.h"
#include "HMI.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define FRQ_out 1000					
#define FRQ_2V 55			//（3）
#define VALUE_out  200
 

#define SAMPLE_NUM 5000  

void key_proc(void);
void uart_proc(void);

float calculateVoltageFromGain(float gain);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern struct keys key[2];
extern uint8_t rx,pointer;
extern char data_rx[30];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char flag_SP=0;
uint8_t view=5;
uint32_t frq_now=FRQ_out;
double frq_fakui_1=0;
uint16_t adc_buf[SAMPLE_NUM]; 
float data_value[1000];
uint8_t Value_now=0;
uint8_t num_1=0,num_2=0;
 float vpp=0;
 uint16_t data_poin=0;
uint16_t parseInt(const char *str);
uint8_t Data[31][12]={	{13,15,16,18,19,20,22,23,25,26,28},//100hz
						{14,15,16,18,19,21,22,24,25,27,28},//200hz
						{14,16,18,19,21,22,24,25,27,28,30},//300hz
						{15,17,19,20,22,24,25,27,29,31,32},//400hz
						{17,19,21,22,24,26,28,30,32,33,35},//500hz
						{18,20,22,24,26,28,30,32,34,36,38},//600hz
						{20,22,24,27,29,31,33,35,37,40,42},//700hz
						{22,24,27,29,31,34,36,39,41,44,46},//800hz
						{24,27,29,32,34,37,39,42,45,47,50},//900hz
						{26,29,32,35,37,40,43,46,49,52,55},//1000hz
						{28,31,34,38,40,44,47,50,53,56,59},//1100hz
						{31,34,38,41,44,47,51,54,57,61,64},//1200hz
						{34,37,41,45,48,52,55,58,62,66,69},//1300hz
						{36,40,43,47,51,55,59,63,67,71,74},//1400hz
						{39,43,47,51,55,59,63,67,72,76,80},//1500hz
						{41,46,50,55,59,63,68,72,77,81,85},//1600hz
						{44,49,54,58,63,68,72,77,82,87,91},//1700hz
						{47,52,57,62,67,72,77,82,87,92,97},//1800hz
						{50,56,61,66,71,77,82,87,93,98,103},//1900hz
						{53,59,65,70,75,81,87,92,99,104,110},//2000hz
						{57,63,69,75,81,87,93,98,105,111,116},//2100hz
						{59,67,73,79,85,91,98,104,111,117,123},//2200hz
						{63,70,77,84,90,97,103,110,117,124,130},//2300hz
						{66,74,81,88,94,102,109,116,124,131,137},//2400hz
						{70,78,86,93,100,107,115,122,131,138,145},//2500hz
						{74,82,90,98,105,113,121,129,137,145,153},//2600hz
						{78,86,94,102,110,119,127,136,144,153,160},//2700hz
						{81,90,99,108,116,124,133,143,152,160,168},//2800hz
						{86,95,104,113,121,130,140,149,159,168,176},//2900hz
						{91,100,109,118,126,136,147,157,165,174,184},//3000hz
						{0}	//NULL
};

//#define DATA_SIZE 600
//#define HYSTERESIS 0.05f // 迟滞阈值防止噪声误判
//#define MIN_CROSSING_DIST 10 // 最小有效穿越间隔

//// 滤波器类型定义（使用整数常量）
//#define FILTER_UNKNOWN 0
//#define FILTER_LOW_PASS 1
//#define FILTER_HIGH_PASS 2
//#define FILTER_BAND_PASS 3
//#define FILTER_BAND_STOP 4

//// 简化的移动平均（边界安全）
//void simple_smooth(float* data, int size) {
//    for (int i = 1; i < size - 1; i++) {
//        data[i] = (data[i-1] + data[i] + data[i+1]) / 3.0f;
//    }
//}

//// 检测有效穿越点
//int find_crossings(float* data, int size, float threshold, int* crossings) {
//    int count = 0;
//    uint8_t last_state = (data[0] > threshold) ? 1 : 0;
//    uint8_t stable_count = 0;
//    
//    for (int i = 1; i < size; i++) {
//        uint8_t current_state = (data[i] > threshold) ? 1 : 0;
//        
//        // 状态变化检测
//        if (current_state != last_state) {
//            stable_count++;
//            
//            // 确认稳定穿越
//            if (stable_count >= 3) {
//                // 检查与前一个穿越点的距离
//                if (count == 0 || (i - crossings[count-1]) > MIN_CROSSING_DIST) {
//                    crossings[count] = i;
//                    count++;
//                    if (count >= 4) break; // 最多检测4个点
//                }
//                last_state = current_state;
//                stable_count = 0;
//            }
//        } else {
//            stable_count = 0;
//        }
//    }
//    return count;
//}

//// 返回值改为 int 类型
// float min_val = 3.3f, max_val = 0.0f;
//int identify_filter(float* data) {
//    // 1. 简化平滑处理
//    simple_smooth(data, DATA_SIZE);
//    
//    // 2. 计算特征阈值（最大最小值的中间值）
//   
//    for (int i = 0; i < DATA_SIZE; i++) {
//        if (data[i] < min_val) min_val = data[i];
//        if (data[i] > max_val) max_val = data[i];
//    }
//    float threshold = (min_val + max_val) * 0.5f;
//    
//    // 3. 检测穿越点
//    int crossings[4] = {0}; // 存储穿越点位置
//    int cross_count = find_crossings(data, DATA_SIZE, threshold, crossings);
//    
//    // 4. 根据穿越点判断类型
//    if (cross_count == 0) {
//        // 无穿越：全通或测量错误
//        return FILTER_UNKNOWN;
//    }
//    else if (cross_count == 1) {
//        // 单穿越：低通或高通
//        // 检查穿越方向（从高到低为低通，从低到高为高通）
//        int idx = crossings[0];
//        if (idx > 10 && idx < DATA_SIZE - 10) {
//            float left_avg = (data[idx-5] + data[idx-4] + data[idx-3]) / 3.0f;
//            float right_avg = (data[idx+3] + data[idx+4] + data[idx+5]) / 3.0f;
//            
//            if (left_avg > threshold && right_avg < threshold) {
//                return FILTER_LOW_PASS; // 从高到低
//            }
//            else if (left_avg < threshold && right_avg > threshold) {
//                return FILTER_HIGH_PASS; // 从低到高
//            }
//        }
//    }
//    else if (cross_count >= 2) {
//        // 多穿越：带通或带阻
//        int first_dir = (data[crossings[0]-1] < threshold) ? 1 : -1; // 1: 上升, -1:下降
//        int second_dir = (data[crossings[1]-1] < threshold) ? 1 : -1;
//        
//        // 带通：先上升后下降
//        if (first_dir == 1 && second_dir == -1) {
//            return FILTER_BAND_PASS;
//        }
//        // 带阻：先下降后上升
//        else if (first_dir == -1 && second_dir == 1) {
//            return FILTER_BAND_STOP;
//        }
//    }
//    
//    return FILTER_UNKNOWN;
//}
void smooth_array(float* arr, uint32_t len, uint32_t window_size)
{
    // 参数校验
    if (arr == NULL || len == 0 || window_size == 0) 
        return;
    
    // 窗口大小不能超过数组长度
    if (window_size > len) 
        window_size = len;
    
    // 分配临时存储空间
    float* temp = (float*)malloc(len * sizeof(float));
    if (temp == NULL) 
        return;  // 内存分配失败
    
    uint32_t half_win = window_size / 2;
    
    for (uint32_t i = 0; i < len; i++) 
    {
        // 计算当前点的有效窗口边界
        uint32_t start = (i >= half_win) ? (i - half_win) : 0;
        uint32_t end = (i + half_win < len) ? (i + half_win) : (len - 1);
        
        // 计算动态窗口内的平均值
        float sum = 0.0f;
        uint32_t count = 0;
        
        for (uint32_t j = start; j <= end; j++) 
        {
            sum += arr[j];
            count++;
        }
        
        temp[i] = sum / (float)count;
    }
    
    // 将结果复制回原数组
    memcpy(arr, temp, len * sizeof(float));
    
    // 释放临时空间
    free(temp);
}

uint8_t classify_by_passband(float* db, int len) {
    if (len < 5) return 0;  // 太短无法判断

    float max_val = -2;
    float min_val = 5;

    for (int i = 0; i < len; i++) {
        if (db[i] > max_val) max_val = db[i];
        if (db[i] < min_val) min_val = db[i];
    }

    float cut_db_v = max_val * 0.707f;
    float notch_check = min_val * 1.414f;

    int rise_index = -1;
    int fall_index = -1;
    int notch_count = 0;

    // 寻找穿越点和 notch 点数
    for (int i = 1; i < len - 1; i++) {
        // 上升穿越 -3dB
        if (db[i - 1] < cut_db_v && db[i] >= cut_db_v && rise_index == -1) {
            rise_index = i;
        }
        // 下降穿越 -3dB
        if (db[i - 1] > cut_db_v && db[i] <= cut_db_v) {
            fall_index = i;
        }

        // 统计凹陷区域（避免把带阻误识为高/低通）
        if (db[i] < notch_check) {
            notch_count++;
        }
    }
   

    // 判断类型
    if (rise_index == -1 && fall_index != -1) {
        return 2; // 低通（只有下降）
    } else if (rise_index != -1 && fall_index == -1) {
        return 1; // 高通（只有上升）
    } else if (rise_index != -1 && fall_index != -1) {
        if (rise_index < fall_index) {
            return 3; // 带通（先升后降）
        } else {
            return 4; // 带阻（先降后升）
        }
    }

    return 5; // 平坦或无法判断
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {  
		frq_fakui_1+=200;
		
		AD9833_FreqSet((uint32_t)frq_fakui_1);  
		AD9833_FreqSet((uint32_t)frq_fakui_1); 

        uint16_t max_val=0 , min_val=4096 ;
        for (uint32_t i = 0; i < SAMPLE_NUM; i++) {
            if (adc_buf[i] > max_val) max_val = adc_buf[i];
            if (adc_buf[i] < min_val) min_val = adc_buf[i];
        }
		vpp = (max_val - min_val) * 3.3f / 4095.0f;
		data_value[data_poin++]=vpp;			

		if(frq_fakui_1>=60000)
		{
			flag_SP=0;
//			 smooth_array(data_value, data_poin, 5);
			uint8_t type=classify_by_passband(data_value,data_poin);
			data_poin=0;
			for(int i=0;i<1000;i++)
			{
				data_value[i]=0;
			}
			char dat_text[50];
			if(type==1)
			sprintf(dat_text,"GAOTONG");
			else if(type==2)
			sprintf(dat_text,"DITONG");
			else if(type==3)
			sprintf(dat_text,"DAITONG");
			else if(type==4)
			sprintf(dat_text,"DAIZHU");
			else if(type==5)
			sprintf(dat_text,"ERROR");
			HMI_send_string("t20.txt",dat_text);
			HMI_send_string("t20.txt",dat_text);
		}  
		
    }
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float value;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  AD9833_GPIOinit();
  AD9833_CtrlSet(0,0,0,0);//初始化正弦波
  
	AD9833_FreqSet(FRQ_out);
	AD9833_FreqSet(FRQ_out);	//初始输出频率

  AD9833_AmpSet(VALUE_out);
  AD9833_AmpSet(VALUE_out);	//初始输出电压

  HAL_TIM_Base_Start_IT(&htim6);		//按键控制
  
  HAL_TIM_Base_Start_IT(&htim2);		//adc采样间隔100hz
  
  HAL_UART_Receive_IT(&huart3,&rx,1);		//串口屏接收
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  key_proc();
	  if(pointer>0)
	  {
		  uint8_t temp=pointer;
		  HAL_Delay(1);
		  if(temp==pointer)
		  {
			  uart_proc();
		  }
}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//uint16_t data=Start_value;
uint8_t num_11=0;

void key_proc(void)
{
    if(key[0].single_flag==1)
    {
        key[0].single_flag=0;
    }
    if(key[1].single_flag==1)
    {
        key[1].single_flag=0;
    }
}

uint16_t parseInt(const char *str) //处理键盘数据
{
    uint16_t num = 0;
	if(str[pointer-1]=='|')pointer--;
    for (int i = 1;i<pointer; i++) {
		if(str[i]=='.') continue;
        num = num * 10 + (str[i] - '0'); // 逐位转换
    }
    return num;
}

void uart_proc(void)
{
	if(data_rx[0]>='0'&&data_rx[0]<='4')
	{
		view=data_rx[0]-'0';
		char dd[50];
		sprintf(dd,"OK");
		if(view==0)								//基本二---->不同frq3V+
		{
			AD9833_AmpSet(250);
			AD9833_AmpSet(250);
			
			HMI_send_string("t6.txt",dd);
			HMI_send_string("t6.txt",dd);
			
			frq_now=1000;
			sprintf(dd,"%dHz",frq_now);
			HMI_send_string("t1.txt",dd);
			
			AD9833_FreqSet(frq_now);  
			AD9833_FreqSet(frq_now); 
		}
		else if(view==1)						//基本三---->1khz2v
		{
			HMI_send_string("t7.txt",dd);
			HMI_send_string("t7.txt",dd);
		}
		else if(view==2)						//基本四//1khz,1v	
		{
			HMI_send_string("t8.txt",dd);
			HMI_send_string("t8.txt",dd);		//OK
			
			frq_now=1000;
			sprintf(dd,"%dHz",frq_now);
			HMI_send_string("t4.txt",dd);
			AD9833_FreqSet(frq_now);  
			AD9833_FreqSet(frq_now); 
			
			sprintf(dd,"1V");
			Value_now=10;
			num_2=0;
			HMI_send_string("t5.txt",dd);
			
			AD9833_AmpSet(Data[9][num_2]);
			AD9833_AmpSet(Data[9][num_2]);
		}
		else if(view==3)					//发挥一
		{
			sprintf(dd,"OK%d",num_11++);
			HMI_send_string("t9.txt",dd);
			HMI_send_string("t9.txt",dd);
		}
		else if(view==4)					//发挥二
		{
			HMI_send_string("t10.txt",dd);
			HMI_send_string("t10.txt",dd);
			frq_now=20000;
			HAL_Delay(2000);
			
			AD9833_FreqSet(frq_now);  
			AD9833_FreqSet(frq_now); 
			
			AD9833_AmpSet(80);
			AD9833_AmpSet(80); 
		}
	}
	else if(view==0)								//基础二通信
	{
	char dat_text[50];
	if(strcmp(data_rx,"(2)set1khz")==0)
	{
		frq_now=1000;
		sprintf(dat_text,"%dHz",frq_now);
		HMI_send_string("t1.txt",dat_text);
		AD9833_FreqSet(frq_now);  
		AD9833_FreqSet(frq_now); 	
	}
	else if(strcmp(data_rx,"(2)set10khz")==0)
	{
		frq_now=10000;
		sprintf(dat_text,"%dHz",frq_now);
		HMI_send_string("t1.txt",dat_text);
		AD9833_FreqSet(frq_now);  
		AD9833_FreqSet(frq_now);   
	}
	else if(strcmp(data_rx,"(2)set100khz")==0)
	{
		frq_now=100000;
		sprintf(dat_text,"%dHz",frq_now);
		HMI_send_string("t1.txt",dat_text);
		AD9833_FreqSet(frq_now);  
		AD9833_FreqSet(frq_now);   
	}
	else if(strcmp(data_rx,"(2)set1mhz")==0)
	{
		frq_now=1000000;
		sprintf(dat_text,"%dHz",frq_now);
		HMI_send_string("t1.txt",dat_text);
		AD9833_FreqSet(frq_now);  
		AD9833_FreqSet(frq_now);   
	}
	else if(strcmp(data_rx,"(2)add100hz")==0)
	{
		frq_now+=100;
		sprintf(dat_text,"%dHz",frq_now);
		HMI_send_string("t1.txt",dat_text);
		AD9833_FreqSet(frq_now);  
		AD9833_FreqSet(frq_now);  
	}
	else if(strcmp(data_rx,"(2)reduce100hz")==0)
	{
		frq_now-=100;
		sprintf(dat_text,"%dHz",frq_now);
		HMI_send_string("t1.txt",dat_text);
		AD9833_FreqSet(frq_now);  
		AD9833_FreqSet(frq_now);  
	}
	}
	else if(view==1)				//基础三通信
	{
	char dat_text[50];
	if(strcmp(data_rx,"(3)start")==0)
	{
		frq_now=1000;
		sprintf(dat_text,"OK");
		HMI_send_string("t3.txt",dat_text);
		
		AD9833_FreqSet(frq_now);  
		AD9833_FreqSet(frq_now); 
		
		AD9833_AmpSet(FRQ_2V);
		AD9833_AmpSet(FRQ_2V);		
	}
	}
	else if(view==2)					//基础四通信
	{
	char dat_text[50];
	if(strcmp(data_rx,"(4)set100hz")==0)
	{
		frq_now=100;
		sprintf(dat_text,"%dHz",frq_now);
		HMI_send_string("t4.txt",dat_text);
		
		AD9833_FreqSet(frq_now);  
		AD9833_FreqSet(frq_now);
		
		AD9833_AmpSet(Data[0][num_2]);
		AD9833_AmpSet(Data[0][num_2]);			
	}
	else if(strcmp(data_rx,"(4)set2khz")==0)
	{
		frq_now=2000;
		sprintf(dat_text,"%dHz",frq_now);
		HMI_send_string("t4.txt",dat_text);
		
		AD9833_FreqSet(frq_now);  
		AD9833_FreqSet(frq_now); 	
		
		HAL_Delay(10);
		AD9833_AmpSet(Data[19][num_2]);
		AD9833_AmpSet(Data[19][num_2]);	
	}
	else if(strcmp(data_rx,"(4)add100hz")==0)
	{
		frq_now+=100;
		if(frq_now>3000) frq_now=3000;
		sprintf(dat_text,"%dHz",frq_now);
		HMI_send_string("t4.txt",dat_text);
		AD9833_FreqSet(frq_now);  
		AD9833_FreqSet(frq_now); 
		
		num_1=frq_now/100-1;
		
		AD9833_AmpSet(Data[num_1][num_2]);
		AD9833_AmpSet(Data[num_1][num_2]);		
	}
	else if(strcmp(data_rx,"(4)reduce100hz")==0)
	{
		frq_now-=100;
		if(frq_now<100)frq_now=100;
		sprintf(dat_text,"%dHz",frq_now);
		HMI_send_string("t4.txt",dat_text);
		
		AD9833_FreqSet(frq_now);  
		AD9833_FreqSet(frq_now); 
		
		num_1=frq_now/100-1;
		AD9833_AmpSet(Data[num_1][num_2]);
		AD9833_AmpSet(Data[num_1][num_2]);		
	}
	else if(strcmp(data_rx,"(4)set2v")==0)
	{
		num_1=frq_now/100-1;
		num_2=10;
		sprintf(dat_text,"2V");
		Value_now=20;
		HMI_send_string("t5.txt",dat_text);
		
		AD9833_AmpSet(Data[num_1][num_2]);
		AD9833_AmpSet(Data[num_1][num_2]);		
	}
	else if(strcmp(data_rx,"(4)set1v")==0)
	{
		num_1=frq_now/100-1;
		num_2=0;
		sprintf(dat_text,"1V");
		Value_now=10;
		HMI_send_string("t5.txt",dat_text);
		
		AD9833_AmpSet(Data[num_1][num_2]);
		AD9833_AmpSet(Data[num_1][num_2]);			
	}
	else if(strcmp(data_rx,"(4)add0.1v")==0)
	{
		Value_now+=1;
		if(Value_now>20)Value_now=20;
		num_1=frq_now/100-1; 
		num_2=Value_now-10;
		sprintf(dat_text,"%.2fV",Value_now*1.0/10);
		HMI_send_string("t5.txt",dat_text);
		AD9833_AmpSet(Data[num_1][num_2]);
		AD9833_AmpSet(Data[num_1][num_2]);			
	}
	else if(strcmp(data_rx,"(4)reduce0.1v")==0)
	{
		Value_now-=1;
		if(Value_now<10)Value_now=10;
		num_1=frq_now/100-1;
		num_2=Value_now-10;
		sprintf(dat_text,"%.2fV",Value_now*1.0/10);
		HMI_send_string("t5.txt",dat_text);
		AD9833_AmpSet(Data[num_1][num_2]);
		AD9833_AmpSet(Data[num_1][num_2]);			
	}
	else					//键盘设置频率以及幅度值
	{
		uint16_t  set_num=0;
		set_num=parseInt(data_rx);
		if(set_num<50)			//解析幅度值
		{
			num_1=frq_now/100-1;
			num_2=set_num-10;
			Value_now=num_2;
			
			AD9833_AmpSet(Data[num_1][num_2]);
			AD9833_AmpSet(Data[num_1][num_2]);	
		
		}
		else					//解析频率值
		{
			uint16_t min,max;
			min=set_num/100*100;
			max=min+100;
			if((max-set_num)>=(set_num-min))
			{
				frq_now=min;
			}
			else
			{
				frq_now=max;
			}

			AD9833_FreqSet(frq_now);  
			AD9833_FreqSet(frq_now); 	
			
			num_1=frq_now/100-1;
			AD9833_AmpSet(Data[num_1][num_2]);
			AD9833_AmpSet(Data[num_1][num_2]);	
		
		}
	}
	}
	else if(view==3)
	{
		if(strcmp(data_rx,"high(1)start")==0)
		{
		char dat_text[30];
		sprintf(dat_text,"Wait...");
		HMI_send_string("t20.txt",dat_text);
		
		AD9833_AmpSet(200);
		AD9833_AmpSet(200);

		frq_fakui_1=500;
		AD9833_FreqSet(frq_fakui_1);  
		AD9833_FreqSet(frq_fakui_1); 	

		flag_SP=1;
		
		}
	}
	else if(view==4)
	{
		
			
//			for(int i=0;i<255;i++)
//			{
//				AD9833_AmpSet(i);
//				AD9833_AmpSet(i);
//				
//			}
		
	}
	memset(data_rx,0,30);
	pointer=0;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
