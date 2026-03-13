#include "delay.h"
#include "stm32f4xx.h"                  // Device header
#include "TypeDef.h"
#include "stm32f4xx_hal.h"                  // Device header





/**
  *@功能： 获取2个时间点之间是否达到期望的延时
  *@参数1：现在的实时时间
  *@参数2：开始计时的时间
  *@参数3：要延时的时间间隔
  *@返回值：0-延时时间未到，1-延时时间已到
  */
  
uint8_t Get_Time_Interval(uint32_t Current_Time, uint32_t Past_Time, uint32_t Delay_Time)
{
    if(Current_Time < Past_Time)//实时时间小于开始时间，时间计数发生回滚
    {
        if(0xFFFFFFFF - Past_Time + Current_Time >= Delay_Time)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else        //实时时间大于开始时间
    {
        if(Current_Time - Past_Time >= Delay_Time)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}









