#ifndef COMPONENTS_TIME_INCLUDE_TIME_H_
#define COMPONENTS_TIME_INCLUDE_TIME_H_
 
#include "driver/timer.h"
#include <stdio.h>
 
void Time_Init();//初始化定时器函数
void IRAM_ATTR timer_group0_isr(void *para); //定时器中断函数
 
#endif /* COMPONENTS_TIME_INCLUDE_TIME_H_ */