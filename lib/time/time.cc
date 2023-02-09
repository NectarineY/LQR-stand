#include <Arduino.h>
#include "driver/timer.h"
#include "time.h"


void app_main(void)
{
//	/**
//	 * 初始化GPIO
//	 */
	
//	inter_init();
	Time_Init();
    while(1) 
    {
        
        vTaskDelay(1000);
    }
}