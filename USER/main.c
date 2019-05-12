
#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "arm_math.h"
#include "ad9954.h"


int main()
{
   delay_init(168);			  
   uart_init(115200);	
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
   
   GPIO_ad9954_initialize();
   ad9954_reset();
   delay_ms(10);
   ad9954_init();
   //Write_amplitude();
   //Write_frequence(80000000);
   //9954_WriteDataToRAM(3000,50000,80000000);
   while(1)
   {
    
   }
}

		




