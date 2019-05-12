
#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "arm_math.h"
#include "ad9954.h"


//过程记录：
//头文件  寄存器，引脚，相关函数申明
//C文件编写前期准备函数：复位，io更新，所有引脚拉高拉低初始化选择函数
//

//注意事项：9954需要先reset再使用
//写ram操作需要修改
//可能影响ram写入时序的部分：发送0x0b后再处理要发送的数据，并且发送给ram
//为什么更改了起始和终止地址就实现了
//uint16_t ADC_Value;
//uint16_t ValueToASF;
//float degree = 0.9;//调制度


int main()
{
   delay_init(168);			  
   uart_init(115200);	
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
   
   GPIO_ad9954_initialize();
   ad9954_reset();
   delay_ms(10);
   ad9954_init();
   MYGPIO_Init(); 
   //MYADC_Init();
   //Write_amplitude();
   //Write_frequence(80000000);
   
   //ad9954_WriteDataToRAM();
   ad9954_WriteDataToRAM(3000,50000,80000000);
   while(1)
   {
    
   }
}

		




