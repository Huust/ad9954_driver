
#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "arm_math.h"
#include "ad9954.h"


//���̼�¼��
//ͷ�ļ�  �Ĵ��������ţ���غ�������
//C�ļ���дǰ��׼����������λ��io���£����������������ͳ�ʼ��ѡ����
//

//ע�����9954��Ҫ��reset��ʹ��
//дram������Ҫ�޸�
//����Ӱ��ramд��ʱ��Ĳ��֣�����0x0b���ٴ���Ҫ���͵����ݣ����ҷ��͸�ram
//Ϊʲô��������ʼ����ֹ��ַ��ʵ����
//uint16_t ADC_Value;
//uint16_t ValueToASF;
//float degree = 0.9;//���ƶ�


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

		




