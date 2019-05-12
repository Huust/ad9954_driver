#include "ad9954.h"
#include "usart.h"
#include "delay.h"
#include "math.h"

#define RamSize (100)
//������ݳ�ʼ��

/*FM��CFR�Ĵ�������*/
//uint8_t CFR1_DATA[4] = {0x80,0x00,0x00,0x00};
//uint8_t CFR2_DATA[3] = {0x00,0x00,0xa4};

/*��Ƶ�����ȿɿ�CFR��ASF�Ĵ�������*/
uint8_t CFR1_DATA[4] = {0x02,0x00,0x20,0x00};
uint8_t CFR2_DATA[3] = {0x00,0x00,0xa4};

/*��Ƶ�µ�CFR�Ĵ�������*/
//uint8_t CFR1_DATA[4] = {0x00,0x00,0x20,0x00};
//uint8_t CFR2_DATA[3] = {0x00,0x00,0xa4};

uint8_t RSCW0_DATA[5] = {0x35,0x0c,0x63,0x00,0x80};//�����ź�Ƶ��Ӧ����   sclk/��ram ramp rate*������ 
                                                   //
uint8_t ByteStorage[4];//���ڴ�����ֽڵ��ĸ���λ   0���ø߰�λ���Դ�����
uint32_t RAM_TABLE[RamSize];
//uint32_t fre_c = 100000000;//�ز�Ƶ��
//uint32_t fre_m = 100000000/(RamSize*3125);//����Ƶ��
//uint32_t fre_m = 1000;
//uint32_t fre_delta = 30000;//���Ƶƫ


/*------
function�������������������͵ĳ�ʼ״̬��������
-------*/
void GPIO_ad9954_initialize(void)
{
    //���ų�ʼ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_6;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD,&GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOG,&GPIO_InitStructure);
    
    //GPIO_ResetBits(GPIOD, GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_6);
    //GPIO_ResetBits(GPIOG, GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13);
    
    PS0 = 0;
    PS1 = 0;
    OSK = 0;
}

/*------
function��io���£������Զ���delayʹ�ã�
-------*/
void ad9954_update(void)
{
    IO_UPDATE=0;
    
	IO_UPDATE = 1;
    delay_us(200);
    IO_UPDATE = 0;
}

/*------
function����λ�������Զ���delayʹ�ã� ��ע��9954��Ҫ��reset��ʹ��
-------*/
void ad9954_reset(void)
{
    SCLK = 0;
	PS0 = 0;
	PS1 = 0;
	IO_UPDATE = 0;
	RESET = 0;
	RESET = 1;
	delay_us(2000);
    RESET = 0;
}
/*--------
function:������ų�ʼ�����壻��λ����ʼ������һЩ�Ĵ���
---------*/
void ad9954_init()
{
    //�������ų�ʼ��
    //��λ
    //��ʼ�Ĵ�������
    ad9954_WriteDataToRegister(CFR1_ADD,CFR1_DATA,4,1);
    ad9954_WriteDataToRegister(CFR2_ADD,CFR2_DATA,3,1);
}



/*----------
function:��Ĵ���д������
parameter:�Ĵ�����ַ����Ҫд������ݣ���д��Ĵ������ֽ����������ֽ�д�룩������flag
return:NULL
note��д��˳��MSB-->LSB
----------*/
void ad9954_WriteDataToRegister(uint8_t RegisterAddress,uint8_t* InputData,uint8_t NumberOfRegister,uint8_t flag)
{
    u8	ControlValue = 0;
	u8	ValueToWrite = 0;
	u8	i = 0;

	ControlValue = RegisterAddress;
//д���ַ
		for (i=0; i<8; i++)
		{
			SCLK = 0;
			if(0x80 == (ControlValue & 0x80))
			SDIO1= 1;	  
			else
			SDIO1= 0;
            __nop();
            __nop();
            __nop();
			SCLK = 1;
            __nop();
            __nop();
            __nop();
			ControlValue <<= 1;
		}
         __nop();
         __nop();
         __nop();
         delay_us(10);
         SCLK = 0;	
//д������
	for (uint8_t RegisterIndex=0; RegisterIndex<NumberOfRegister; RegisterIndex++)//�����ֽ�д��
	{
		
        ValueToWrite = InputData[RegisterIndex];
		for (i=0; i<8; i++)
		{
			SCLK = 0;
			if(0x80 == (ValueToWrite & 0x80))
			SDIO1= 1;	  
			else
			SDIO1= 0;
            __nop();
            __nop();
            __nop();
			SCLK = 1;
            __nop();
            __nop();
            __nop();
			ValueToWrite <<= 1;
		}
         __nop();
         __nop();
         __nop();
         delay_us(10);
         SCLK = 0;		
	}	
	if(1==flag)
    ad9954_update();
    
}
void ad9954_WriteByte(uint8_t ValueToWrite)
{
    for (uint8_t i=0; i<8; i++)
		{
			SCLK = 0;
			if(0x80 == (ValueToWrite & 0x80))
			SDIO1= 1;	  
			else
			SDIO1= 0;
            __nop();
            __nop();
            __nop();
			SCLK = 1;
            __nop();
            __nop();
            __nop();
			ValueToWrite <<= 1;
		}
         __nop();
         __nop();
         __nop();
         delay_us(10);
         SCLK = 0;		
        

}
/*-------
function:�����д���д�����ݵĲ���������Ҫʶ��Ĵ�����ַ
--------*/
void ad9954_WriteData(uint8_t* InputData,uint8_t NumberOfArrayElement,uint8_t flag)
{
	u8	ValueToWrite = 0;
	u8	i = 0;
    
    //д������
	for (uint8_t ArrayElementIndex=0; ArrayElementIndex<NumberOfArrayElement; ArrayElementIndex++)//�����ֽ�д��
	{
		
        ValueToWrite = InputData[ArrayElementIndex];
		for (i=0; i<8; i++)
		{
			SCLK = 0;
			if(0x80 == (ValueToWrite & 0x80))
			SDIO1= 1;	  
			else
			SDIO1= 0;
            __nop();
            __nop();
            __nop();
			SCLK = 1;
            __nop();
            __nop();
            __nop();
			ValueToWrite <<= 1;
		}
         __nop();
         __nop();
         __nop();
         delay_us(10);
         SCLK = 0;		
	}	
	if(1==flag)
    ad9954_update();
    
}



/*--------
function����ramд����
return��null
parameter��null
issue����Ҫ�Ľ���������ڲ���Ϊ��ʼ��ַ��������ַ��������
Ŀǰ֧��1024ȫ��д��
ĿǰramsizeΪ100������ramp rateΪ1000000/fre_m
---------*/
void ad9954_WriteDataToRAM(u32 fre_m,u32 fre_delta,u32 fre_c)
{
    //FM�µļĴ�������
    uint8_t CFR1_DATA_FM[4] = {0x80,0x00,0x00,0x00};
    uint8_t CFR2_DATA_FM[3] = {0x00,0x00,0xa4};
    uint8_t instruction_byte[1] = {0x0b};//У���ֽ�
    //��ʼ������
    
    //fre_m = 100000000/(RamSize*3125);//����Ƶ��
    
    //fre_delta = 30000;//���Ƶƫ
    //��ʽд��ram��
    for(uint16_t j = 0;j<RamSize;j++)
    {
        
        RAM_TABLE[j] = fre_c+fre_delta*sin(2*3.14*j/(RamSize-1));
    }
    
    //��ʵ����Ҫ��ramp rateд��Ĵ��� ramp rateռ��16λ
    uint16_t temp2;
    temp2 = 1000000/fre_m;
    RSCW0_DATA[0] = (u8)(temp2);
    RSCW0_DATA[1] = (u8)(temp2>>8);
    ad9954_WriteDataToRegister(CFR1_ADD,CFR1_DATA_FM,4,1);
    ad9954_WriteDataToRegister(CFR2_ADD,CFR2_DATA_FM,3,1);
    ad9954_WriteDataToRegister(RSCW0_ADD,RSCW0_DATA,5,1);
    PS0 = 0;
    PS1 = 0;
    //����0x0b��ΪУ���ֽ�
    ad9954_WriteData(instruction_byte,1,0);
    
    //�����ݷ��͸�ram
    
    for(uint16_t RamIndex=0; RamIndex<RamSize; RamIndex++)
    {
       uint32_t temp;
       temp = RAM_TABLE[RamIndex]*10.73741824;
       ad9954_WriteByte((u8)(temp>>24));
       ad9954_WriteByte((u8)(temp>>16));
       ad9954_WriteByte((u8)(temp>>8));
       ad9954_WriteByte((u8)temp);
    }
    ad9954_update();
    
}
void Write_frequence(u32 Freq)
{	 
      u8 CFTW0_DATA[4] ={0x00,0x00,0x00,0x00};
	  u32 Temp;            
	  Temp=(u32)Freq*10.73741824;	 
	  CFTW0_DATA[3]=(u8)Temp;
	  CFTW0_DATA[2]=(u8)(Temp>>8);
	  CFTW0_DATA[1]=(u8)(Temp>>16);
	  CFTW0_DATA[0]=(u8)(Temp>>24);
      ad9954_WriteDataToRegister(FTW0_ADD,CFTW0_DATA,4,1);
      
}
void Write_amplitude(u16 ScaleFactor)
{
     u8 ASF_DATA[2] = {0x00,0x00};
     ASF_DATA[1] = (u8)ScaleFactor;
     ASF_DATA[0] = (u8)(ScaleFactor>>8);
    
    ad9954_WriteDataToRegister(ASF_ADD,ASF_DATA,2,1);
}
