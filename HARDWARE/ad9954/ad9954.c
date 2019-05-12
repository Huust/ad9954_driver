#include "ad9954.h"
#include "usart.h"
#include "delay.h"
#include "math.h"

#define RamSize (100)
//相关数据初始化

/*FM的CFR寄存器配置*/
//uint8_t CFR1_DATA[4] = {0x80,0x00,0x00,0x00};
//uint8_t CFR2_DATA[3] = {0x00,0x00,0xa4};

/*点频，幅度可控CFR与ASF寄存器配置*/
uint8_t CFR1_DATA[4] = {0x02,0x00,0x20,0x00};
uint8_t CFR2_DATA[3] = {0x00,0x00,0xa4};

/*点频下的CFR寄存器配置*/
//uint8_t CFR1_DATA[4] = {0x00,0x00,0x20,0x00};
//uint8_t CFR2_DATA[3] = {0x00,0x00,0xa4};

uint8_t RSCW0_DATA[5] = {0x35,0x0c,0x63,0x00,0x80};//调制信号频率应等于   sclk/（ram ramp rate*点数） 
                                                   //
uint8_t ByteStorage[4];//用于存放四字节的四个八位   0放置高八位，以此类推
uint32_t RAM_TABLE[RamSize];
//uint32_t fre_c = 100000000;//载波频率
//uint32_t fre_m = 100000000/(RamSize*3125);//基波频率
//uint32_t fre_m = 1000;
//uint32_t fre_delta = 30000;//最大频偏


/*------
function：对所有引脚拉高拉低的初始状态进行配置
-------*/
void GPIO_ad9954_initialize(void)
{
    //引脚初始化
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
function：io更新（搭配自定义delay使用）
-------*/
void ad9954_update(void)
{
    IO_UPDATE=0;
    
	IO_UPDATE = 1;
    delay_us(200);
    IO_UPDATE = 0;
}

/*------
function：复位（搭配自定义delay使用） 需注意9954需要先reset再使用
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
function:相关引脚初始化定义；复位；初始化配置一些寄存器
---------*/
void ad9954_init()
{
    //配置引脚初始化
    //复位
    //初始寄存器设置
    ad9954_WriteDataToRegister(CFR1_ADD,CFR1_DATA,4,1);
    ad9954_WriteDataToRegister(CFR2_ADD,CFR2_DATA,3,1);
}



/*----------
function:向寄存器写入数据
parameter:寄存器地址；需要写入的数据；被写入寄存器的字节数（控制字节写入）；更新flag
return:NULL
note：写入顺序：MSB-->LSB
----------*/
void ad9954_WriteDataToRegister(uint8_t RegisterAddress,uint8_t* InputData,uint8_t NumberOfRegister,uint8_t flag)
{
    u8	ControlValue = 0;
	u8	ValueToWrite = 0;
	u8	i = 0;

	ControlValue = RegisterAddress;
//写入地址
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
//写入数据
	for (uint8_t RegisterIndex=0; RegisterIndex<NumberOfRegister; RegisterIndex++)//控制字节写入
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
function:仅进行串口写入数据的操作，不需要识别寄存器地址
--------*/
void ad9954_WriteData(uint8_t* InputData,uint8_t NumberOfArrayElement,uint8_t flag)
{
	u8	ValueToWrite = 0;
	u8	i = 0;
    
    //写入数据
	for (uint8_t ArrayElementIndex=0; ArrayElementIndex<NumberOfArrayElement; ArrayElementIndex++)//控制字节写入
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
function：向ram写数据
return：null
parameter：null
issue：仍要改进：设置入口参数为起始地址，结束地址，数据量
目前支持1024全部写入
目前ramsize为100，则反推ramp rate为1000000/fre_m
---------*/
void ad9954_WriteDataToRAM(u32 fre_m,u32 fre_delta,u32 fre_c)
{
    //FM下的寄存器配置
    uint8_t CFR1_DATA_FM[4] = {0x80,0x00,0x00,0x00};
    uint8_t CFR2_DATA_FM[3] = {0x00,0x00,0xa4};
    uint8_t instruction_byte[1] = {0x0b};//校验字节
    //初始化数据
    
    //fre_m = 100000000/(RamSize*3125);//基波频率
    
    //fre_delta = 30000;//最大频偏
    //公式写入ram表
    for(uint16_t j = 0;j<RamSize;j++)
    {
        
        RAM_TABLE[j] = fre_c+fre_delta*sin(2*3.14*j/(RamSize-1));
    }
    
    //将实际需要的ramp rate写入寄存器 ramp rate占高16位
    uint16_t temp2;
    temp2 = 1000000/fre_m;
    RSCW0_DATA[0] = (u8)(temp2);
    RSCW0_DATA[1] = (u8)(temp2>>8);
    ad9954_WriteDataToRegister(CFR1_ADD,CFR1_DATA_FM,4,1);
    ad9954_WriteDataToRegister(CFR2_ADD,CFR2_DATA_FM,3,1);
    ad9954_WriteDataToRegister(RSCW0_ADD,RSCW0_DATA,5,1);
    PS0 = 0;
    PS1 = 0;
    //发送0x0b做为校验字节
    ad9954_WriteData(instruction_byte,1,0);
    
    //将数据发送给ram
    
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
