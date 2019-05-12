#include "ad9954.h"
#include "usart.h"
#include "delay.h"
#include "math.h"

/*使用ad9954实现基本的点频，调幅及FM*/

#define RamSize (100)//ram的采样点数为100

/*FM的CFR寄存器配置*/
//uint8_t CFR1_DATA[4] = {0x80,0x00,0x00,0x00};
//uint8_t CFR2_DATA[3] = {0x00,0x00,0xa4};

/*点频下，幅度可控CFR与ASF寄存器配置*/
uint8_t CFR1_DATA[4] = {0x02,0x00,0x20,0x00};
uint8_t CFR2_DATA[3] = {0x00,0x00,0xa4};

/*ram相关配置*/
uint32_t RAM_TABLE[RamSize];
uint8_t RSCW0_DATA[5] = {0x35,0x0c,0x63,0x00,0x80};// 调制信号频率应等于   sclk/（ram ramp rate*点数） 
                                                   
uint8_t ByteStorage[4];//用于存放四字节的四个八位   0放置高八位，以此类推


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
    
	//下面注释的两步是确保初始化引脚保证为低
    	//GPIO_ResetBits(GPIOD, GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_6);
    	//GPIO_ResetBits(GPIOG, GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13);
	
	/*※这里需要说明一点，不同板子引脚状态有所不同，所以引脚配置为低；另外引脚初始化时首先将sclk引脚进行设置*/
    
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
    	SCLK = 0;/*有限拉低sclk*/
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
    ad9954_WriteDataToRegister(CFR1_ADD,CFR1_DATA,4,1);
    ad9954_WriteDataToRegister(CFR2_ADD,CFR2_DATA,3,1);
}



/*----------
function:向寄存器写入数据
parameter:寄存器地址；需要写入的数据；被写入寄存器的字节数（控制字节写入）；更新flag
return:NULL
note1：写入顺序：MSB-->LSB
note2: MSB写入时，从寄存器的最高位开始写入
----------*/
void ad9954_WriteDataToRegister(uint8_t RegisterAddress,uint8_t* InputData,uint8_t NumberOfRegister,uint8_t flag)
{
    	u8	ControlValue = 0;
	u8	ValueToWrite = 0;
	u8	i = 0;

	ControlValue = RegisterAddress;

//写入寄存器地址		
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


/*-------
function:用于直接发送8bit；与后一个函数不同的是，后一个函数对于多个字节需要将每个字节存入数组
--------*/
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
parameter：fre_m:基波频率;fre_delta:最大频偏;fre_c:载波频率
目前ramsize为100，则反推ramp rate为1000000/fre_m
---------*/
void ad9954_WriteDataToRAM(u32 fre_m,u32 fre_delta,u32 fre_c)
{
    //FM下的寄存器配置
    uint8_t CFR1_DATA_FM[4] = {0x80,0x00,0x00,0x00};
    uint8_t CFR2_DATA_FM[3] = {0x00,0x00,0xa4};
    uint8_t instruction_byte[1] = {0x0b};//校验字节

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

/*-------
function:单频模式下写入频率
parameter：填入的频率（将填入的频率划分成四个8bit，按照data sheet发给FTW）
note：注意如果先使用FM再使用单频要记得修改寄存器；程序初始化用的单频配置
--------*/
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

/*-------
function:单频模式下写入幅度
parameter：填入的幅度控制字（不是实际电压值）
note：根据DDS出频率的方法需要知道频率较高时，幅度会有衰减
--------*/
void Write_amplitude(u16 ScaleFactor)
{
     u8 ASF_DATA[2] = {0x00,0x00};
     ASF_DATA[1] = (u8)ScaleFactor;
     ASF_DATA[0] = (u8)(ScaleFactor>>8);
    
    ad9954_WriteDataToRegister(ASF_ADD,ASF_DATA,2,1);
}
