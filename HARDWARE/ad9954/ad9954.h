#ifndef AD9954_H
#define AD9954_H
#endif

#include "sys.h"
#include "stdint.h"
#include "delay.h"
#include "usart.h"

//寄存器定义
#define CFR1_ADD 0x00
#define CFR2_ADD 0x01
#define ASF_ADD  0x02/*Amplitude Scale Factor*/

#define FTW0_ADD 0x04
#define POW0_ADD 0x05
#define FTW1_ADD 0x06
#define RSCW0_ADD 0x07
#define RSCW1_ADD 0x08
#define RSCW2_ADD 0x09
#define RSCW3_ADD 0x0A
/*0x0B开始为 32*1024 bit RAM segment*/

//引脚定义
#define IO_UPDATE PDout(0)
#define PS1 PDout(2)
#define PS0 PDout(4)
#define OSK PDout(6)
#define SDIO1 PGout(9)
#define SCLK PGout(11)
#define RESET PGout(13)

//相关函数申明
void ad9954_WriteDataToRegister(uint8_t RegisterAddress,uint8_t* InputData,uint8_t NumberOfRegister,uint8_t flag);
void GPIO_ad9954_initialize(void);
void ad9954_update(void);
void ad9954_reset(void);
void ad9954_init(void);
void ad9954_WriteDataToRegister(uint8_t RegisterAddress,uint8_t* InputData,uint8_t NumberOfRegister,uint8_t flag);
void ad9954_WriteData(uint8_t* InputData,uint8_t NumberOfArray,uint8_t flag);
void ad9954_WriteDataToRAM(u32 fre_m,u32 fre_delta,u32 fre_c);
void Write_frequence(u32 Freq);
void Write_amplitude(u16 ScaleFactor);
void ad9954_WriteByte(uint8_t ValueToWrite);
