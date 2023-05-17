#ifndef OLED_H
#define OLED_H

#include "stm32f1xx_hal_i2c.h"

 #define u8 uint8_t
 #define u32 uint32_t

#define OLED0561_ADD	0x78  // OLED��I2C��ַ����ֹ�޸ģ�
#define COM				0x00  // OLED ָ���ֹ�޸ģ�
#define DAT 			0x40  // OLED ���ݣ���ֹ�޸ģ�

//uint8_t WriteCmd(unsigned char I2C_Command);//д����
//uint8_t WriteDat(unsigned char I2C_Data);//д����
uint8_t OLED_Init(void);//��ʼ��
uint8_t OLED_SetPos(unsigned char x, unsigned char y);
uint8_t OLED_Fill(unsigned char fill_Data);//ȫ�����
uint8_t OLED_CLS(void);
uint8_t OLED_ON(void);
uint8_t OLED_OFF(void);
uint8_t OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);//��ʾ�ַ���
uint8_t OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);//��ʾ����
uint8_t OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);//��ʾͼƬ

uint8_t OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size);
u32 oled_pow(u8 m,u8 n);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2);//size2(16|12)
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 Char_Size);

#endif