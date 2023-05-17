#include "stm32f1xx_hal.h"
#include "oled.h"
#include "oledfont.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
uint8_t writeCmd(unsigned char I2C_Command)//д����
 {

		return HAL_I2C_Mem_Write(&hi2c1,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&I2C_Command,1,100);

 }

uint8_t writeDat(unsigned char I2C_Data)//д����
 {

		return HAL_I2C_Mem_Write(&hi2c1,OLED0561_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&I2C_Data,1,100);

  }

#define WriteCmd(I2C_Command) if(writeCmd(I2C_Command)) return 1;
#define WriteDat(I2C_Data) if(writeDat(I2C_Data)) return 1;
uint8_t OLED_Init(void)
{
	HAL_Delay(100); //�������ʱ����Ҫ

	WriteCmd(0xAE); //display off
	WriteCmd(0x20);	//Set Memory Addressing Mode
	WriteCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(0xc8);	//Set COM Output Scan Direction
	WriteCmd(0x00); //---set low column address
	WriteCmd(0x10); //---set high column address
	WriteCmd(0x40); //--set start line address
	WriteCmd(0x81); //--set contrast control register
	WriteCmd(0xff); //���ȵ��� 0x00~0xff
	WriteCmd(0xa1); //--set segment re-map 0 to 127
	WriteCmd(0xa6); //--set normal display
	WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	WriteCmd(0x3F); //
	WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xd3); //-set display offset
	WriteCmd(0x00); //-not offset
	WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0xf0); //--set divide ratio
	WriteCmd(0xd9); //--set pre-charge period
	WriteCmd(0x22); //
	WriteCmd(0xda); //--set com pins hardware configuration
	WriteCmd(0x12);
	WriteCmd(0xdb); //--set vcomh
	WriteCmd(0x20); //0x20,0.77xVcc
	WriteCmd(0x8d); //--set DC-DC enable
	WriteCmd(0x14); //
	WriteCmd(0xaf); //--turn on oled panel

	return 0;
}

uint8_t OLED_SetPos(unsigned char x, unsigned char y) //������ʼ������
{
	WriteCmd(0xb0+y);
	WriteCmd(((x&0xf0)>>4)|0x10);
	WriteCmd((x&0x0f)|0x01);

	return 0;
}

uint8_t OLED_Fill(unsigned char fill_Data)//ȫ�����
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		WriteCmd(0xb0+m);		//page0-page1
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address
		for(n=0;n<128;n++)
		{
			WriteDat(fill_Data);
		}
	}

	return 0;
}


uint8_t OLED_CLS(void)//����
{
	return OLED_Fill(0x00);
}

uint8_t OLED_ON(void)
{
	WriteCmd(0X8D);  //���õ�ɱ�
	WriteCmd(0X14);  //������ɱ�
	WriteCmd(0XAF);  //OLED����

	return 0;
}

uint8_t OLED_OFF(void)
{
	WriteCmd(0X8D);  //���õ�ɱ�
	WriteCmd(0X10);  //�رյ�ɱ�
	WriteCmd(0XAE);  //OLED����

	return 0;
}


// Parameters     : x,y -- ��ʼ������(x:0~127, y:0~7); ch[] -- Ҫ��ʾ���ַ���; TextSize -- �ַ���С(1:6*8 ; 2:8*16)
uint8_t OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
{
	unsigned char c = 0,i = 0,j = 0;
	switch(TextSize)
	{
		case 1:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 126)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<6;i++)
					WriteDat(F6x8[c][i]);
				x += 6;
				j++;
			}
		}break;
		case 2:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 120)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<8;i++)
					WriteDat(F8X16[c*16+i]);
				OLED_SetPos(x,y+1);
				for(i=0;i<8;i++)
					WriteDat(F8X16[c*16+i+8]);
				x += 8;
				j++;
			}
		}break;
	}
	return 0;
}


// Parameters     : x,y -- ��ʼ������(x:0~127, y:0~7); N:������.h�е�����
// Description    : ��ʾASCII_8x16.h�еĺ���,16*16����
uint8_t OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
{
	unsigned int adder=16*8*N;
	y += 4 * 2;
	for(uint8_t i = 0; i < 8; i++){ //8 row
			y--;
			OLED_SetPos(x, y);
			for(uint8_t j = 0; j < 16; j++){
				WriteDat(F16x8[adder + 16 * i + 15 - j]);
			}
		
	}
	return 0;
}



// Parameters     : x0,y0 -- ��ʼ������(x0:0~127, y0:0~7); x1,y1 -- ���Խ���(������)������(x1:1~128,y1:1~8)
// Description    : ��ʾBMPλͼ
uint8_t OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[])
{
	unsigned int j=0;
	unsigned char x,y;

  if(y1%8==0)
		y = y1/8;
  else
		y = y1/8 + 1;
	for(y=y0;y<y1;y++)
	{
		OLED_SetPos(x0,y);
    for(x=x0;x<x1;x++)
		{
			WriteDat(BMP[j++]);
		}
	}
	return 0;
}






uint8_t OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size)
{
	unsigned char c=0,i=0;
	c=chr-' ';//�õ�ƫ�ƺ��ֵ
	if(x>128-1){x=0;y=y+2;}
	if(Char_Size ==16)
	{
	OLED_SetPos(x,y);
	for(i=0;i<8;i++)
	WriteDat(F8X16[c*16+i]);
	OLED_SetPos(x,y+1);
	for(i=0;i<8;i++)
	WriteDat(F8X16[c*16+i+8]);
	}
	else {
		OLED_SetPos(x,y);
		for(i=0;i<6;i++)
		WriteDat(F6x8[c][i]);

	}
	return 0;
}
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--)result*=m;
	return result;
}


//��ʾ2������
//x,y :�������
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2)
{
	u8 t,temp;
	u8 enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
				continue;
			}else enshow=1;

		}
	 	OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2);
	}
}