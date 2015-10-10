#include "display_functions.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "font5x8.h"

void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal)
{
  GPIOx->ODR = PortVal;
}

void delay(volatile uint32_t nCount)
{

	for(;nCount!=0;nCount--);
}


void otpravka(uint32_t kill) // ОТПРАВКА ДАННЫХ НА LCD
{
 GPIO_Write(GPIOC,kill);
	delay(0x000F);
	HAL_GPIO_WritePin( GPIOC,  GPIO_PIN_9,GPIO_PIN_RESET);
	delay(0x000F);
	HAL_GPIO_WritePin( GPIOC,  GPIO_PIN_9,GPIO_PIN_RESET);
	delay(0x000F);
}
unsigned char GLCD_ReadByteFromROMMemory(uint8_t * ptr) // ПРИХОДИТ АДРЕС ,РАЗЫМЕНОВВЫВАЮ И ПОЛУЧАЮ ДАННЫЕ
{
  return *(ptr);
}
void komandaL(uint32_t lol) 
{
	lol|=0x0900;
otpravka(lol);
	
}

void komandaR(uint32_t mol)
{
	mol|=0x1100;
otpravka(mol);
	
}
void dataL(uint32_t ass)
{
	ass|=0x2900;
otpravka(ass);
	
}

void dataR(uint32_t byte)
{
	byte|=0x3100;
otpravka(byte);
	
}
void GLCD_WriteCharL(uint8_t charToWriteL) // ЗАПИСЬ СИМВОЛА В ЛЕВЫЙ 
{
int i;
	charToWriteL -= 32; 
for(i = 0; i < 5; i++) 
	
  dataL(GLCD_ReadByteFromROMMemory((uint8_t *)((int)font5x8 + (5 * charToWriteL) + i))); 
dataL(0x00);
}

void GLCD_WriteCharR(uint8_t charToWriteR) // ЗАПИСЬ СИМВОЛА В ПРАВЫЙ
{
int i;
charToWriteR -= 32; 
for(i = 0; i < 5; i++) 
	
  dataR(GLCD_ReadByteFromROMMemory((uint8_t *)((int)font5x8 + (5 * charToWriteR) + i))); 
dataR(0x00);
}
void kristalli(uint32_t chislo)//1 OR 2(ИСПРАВИТЬ ВКЛЮЧЕНИЕ КРИСТАЛЛОВ)
{
//	komandaL(0x3E);
//	komandaR(0x3E);
if(chislo==1)
{
komandaL(0x3F);
}

	if(chislo==2)
{
komandaR(0x3F);
}
	if(chislo==12)
{
komandaR(0x3F);
	komandaL(0x3F);
}
}
void set_xL(uint32_t pos_xL) //ПО ВЫСОТЕ ПОЛОЖЕНИЕ В ЛЕВОМ И ТД
 {
	 
	 pos_xL|=0xB8;
	 komandaL(pos_xL);
 } 
 void set_xR(uint32_t pos_xR)
 {
	 
	 pos_xR|=0xB8;
	 komandaR(pos_xR);
 } 
 void set_yL(uint32_t pos_yL)
 {
	 
	 pos_yL|=0x40;
	 komandaL(pos_yL);
 } 
 void set_yR(uint32_t pos_yR)
 {
	 
	 pos_yR|=0x40;
	 komandaR(pos_yR);
 } 
 
void GLCD_WriteStringL(char  *stringToWrite)// ЗАПИСЬ СТРОКИ В ЛЕВУЮ ЧАСТЬ ЭКРАНА(ПЕРЕДЕЛЕЛАТЬ,ЧТОБЫ 
//В ЗАВИСИМОСТИ ОТ ТОГО КАКОЙ БАЙТ,ЗАПИСЫВАИСЬ ДДАНЫЕ В СВОЕ МЕСТО НА ЭКРАНЕ)СДЕЛАТЬ СЧЕТЧИК БАЙТ 
{
	
	uint8_t g=0;
	 int length = 0;
while(*stringToWrite)// ПОКА НЕ КОНЧИТСЯ СТРОКА
{	
if(length<=9)//РАЗМЕР ЧАСТИ ДИСПЛЕЯ В СИМВОЛАХ
	{
//???????

	 length++; 
     
	GLCD_WriteCharL(*stringToWrite);
	 *stringToWrite++;  
}
	
	else
	
		if(g<1)
		{
		set_xL(1);
set_yL(0);
			g++;
		}
	
else

		if(length<=19)//ВТОРАЯ СТРОКА
		{
	
	 length++; 
      
	GLCD_WriteCharL(*stringToWrite);
	 *stringToWrite++; 
		}
			else
	
		if(g<2)
		{
		set_xL(2);
set_yL(0);
			g++;
		}
	
//else

//		if(length<=29)//3 СТРОКА
//		{
//	
//	 length++; 
//      
//	GLCD_WriteCharL(*stringToWrite);
//	 *stringToWrite++; 
//				set_xL(3);
//set_yL(0);
//		}
		else 
		{break;}
}
}
void GLCD_WriteStringR(char * stringToWrite)// СТРОКА В ПРАВУЮ ЧАСТЬ(БЕЗ ОГРАНИЧЕНИЯ И ВЫРЕЗАНИЯ,ЕСЛИ ПРЕВЫСИТ ДЛИНУ ЭКРАНА)
{ 
uint8_t g=0;
	 int length = 0;
while(*stringToWrite)// ПОКА НЕ КОНЧИТСЯ СТРОКА
{	
if(length<=9)//РАЗМЕР ЧАСТИ ДИСПЛЕЯ В СИМВОЛАХ
	{
//???????

	 length++; 
     
	GLCD_WriteCharR(*stringToWrite);
	 *stringToWrite++;  
}
	else 
		{break;}
}	
}
void lcd_fillL()//ЗАПОЛНЕНИЕ ЕДИНИЦАМИ ЛЕВОЙ ЧАСТИ ЭКРАНА
 {
komandaR(0x3F);
	komandaL(0x3F);
  for(unsigned char i=0; i<8; i++)//X
  {
  set_xL(i);
   for(unsigned char j=0; j<64; j++)//Y
    {
     set_yL(j);
     dataL(0xFF);
    }

 
	}
 }
 
 
void lcd_clearL()//НУЛИ В ЛЕВУЮ ЧАСТЬ ЭКРАНА (НЕКОРРЕКТНАЯ РАБОТА ПРИ ПРЕРЫВАНИИ)
 {
komandaR(0x3F);
	komandaL(0x3F);
  for(unsigned char i=0; i<8; i++)//X
  {
  set_xL(i);
   for(unsigned char j=0; j<64; j++)//Y
    {
     set_yL(j);
     dataL(0x00);
    }

 
	}
 }
 void lcd_fillR()
 {
komandaR(0x3F);
	komandaL(0x3F);
  for(unsigned char i=0; i<8; i++)//X
  {
  set_xR(i);
   for(unsigned char j=0; j<64; j++)//Y
    {
     set_yR(j);
     dataR(0xFF);
    }

 
	}
 }
void lcd_clearR()
 {
komandaR(0x3F);
	komandaL(0x3F);
  for(unsigned char i=0; i<8; i++)//X
  {
  set_xR(i);
   for(unsigned char j=0; j<64; j++)//Y
    {
     set_yR(j);
     dataR(0x00);
    }
	}
 }
 
 void vivod(unsigned char sector,unsigned char  pos_x,unsigned char  pos_y, unsigned char *abc)
 {
 
	 
	 
  switch(sector)
  {
   case 1: 
	 {
		set_xL(pos_x);
   set_yL(pos_y);
		GLCD_WriteStringL(abc);
   break;
  }		
   case 2:  // В РАЗРАБОТКЕ
		{
		set_xR(pos_x);
   set_yR(pos_y);
			GLCD_WriteStringR(abc);
   break;
  }
 }
}


