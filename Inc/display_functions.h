#ifndef PERIPH_H
#define PERIPH_H
#include "main.h"


void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void otpravka(uint32_t kill);
unsigned char GLCD_ReadByteFromROMMemory(uint8_t * ptr);
void komandaL(uint32_t lol) ;
void komandaR(uint32_t mol);
void dataL(uint32_t ass);
void dataR(uint32_t byte);
void GLCD_WriteCharL(uint8_t charToWriteL) ;
void GLCD_WriteCharR(uint8_t charToWriteR) ;
void GLCD_WriteCharL_ex(uint8_t charToWriteL) ;
void GLCD_WriteCharR_ex(uint8_t charToWriteR) ;
void GLCD_WriteCharL_ex(uint8_t charToWriteL) ;
void GLCD_WriteCharR_ex(uint8_t charToWriteR) ;
void kristalli(uint32_t chislo);
void set_xL(uint32_t pos_xL);
void set_xR(uint32_t pos_xR);
void set_yL(uint32_t pos_yL);
void set_yR(uint32_t pos_yR);
void GLCD_WriteStringL(char  *stringToWrite);
void GLCD_WriteStringR(char * stringToWrite);
void lcd_fillL();
void lcd_clearL();
void lcd_fillR();
void lcd_clearR();
void vivod(unsigned char sector,unsigned char  pos_x,unsigned char  pos_y, unsigned char *abc);
void vivod_ex(unsigned char sector,unsigned char  pos_x,unsigned char  pos_y, unsigned char *abc);
	




	#endif