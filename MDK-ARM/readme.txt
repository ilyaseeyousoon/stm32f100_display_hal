.LOG
16:03 27.02.2016

void vivod(unsigned char sector,unsigned char  pos_x,unsigned char  pos_y, uint16_t data,uint8_t notation);

sector - левая или правая часть экрана (1,2)
pos_x, pos_y- номер столбца и строки с которых начнется вывод (0..7)
data - данные числовые
notation - система счисления в которой данные выводятся на экран(10,16)

void vivod_ex(unsigned char sector,unsigned char  pos_x,unsigned char  pos_y, unsigned char *abc);

abc - вывод строки до 8ми символов