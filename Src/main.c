/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
 #include "i2c.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t delay_count=0;	
char datas=88;
	char s[17];
	uint8_t data[12] ;
		uint8_t datar[12] ;
		uint8_t gg=0x33;
		#define I2C_ADDRESS        0x3E
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void delay_ms(uint32_t delay_temp)
{
delay_count = delay_temp;
	while(delay_count){}
}

void spi_receive(uint8_t m)
{
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
			
//	HAL_SPI_TransmitReceive(&hspi1, &data[h],  &datar[h], 1,1000);
			HAL_SPI_Receive(&hspi1,  &datar[m], 1,10000);
//				HAL_SPI_Transmit_IT(&hspi1, &data[h], 1);
//			
//				HAL_SPI_Receive_IT(&hspi1,  &datar[h], 1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);
//		HAL_Delay(1000);
}

//void striiing(unsigned char sector,unsigned char  pos_x,unsigned char  pos_y,uint16_t datas)
//{
//uint16_t i,j,o;
//	uint32_t arg[6];
//	char _str[255];
//	sprintf(_str, " % d", datas);
//				vivod(sector,pos_x,pos_y, _str);

//}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
	 I2C1_Initialize();
//  MX_SPI2_Init();
//  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	lcd_fillL();
	delay(0xFF);
	lcd_clearL();
	delay(0xFF);
	lcd_fillR();
	delay(0xFF);
	lcd_clearR();
	delay(0xFF);
	set_xL(0);
	set_yL(0);
	delay(0xFF);
	data[0]=0x01;
//	data[1]=0x02;
//	data[2]=0x03;
//	data[3]=0x04;
//	data[4]=0x05;
//	data[5]=0x06;
//	data[6]=6;
//	data[7]=0x08;
//	data[8]=0x09;
//	data[9]=0x10;
//	data[10]=0x11;
//	data[11]=0x12;
//	data[12]=0x13;
//	data[13]=0x14;
//	data[14]=0x15;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t ghg=0;
	 HAL_I2C_Master_Receive_IT(&hi2c1, (uint16_t)I2C_ADDRESS, (uint8_t*)&gg, 1);
  while (1)
  {
		
		
//			HAL_Delay(3000);
		
		
//		if (ghg==0){
//		spi_receive(0);
//			ghg=ghg+1;
//		}
//		for(uint8_t h=0;h<12;h++)
//		{
//			if(h!=0) {
//spi_receive(h);
//			}
////			HAL_Delay(1000);
//		}
////		char	charka[]=datar[5];
//	set_xL(0);
//	set_yL(0);
//	set_xR(0);
//	set_yR(0);
//			 for(int l=0;l<4;l++)
//			 {
//				switch(l)
//				{
//					case 1:
//				{

//			 vivod(1,0,0, datar[1],16);
//			  vivod(1,1,0, datar[1],10);
//					vivod_ex(1,2,0, "laserstate");
//			 vivod_ex(1,3,0, "4");
//			 vivod_ex(1,4,0, "5");
//			 vivod_ex(1,5,0, "6");
//			 vivod_ex(1,6,0, "7");
//			 vivod_ex(1,7,0, "8");


//            vivod(2,0,0, data[0]);
////            vivod_ex(2,1,0, &s);
//						vivod_ex(2,2,0, " off");
//						vivod_ex(2,1,0, "-3");
//						vivod_ex(2,3,0, "-4");
//						vivod_ex(2,4,0, "-5");
//						vivod_ex(2,5,0, "-6");
//						vivod_ex(2,6,0, "-7");
//						vivod_ex(2,7,0, "-8");
//	
//					break;
//			 }			
//				case 2:
//				{
//  	HAL_Delay(3000);
//	   lcd_clearL();
//	   lcd_clearR();
//			 vivod_ex(1,0,0,"9");
//			 vivod_ex(1,1,0, "10");
//			 vivod_ex(1,2,0, "11");
//			 vivod_ex(1,3,0, "12");
//			 vivod_ex(1,4,0, "13");
//			 vivod_ex(1,5,0, "14");
//			 vivod_ex(1,6,0, "15");
//			 vivod_ex(1,7,0, "16");
//						vivod_ex(2,0,0, "-9");
//						vivod_ex(2,1,0, "-10");
//						vivod_ex(2,2,0, "-11");
//						vivod_ex(2,3,0, "-12");
//						vivod_ex(2,4,0, "-13");
//						vivod_ex(2,5,0, "-14");
//						vivod_ex(2,6,0, "-15");
//						vivod_ex(2,7,0, "-16");
//				break;
//				}
//					case 3:
//				{
// 	HAL_Delay(3000);
//		 lcd_clearL();
//	   lcd_clearR();
//			 vivod_ex(1,0,0,"17");
//			 vivod_ex(1,1,0, "18");
//			 vivod_ex(1,2,0, "19");
//			 vivod_ex(1,3,0, "20");
//			 vivod_ex(1,4,0, "21");
//			 vivod_ex(1,5,0, "22");
//			 vivod_ex(1,6,0, "23");
//			 vivod_ex(1,7,0, "24");
//						vivod_ex(2,0,0, "-17");
//						vivod_ex(2,1,0, "-18");
//						vivod_ex(2,2,0, "-19");
//						vivod_ex(2,3,0, "-20");
//						vivod_ex(2,4,0, "-21");
//						vivod_ex(2,5,0, "-22");
//						vivod_ex(2,6,0, "-23");
//						vivod_ex(2,7,0, "-24");
//				break;
//				}
//				
//		 }
//	 }
//	 	HAL_Delay(3000);

//	set_xL(0);
//	set_yL(0);
//	set_xR(0);
//	set_yR(0);

//	lcd_clearL();
//	lcd_clearR();


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
