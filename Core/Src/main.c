/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "lcd.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
void AlternateFunction(GPIO_TypeDef * port, uint8_t pin, uint8_t af);
void PWM(TIM_TypeDef * timer, uint8_t canal, uint32_t HCLKFrequency, uint32_t PWMFrequency ,float duty_cycle);
int _write(int file,char * ptr, int len)
{
for (int i=0; i<len;i++)
{
while(LL_USART_IsActiveFlag_TXE(USART2)==0);
LL_USART_TransmitData8(USART2,*ptr);
ptr++;
}
return len;
}


double distance = 0;

void ultrason()
{
	distance = 0;

		GPIOA->ODR|=(1<<1);  // activation du trig
		LL_mDelay(10); // delai d'activation
		GPIOA->ODR &=~(1<<1); // desactivation


		while((GPIOA->IDR & (1<<4)) == 0)  // attendre que le echo passe à 0
		{

		}

		while(GPIOA->IDR & (1<<4))
		{
			LL_mDelay(58);
			distance = distance +1 ; // calcule de la distance
		}

		printf("distance : %f\n", distance); // affichage de la distance
		buzzer();
		LL_mDelay(100000);

}

void buzzer(){

	if(distance > 10){
		GPIOA->ODR  &= ~(1<<0); // desactivation du buzzer si la distance du echo est inferieur à 10 cm
		lcdinit4();
		char ligne1[20] = "desinfectez vous"; // meessage a afficher ligne 1
		char ligne2[20] = "les mains svpl"; // meessage a afficher ligne 2
		Affichage_LCD(ligne1, ligne2);

	}else{


		GPIOA->ODR  |= (1<<0);// sactivation du buzzer si la distance du echo est inferieur à 10 cm
		LL_mDelay(30000);// delai d'activation
		GPIOA->ODR  &= ~(1<<0);// desactivation d
		lcdinit4();


		char ligne3[20] = "Merci";
		char ligne4[20] = "";
		Affichage_LCD(ligne3, ligne4);
		for(int i = 0;i<1500;i++){
				AlternateFunction(GPIOC, 7,0);
				PWM(TIM22, 2, 16000000, 15,0.125);
			   LL_mDelay(100);
			   PWM(TIM22, 2, 16000000, 15,0.125);
			   LL_mDelay(100);
		}

	}

}

int main(void)
{

	RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

	GPIOA->MODER&=~(0b11<<(2*1));
	GPIOA->MODER|=(0b01<<(2*1)); //trig output

	GPIOA->MODER&=~(0b11<<(2*4)); //echo input
	GPIOA->MODER&=~(0b11<<(2*0));
	GPIOA->MODER |= (0b01<<(2*0));


  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/



  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  LL_Init1msTick(16000);
  while (1)
  {
    /* USER CODE END WHILE */

	   ultrason();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_5);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
  {

  }

  LL_Init1msTick(2097000);

  LL_SetSystemCoreClock(2097000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
