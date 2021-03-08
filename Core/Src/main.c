/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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

#define PITCH_READ  HAL_GPIO_ReadPin(PITCH_GPIO_Port, PITCH_Pin)
#define YAW_READ  HAL_GPIO_ReadPin(YAW_GPIO_Port, YAW_Pin)
#define SWITCH_READ  HAL_GPIO_ReadPin(SWITCH_GPIO_Port, SWITCH_Pin)
#define FIRE_READ  HAL_GPIO_ReadPin(FIRE_GPIO_Port, FIRE_Pin)
#define FIRE_OFF HAL_GPIO_WritePin(FIRE_CON_GPIO_Port, FIRE_CON_Pin, GPIO_PIN_RESET)
#define FIRE_ON HAL_GPIO_WritePin(FIRE_CON_GPIO_Port, FIRE_CON_Pin, GPIO_PIN_SET)

#define PITCH_CON(value) __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,value)
#define YAW_CON(value) __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,value)

uint8_t pitch_flag = 0, yaw_flag = 0, switch_flag = 0, fire_flag = 0, counter=0;
uint16_t pitch_sig = 0, yaw_sig = 0, switch_sig = 0, fire_sig = 0;

uint16_t pitch_pwm = 1500, yaw_pwm = 1500;
int16_t pitch_fix = 0, yaw_fix = 0;

uint8_t RxBuffer[8] = {0};
uint8_t Rx_flag = 0, mode_last = 0, fire_last = 0;
uint16_t blob_x = 0, blob_y = 0;

uint32_t time1_s = 0;
int32_t time2_s = 0;

void Delay_us(int16_t nus);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	/*
		pwm脉宽：1200->1.22ms 1000->1.02ms (+20us)
	*/
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("=========================\n");
	printf("Designed and made by HUAN & PIECE in 2020.10 at SWU.\n");
	printf("All rights reserved.\nHello from SWU! :-)\n");
	printf("=========================\n");
//	HAL_Delay(18000);
	My_UART_DMA_Init();	
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);  // pitch
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);  // yaw
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1500);  // NC
	HAL_TIM_Base_Start_IT(&htim2);
	
	printf("Initing ok!\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		if(counter==0)
		{
			pitch_flag = 0;
			counter++;
		}
		else if(counter==1)
		{
			yaw_flag = 0;
			counter++;
		}
		else if(counter==2)
		{
			switch_flag = 0;
			counter++;
		}
		else if(counter==3)
		{
			fire_flag = 0;
			counter=0;
		}
		if(pitch_sig > 2500) pitch_sig = 1500;
		if(yaw_sig > 2500) yaw_sig = 1500;
		if(switch_sig > 2500) switch_sig = 0;
		if(fire_sig > 2500) fire_sig = 0;
			
		if(switch_sig > 1250)  // 1000 auto mode 
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,GPIO_PIN_SET);
			
			if(mode_last == 1)
			{
				pitch_pwm = 1500;
				yaw_pwm = 1500;
				FIRE_OFF;
			}
			mode_last = 0;
			
			if(Rx_flag == 1)
			{
				if(RxBuffer[1] == 0)
				{
					blob_x = RxBuffer[2] * 100 + RxBuffer[3] * 10 + RxBuffer[4];
					blob_y = RxBuffer[5] * 100 + RxBuffer[6] * 10 + RxBuffer[7];
					pitch_fix = -(120 - blob_y) / 6;
					yaw_fix = (160 - blob_x) / 35;
				}
				Rx_flag = 0;
				time1_s = 0;
				
				if((blob_x-160<=5 || blob_x-160>=-5) && (blob_y-120<=3 || blob_y-120>=-3))
				{
					if(fire_last == 0)
					{
						fire_last = 1;
						time2_s = 0;
					}
					if(time2_s >= 2 && time2_s < 4)
					{
						FIRE_ON;
					}
					else if(time2_s >= 4)
					{
						FIRE_OFF;
						fire_last = 0;
					}
				}
			}
			else
			{
				if(time1_s >= 4)
				{
					pitch_pwm = 1500;
					yaw_pwm = 1500;
				}
			}
		}
		else if(switch_sig < 1250)   // 1500 relative manul mode
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,GPIO_PIN_RESET);
			if(mode_last == 0)
			{
				pitch_pwm = 1500;
				yaw_pwm = 1500;
				FIRE_OFF;
			}
			mode_last = 1;
			pitch_fix = (pitch_sig - 1500) / 20;
			yaw_fix = (1500 - yaw_sig) / 60;
			if(fire_sig >= 800&& fire_sig <= 1200) 
				FIRE_OFF;
			else if (fire_sig >= 1800 && fire_sig <= 2200) 
				FIRE_ON;
		} 
		
		if(pitch_pwm > 2000) pitch_pwm = 2000;
		else if(pitch_pwm < 1000) pitch_pwm = 1000;
		if(yaw_pwm > 2000) yaw_pwm = 2000;
		else if(yaw_pwm < 1000) yaw_pwm = 1000;
		
		PITCH_CON(pitch_pwm);
		YAW_CON(yaw_pwm);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if((GPIO_Pin == PITCH_Pin) || (GPIO_Pin == YAW_Pin) || (GPIO_Pin == SWITCH_Pin) || (GPIO_Pin == FIRE_Pin))
	{	
		// TODO 三路信号可能不会同时到达！直接延时800us可能出错
		Delay_us(799);
		if(pitch_flag == 0 && PITCH_READ == GPIO_PIN_SET)	pitch_sig = 800;

		if(yaw_flag == 0 && YAW_READ == GPIO_PIN_SET)	yaw_sig = 800;

		if(switch_flag == 0 && SWITCH_READ == GPIO_PIN_SET)	switch_sig = 800;
		
		if(fire_flag == 0 && FIRE_READ == GPIO_PIN_SET)	fire_sig = 800;
		
		while((pitch_flag == 0) || (yaw_flag == 0) || (switch_flag == 0) || (fire_flag == 0))
		{
			Delay_us(18);
			if((pitch_flag == 0) && (PITCH_READ == GPIO_PIN_SET))	
				pitch_sig += 20;
			else
				pitch_flag = 1;
			
			if((yaw_flag == 0) && (YAW_READ == GPIO_PIN_SET))
				yaw_sig += 20;
			else
				yaw_flag = 1;
		
			if((switch_flag == 0) && (SWITCH_READ == GPIO_PIN_SET))	
				switch_sig += 20;
			else
				switch_flag = 1;
			
			if((fire_flag == 0) && (FIRE_READ == GPIO_PIN_SET))	
				fire_sig += 20;
			else
				fire_flag = 1;
			
		}
	}
}

/*
	HAL_Delay重定义，保留原有功能，并消除+1ms误差
*/
void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;
	if (wait == 0)
	{
		wait += 1U;
	}
  while ((HAL_GetTick() - tickstart) < wait)
  {
  }
}

/*
	us延时函数，存在误差(~1us)
*/
void Delay_us(int16_t nus) 
{
  int32_t temp; 
  SysTick->LOAD = nus*9; //72MHz
  SysTick->VAL=0X00;
  SysTick->CTRL=0X01;
  do 
  { 
    temp=SysTick->CTRL;
  }
  while((temp&0x01)&&(!(temp&(1<<16))));
     
  SysTick->CTRL=0x00; 
  SysTick->VAL =0X00; 
}

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	static uint16_t tick = 0;
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	else if (htim->Instance == TIM2)  // 20ms
	{
//		printf("%d %d %d %d\n", pitch_sig, yaw_sig, switch_sig, fire_sig);
//		printf("x: %d\ty: %d\n", blob_x, blob_y);
		pitch_pwm += pitch_fix; pitch_fix = 0;
		yaw_pwm += yaw_fix; yaw_fix = 0;
		tick++;
		if(tick == 50)
		{
			tick = 0;
			time1_s++;
			time2_s++;
		}
	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
