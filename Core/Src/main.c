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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "dfrobot_imu.h"
#include "motor_driver.h"
#include "pd_algorithm.h"
#include "encoder.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * *****************************************
 * Stable Values for Attitude Control System
 * Yaw (Heading) 	- Z
 * Pitch 			- Y
 * Roll				- X
 * *****************************************
 */
int angle_roll 	= 	0;
int angle_pitch	=	0;
int angle_yaw	=	180;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//Global Variables to read Euler data
double x,y,z;
double _z,_x,_y;
int time = 0;
unsigned int rot;
volatile int flag = 0;
//int duty;
//uint16_t pulse_count; // Licznik impulsow
//double vel = 0;

//Global variables for PD algorithm - Motor 1
double blad_1;
double ostatni_blad_1;
double pochodna_1;

//Global variables for PD algorithm - Motor 2
double blad_2;
double ostatni_blad_2;
double pochodna_2;

//Global variables for PD algorithm - Motor 3
double blad_3;
double ostatni_blad_3;
double pochodna_3;

//Variables for PD corrected values
double out_roll = 0;
double out_pitch = 0;
double out_yaw = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{

		 uint8_t data[100];   	// Tablica przechowujaca wysylana wiadomosc.
		 uint16_t size = 0; 	// Rozmiar wysylanej wiadomosci ++cnt;

		 //Zliczanie czasu
		 time += 200;

		 // Wysylanie informacji do zewnetrznego urzadzenia
		 size = sprintf(data, "%.4f \t %d \n\r", z, time);
		 HAL_UART_Transmit_IT(&huart2, data, size);
		 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{


	if(flag == 0)
	flag = 1;

	else if (flag == 1){
	flag = 0;
	}
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
}
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

	//Objects for motor driver structure
	Motor_HandleTypeDef motor1;
	Motor_HandleTypeDef motor2;
	Motor_HandleTypeDef motor3;

	//Encoder object structure
	Encoder_HandleTypeDef encoder_z;

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //Values set for motor1 structure
  motor1.timer = &htim1;
  motor1.channel = TIM_CHANNEL_1;
  motor1.gpio_port = GPIOA;
  motor1.pin_gpio = GPIO_PIN_10;

  //Values set for motor2 structure
  motor2.timer = &htim1;
  motor2.channel = TIM_CHANNEL_2;
  motor2.gpio_port = GPIOA;
  motor2.pin_gpio = GPIO_PIN_9;

  //Values set for motor3 structure
  motor3.timer = &htim1;
  motor3.channel = TIM_CHANNEL_3;
  motor3.gpio_port = GPIOA;
  motor3.pin_gpio = GPIO_PIN_8;

  //Values set for encoder structure
  encoder_z.timer = &htim2;
  encoder_z.channel = TIM_CHANNEL_ALL;
  encoder_z.max_impulse = 600;
  encoder_z.actual_impulse = 0;

  HAL_TIM_Base_Start_IT(&htim2);

  //Start PWM for motor 1,2,3
  Start_PWM_Motor_Z(&motor1);
  Start_PWM_Motor_Z(&motor2);
  Start_PWM_Motor_Z(&motor3);

  //Initialize DFROBOT_IMU
  IMU_Initialize(&hi2c1);

  //Start timer for encoder
  Encoder_Start(&encoder_z);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // IMU SENSOR
	  Euler_Data(&hi2c1, &x, &y, &z);
	  //PD algorithm for all axes

	  //Out value must be set from -1000 to 1000
	  //Cutting the noise values
	  if(out_roll >= 600) out_roll = 600;
	  else if(out_roll <= -600) out_roll = -600;

	  if(out_pitch >= 600) out_pitch = 600;
	  else if(out_pitch <= -600) out_pitch = -600;

	  if(out_yaw >= 700) out_yaw = 700;
	  else if(out_yaw <= -700) out_yaw = -700;

	  //DODATNIA W PRAWO - UJEMNA W LEWO
	  //Setting speed and rotation direction for MOTOR

	  if(blad_1 >= 0) Speed_Motor(&motor1, 1, (uint16_t)out_roll);
	  else if(blad_1 < 0) Speed_Motor(&motor1, 0, (uint16_t)(out_roll*(-1)));

	  if(blad_2 >= 0) Speed_Motor(&motor2, 1, (uint16_t)out_pitch);
	  else if(blad_2 < 0) Speed_Motor(&motor2, 0, (uint16_t)(out_pitch*(-1)));

	  if(blad_3 >= 0) Speed_Motor(&motor3, 0, (uint16_t)out_yaw);
	  else if(blad_3 < 0) Speed_Motor(&motor3, 1, (uint16_t)(out_yaw*(-1)));


	  HAL_Delay(100);

/*
	  Encoder readings
	  vel = Get_encoder_readings(&encoder_z, 0.1);
	  pulse_count = encoder_z.actual_impulse;

*/
    /* USER CODE END WHILE */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE END 4 */

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
