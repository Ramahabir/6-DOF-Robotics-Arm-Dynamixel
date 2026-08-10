/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "dynamixel.h"
#include <string.h>
#include <stdio.h>

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
DXL_HandleTypeDef hdxl;
DXL_Result result;

uint16_t base_raw;
uint16_t shoulder_raw;
uint16_t elbow_raw;

float base_angle;    
float shoulder_angle;    
float elbow_angle;    

volatile uint16_t base_position = 512; // ID 1
volatile uint16_t joint1_position = 512; // ID 9
volatile uint16_t joint2_position = 512; // ID 13

#define BASE_OFFSET 144.57f
#define SHOULDER_OFFSET 90.32f
#define ELBOW_OFFSET 141.35f
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MG996R_SetAngle(uint8_t angle)
{
    uint16_t pulse;

    if(angle > 180)
        angle = 180;

    // 1000us - 2000us pulse
    pulse = 1000 + ((uint16_t)angle * 1000) / 180;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();

while(1)
{

    char msg[] = "STM32 OK\r\n";

    HAL_UART_Transmit(
        &huart1,
        (uint8_t*)msg,
        strlen(msg),
        HAL_MAX_DELAY
    );

    HAL_Delay(500);
}
}
//   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//   /* USER CODE BEGIN 2 */
//   DXL_Init(&hdxl, &huart2, GPIOB, GPIO_PIN_0);

//   HAL_Delay(500);

//   /* Enable torque off Dynamixel ID 1,9,13 */
//   result = DXL_SetTorque(&hdxl, 1, false);
//   result = DXL_SetTorque(&hdxl, 9, false);
//   result = DXL_SetTorque(&hdxl, 13, false);

//   // HAL_Delay(100);

//   /* Set moderate movement speed */
//   result = DXL_SetMovingSpeed(&hdxl, 1, 100);
//   result = DXL_SetMovingSpeed(&hdxl, 9, 100);
//   result = DXL_SetMovingSpeed(&hdxl, 13, 100);

//   HAL_Delay(500);

//   // Set home pose
//   DXL_SetGoalPosition(&hdxl, 1, 493);
//   DXL_SetGoalPosition(&hdxl, 9, 308);
//   DXL_SetGoalPosition(&hdxl, 13, 481);

//     // Wait until servo reach home position
//   while (1)
//   {
//     DXL_GetPresentPosition(&hdxl, 1, &base_raw);
//     DXL_GetPresentPosition(&hdxl, 9, &shoulder_raw);
//     DXL_GetPresentPosition(&hdxl, 13, &elbow_raw);

//     if (
//       (abs((int)base_raw - 493 < 5)) &&
//       (abs((int)shoulder_raw - 308)) < 5 &&
//       (abs((int)elbow_raw - 481) < 5))
//       {
//         break;
//       } 
//   }
//   /* USER CODE END 2 */

//   /* Infinite loop */
//   /* USER CODE BEGIN WHILE */
//   while (1)
//   {
//     /* USER CODE END WHILE */

//     DXL_GetPresentPosition(&hdxl, 1, &base_raw);
//     DXL_GetPresentPosition(&hdxl, 9, &shoulder_raw);
//     DXL_GetPresentPosition(&hdxl, 13, &elbow_raw);

//     base_angle = DXL_PositionToAngle(base_raw) - BASE_OFFSET;
//     shoulder_angle = DXL_PositionToAngle(shoulder_raw) - SHOULDER_OFFSET;
//     elbow_angle = DXL_PositionToAngle(elbow_raw) - ELBOW_OFFSET;

//     char buffer[100];

//     sprintf(buffer,"Base:%.2f Shoulder:%.2f Elbow:%.2f\r\n", base_angle, shoulder_angle, elbow_angle);
//     HAL_UART_Transmit(
//       &huart1,
//       (uint8_t *)buffer,
//       strlen(buffer),
//       HAL_MAX_DELAY
//     );
//     // MG996R_SetAngle(0);
//     // HAL_Delay(1000);
//     // MG996R_SetAngle(90);
//     // HAL_Delay(1000);
//     // MG996R_SetAngle(180);
//     // HAL_Delay(1000);
//     /* USER CODE BEGIN 3 */
//   }
//   /* USER CODE END 3 */
// }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
