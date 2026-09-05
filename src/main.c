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
#include <stdio.h>
#include <string.h>

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
static DXL_HandleTypeDef hdxl;

#define BASE_ID       1U
#define SHOULDER_ID   9U
#define ELBOW_ID     13U
#define WRIST_ID       3U

/* Set to 1 to run the Dynamixel joints and log their telemetry. */
#define DYNAMIXEL_ENABLED 1U
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Robot_ReadAndLogDynamixelTelemetry(void);

static void UART_SendUART1(const char *message)
{
  size_t message_length = strlen(message);
  if (message_length > 0U)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)message,
                            (uint16_t)message_length, HAL_MAX_DELAY);
  }
}

static void Robot_ReadAndLogDynamixelTelemetry(void)
{
  char buffer[256];
  int length;

  static const uint8_t joint_ids[] =
  {
    BASE_ID,
    SHOULDER_ID,
    ELBOW_ID,
    WRIST_ID
  };
  static const char *const joint_names[] =
  {
    "BASE",
    "SHD",
    "ELB",
    "WRI"
  };

  size_t offset = 0U;

  for (size_t i = 0U; i < sizeof(joint_ids) / sizeof(joint_ids[0]); i++)
  {
    uint8_t id = joint_ids[i];
    uint16_t position = 0U;
    uint16_t speed = 0U;
    uint8_t voltage_x10 = 0U;
    uint8_t temperature = 0U;
    uint16_t load = 0U;
    bool moving = false;

    bool all_ok =
        (DXL_GetPresentPosition(&hdxl, id, &position) == DXL_OK) &&
        (DXL_GetPresentSpeed(&hdxl, id, &speed) == DXL_OK) &&
        (DXL_GetPresentVoltage(&hdxl, id, &voltage_x10) == DXL_OK) &&
        (DXL_GetPresentTemperature(&hdxl, id, &temperature) == DXL_OK) &&
        (DXL_GetPresentLoad(&hdxl, id, &load) == DXL_OK) &&
        (DXL_IsMoving(&hdxl, id, &moving) == DXL_OK);

    if (!all_ok)
    {
      length = snprintf(
          &buffer[offset],
          sizeof(buffer) - offset,
          "%s%s,ERROR,%d",
          (i == 0U) ? "" : " ",
          joint_names[i],
          (int)hdxl.last_servo_error
      );
      offset += (size_t)length;
    }
    else
    {
      /*
       * AX-12A PRESENT_LOAD is a signed value centered on 0x400:
       * 0..0x3FF is counterclockwise load, 0x401..0x7FF is clockwise.
       * Report the raw signed value so the sign can be interpreted.
       */
      int16_t signed_load = (int16_t)load;
      if (load >= 0x400U)
      {
        signed_load = (int16_t)load - 0x400;
      }
      else
      {
        signed_load = (int16_t)load;
      }

      buffer[offset] = (i == 0U) ? '\0' : ' ';
      offset += (i == 0U) ? 0U : 1U;
      length = snprintf(
          &buffer[offset],
          sizeof(buffer) - offset,
          "%s,%u,%u,%d,%u,%u,%s",
          joint_names[i],
          (unsigned)position,
          (unsigned)speed,
          (int)signed_load,
          (unsigned)voltage_x10,
          (unsigned)temperature,
          moving ? "MOV" : "IDL"
      );
      offset += (size_t)length;
    }
  }

  length = snprintf(&buffer[offset], sizeof(buffer) - offset, "\r\n");
  offset += (size_t)length;

  (void)HAL_UART_Transmit(&huart6, (uint8_t *)buffer,
                          (uint16_t)offset, HAL_MAX_DELAY);
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)buffer,
                          (uint16_t)offset, HAL_MAX_DELAY);
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
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /*
   * UART2: PA2=TX, PA3=RX, 1 Mbps, Dynamixel Protocol 1.0.
   * PB0 controls the external half-duplex buffer: high=TX, low=RX.
   */
#if DYNAMIXEL_ENABLED  /* ===== Dynamixel active ===== */
  DXL_Init(&hdxl, &huart2, GPIOB, GPIO_PIN_0);
  HAL_Delay(500U);

  uint16_t base_position, shoulder_position, elbow_position, wrist_position;
  char buffer[128];
  int length;

  DXL_SetTorque(&hdxl, BASE_ID, false);
  DXL_SetTorque(&hdxl, SHOULDER_ID, false);
  DXL_SetTorque(&hdxl, ELBOW_ID, false);
  DXL_SetTorque(&hdxl, WRIST_ID, false);
  HAL_Delay(100U);

  length = snprintf(buffer, sizeof(buffer),
                    "Dynamixel ready (torque off)\r\n");
  if (length > 0)
  {
    UART_SendUART1(buffer);
  }
#endif /* DYNAMIXEL_ENABLED */

  while (1)
  {
#if DYNAMIXEL_ENABLED
    bool positions_ok =
        (DXL_GetPresentPosition(&hdxl, BASE_ID, &base_position) == DXL_OK) &&
        (DXL_GetPresentPosition(&hdxl, SHOULDER_ID, &shoulder_position) == DXL_OK) &&
        (DXL_GetPresentPosition(&hdxl, ELBOW_ID, &elbow_position) == DXL_OK) &&
        (DXL_GetPresentPosition(&hdxl, WRIST_ID, &wrist_position) == DXL_OK);

    if (positions_ok)
    {
      HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin,
                        GPIO_PIN_RESET);

      Robot_ReadAndLogDynamixelTelemetry();
    }
    else
    {
      length = snprintf(buffer, sizeof(buffer),
                        "DXL error reading positions err=%d\r\n",
                        (int)hdxl.last_servo_error);
      HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
    }

#endif /* DYNAMIXEL_ENABLED */

    HAL_Delay(200U);
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
