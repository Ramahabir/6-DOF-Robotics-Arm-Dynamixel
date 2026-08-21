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

#define TARGET_BASE_POS    512U
#define TARGET_SHOULDER_POS 315U
#define TARGET_ELBOW_POS   512U
#define TARGET_WRIST_POS   512U
#define POSITION_TOLERANCE  5U

/* Home position offsets - these will be displayed as 0 degrees */
#define HOME_BASE_POS       512U
#define HOME_SHOULDER_POS   315U
#define HOME_ELBOW_POS      512U
#define HOME_WRIST_POS      512U

#define ANIMATION_POSE_TIMEOUT_MS  5000U
#define ANIMATION_STABLE_READS      3U
#define ANIMATION_POSE_HOLD_MS   2000U

typedef struct
{
  uint16_t base;
  uint16_t shoulder;
  uint16_t elbow;
  uint16_t wrist;
  uint16_t speed;
  uint16_t pause_ms;
} RobotPose;

/*
 * Dramatic "sentinel wake-up" sequence. The wide scan takes about two
 * seconds, depending on servo load and supply voltage. The final pose is home.
 */
static const RobotPose startup_animation[] =
{
  {512U, 315U, 512U, 512U, 50U, ANIMATION_POSE_HOLD_MS}, /* home */
  {392U, 315U, 512U, 512U, 50U, ANIMATION_POSE_HOLD_MS}, /* scan left */
  {632U, 315U, 512U, 512U, 50U, ANIMATION_POSE_HOLD_MS}, /* scan right */
  {512U, 315U, 512U, 512U, 50U, ANIMATION_POSE_HOLD_MS}, /* center */
  {512U, 365U, 457U, 402U, 30U, ANIMATION_POSE_HOLD_MS}, /* bow */
  {512U, 365U, 457U, 622U, 55U, ANIMATION_POSE_HOLD_MS}, /* wrist flick */
  {512U, 365U, 457U, 512U, 55U, ANIMATION_POSE_HOLD_MS}, /* wrist center */
  {512U, 315U, 512U, 512U, 25U, ANIMATION_POSE_HOLD_MS}  /* home */
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static bool Robot_MoveToPose(const RobotPose *pose, uint32_t timeout_ms);
static bool Robot_PlayStartupAnimation(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t PositionDifference(uint16_t actual, uint16_t target)
{
  return (actual > target) ? (actual - target) : (target - actual);
}

static bool Robot_MoveToPose(const RobotPose *pose, uint32_t timeout_ms)
{
  uint16_t base;
  uint16_t shoulder;
  uint16_t elbow;
  uint16_t wrist;
  uint32_t start_ms;
  uint8_t stable_reads = 0U;

  if (pose == NULL)
  {
    return false;
  }

  if ((DXL_SetMovingSpeed(&hdxl, BASE_ID, pose->speed) != DXL_OK) ||
      (DXL_SetMovingSpeed(&hdxl, SHOULDER_ID, pose->speed) != DXL_OK) ||
      (DXL_SetMovingSpeed(&hdxl, ELBOW_ID, pose->speed) != DXL_OK) ||
      (DXL_SetMovingSpeed(&hdxl, WRIST_ID, pose->speed) != DXL_OK))
  {
    return false;
  }

  if ((DXL_SetGoalPosition(&hdxl, BASE_ID, pose->base) != DXL_OK) ||
      (DXL_SetGoalPosition(&hdxl, SHOULDER_ID, pose->shoulder) != DXL_OK) ||
      (DXL_SetGoalPosition(&hdxl, ELBOW_ID, pose->elbow) != DXL_OK) ||
      (DXL_SetGoalPosition(&hdxl, WRIST_ID, pose->wrist) != DXL_OK))
  {
    return false;
  }

  start_ms = HAL_GetTick();
  while ((HAL_GetTick() - start_ms) < timeout_ms)
  {
    bool at_target =
        (DXL_GetPresentPosition(&hdxl, BASE_ID, &base) == DXL_OK) &&
        (DXL_GetPresentPosition(&hdxl, SHOULDER_ID, &shoulder) == DXL_OK) &&
        (DXL_GetPresentPosition(&hdxl, ELBOW_ID, &elbow) == DXL_OK) &&
        (DXL_GetPresentPosition(&hdxl, WRIST_ID, &wrist) == DXL_OK) &&
        (PositionDifference(base, pose->base) <= POSITION_TOLERANCE) &&
        (PositionDifference(shoulder, pose->shoulder) <= POSITION_TOLERANCE) &&
        (PositionDifference(elbow, pose->elbow) <= POSITION_TOLERANCE) &&
        (PositionDifference(wrist, pose->wrist) <= POSITION_TOLERANCE);

    if (at_target)
    {
      stable_reads++;
      if (stable_reads >= ANIMATION_STABLE_READS)
      {
        HAL_Delay(pose->pause_ms);
        return true;
      }
    }
    else
    {
      stable_reads = 0U;
    }

    HAL_Delay(40U);
  }

  return false;
}

static bool Robot_PlayStartupAnimation(void)
{
  size_t pose_index;
  const size_t pose_count =
      sizeof(startup_animation) / sizeof(startup_animation[0]);

  for (pose_index = 0U; pose_index < pose_count; pose_index++)
  {
    if (!Robot_MoveToPose(&startup_animation[pose_index],
                          ANIMATION_POSE_TIMEOUT_MS))
    {
      return false;
    }
  }

  return true;
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

  /*
   * UART2: PA2=TX, PA3=RX, 1 Mbps, Dynamixel Protocol 1.0.
   * PB0 controls the external half-duplex buffer: high=TX, low=RX.
   */
  DXL_Init(&hdxl, &huart2, GPIOB, GPIO_PIN_0);
  HAL_Delay(500U);

  /* ========== STARTUP INITIALIZATION SEQUENCE ========== */
  uint16_t base_position, shoulder_position, elbow_position, wrist_position;
  uint16_t attempts = 0;
  const uint16_t MAX_ATTEMPTS = 500;  /* ~50 seconds at 100ms polling */
  char buffer[128];
  int length;
  bool init_success = false;

  /* Enable torque on all servos */
  DXL_SetTorque(&hdxl, BASE_ID, true);
  DXL_SetTorque(&hdxl, SHOULDER_ID, true);
  DXL_SetTorque(&hdxl, ELBOW_ID, true);
  DXL_SetTorque(&hdxl, WRIST_ID, true);
  HAL_Delay(100U);

  /* Set moderate movement speed for all servos */
  DXL_SetMovingSpeed(&hdxl, BASE_ID, 256);
  DXL_SetMovingSpeed(&hdxl, SHOULDER_ID, 256);
  DXL_SetMovingSpeed(&hdxl, ELBOW_ID, 256);
  DXL_SetMovingSpeed(&hdxl, WRIST_ID, 256);
  HAL_Delay(100U);

  /* Set initial goal positions */
  DXL_SetGoalPosition(&hdxl, BASE_ID, TARGET_BASE_POS);
  DXL_SetGoalPosition(&hdxl, SHOULDER_ID, TARGET_SHOULDER_POS);
  DXL_SetGoalPosition(&hdxl, ELBOW_ID, TARGET_ELBOW_POS);
  DXL_SetGoalPosition(&hdxl, WRIST_ID, TARGET_WRIST_POS);
  HAL_Delay(500U);

  /* Print startup message */
  length = snprintf(buffer, sizeof(buffer),
                    "Robot startup: Moving to initial position...\r\n");
  if (length > 0) {
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
  }

  /* Wait for all servos to reach target positions */
  while (attempts < MAX_ATTEMPTS && !init_success) {
    DXL_Result base_result = DXL_GetPresentPosition(&hdxl, BASE_ID, &base_position);
    DXL_Result shoulder_result = DXL_GetPresentPosition(&hdxl, SHOULDER_ID, &shoulder_position);
    DXL_Result elbow_result = DXL_GetPresentPosition(&hdxl, ELBOW_ID, &elbow_position);
    DXL_Result wrist_result = DXL_GetPresentPosition(&hdxl, WRIST_ID, &wrist_position);

    /* Check if all positions are within tolerance */
    if ((base_result == DXL_OK) && (shoulder_result == DXL_OK) &&
        (elbow_result == DXL_OK) && (wrist_result == DXL_OK)) {
      
      int base_diff = (base_position > TARGET_BASE_POS) ? 
                      (base_position - TARGET_BASE_POS) : 
                      (TARGET_BASE_POS - base_position);
      int shoulder_diff = (shoulder_position > TARGET_SHOULDER_POS) ? 
                          (shoulder_position - TARGET_SHOULDER_POS) : 
                          (TARGET_SHOULDER_POS - shoulder_position);
      int elbow_diff = (elbow_position > TARGET_ELBOW_POS) ? 
                       (elbow_position - TARGET_ELBOW_POS) : 
                       (TARGET_ELBOW_POS - elbow_position);
      int wrist_diff = (wrist_position > TARGET_WRIST_POS) ? 
                       (wrist_position - TARGET_WRIST_POS) : 
                       (TARGET_WRIST_POS - wrist_position);

      if ((base_diff <= POSITION_TOLERANCE) &&
          (shoulder_diff <= POSITION_TOLERANCE) &&
          (elbow_diff <= POSITION_TOLERANCE) &&
          (wrist_diff <= POSITION_TOLERANCE)) {
        init_success = true;
        break;
      }
    }

    attempts++;
    HAL_Delay(100U);
  }

  /* Print initialization result */
  if (init_success) {
    /* Apply home offset so home position displays as 0 degrees */
    float base_deg = DXL_PositionToAngle(base_position > HOME_BASE_POS ? 
                                         base_position - HOME_BASE_POS : 
                                         HOME_BASE_POS - base_position);
    float shoulder_deg = DXL_PositionToAngle(shoulder_position > HOME_SHOULDER_POS ? 
                                             shoulder_position - HOME_SHOULDER_POS : 
                                             HOME_SHOULDER_POS - shoulder_position);
    float elbow_deg = DXL_PositionToAngle(elbow_position > HOME_ELBOW_POS ? 
                                          elbow_position - HOME_ELBOW_POS : 
                                          HOME_ELBOW_POS - elbow_position);
    float wrist_deg = DXL_PositionToAngle(wrist_position > HOME_WRIST_POS ? 
                                          wrist_position - HOME_WRIST_POS : 
                                          HOME_WRIST_POS - wrist_position);
    length = snprintf(buffer, sizeof(buffer),
                      "[SUCCESS] Robot at initial position: B:%.2f° S:%.2f° E:%.2f° W:%.2f°\r\n",
                      base_deg, shoulder_deg, elbow_deg, wrist_deg);
    HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
  } else {
    length = snprintf(buffer, sizeof(buffer),
                      "[FAILED] Timeout reaching initial position (attempts: %u)\r\n",
                      attempts);
    HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
  }

  if (length > 0) {
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
  }

  /* Play the wake-up sequence only after the robot has safely reached home. */
  if (init_success)
  {
    length = snprintf(buffer, sizeof(buffer),
                      "Robot startup: Playing sentinel animation...\r\n");
    if (length > 0)
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)length,
                        HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart6, (uint8_t *)buffer, (uint16_t)length,
                        HAL_MAX_DELAY);
    }

    if (Robot_PlayStartupAnimation())
    {
      length = snprintf(buffer, sizeof(buffer),
                        "[SUCCESS] Animation complete; robot is home.\r\n");
      HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin,
                        GPIO_PIN_RESET);
    }
    else
    {
      length = snprintf(buffer, sizeof(buffer),
                        "[FAILED] Animation stopped: servo error or timeout.\r\n");
      HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin,
                        GPIO_PIN_SET);
    }

    if (length > 0)
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)length,
                        HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart6, (uint8_t *)buffer, (uint16_t)length,
                        HAL_MAX_DELAY);
    }
  }

  /* ========== END STARTUP SEQUENCE ========== */

  while (1)
  {
    DXL_Result base_result;
    DXL_Result shoulder_result;
    DXL_Result elbow_result;
    DXL_Result wrist_result;

    base_result = DXL_GetPresentPosition(&hdxl, BASE_ID, &base_position);
    shoulder_result = DXL_GetPresentPosition(&hdxl, SHOULDER_ID,
                                             &shoulder_position);
    elbow_result = DXL_GetPresentPosition(&hdxl, ELBOW_ID, &elbow_position);
    wrist_result = DXL_GetPresentPosition(&hdxl, WRIST_ID, &wrist_position);

    if ((base_result == DXL_OK) &&
        (shoulder_result == DXL_OK) &&
        (elbow_result == DXL_OK) &&
        (wrist_result == DXL_OK))
    {
      /* Apply home offset so home position displays as 0 degrees */
      float base_deg = DXL_PositionToAngle(base_position > HOME_BASE_POS ? 
                                           base_position - HOME_BASE_POS : 
                                           HOME_BASE_POS - base_position);
      float shoulder_deg = DXL_PositionToAngle(shoulder_position > HOME_SHOULDER_POS ? 
                                               shoulder_position - HOME_SHOULDER_POS : 
                                               HOME_SHOULDER_POS - shoulder_position);
      float elbow_deg = DXL_PositionToAngle(elbow_position > HOME_ELBOW_POS ? 
                                            elbow_position - HOME_ELBOW_POS : 
                                            HOME_ELBOW_POS - elbow_position);
      float wrist_deg = DXL_PositionToAngle(wrist_position > HOME_WRIST_POS ? 
                                            wrist_position - HOME_WRIST_POS : 
                                            HOME_WRIST_POS - wrist_position);
      length = snprintf(buffer, sizeof(buffer),
                        "Base:%.2f° Shoulder:%.2f° Elbow:%.2f° Wrist:%.2f°\r\n",
                        base_deg, shoulder_deg, elbow_deg, wrist_deg);
      HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin,
                        GPIO_PIN_RESET);
    }
    else
    {
      length = snprintf(buffer, sizeof(buffer),
                        "DXL error Base:%d Shoulder:%d Elbow:%d Wrist:%d\r\n",
                        (int)base_result,
                        (int)shoulder_result,
                        (int)elbow_result,
                        (int)wrist_result);
      HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
    }

    if (length > 0)
    {
      size_t transmit_length = (size_t)length;
      if (transmit_length >= sizeof(buffer))
      {
        transmit_length = sizeof(buffer) - 1U;
      }

      (void)HAL_UART_Transmit(&huart1, (uint8_t *)buffer,
                              (uint16_t)transmit_length, HAL_MAX_DELAY);
      (void)HAL_UART_Transmit(&huart6, (uint8_t *)buffer,
                              (uint16_t)transmit_length, HAL_MAX_DELAY);
    }

    HAL_Delay(100U);
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
