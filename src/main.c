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
#include "tim.h"
#include "dynamixel.h"
#include "laserping.h"
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

/* Set to 0 to restore the complete robot-arm application. */
#define SERVO_ONLY_TEST 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#if !SERVO_ONLY_TEST
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

/*
 * Machine-readable logger status bitmask. A value of zero means that the
 * LaserPING result and all four present-position reads are valid.
 */
#define LOGGER_STATUS_LASER_OUT_OF_RANGE  (1U << 0)
#define LOGGER_STATUS_LASER_SENSOR_ERROR  (1U << 1)
#define LOGGER_STATUS_LASER_TIMEOUT       (1U << 2)
#define LOGGER_STATUS_BASE_ERROR          (1U << 4)
#define LOGGER_STATUS_SHOULDER_ERROR      (1U << 5)
#define LOGGER_STATUS_ELBOW_ERROR         (1U << 6)
#define LOGGER_STATUS_WRIST_ERROR         (1U << 7)

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
#endif /* !SERVO_ONLY_TEST */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#if !SERVO_ONLY_TEST
static bool Robot_MoveToPose(const RobotPose *pose, uint32_t timeout_ms);
static bool Robot_PlayStartupAnimation(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if !SERVO_ONLY_TEST
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
#endif /* !SERVO_ONLY_TEST */
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
#if !SERVO_ONLY_TEST
  MX_TIM4_Init();
  LaserPING_Init(&htim4);
#endif

#if SERVO_ONLY_TEST
  /*
   * Breadboard test mode: MG90S only, with no LaserPING, Dynamixel
   * initialization, or robot movement. USART1 uses PB6/PB7 in this codebase.
  */
  char buffer[128];
  int length;
  uint16_t servo_pulse_us = 1000U;
  int16_t servo_step_us = 10;
  uint32_t last_servo_move_ms;

  /* Start near 0 degrees on PA6/TIM3_CH1. */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_pulse_us);
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  last_servo_move_ms = HAL_GetTick();

  HAL_Delay(100U);
  length = snprintf(buffer, sizeof(buffer),
                    "MG90S sweeping 0-180-0 degrees on PA6\r\n");
  if (length > 0)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buffer,
                            (uint16_t)length, HAL_MAX_DELAY);
    (void)HAL_UART_Transmit(&huart6, (uint8_t *)buffer,
                            (uint16_t)length, HAL_MAX_DELAY);
  }

  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* Smoothly sweep approximately 0 -> 180 -> 0 degrees. */
    if ((now - last_servo_move_ms) >= 20U)
    {
      int32_t next_pulse_us = (int32_t)servo_pulse_us + servo_step_us;
      const char *endpoint = NULL;

      if (next_pulse_us >= 2000)
      {
        next_pulse_us = 2000;
        servo_step_us = -10;
        endpoint = "180 degrees";
      }
      else if (next_pulse_us <= 1000)
      {
        next_pulse_us = 1000;
        servo_step_us = 10;
        endpoint = "0 degrees";
      }

      servo_pulse_us = (uint16_t)next_pulse_us;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_pulse_us);
      last_servo_move_ms = now;

      if (endpoint != NULL)
      {
        length = snprintf(buffer, sizeof(buffer), "MG90S: %s (%u us)\r\n",
                          endpoint, (unsigned int)servo_pulse_us);
        if (length > 0)
        {
          (void)HAL_UART_Transmit(&huart1, (uint8_t *)buffer,
                                  (uint16_t)length, HAL_MAX_DELAY);
          (void)HAL_UART_Transmit(&huart6, (uint8_t *)buffer,
                                  (uint16_t)length, HAL_MAX_DELAY);
        }
      }
    }

    HAL_Delay(1U);
  }
#else
  MX_USART2_UART_Init();

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

  /*
   * Start the first asynchronous LaserPING measurement. Data records use:
   * LP,timestamp_ms,q1_raw,q2_raw,q3_raw,q4_raw,distance_mm,status
   */
  (void)LaserPING_StartMeasurement();

  while (1)
  {
    DXL_Result base_result;
    DXL_Result shoulder_result;
    DXL_Result elbow_result;
    DXL_Result wrist_result;
    uint16_t laser_distance_mm;
    LaserPING_Status laser_status;

    LaserPING_Process();
    if (LaserPING_GetResult(&laser_distance_mm, &laser_status))
    {
      uint32_t sample_timestamp_ms = HAL_GetTick();
      uint16_t logger_status = 0U;

      /* Zero failed fields rather than publishing a previous sample as new. */
      base_position = 0U;
      shoulder_position = 0U;
      elbow_position = 0U;
      wrist_position = 0U;

      base_result = DXL_GetPresentPosition(&hdxl, BASE_ID, &base_position);
      shoulder_result = DXL_GetPresentPosition(&hdxl, SHOULDER_ID,
                                               &shoulder_position);
      elbow_result = DXL_GetPresentPosition(&hdxl, ELBOW_ID,
                                            &elbow_position);
      wrist_result = DXL_GetPresentPosition(&hdxl, WRIST_ID,
                                            &wrist_position);

      if (laser_status == LASERPING_STATUS_OUT_OF_RANGE)
      {
        logger_status |= LOGGER_STATUS_LASER_OUT_OF_RANGE;
      }
      else if (laser_status == LASERPING_STATUS_SENSOR_ERROR)
      {
        logger_status |= LOGGER_STATUS_LASER_SENSOR_ERROR;
      }
      else if (laser_status != LASERPING_STATUS_VALID)
      {
        logger_status |= LOGGER_STATUS_LASER_TIMEOUT;
      }

      if (base_result != DXL_OK)
      {
        logger_status |= LOGGER_STATUS_BASE_ERROR;
      }
      if (shoulder_result != DXL_OK)
      {
        logger_status |= LOGGER_STATUS_SHOULDER_ERROR;
      }
      if (elbow_result != DXL_OK)
      {
        logger_status |= LOGGER_STATUS_ELBOW_ERROR;
      }
      if (wrist_result != DXL_OK)
      {
        logger_status |= LOGGER_STATUS_WRIST_ERROR;
      }

      if (laser_status != LASERPING_STATUS_VALID)
      {
        laser_distance_mm = 0U;
      }

      length = snprintf(buffer, sizeof(buffer),
                        "LP,%lu,%u,%u,%u,%u,%u,%u\r\n",
                        (unsigned long)sample_timestamp_ms,
                        (unsigned int)base_position,
                        (unsigned int)shoulder_position,
                        (unsigned int)elbow_position,
                        (unsigned int)wrist_position,
                        (unsigned int)laser_distance_mm,
                        (unsigned int)logger_status);

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

      HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin,
                        (logger_status == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    /* The driver enforces the sensor's 65 ms minimum measurement interval. */
    (void)LaserPING_StartMeasurement();
    HAL_Delay(1U);
  }
#endif /* SERVO_ONLY_TEST */
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
