#include "laserping.h"

#define LASERPING_GPIO_PORT              GPIOB
#define LASERPING_GPIO_PIN               GPIO_PIN_8
#define LASERPING_TIMER_CHANNEL           TIM_CHANNEL_3
#define LASERPING_TIMER_ACTIVE_CHANNEL    HAL_TIM_ACTIVE_CHANNEL_3
#define LASERPING_TIMER_AF                GPIO_AF2_TIM4

#define LASERPING_TRIGGER_US              5U
#define LASERPING_RESPONSE_TIMEOUT_MS     20U
#define LASERPING_MIN_INTERVAL_MS         65U
#define LASERPING_MIN_VALID_PULSE_US      115U
#define LASERPING_MAX_VALID_PULSE_US      12000U
#define LASERPING_OUT_OF_RANGE_US         13000U
#define LASERPING_SENSOR_ERROR_US         14000U

typedef enum
{
  LASERPING_CAPTURE_IDLE = 0,
  LASERPING_CAPTURE_WAIT_RISE,
  LASERPING_CAPTURE_WAIT_FALL,
  LASERPING_CAPTURE_COMPLETE
} LaserPING_CaptureState;

static TIM_HandleTypeDef *laserping_timer;
static volatile LaserPING_CaptureState capture_state;
static volatile LaserPING_Status result_status;
static volatile uint16_t rising_count;
static volatile uint16_t measured_distance_mm;
static uint32_t trigger_time_ms;
static uint32_t previous_trigger_time_ms;
static bool has_triggered;

static void LaserPING_ConfigureSignalOutput(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* Set ODR low before enabling the output to avoid an unintended pulse. */
  HAL_GPIO_WritePin(LASERPING_GPIO_PORT, LASERPING_GPIO_PIN, GPIO_PIN_RESET);
  gpio.Pin = LASERPING_GPIO_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LASERPING_GPIO_PORT, &gpio);
}

static void LaserPING_ConfigureSignalCapture(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = LASERPING_GPIO_PIN;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = LASERPING_TIMER_AF;
  HAL_GPIO_Init(LASERPING_GPIO_PORT, &gpio);
}

void LaserPING_Init(TIM_HandleTypeDef *timer)
{
  laserping_timer = timer;
  capture_state = LASERPING_CAPTURE_IDLE;
  result_status = LASERPING_STATUS_IDLE;
  rising_count = 0U;
  measured_distance_mm = 0U;
  trigger_time_ms = 0U;
  previous_trigger_time_ms = 0U;
  has_triggered = false;

  LaserPING_ConfigureSignalCapture();
}

bool LaserPING_StartMeasurement(void)
{
  uint32_t now;

  if ((laserping_timer == NULL) ||
      (capture_state == LASERPING_CAPTURE_WAIT_RISE) ||
      (capture_state == LASERPING_CAPTURE_WAIT_FALL) ||
      (capture_state == LASERPING_CAPTURE_COMPLETE))
  {
    return false;
  }

  now = HAL_GetTick();
  if (has_triggered && ((now - previous_trigger_time_ms) < LASERPING_MIN_INTERVAL_MS))
  {
    return false;
  }

  (void)HAL_TIM_IC_Stop_IT(laserping_timer, LASERPING_TIMER_CHANNEL);
  LaserPING_ConfigureSignalOutput();

  /* TIM4 runs at 1 MHz, so one counter tick equals one microsecond. */
  __HAL_TIM_SET_COUNTER(laserping_timer, 0U);
  __HAL_TIM_CLEAR_FLAG(laserping_timer, TIM_FLAG_UPDATE | TIM_FLAG_CC3);
  __HAL_TIM_ENABLE(laserping_timer);
  HAL_GPIO_WritePin(LASERPING_GPIO_PORT, LASERPING_GPIO_PIN, GPIO_PIN_SET);
  while (__HAL_TIM_GET_COUNTER(laserping_timer) < LASERPING_TRIGGER_US)
  {
  }
  HAL_GPIO_WritePin(LASERPING_GPIO_PORT, LASERPING_GPIO_PIN, GPIO_PIN_RESET);
  __HAL_TIM_DISABLE(laserping_timer);

  /* Release SIG and reconnect PB8 to the TIM4_CH3 capture input. */
  LaserPING_ConfigureSignalCapture();
  __HAL_TIM_SET_COUNTER(laserping_timer, 0U);
  __HAL_TIM_SET_CAPTUREPOLARITY(laserping_timer, LASERPING_TIMER_CHANNEL,
                                TIM_INPUTCHANNELPOLARITY_RISING);
  __HAL_TIM_CLEAR_FLAG(laserping_timer, TIM_FLAG_UPDATE | TIM_FLAG_CC3);

  capture_state = LASERPING_CAPTURE_WAIT_RISE;
  result_status = LASERPING_STATUS_BUSY;
  trigger_time_ms = now;
  previous_trigger_time_ms = now;
  has_triggered = true;

  if (HAL_TIM_IC_Start_IT(laserping_timer, LASERPING_TIMER_CHANNEL) != HAL_OK)
  {
    capture_state = LASERPING_CAPTURE_COMPLETE;
    result_status = LASERPING_STATUS_SENSOR_ERROR;
    return false;
  }

  return true;
}

void LaserPING_Process(void)
{
  if (((capture_state == LASERPING_CAPTURE_WAIT_RISE) ||
       (capture_state == LASERPING_CAPTURE_WAIT_FALL)) &&
      ((HAL_GetTick() - trigger_time_ms) > LASERPING_RESPONSE_TIMEOUT_MS))
  {
    (void)HAL_TIM_IC_Stop_IT(laserping_timer, LASERPING_TIMER_CHANNEL);
    capture_state = LASERPING_CAPTURE_COMPLETE;
    result_status = LASERPING_STATUS_TIMEOUT;
  }
}

bool LaserPING_GetResult(uint16_t *distance_mm, LaserPING_Status *status)
{
  if ((capture_state != LASERPING_CAPTURE_COMPLETE) ||
      (distance_mm == NULL) || (status == NULL))
  {
    return false;
  }

  *distance_mm = measured_distance_mm;
  *status = result_status;
  capture_state = LASERPING_CAPTURE_IDLE;
  result_status = LASERPING_STATUS_IDLE;
  return true;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint16_t falling_count;
  uint16_t pulse_width_us;

  if ((htim != laserping_timer) ||
      (htim->Channel != LASERPING_TIMER_ACTIVE_CHANNEL))
  {
    return;
  }

  if (capture_state == LASERPING_CAPTURE_WAIT_RISE)
  {
    rising_count = (uint16_t)HAL_TIM_ReadCapturedValue(
        htim, LASERPING_TIMER_CHANNEL);
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, LASERPING_TIMER_CHANNEL,
                                  TIM_INPUTCHANNELPOLARITY_FALLING);
    capture_state = LASERPING_CAPTURE_WAIT_FALL;
    return;
  }

  if (capture_state != LASERPING_CAPTURE_WAIT_FALL)
  {
    return;
  }

  falling_count = (uint16_t)HAL_TIM_ReadCapturedValue(
      htim, LASERPING_TIMER_CHANNEL);
  pulse_width_us = (uint16_t)(falling_count - rising_count);
  (void)HAL_TIM_IC_Stop_IT(htim, LASERPING_TIMER_CHANNEL);

  if ((pulse_width_us >= LASERPING_MIN_VALID_PULSE_US) &&
      (pulse_width_us <= LASERPING_MAX_VALID_PULSE_US))
  {
    /* distance_mm = pulse_us * 171.5 / 1000, rounded to nearest mm. */
    measured_distance_mm =
        (uint16_t)(((uint32_t)pulse_width_us * 343U + 1000U) / 2000U);
    result_status = LASERPING_STATUS_VALID;
  }
  else if (pulse_width_us < LASERPING_OUT_OF_RANGE_US)
  {
    result_status = LASERPING_STATUS_OUT_OF_RANGE;
  }
  else if (pulse_width_us < LASERPING_SENSOR_ERROR_US)
  {
    result_status = LASERPING_STATUS_OUT_OF_RANGE;
  }
  else if (pulse_width_us < 15000U)
  {
    result_status = LASERPING_STATUS_SENSOR_ERROR;
  }
  else
  {
    result_status = LASERPING_STATUS_TIMEOUT;
  }

  capture_state = LASERPING_CAPTURE_COMPLETE;
}
