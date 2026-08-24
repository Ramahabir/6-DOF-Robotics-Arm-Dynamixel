#ifndef LASERPING_H
#define LASERPING_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  LASERPING_STATUS_IDLE = 0,
  LASERPING_STATUS_BUSY,
  LASERPING_STATUS_VALID,
  LASERPING_STATUS_OUT_OF_RANGE,
  LASERPING_STATUS_SENSOR_ERROR,
  LASERPING_STATUS_TIMEOUT
} LaserPING_Status;

void LaserPING_Init(TIM_HandleTypeDef *timer);
bool LaserPING_StartMeasurement(void);
void LaserPING_Process(void);
bool LaserPING_GetResult(uint16_t *distance_mm, LaserPING_Status *status);

#endif /* LASERPING_H */
