#ifndef INC_DYNAMIXEL_H_
#define INC_DYNAMIXEL_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

// Protocol 1.0 instructions
#define DXL_INST_PING       0x01U
#define DXL_INST_READ       0x02U
#define DXL_INST_WRITE      0x03U

// AX-12A control-table addresses
#define DXL_ADDR_TORQUE_ENABLE          24U
#define DXL_ADDR_LED                    25U
#define DXL_ADDR_CW_MARGIN              26U
#define DXL_ADDR_CCW_MARGIN             27U
#define DXL_ADDR_CW_SLOPE               28U
#define DXL_ADDR_CCW_SLOPE              29U
#define DXL_ADDR_GOAL_POSITION          30U
#define DXL_ADDR_MOVING_SPEED           32U
#define DXL_ADDR_PRESENT_POSITION       36U
#define DXL_ADDR_PRESENT_SPEED          38U
#define DXL_ADDR_PRESENT_LOAD           40U
#define DXL_ADDR_PRESENT_VOLTAGE        42U
#define DXL_ADDR_PRESENT_TEMP           43U
#define DXL_ADDR_MOVING                 46U

// Broadcast ID does not generate an ordinary status response
#define DXL_BROADCAST_ID                0xFEU

typedef enum{
    DXL_OK = 0,
    DXL_ERROR_ARGUMENT,
    DXL_ERROR_UART,
    DXL_ERROR_TIMEOUT,
    DXL_ERROR_HEADER,
    DXL_ERROR_ID,
    DXL_ERROR_LENGTH,
    DXL_ERROR_CHECKSUM,
    DXL_ERROR_SERVO
} DXL_Result;

typedef struct{
    UART_HandleTypeDef *uart;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    uint8_t last_servo_error;
} DXL_HandleTypeDef;

void DXL_Init(
    DXL_HandleTypeDef *dxl,
    UART_HandleTypeDef *uart,
    GPIO_TypeDef *dir_port,
    uint16_t dir_pin
);

float DXL_PositionToAngle(uint16_t position);

DXL_Result DXL_Ping(DXL_HandleTypeDef *dxl, uint8_t id);

DXL_Result DXL_Write8(DXL_HandleTypeDef *dxl, uint8_t id, uint8_t address, uint8_t value);

DXL_Result DXL_Write16(DXL_HandleTypeDef *dxl, uint8_t id, uint8_t address, uint16_t value);

DXL_Result DXL_Read8(DXL_HandleTypeDef *dxl, uint8_t id, uint8_t address, uint8_t *value);

DXL_Result DXL_Read16(DXL_HandleTypeDef *dxl, uint8_t id, uint8_t address, uint16_t *value);

DXL_Result DXL_SetTorque(DXL_HandleTypeDef *dxl, uint8_t id, bool enabled);

DXL_Result DXL_SetLED(DXL_HandleTypeDef *dxl, uint8_t id, bool enabled);

DXL_Result DXL_SetGoalPosition(DXL_HandleTypeDef *dxl, uint8_t id, uint16_t position);

DXL_Result DXL_SetMovingSpeed(DXL_HandleTypeDef *dxl, uint8_t id, uint16_t speed);

DXL_Result DXL_GetPresentPosition(DXL_HandleTypeDef *dxl, uint8_t id, uint16_t *position);

DXL_Result DXL_GetPresentSpeed(DXL_HandleTypeDef *dxl, uint8_t id, uint16_t *speed);

DXL_Result DXL_GetPresentVoltage(DXL_HandleTypeDef *dxl, uint8_t id, uint8_t *voltage_tenths);

DXL_Result DXL_GetPresentTemperature(DXL_HandleTypeDef *dxl, uint8_t id, uint8_t *temperature);

DXL_Result DXL_GetPresentLoad(DXL_HandleTypeDef *dxl, uint8_t id, uint16_t *load);

DXL_Result DXL_IsMoving(DXL_HandleTypeDef *dxl, uint8_t id, bool *is_moving);
#endif

