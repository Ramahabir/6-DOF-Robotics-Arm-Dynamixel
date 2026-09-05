#include "dynamixel.h"

#include <stddef.h>
#include <string.h>

#define DXL_HEADER                0xFFU
#define DXL_MAX_PARAMETERS        16U
#define DXL_MAX_PACKET_SIZE       (DXL_MAX_PARAMETERS + 6U)

/*
 * These values are deliberately longer than the AX-12A packet duration.
 * At 1 Mbps, one UART byte takes approximately 10 us with 8-N-1 framing.
 */
#define DXL_TX_TIMEOUT_MS         10U
#define DXL_RX_BYTE_TIMEOUT_MS    5U

static void DXL_SetTransmitMode(DXL_HandleTypeDef *dxl)
{
    HAL_GPIO_WritePin(
        dxl->dir_port,
        dxl->dir_pin,
        GPIO_PIN_SET
    );
}

static void DXL_SetReceiveMode(DXL_HandleTypeDef *dxl)
{
    HAL_GPIO_WritePin(
        dxl->dir_port,
        dxl->dir_pin,
        GPIO_PIN_RESET
    );
}

/*
 * Because STM32 RX is connected to the same DATA bus, it can receive an echo
 * while STM32 TX is driving the bus. Clear those bytes before reading the
 * AX-12A response.
 */
static void DXL_FlushUART(UART_HandleTypeDef *uart)
{
    while (__HAL_UART_GET_FLAG(uart, UART_FLAG_RXNE) != RESET)
    {
        volatile uint8_t discarded =
            (uint8_t)(uart->Instance->DR & 0xFFU);

        (void)discarded;
    }

    if (__HAL_UART_GET_FLAG(uart, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(uart);
    }

    if (__HAL_UART_GET_FLAG(uart, UART_FLAG_NE) != RESET)
    {
        __HAL_UART_CLEAR_NEFLAG(uart);
    }

    if (__HAL_UART_GET_FLAG(uart, UART_FLAG_FE) != RESET)
    {
        __HAL_UART_CLEAR_FEFLAG(uart);
    }
}

static uint8_t DXL_CalculateChecksum(const uint8_t *packet,
                                     uint16_t packet_size)
{
    uint8_t sum = 0U;

    /*
     * Skip the two FF header bytes.
     * Do not include the final checksum byte.
     */
    for (uint16_t i = 2U; i < packet_size - 1U; i++)
    {
        sum = (uint8_t)(sum + packet[i]);
    }

    return (uint8_t)(~sum);
}

static DXL_Result DXL_TransmitPacket(DXL_HandleTypeDef *dxl,
                                     const uint8_t *packet,
                                     uint16_t packet_size)
{
    if ((dxl == NULL) ||
        (dxl->uart == NULL) ||
        (packet == NULL) ||
        (packet_size < 6U))
    {
        return DXL_ERROR_ARGUMENT;
    }

    /*
     * Remove any old data before beginning another transaction.
     */
    DXL_FlushUART(dxl->uart);

    DXL_SetTransmitMode(dxl);

    /*
     * A few CPU cycles are enough for the external OE signal to settle.
     */
    __NOP();
    __NOP();
    __NOP();

    HAL_StatusTypeDef hal_status = HAL_UART_Transmit(
        dxl->uart,
        (uint8_t *)packet,
        packet_size,
        DXL_TX_TIMEOUT_MS
    );

    if (hal_status != HAL_OK)
    {
        DXL_SetReceiveMode(dxl);
        return DXL_ERROR_UART;
    }

    /*
     * HAL_UART_Transmit waits for transmission complete in blocking mode,
     * but explicitly checking TC makes the direction transition clear:
     * do not release the buffer before the final stop bit has left USART.
     */
    while (__HAL_UART_GET_FLAG(dxl->uart, UART_FLAG_TC) == RESET)
    {
    }

    /*
     * Clear our transmitted echo before enabling the servo reply direction.
     * This must remain short because the AX-12A replies after Return Delay.
     */
    DXL_FlushUART(dxl->uart);

    DXL_SetReceiveMode(dxl);

    return DXL_OK;
}

static DXL_Result DXL_ReceiveStatusPacket(DXL_HandleTypeDef *dxl,
                                          uint8_t expected_id,
                                          uint8_t *parameters,
                                          uint8_t parameters_capacity,
                                          uint8_t *parameters_received)
{
    if ((dxl == NULL) ||
        (dxl->uart == NULL) ||
        (parameters_received == NULL))
    {
        return DXL_ERROR_ARGUMENT;
    }

    *parameters_received = 0U;

    uint8_t byte = 0U;
    uint8_t previous = 0U;

    /*
     * Find FF FF header. Searching rather than assuming alignment makes the
     * receiver more tolerant of stale bytes or line noise.
     */
    uint32_t search_start = HAL_GetTick();

    while ((HAL_GetTick() - search_start) < DXL_RX_BYTE_TIMEOUT_MS)
    {
        if (HAL_UART_Receive(dxl->uart,
                             &byte,
                             1U,
                             1U) != HAL_OK)
        {
            continue;
        }

        if ((previous == DXL_HEADER) && (byte == DXL_HEADER))
        {
            break;
        }

        previous = byte;
    }

    if (!((previous == DXL_HEADER) && (byte == DXL_HEADER)))
    {
        return DXL_ERROR_TIMEOUT;
    }

    uint8_t id = 0U;
    uint8_t length = 0U;

    if (HAL_UART_Receive(dxl->uart,
                         &id,
                         1U,
                         DXL_RX_BYTE_TIMEOUT_MS) != HAL_OK)
    {
        return DXL_ERROR_TIMEOUT;
    }

    if (HAL_UART_Receive(dxl->uart,
                         &length,
                         1U,
                         DXL_RX_BYTE_TIMEOUT_MS) != HAL_OK)
    {
        return DXL_ERROR_TIMEOUT;
    }

    /*
     * Status packet LENGTH includes:
     *   Error byte + parameter bytes + checksum byte
     *
     * Therefore:
     *   parameter_count = LENGTH - 2
     */
    if (length < 2U)
    {
        return DXL_ERROR_LENGTH;
    }

    uint8_t parameter_count = (uint8_t)(length - 2U);

    if (parameter_count > parameters_capacity)
    {
        return DXL_ERROR_LENGTH;
    }

    uint8_t error = 0U;

    if (HAL_UART_Receive(dxl->uart,
                         &error,
                         1U,
                         DXL_RX_BYTE_TIMEOUT_MS) != HAL_OK)
    {
        return DXL_ERROR_TIMEOUT;
    }

    if (parameter_count > 0U)
    {
        if (parameters == NULL)
        {
            return DXL_ERROR_ARGUMENT;
        }

        if (HAL_UART_Receive(dxl->uart,
                             parameters,
                             parameter_count,
                             DXL_RX_BYTE_TIMEOUT_MS) != HAL_OK)
        {
            return DXL_ERROR_TIMEOUT;
        }
    }

    uint8_t received_checksum = 0U;

    if (HAL_UART_Receive(dxl->uart,
                         &received_checksum,
                         1U,
                         DXL_RX_BYTE_TIMEOUT_MS) != HAL_OK)
    {
        return DXL_ERROR_TIMEOUT;
    }

    uint8_t checksum_sum = 0U;

    checksum_sum = (uint8_t)(checksum_sum + id);
    checksum_sum = (uint8_t)(checksum_sum + length);
    checksum_sum = (uint8_t)(checksum_sum + error);

    for (uint8_t i = 0U; i < parameter_count; i++)
    {
        checksum_sum = (uint8_t)(checksum_sum + parameters[i]);
    }

    uint8_t calculated_checksum = (uint8_t)(~checksum_sum);

    if (received_checksum != calculated_checksum)
    {
        return DXL_ERROR_CHECKSUM;
    }

    if (id != expected_id)
    {
        return DXL_ERROR_ID;
    }

    dxl->last_servo_error = error;
    *parameters_received = parameter_count;

    if (error != 0U)
    {
        return DXL_ERROR_SERVO;
    }

    return DXL_OK;
}

static DXL_Result DXL_SendInstruction(DXL_HandleTypeDef *dxl,
                                      uint8_t id,
                                      uint8_t instruction,
                                      const uint8_t *parameters,
                                      uint8_t parameter_count,
                                      uint8_t *response_parameters,
                                      uint8_t response_capacity,
                                      uint8_t *response_count)
{
    if ((dxl == NULL) ||
        (parameter_count > DXL_MAX_PARAMETERS) ||
        ((parameter_count > 0U) && (parameters == NULL)))
    {
        return DXL_ERROR_ARGUMENT;
    }

    uint8_t packet[DXL_MAX_PACKET_SIZE] = {0};

    uint8_t length = (uint8_t)(parameter_count + 2U);
    uint16_t packet_size = (uint16_t)(parameter_count + 6U);

    packet[0] = DXL_HEADER;
    packet[1] = DXL_HEADER;
    packet[2] = id;
    packet[3] = length;
    packet[4] = instruction;

    if (parameter_count > 0U)
    {
        memcpy(&packet[5], parameters, parameter_count);
    }

    packet[packet_size - 1U] =
        DXL_CalculateChecksum(packet, packet_size);

    DXL_Result result =
        DXL_TransmitPacket(dxl, packet, packet_size);

    if (result != DXL_OK)
    {
        return result;
    }

    /*
     * Broadcast instructions normally produce no status packet.
     */
    if (id == DXL_BROADCAST_ID)
    {
        if (response_count != NULL)
        {
            *response_count = 0U;
        }

        return DXL_OK;
    }

    if (response_count == NULL)
    {
        return DXL_ERROR_ARGUMENT;
    }

    return DXL_ReceiveStatusPacket(
        dxl,
        id,
        response_parameters,
        response_capacity,
        response_count
    );
}

void DXL_Init(DXL_HandleTypeDef *dxl,
              UART_HandleTypeDef *uart,
              GPIO_TypeDef *dir_port,
              uint16_t dir_pin)
{
    if (dxl == NULL)
    {
        return;
    }

    dxl->uart = uart;
    dxl->dir_port = dir_port;
    dxl->dir_pin = dir_pin;
    dxl->last_servo_error = 0U;

    /*
     * Start with the bus released so the LS241 output is High-Z.
     */
    DXL_SetReceiveMode(dxl);
    DXL_FlushUART(uart);
}

DXL_Result DXL_Ping(DXL_HandleTypeDef *dxl, uint8_t id)
{
    uint8_t response_count = 0U;

    return DXL_SendInstruction(
        dxl,
        id,
        DXL_INST_PING,
        NULL,
        0U,
        NULL,
        0U,
        &response_count
    );
}

DXL_Result DXL_Write8(DXL_HandleTypeDef *dxl,
                      uint8_t id,
                      uint8_t address,
                      uint8_t value)
{
    uint8_t parameters[2];

    parameters[0] = address;
    parameters[1] = value;

    uint8_t response_count = 0U;

    return DXL_SendInstruction(
        dxl,
        id,
        DXL_INST_WRITE,
        parameters,
        sizeof(parameters),
        NULL,
        0U,
        &response_count
    );
}

DXL_Result DXL_Write16(DXL_HandleTypeDef *dxl,
                       uint8_t id,
                       uint8_t address,
                       uint16_t value)
{
    uint8_t parameters[3];

    parameters[0] = address;

    /*
     * Protocol 1.0 multi-byte control-table values are little-endian:
     * low byte first, then high byte.
     */
    parameters[1] = (uint8_t)(value & 0xFFU);
    parameters[2] = (uint8_t)((value >> 8U) & 0xFFU);

    uint8_t response_count = 0U;

    return DXL_SendInstruction(
        dxl,
        id,
        DXL_INST_WRITE,
        parameters,
        sizeof(parameters),
        NULL,
        0U,
        &response_count
    );
}

DXL_Result DXL_Read8(DXL_HandleTypeDef *dxl,
                     uint8_t id,
                     uint8_t address,
                     uint8_t *value)
{
    if (value == NULL)
    {
        return DXL_ERROR_ARGUMENT;
    }

    uint8_t request_parameters[2];

    request_parameters[0] = address;
    request_parameters[1] = 1U;

    uint8_t response[1] = {0};
    uint8_t response_count = 0U;

    DXL_Result result = DXL_SendInstruction(
        dxl,
        id,
        DXL_INST_READ,
        request_parameters,
        sizeof(request_parameters),
        response,
        sizeof(response),
        &response_count
    );

    if (result != DXL_OK)
    {
        return result;
    }

    if (response_count != 1U)
    {
        return DXL_ERROR_LENGTH;
    }

    *value = response[0];

    return DXL_OK;
}

DXL_Result DXL_Read16(DXL_HandleTypeDef *dxl,
                      uint8_t id,
                      uint8_t address,
                      uint16_t *value)
{
    if (value == NULL)
    {
        return DXL_ERROR_ARGUMENT;
    }

    uint8_t request_parameters[2];

    request_parameters[0] = address;
    request_parameters[1] = 2U;

    uint8_t response[2] = {0};
    uint8_t response_count = 0U;

    DXL_Result result = DXL_SendInstruction(
        dxl,
        id,
        DXL_INST_READ,
        request_parameters,
        sizeof(request_parameters),
        response,
        sizeof(response),
        &response_count
    );

    if (result != DXL_OK)
    {
        return result;
    }

    if (response_count != 2U)
    {
        return DXL_ERROR_LENGTH;
    }

    *value = (uint16_t)response[0] |
             ((uint16_t)response[1] << 8U);

    return DXL_OK;
}

DXL_Result DXL_SetTorque(DXL_HandleTypeDef *dxl,
                         uint8_t id,
                         bool enabled)
{
    return DXL_Write8(
        dxl,
        id,
        DXL_ADDR_TORQUE_ENABLE,
        enabled ? 1U : 0U
    );
}

DXL_Result DXL_SetLED(DXL_HandleTypeDef *dxl,
                      uint8_t id,
                      bool enabled)
{
    return DXL_Write8(
        dxl,
        id,
        DXL_ADDR_LED,
        enabled ? 1U : 0U
    );
}

DXL_Result DXL_SetGoalPosition(DXL_HandleTypeDef *dxl,
                               uint8_t id,
                               uint16_t position)
{
    /*
     * AX-12A position range is 0..1023 in joint mode.
     */
    if (position > 1023U)
    {
        return DXL_ERROR_ARGUMENT;
    }

    return DXL_Write16(
        dxl,
        id,
        DXL_ADDR_GOAL_POSITION,
        position
    );
}

DXL_Result DXL_SetMovingSpeed(DXL_HandleTypeDef *dxl,
                              uint8_t id,
                              uint16_t speed)
{
    if (speed > 1023U)
    {
        return DXL_ERROR_ARGUMENT;
    }

    return DXL_Write16(
        dxl,
        id,
        DXL_ADDR_MOVING_SPEED,
        speed
    );
}

DXL_Result DXL_GetPresentPosition(DXL_HandleTypeDef *dxl,
                                  uint8_t id,
                                  uint16_t *position)
{
    return DXL_Read16(
        dxl,
        id,
        DXL_ADDR_PRESENT_POSITION,
        position
    );
}

DXL_Result DXL_GetPresentSpeed(DXL_HandleTypeDef *dxl,
                               uint8_t id,
                               uint16_t *speed)
{
    return DXL_Read16(
        dxl,
        id,
        DXL_ADDR_PRESENT_SPEED,
        speed
    );
}

DXL_Result DXL_GetPresentVoltage(DXL_HandleTypeDef *dxl,
                                 uint8_t id,
                                 uint8_t *voltage_tenths)
{
    uint8_t value = 0U;
    DXL_Result result = DXL_Read8(
        dxl,
        id,
        DXL_ADDR_PRESENT_VOLTAGE,
        &value
    );

    if (result != DXL_OK)
    {
        return result;
    }

    if (voltage_tenths != NULL)
    {
        *voltage_tenths = value;
    }

    return DXL_OK;
}

DXL_Result DXL_GetPresentTemperature(DXL_HandleTypeDef *dxl,
                                     uint8_t id,
                                     uint8_t *temperature)
{
    return DXL_Read8(
        dxl,
        id,
        DXL_ADDR_PRESENT_TEMP,
        temperature
    );
}

DXL_Result DXL_GetPresentLoad(DXL_HandleTypeDef *dxl,
                              uint8_t id,
                              uint16_t *load)
{
    return DXL_Read16(
        dxl,
        id,
        DXL_ADDR_PRESENT_LOAD,
        load
    );
}

DXL_Result DXL_IsMoving(DXL_HandleTypeDef *dxl,
                        uint8_t id,
                        bool *is_moving)
{
    uint8_t value = 0U;
    DXL_Result result = DXL_Read8(
        dxl,
        id,
        DXL_ADDR_MOVING,
        &value
    );

    if (result != DXL_OK)
    {
        return result;
    }

    if (is_moving != NULL)
    {
        *is_moving = (value != 0U);
    }

    return DXL_OK;
}

float DXL_PositionToAngle(uint16_t position)
{
    return ((float)position * 300.0f / 1023.0f);
}
