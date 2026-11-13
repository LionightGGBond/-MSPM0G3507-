#include "f32c_motor.h"

/* ==================== 云台转速指令上限 ==================== */
#define F32C_MAX_COMMAND_RPM      40    /* 两轴共用上限，限制到 ±40 RPM。 */

/* ==================== 串口协议与时序 ==================== */
#define F32C_HEADER               0x7AU
#define F32C_TAIL                 0x7BU
#define F32C_CMD_MODE             0x00U
#define F32C_CMD_SPEED            0x01U
#define F32C_CMD_ENABLE           0x06U
#define F32C_MODE_SPEED           0x0000U
#define F32C_FRAME_GAP_MS         1U
#define F32C_TX_TIMEOUT_MS        3U

/* ==================== 完整帧发送 ==================== */
static HAL_StatusTypeDef F32C_Send(UART_HandleTypeDef *uart, uint8_t address,
                                  uint8_t command, uint16_t data, uint8_t has_data)
{
    uint8_t frame[7] = {F32C_HEADER, address, command};
    uint16_t length = 3U;

    if (has_data != 0U)
    {
        frame[length++] = (uint8_t)(data >> 8);
        frame[length++] = (uint8_t)data;
    }

    /* BCC 为从帧头到最后一个数据字节的异或。 */
    uint8_t bcc = 0U;
    for (uint16_t index = 0U; index < length; ++index)
    {
        bcc ^= frame[index];
    }
    frame[length++] = bcc;
    frame[length++] = F32C_TAIL;

    /* 电机用串口空闲判断帧结束。USART2 由本驱动独占，接口在主循环调用。 */
    HAL_StatusTypeDef status = HAL_UART_Transmit(uart, frame, length, F32C_TX_TIMEOUT_MS);
    HAL_Delay(F32C_FRAME_GAP_MS);
    return status;
}

/* ==================== 速度模式接口 ==================== */
HAL_StatusTypeDef F32C_SetSpeed(UART_HandleTypeDef *uart, uint8_t address, int16_t rpm)
{
    if (rpm > F32C_MAX_COMMAND_RPM)
    {
        rpm = F32C_MAX_COMMAND_RPM;
    }
    else if (rpm < -F32C_MAX_COMMAND_RPM)
    {
        rpm = -F32C_MAX_COMMAND_RPM;
    }

    /* 转为 16 位无符号值后保留负转速的补码位型。 */
    return F32C_Send(uart, address, F32C_CMD_SPEED, (uint16_t)rpm, 1U);
}

HAL_StatusTypeDef F32C_StartSpeedMode(UART_HandleTypeDef *uart, uint8_t address)
{
    HAL_StatusTypeDef status = F32C_Send(uart, address, F32C_CMD_ENABLE, 0U, 0U);
    if (status != HAL_OK)
    {
        return status;
    }

    status = F32C_Send(uart, address, F32C_CMD_MODE, F32C_MODE_SPEED, 1U);
    if (status != HAL_OK)
    {
        return status;
    }

    return F32C_SetSpeed(uart, address, 0);
}
