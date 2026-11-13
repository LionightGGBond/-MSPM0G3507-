#ifndef F32C_MOTOR_H
#define F32C_MOTOR_H

#include "stm32f1xx_hal.h"

/* 启动指定地址的电机，切换到速度模式并将目标转速清零。 */
HAL_StatusTypeDef F32C_StartSpeedMode(UART_HandleTypeDef *uart, uint8_t address);

/* 设置有符号整数转速，单位 RPM。驱动发送前按云台上限限幅。 */
HAL_StatusTypeDef F32C_SetSpeed(UART_HandleTypeDef *uart, uint8_t address, int16_t rpm);

#endif
