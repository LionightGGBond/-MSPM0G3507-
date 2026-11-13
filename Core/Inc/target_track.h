#ifndef TARGET_TRACK_H
#define TARGET_TRACK_H

#include "stm32f1xx_hal.h"

/* 在两路 UART 初始化完成后调用一次。 */
HAL_StatusTypeDef TargetTrack_Init(UART_HandleTypeDef *vision_uart,
                                   UART_HandleTypeDef *motor_uart);

/* 主循环中持续调用，内部按固定周期更新两轴目标转速。 */
void TargetTrack_Task(void);

#endif
