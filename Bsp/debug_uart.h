#ifndef ROBOT_DEBUG_UART_H
#define ROBOT_DEBUG_UART_H
#include <stdbool.h>
#include <stdint.h>

/* 仅主循环调用；SysConfig未添加DEBUG_UART时自动禁用，不占用其他串口。 */
bool DebugUart_Enabled(void);
void DebugUart_Poll(void);
bool DebugUart_Read(char *character);
/* 队列不足时整条丢弃并计数，绝不为等待串口阻塞电机控制。 */
bool DebugUart_Write(const char *text);
uint32_t DebugUart_Dropped(void);
uint32_t DebugUart_RxErrors(void);
#endif
