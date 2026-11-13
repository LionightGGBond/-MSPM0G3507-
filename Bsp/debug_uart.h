/**
 * @file    Bsp/debug_uart.h
 * @brief   调试串口的非阻塞读写与丢包诊断接口。
 * @details 串口命令解析在empty.c，中文日志格式化在App/robot_app.c。
 */
#ifndef ROBOT_DEBUG_UART_H
#define ROBOT_DEBUG_UART_H
#include <stdbool.h>
#include <stdint.h>

/* 仅主循环调用；SysConfig未添加DEBUG_UART时自动禁用，不占用其他串口。 */
/**
 * @brief 判断SysConfig是否提供DEBUG_UART实例。
 * @retval true已配置；false未配置，此时收发接口自动禁用。
 */
bool DebugUart_Enabled(void);
/**
 * @brief 把已排队字符送入UART TX FIFO，FIFO满立即返回。
 * @note 每次最多装32字节，仅主循环调用，无阻塞等待。
 */
void DebugUart_Poll(void);
/**
 * @brief 从RX FIFO尝试取一个字符，错误字节置零并统计。
 * @param[out] character 非空输出字符。
 * @retval true消费了一个字节；false无可读数据。
 */
bool DebugUart_Read(char *character);
/* 队列不足时整条丢弃并计数，绝不为等待串口阻塞电机控制。 */
/**
 * @brief 尝试把一整条文本加入非阻塞TX队列。
 * @param[in] text 以零结尾的UTF-8/ASCII文本。
 * @retval true已入队；false队列不足或UART未配置。
 * @note 空间不足整条丢弃，不把半个UTF-8字符放入队列。
 */
bool DebugUart_Write(const char *text);
/**
 * @brief 读取发送队列空间不足导致的累计整条丢弃次数。
 * @retval 发送丢弃计数。
 */
uint32_t DebugUart_Dropped(void);
/**
 * @brief 读取UART帧错误、奇偶、BREAK或溢出的累计计数。
 * @retval 接收错误计数。
 */
uint32_t DebugUart_RxErrors(void);
#endif
