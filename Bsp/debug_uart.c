/**
 * @file    Bsp/debug_uart.c
 * @brief   Type-C调试串口的非阻塞发送队列和字符接收。
 * @details UART0/PA10/PA11与115200配置在SysConfig；顶部设置TX队列容量。
 *          主循环周期性调用Poll，FIFO满立即返回；不使用UART中断。
 */
#include "debug_uart.h"
#include "board.h"
#include <string.h>

#ifdef DEBUG_UART_INST
#define TX_CAPACITY 2048U /* 非阻塞发送队列字节数，须大于单组日志长度。 */
static char s_tx[TX_CAPACITY];
static uint32_t s_read, s_write, s_used, s_dropped, s_rx_errors;
#endif

/**
 * @brief 判断SysConfig是否提供DEBUG_UART实例。
 * @retval true已配置；false未配置，此时收发接口自动禁用。
 */
bool DebugUart_Enabled(void)
{
#ifdef DEBUG_UART_INST
    return true;
#else
    return false;
#endif
}

/**
 * @brief 把已排队字符送入UART TX FIFO，FIFO满立即返回。
 * @note 每次最多装32字节，仅主循环调用，无阻塞等待。
 */
void DebugUart_Poll(void)
{
#ifdef DEBUG_UART_INST
    /* 每次最多装32字节，FIFO满立即返回；不用发送完成中断。 */
    for (unsigned int n = 0U; n < 32U && s_used != 0U; ++n) {
        if (DL_UART_Main_isTXFIFOFull(DEBUG_UART_INST)) break;
        DL_UART_Main_transmitData(DEBUG_UART_INST, (uint8_t)s_tx[s_read]);
        s_read = (s_read + 1U) % TX_CAPACITY;
        --s_used;
    }
#endif
}

/**
 * @brief 从RX FIFO尝试取一个字符，错误字节置零并统计。
 * @param[out] character 非空输出字符。
 * @retval true消费了一个字节；false无可读数据。
 */
bool DebugUart_Read(char *character)
{
#ifdef DEBUG_UART_INST
    if (DL_UART_Main_isRXFIFOEmpty(DEBUG_UART_INST)) return false;
    /* RXDATA只能读一次：读取错误标志同样会弹出FIFO，不能先查错再收数据。 */
    uint32_t received = DEBUG_UART_INST->RXDATA;
    uint32_t errors = DL_UART_ERROR_OVERRUN | DL_UART_ERROR_BREAK |
                      DL_UART_ERROR_PARITY | DL_UART_ERROR_FRAMING;
    if ((received & errors) != 0U) {
        ++s_rx_errors;
        *character = '\0';
    } else {
        *character = (char)(received & UART_RXDATA_DATA_MASK);
    }
    return true;
#else
    (void)character;
    return false;
#endif
}

/**
 * @brief 尝试把一整条文本加入非阻塞TX队列。
 * @param[in] text 以零结尾的UTF-8/ASCII文本。
 * @retval true已入队；false队列不足或UART未配置。
 * @note 空间不足整条丢弃，不把半个UTF-8字符放入队列。
 */
bool DebugUart_Write(const char *text)
{
#ifdef DEBUG_UART_INST
    size_t length = strlen(text);
    if (length > TX_CAPACITY - s_used) {
        ++s_dropped;
        return false;
    }
    for (size_t n = 0U; n < length; ++n) {
        s_tx[s_write] = text[n];
        s_write = (s_write + 1U) % TX_CAPACITY;
    }
    s_used += (uint32_t)length;
    return true;
#else
    (void)text;
    return false;
#endif
}

/**
 * @brief 读取发送队列空间不足导致的累计整条丢弃次数。
 * @retval 发送丢弃计数。
 */
uint32_t DebugUart_Dropped(void)
{
#ifdef DEBUG_UART_INST
    return s_dropped;
#else
    return 0U;
#endif
}

/**
 * @brief 读取UART帧错误、奇偶、BREAK或溢出的累计计数。
 * @retval 接收错误计数。
 */
uint32_t DebugUart_RxErrors(void)
{
#ifdef DEBUG_UART_INST
    return s_rx_errors;
#else
    return 0U;
#endif
}
