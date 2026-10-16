#include "debug_uart.h"
#include "board.h"
#include <string.h>

#ifdef DEBUG_UART_INST
#define TX_CAPACITY 2048U
static char s_tx[TX_CAPACITY];
static uint32_t s_read, s_write, s_used, s_dropped, s_rx_errors;
#endif

bool DebugUart_Enabled(void)
{
#ifdef DEBUG_UART_INST
    return true;
#else
    return false;
#endif
}

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

uint32_t DebugUart_Dropped(void)
{
#ifdef DEBUG_UART_INST
    return s_dropped;
#else
    return 0U;
#endif
}

uint32_t DebugUart_RxErrors(void)
{
#ifdef DEBUG_UART_INST
    return s_rx_errors;
#else
    return 0U;
#endif
}
