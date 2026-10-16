#include "quadrature.h"

int8_t Quadrature_Delta(uint8_t previous_ab, uint8_t current_ab)
{
    /* 一个完整 AB 周期累计 ±4，与 STM32 TIM_ENCODERMODE_TI12 对齐。
     * 00→01→11→10→00 为正；反向为负；同时改变两位属于漏边沿/干扰。 */
    static const int8_t table[16] = {
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0
    };
    return table[((previous_ab & 3U) << 2) | (current_ab & 3U)];
}
