/**
 * @file    Motion/quadrature.c
 * @brief   AB正交编码器四倍频状态表解码。
 * @details 由右轮GPIO中断调用；每个有效边沿返回±1，两位同时变化返回0。
 */
#include "quadrature.h"

/**
 * @brief 根据前后AB状态计算一次正交编码器边沿增量。
 * @param[in] previous_ab 前一AB状态，范围0..3。
 * @param[in] current_ab 当前AB状态，范围0..3。
 * @retval +1/-1有效边沿；0无变化或非法跳变。
 */
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
