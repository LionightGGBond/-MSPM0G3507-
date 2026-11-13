/**
 * @file    Motion/quadrature.h
 * @brief   无硬件依赖的AB正交解码接口。
 * @details AB编码=(A<<1)|B；调用者自行统计非法跳变与累计位置。
 */
#ifndef ROBOT_QUADRATURE_H
#define ROBOT_QUADRATURE_H
#include <stdint.h>
/* AB 编码为 (A<<1)|B。非法跳变返回 0；调用者记录诊断计数。 */
/**
 * @brief 根据前后AB状态计算一次正交编码器边沿增量。
 * @param[in] previous_ab 前一AB状态，范围0..3。
 * @param[in] current_ab 当前AB状态，范围0..3。
 * @retval +1/-1有效边沿；0无变化或非法跳变。
 */
int8_t Quadrature_Delta(uint8_t previous_ab, uint8_t current_ab);
#endif
