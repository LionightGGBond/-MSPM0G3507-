/**
 * @file    Bsp/encoder.h
 * @brief   编码器10ms增量、64位累计位置及诊断数据接口。
 * @details 读取反馈的周期由empty.c控制中断决定；模块不自行创建采样任务。
 */
#ifndef ROBOT_ENCODER_H
#define ROBOT_ENCODER_H
#include <stdint.h>
typedef struct {
    int64_t left_position, right_position; /**< 四倍频累计计数；物理前进为正。 */
    int32_t left_delta, right_delta;       /**< 相邻10ms采样之间的计数增量。 */
    uint32_t right_invalid_transitions;    /**< 右轮AB同时跳两位的累计次数。 */
} EncoderSample;
/**
 * @brief 初始化左右计数历史，启动左QEI并开放右轮GPIO边沿中断。
 * @note 统一初始化时调用；不清零运行中的QEI来获得速度。
 */
void Encoder_Init(void);
/* 仅在 10 ms 控制中断调用。右轮用极短临界区与 GPIO ISR 对齐。 */
/**
 * @brief 读取两轮相邻采样差值和累计位置。
 * @param[out] sample 反馈样本，增量单位为计数/本次10ms窗口。
 * @note 只在控制IRQ调用；内部短临界区保证读取右轮计数一致。
 */
void Encoder_Sample(EncoderSample *sample);
#endif
