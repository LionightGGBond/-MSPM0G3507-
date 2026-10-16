#ifndef ROBOT_ENCODER_H
#define ROBOT_ENCODER_H
#include <stdint.h>
typedef struct {
    int64_t left_position, right_position;
    int32_t left_delta, right_delta;
    uint32_t right_invalid_transitions;
} EncoderSample;
void Encoder_Init(void);
/* 仅在 10 ms 控制中断调用。右轮用极短临界区与 GPIO ISR 对齐。 */
void Encoder_Sample(EncoderSample *sample);
#endif
