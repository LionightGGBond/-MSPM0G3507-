#ifndef ROBOT_LINE_FOLLOW_H
#define ROBOT_LINE_FOLLOW_H
#include <stdint.h>

typedef enum {
    LINE_NORMAL = 0,
    LINE_LOST,
    LINE_AMBIGUOUS
} LineResult;

/* bit0=最左，bit4=最右。只接受原 STM32 的九种单线状态。
 * 多线/宽线/交叉线在五个数字输入上不可完全区分，统一报告歧义。 */
LineResult LineFollow_Targets(uint8_t state, float base, float *left, float *right);
#endif
