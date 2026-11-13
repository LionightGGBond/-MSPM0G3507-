/**
 * @file    Motion/line_follow.h
 * @brief   循迹线型判断和差速目标接口。
 * @details 速度单位为四倍频编码器计数/10ms；不能把目标速度当作PWM百分比。
 */
#ifndef ROBOT_LINE_FOLLOW_H
#define ROBOT_LINE_FOLLOW_H
#include <stdint.h>

/* 调参位置：line_follow.c顶部。其他模块只读取，不重复定义。 */
extern const float ROBOT_BASE_SPEED;
extern const float ROBOT_LINE_FALLBACK_RATIO;

typedef enum {
    LINE_NORMAL = 0,
    LINE_LOST,
    LINE_AMBIGUOUS
} LineResult;

/* bit0=最左，bit4=最右。只接受原 STM32 的九种单线状态。
 * 多线/宽线/交叉线在五个数字输入上不可完全区分，统一报告歧义。 */
/**
 * @brief 把标准线型映射为左右轮目标速度。
 * @param[in] state 五路位图，bit0最左，检测黑线为1。
 * @param[in] base 基础速度，单位四倍频计数/10ms。
 * @param[out] left 左轮目标。
 * @param[out] right 右轮目标。
 * @retval LINE_NORMAL 标准九档；LINE_LOST 全0；LINE_AMBIGUOUS 其他组合。
 * @note 无效线型把输出置0；是否保持旧目标由controller.c决定。
 */
LineResult LineFollow_Targets(uint8_t state, float base, float *left, float *right);
#endif
