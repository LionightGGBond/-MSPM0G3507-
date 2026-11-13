/**
 * @file    Motion/line_follow.c
 * @brief   五路数字循迹的九种标准线型与左右轮目标速度映射。
 * @details 顶部修改基础速度、首次无有效线型时的速度比例以及九档差速表。
 *          bit0为最左探头；输入由Board_LineBits统一转换成黑线=1。
 */
#include "line_follow.h"

/* ======================== 速度与差速表（优先调这里） ======================== */
/* 单位：四倍频编码器计数/10ms；不是PWM百分比，也不是RPM。 */
const float ROBOT_BASE_SPEED = 20.0f;
/* 从未识别到有效线型时，直行目标=基础速度×此比例。 */
const float ROBOT_LINE_FALLBACK_RATIO = 0.5f;

/* states按最左→中间→最右排列；bit0代表最左探头。
 * speeds每项依次为左轮、右轮，40代表100%基础目标。
 * 中线{40,40}；左偏降低左轮；右偏降低右轮。仅改变比例，不改变引脚。 */
static const uint8_t states[9] = {1, 3, 2, 6, 4, 12, 8, 24, 16};
static const uint8_t speeds[9][2] = {
    {24, 40}, /* 10000：黑线在最左，左轮60%，右轮100%。 */
    {27, 40}, /* 11000：左轮70%，右轮100%。 */
    {31, 40}, /* 01000：左轮80%，右轮100%。 */
    {36, 40}, /* 01100：左轮90%，右轮100%。 */

    {40, 40}, /* 00100：中线，两轮100%。 */
    
    {40, 36}, /* 00110：左轮100%，右轮90%。 */
    {40, 31}, /* 00010：左轮100%，右轮80%。 */
    {40, 27}, /* 00011：左轮100%，右轮70%。 */
    {40, 24}  /* 00001：黑线在最右，左轮100%，右轮60%。 */
};

/**
 * @brief 把标准线型映射为左右轮目标速度。
 * @param[in] state 五路位图，bit0最左，检测黑线为1。
 * @param[in] base 基础速度，单位四倍频计数/10ms。
 * @param[out] left 左轮目标。
 * @param[out] right 右轮目标。
 * @retval LINE_NORMAL 标准九档；LINE_LOST 全0；LINE_AMBIGUOUS 其他组合。
 * @note 无效线型把输出置0；是否保持旧目标由controller.c决定。
 */
LineResult LineFollow_Targets(uint8_t state, float base, float *left, float *right)
{
    *left = 0.0f;
    *right = 0.0f;
    for (unsigned int i = 0; i < 9U; ++i) {
        if (state == states[i]) {
            *left = base * (float)speeds[i][0] / 40.0f;
            *right = base * (float)speeds[i][1] / 40.0f;
            return LINE_NORMAL;
        }
    }
    return (state == 0U) ? LINE_LOST : LINE_AMBIGUOUS;
}
