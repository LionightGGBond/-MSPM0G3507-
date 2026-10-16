#include "line_follow.h"

LineResult LineFollow_Targets(uint8_t state, float base, float *left, float *right)
{
    static const uint8_t states[9] = {1, 3, 2, 6, 4, 12, 8, 24, 16};
    static const uint8_t speeds[9][2] = {
        {24, 40}, {28, 40}, {32, 40}, {36, 40}, {40, 40},
        {40, 36}, {40, 32}, {40, 28}, {40, 24}
    };
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
