#ifndef ROBOT_TB6612_H
#define ROBOT_TB6612_H
#include <stdbool.h>

typedef struct {
    bool ain1_high, ain2_high, bin1_high, bin2_high;
} TB6612_InputLevels;

void TB6612_Init(void);
/* 左轮=B/PWMB(CCP1)，右轮=A/PWMA(CCP0)，输入为有符号占空比百分数。 */
void TB6612_Set(float left_percent, float right_percent);
void TB6612_Brake(void);
/* 读取GPIO输出锁存值用于诊断，不是实测引脚电压。
 * 与控制快照一起调用时，调用者需用短临界区避免跨过换向/刹车时刻。 */
void TB6612_ReadInputLevels(TB6612_InputLevels *levels);
#endif
