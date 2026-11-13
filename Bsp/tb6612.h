/**
 * @file    Bsp/tb6612.h
 * @brief   TB6612初始化、带符号PWM输出、制动和方向锁存诊断接口。
 * @details 驱动层不决定循迹策略；正常电机指令只从empty.c控制中断写入。
 */
#ifndef ROBOT_TB6612_H
#define ROBOT_TB6612_H
#include <stdbool.h>

typedef struct {
    bool ain1_high, ain2_high, bin1_high, bin2_high;
} TB6612_InputLevels;

/**
 * @brief 先建立两轮制动，再启动电机PWM定时器。
 * @note 在控制中断启用之前调用。
 */
void TB6612_Init(void);
/* 左轮=B/PWMB(CCP1)，右轮=A/PWMA(CCP0)，输入为有符号占空比百分数。 */
/**
 * @brief 设置两轮有符号PWM命令。
 * @param[in] left_percent 左轮命令%，左轮使用B/PWMB。
 * @param[in] right_percent 右轮命令%，右轮使用A/PWMA。
 * @note 正常运行只由empty.c的控制IRQ调用；函数内部进行死区和限幅处理。
 */
void TB6612_Set(float left_percent, float right_percent);
/**
 * @brief 两轮执行短路制动，复位方向切换历史。
 * @note 不使用STBY脚；电机机械停止需要实际减速时间。
 */
void TB6612_Brake(void);
/* 读取GPIO输出锁存值用于诊断，不是实测引脚电压。
 * 与控制快照一起调用时，调用者需用短临界区避免跨过换向/刹车时刻。 */
/**
 * @brief 读取四个方向GPIO的输出锁存值。
 * @param[out] levels AIN1/AIN2/BIN1/BIN2的逻辑高低状态。
 * @note 这是软件输出寄存器，不等同于万用表实测电压；主循环在临界区中读取。
 */
void TB6612_ReadInputLevels(TB6612_InputLevels *levels);
#endif
