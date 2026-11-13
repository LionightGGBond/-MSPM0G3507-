/**
 * @file    App/robot_app.h
 * @brief   主循环提供给显示和日志模块的只读快照及接口声明。
 * @details 快照由empty.c在短临界区采集；所有驱动/状态初始化都在empty.c统一安排。
 */
#ifndef ROBOT_APP_H
#define ROBOT_APP_H
#include "../Motion/controller.h"
#include "../Bsp/encoder.h"
#include "../Bsp/board.h"
#include "../Bsp/tb6612.h"
#include <stdbool.h>

typedef struct {
    RobotState state;
    BoardTimerDiagnostics timer_diagnostics; /**< 初始化时保存的定时器检查结果。 */
    bool imu_ready;                          /**< IMU驱动是否初始化成功。 */
    uint8_t line_bits; /* 当前GPIO采样，故障停车时也更新，不是故障瞬间的锁存值。 */
    bool line_fallback;                 /**< 无效线型时保持目标的标志。 */
    float left_target, right_target;     /**< 目标速度，四倍频计数/10ms。 */
    float left_speed, right_speed;       /**< 实测速度，四倍频计数/10ms。 */
    float left_pwm, right_pwm;           /**< 计算的PWM命令，单位%。 */
    EncoderSample encoder;              /**< 编码器本拍增量、累计位置、跳变诊断。 */
    TB6612_InputLevels motor_inputs;     /**< 方向GPIO的输出锁存值。 */
    uint32_t now_ms;                     /**< 快照采集时刻，ms。 */
    uint32_t heartbeat_seconds, control_irq_count, control_dt_ms; /**< 心跳秒数、ISR次数、间隔ms。 */
    bool heartbeat_led_on;               /**< 心跳灯与OLED星号的共同状态。 */
} RobotSnapshot;

/**
 * @brief 根据一致性快照绘制OLED帧缓存，不执行I2C刷新。
 * @param[in] snapshot 由empty.c的Robot_GetSnapshot生成的只读快照。
 * @note 主循环随后逐页调用Oled_FlushPage；禁止在控制IRQ调用。
 */
void RobotApp_Render(const RobotSnapshot *snapshot);
/**
 * @brief 把底盘、IMU和硬件诊断格式化为中文文本，放入非阻塞UART队列。
 * @param[in] snapshot 由主循环读取的一致性快照。
 * @note 不修改控制状态；任务周期在empty.c顶部配置。
 */
void RobotApp_Report(const RobotSnapshot *snapshot);
#endif
