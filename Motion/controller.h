/**
 * @file    Motion/controller.h
 * @brief   底盘控制状态、运行数据和固定周期控制接口。
 * @details 控制器对象由empty.c持有；控制中断更新，主循环通过一致性快照读取。
 */
#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ROBOT_WAIT = 0, ROBOT_RUNNING, ROBOT_FINISHED,
    ROBOT_TEST_RUNNING, ROBOT_TEST_STOPPED, ROBOT_TEST_PAUSED,
    ROBOT_LINE_LOST, ROBOT_LINE_AMBIGUOUS,
    ROBOT_ENCODER_STALL, ROBOT_MAIN_TIMEOUT, ROBOT_TIMING_ERROR
} RobotState;

typedef struct {
    Pid left_pid, right_pid;             /**< 双轮各自的速度PID历史。 */
    RobotState state;                   /**< 当前运行状态或锁存的停止原因。 */
    uint32_t start_ms, run_start_ms, last_step_ms; /**< 初始化、运行、上拍时间，ms。 */
    uint32_t left_stall_ms, right_stall_ms; /**< 连续无反馈诊断累计，ms。 */
    volatile bool test_pause_requested;  /**< 主循环发出、控制ISR执行的测试暂停请求。 */
    uint8_t line_bits;                   /**< 黑线=1，bit0为最左传感器。 */
    bool line_fallback; /* 当前线型无效，正在沿用最近差速或首次低速直行。 */
    float left_target, right_target;     /**< 目标速度，四倍频计数/10ms。 */
    float left_speed, right_speed;       /**< 本拍反馈速度，四倍频计数/10ms。 */
    float left_pwm, right_pwm;           /**< 有符号PWM命令，单位%。 */
} Controller;

/**
 * @brief 初始化底盘状态机和左右速度PID。
 * @param[out] c 由empty.c持有的控制器实例。
 * @param[in] now_ms 启动等待计时起点，单位ms。
 * @note 只在统一初始化调用；不会启动定时器或操作电机GPIO。
 */
void Controller_Init(Controller *c, uint32_t now_ms);
/* 所有状态只在控制中断内修改。终止/故障锁存，复位 MCU 才会重新启动。
 * 主循环在短临界区内写test_pause_requested，控制中断在下一拍响应。 */
/**
 * @brief 执行一次固定周期底盘控制：检查时序→更新目标→速度PID。
 * @param[in,out] c 控制器及其PID历史。
 * @param[in] now_ms 当前毫秒时间。
 * @param[in] main_alive_ms 主循环最近一次存活时间。
 * @param[in] line_bits 黑线=1的五路位图，bit0最左。
 * @param[in] left_delta 左轮四倍频计数/10ms。
 * @param[in] right_delta 右轮四倍频计数/10ms。
 * @note 只由10ms中断调用；输出写入c->left_pwm/right_pwm，由入口文件统一驱动电机。
 */
void Controller_Step(Controller *c, uint32_t now_ms, uint32_t main_alive_ms,
                     uint8_t line_bits, int32_t left_delta, int32_t right_delta);
/**
 * @brief 取得用于OLED的ASCII状态名。
 * @param[in] state 底盘状态。
 * @retval 静态字符串指针，调用者不释放。
 */
const char *Controller_StateName(RobotState state);
#endif
