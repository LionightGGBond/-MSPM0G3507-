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
    Pid left_pid, right_pid;
    RobotState state;
    uint32_t start_ms, run_start_ms, last_step_ms;
    uint32_t left_stall_ms, right_stall_ms;
    volatile bool test_pause_requested;
    uint8_t line_bits;
    bool line_fallback; /* 当前线型无效，正在沿用最近差速或首次低速直行。 */
    float left_target, right_target;
    float left_speed, right_speed;
    float left_pwm, right_pwm;
} Controller;

void Controller_Init(Controller *control, uint32_t now_ms);
/* 所有状态只在控制中断内修改。终止/故障锁存，复位 MCU 才会重新启动。
 * 主循环在短临界区内写test_pause_requested，控制中断在下一拍响应。 */
void Controller_Step(Controller *control, uint32_t now_ms, uint32_t main_alive_ms,
                     uint8_t line_bits, int32_t left_delta, int32_t right_delta);
const char *Controller_StateName(RobotState state);
#endif
