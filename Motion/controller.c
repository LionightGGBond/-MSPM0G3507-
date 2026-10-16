#include "controller.h"
#include "line_follow.h"
#include "../robot_config.h"
#include <math.h>
#include <string.h>

static void Stop(Controller *c, RobotState state)
{
    c->state = state;
    c->left_target = c->right_target = 0.0f;
    c->left_pwm = c->right_pwm = 0.0f;
    Pid_Reset(&c->left_pid);
    Pid_Reset(&c->right_pid);
}

void Controller_Init(Controller *c, uint32_t now_ms)
{
    memset(c, 0, sizeof(*c));
    c->state = ROBOT_WAIT;
    c->start_ms = c->last_step_ms = now_ms;
    Pid_Init(&c->left_pid, ROBOT_LEFT_KP, ROBOT_LEFT_KI, ROBOT_LEFT_KD,
             ROBOT_PWM_LIMIT, ROBOT_INTEGRAL_LIMIT, ROBOT_I_SEPARATE);
    Pid_Init(&c->right_pid, ROBOT_RIGHT_KP, ROBOT_RIGHT_KI, ROBOT_RIGHT_KD,
             ROBOT_PWM_LIMIT, ROBOT_INTEGRAL_LIMIT, ROBOT_I_SEPARATE);
}

#if !ROBOT_MOTOR_TEST_MODE
static uint32_t StallTime(uint32_t previous, float pwm, int32_t delta)
{
    if (fabsf(pwm) < ROBOT_STALL_PWM || delta != 0) return 0U;
    /* 持续运行时只统计到门限，避免长时间无反馈使累计毫秒溢出。 */
    return previous < ROBOT_STALL_MS ? previous + ROBOT_CONTROL_MS : ROBOT_STALL_MS;
}
#endif

void Controller_Step(Controller *c, uint32_t now_ms, uint32_t main_alive_ms,
                     uint8_t line_bits, int32_t left_delta, int32_t right_delta)
{
    uint32_t elapsed = now_ms - c->last_step_ms;
    c->last_step_ms = now_ms;
    c->line_bits = line_bits;
    c->left_speed = (float)left_delta;
    c->right_speed = (float)right_delta;
    if (c->state != ROBOT_WAIT && c->state != ROBOT_RUNNING &&
        c->state != ROBOT_TEST_RUNNING && c->state != ROBOT_TEST_STOPPED &&
        c->state != ROBOT_TEST_PAUSED) return;

    /* 不补跑迟到的多次 PID：反馈已不属于原来的 10 ms 窗口。 */
    if (elapsed < ROBOT_CONTROL_MS - 2U || elapsed > ROBOT_CONTROL_MS + 2U) {
        Stop(c, ROBOT_TIMING_ERROR);
        return;
    }
    if ((uint32_t)(now_ms - main_alive_ms) > ROBOT_MAIN_TIMEOUT_MS) {
        Stop(c, ROBOT_MAIN_TIMEOUT);
        return;
    }
#if ROBOT_MOTOR_TEST_MODE
    /* 开环台架测试故意绕过循迹、速度PID和编码器缺失停车。
     * 仍保留上方时基/主循环超时保护；故障后r命令不能解除锁存。 */
    if (c->test_pause_requested) {
        Stop(c, ROBOT_TEST_PAUSED);
        return;
    }
    if (c->state == ROBOT_WAIT) {
        if ((uint32_t)(now_ms - c->start_ms) < ROBOT_START_DELAY_MS) return;
        c->run_start_ms = now_ms;
    } else if (c->state == ROBOT_TEST_PAUSED) {
        c->run_start_ms = now_ms; /* r继续时从完整的2秒转动段开始。 */
    }
    uint32_t cycle_ms = ROBOT_TEST_ON_MS + ROBOT_TEST_OFF_MS;
    uint32_t phase_ms = now_ms - c->run_start_ms;
    /* 每圈推进基准，避免运行49.7天毫秒回绕时出现额外相位跳变。 */
    c->run_start_ms += (phase_ms / cycle_ms) * cycle_ms;
    if (phase_ms % cycle_ms < ROBOT_TEST_ON_MS) {
        c->state = ROBOT_TEST_RUNNING;
        c->left_target = c->right_target = 0.0f; /* 开环没有速度目标。 */
        c->left_pwm = (float)ROBOT_TEST_LEFT_PWM;
        c->right_pwm = (float)ROBOT_TEST_RIGHT_PWM;
    } else {
        Stop(c, ROBOT_TEST_STOPPED);
    }
    return;
#else
    if (c->state == ROBOT_WAIT) {
        if ((uint32_t)(now_ms - c->start_ms) < ROBOT_START_DELAY_MS) return;
        c->run_start_ms = now_ms;
        c->state = ROBOT_RUNNING;
    }
#if ROBOT_RUN_MS > 0U
    if ((uint32_t)(now_ms - c->run_start_ms) >= ROBOT_RUN_MS) {
        Stop(c, ROBOT_FINISHED);
        return;
    }
#endif

    /* 先写临时目标：无效输入不能覆盖最近有效差速，再由开关决定是否停车。 */
    float next_left, next_right;
    LineResult result = LineFollow_Targets(line_bits, ROBOT_BASE_SPEED,
                                           &next_left, &next_right);
    c->line_fallback = result != LINE_NORMAL;
    if (result == LINE_NORMAL) {
        c->left_target = next_left;
        c->right_target = next_right;
    }
    if (result != LINE_NORMAL) {
#if ROBOT_STOP_ON_INVALID_LINE
        Stop(c, result == LINE_LOST ? ROBOT_LINE_LOST : ROBOT_LINE_AMBIGUOUS);
        return;
#else
        /* 短时干扰沿用上一次有效转向；还没有有效线型时以半基础速度直行。
         * 恢复有效线型后下一拍立即更新差速，不锁存、不等复位。 */
        if (c->left_target <= 0.0f || c->right_target <= 0.0f)
            c->left_target = c->right_target = ROBOT_BASE_SPEED * 0.5f;
#endif
    }
    /* 每轮只前进；超速时降到最低PWM，避免负PID命令触发反转前刹车。 */
    c->left_pwm = Pid_StepForward(&c->left_pid, c->left_target, c->left_speed,
                                  (float)ROBOT_LINE_MIN_PWM);
    c->right_pwm = Pid_StepForward(&c->right_pid, c->right_target, c->right_speed,
                                   (float)ROBOT_LINE_MIN_PWM);
    c->left_stall_ms = StallTime(c->left_stall_ms, c->left_pwm, left_delta);
    c->right_stall_ms = StallTime(c->right_stall_ms, c->right_pwm, right_delta);
#if ROBOT_STOP_ON_ENCODER_STALL
    if (c->left_stall_ms >= ROBOT_STALL_MS || c->right_stall_ms >= ROBOT_STALL_MS) {
        Stop(c, ROBOT_ENCODER_STALL);
    }
#endif
#endif
}

const char *Controller_StateName(RobotState state)
{
    switch (state) {
        case ROBOT_WAIT: return "WAIT";
        case ROBOT_RUNNING: return "RUN";
        case ROBOT_FINISHED: return "DONE";
        case ROBOT_TEST_RUNNING: return "TEST RUN";
        case ROBOT_TEST_STOPPED: return "TEST STOP";
        case ROBOT_TEST_PAUSED: return "TEST PAUSE";
        case ROBOT_LINE_LOST: return "LOST";
        case ROBOT_LINE_AMBIGUOUS: return "CROSS/INVALID";
        case ROBOT_ENCODER_STALL: return "ENC STALL";
        case ROBOT_MAIN_TIMEOUT: return "MAIN TIMEOUT";
        case ROBOT_TIMING_ERROR: return "TICK ERROR";
        default: return "UNKNOWN";
    }
}
