/**
 * @file    Motion/controller.c
 * @brief   底盘状态机：启动等待、循迹目标、速度PID与可选故障停车。
 * @details 每10ms由empty.c中的控制中断调用一次Controller_Step。
 *          参数区配置运行时间、最低PWM与保护门限；速度在line_follow.c、PID在pid.c。
 *          默认保持已验证的连续循迹：无效线型保持差速、最低PWM10%、不按时长停车。
 */
#include "controller.h"
#include "line_follow.h"
#include "../robot_config.h"
#include <math.h>
#include <string.h>

/* ======================== 本模块参数（修改后重新编译） ======================== */
#define ROBOT_START_DELAY_MS      2000U /* 初始化完成后的启动等待，单位ms。 */
#define ROBOT_RUN_MS              0U /* 0=持续循迹，不按运行时长停车。 */
#define ROBOT_LINE_MIN_PWM        10U /* 循迹运行时每轮最小正向PWM百分比。 */
#define ROBOT_TEST_ON_MS           2000U /* 台架模式每段转动时间，单位ms。 */
#define ROBOT_TEST_OFF_MS          2000U /* 台架模式每段刹车时间，单位ms。 */
#define ROBOT_TEST_LEFT_PWM        30 /* 台架左轮PWM%，负值反转，0停止。 */
#define ROBOT_TEST_RIGHT_PWM       30 /* 台架右轮PWM%，与左轮独立。 */
#define ROBOT_STALL_PWM           35.0f /* 累计无反馈时间的PWM阈值%。 */
#define ROBOT_STALL_MS            500U /* 无反馈诊断时间门限，单位ms。 */
#define ROBOT_MAIN_TIMEOUT_MS     250U /* 主循环超过此时间未响应则故障停车，单位ms。 */

/* PID软件输出上限；TB6612硬件驱动也会独立限制在0..100%。 */
#define ROBOT_PWM_LIMIT 100.0f

#if ROBOT_LINE_MIN_PWM < 1U || ROBOT_LINE_MIN_PWM > 100U
#error "最低正向PWM必须为1..100的整数，并须高于tb6612.c的死区。"
#endif
#if ROBOT_TEST_ON_MS == 0U || ROBOT_TEST_OFF_MS == 0U || \
    ROBOT_TEST_ON_MS > 60000U || ROBOT_TEST_OFF_MS > 60000U
#error "台架测试转动/停车时间必须为1..60000ms。"
#endif
#if ROBOT_TEST_LEFT_PWM < -100 || ROBOT_TEST_LEFT_PWM > 100 || \
    ROBOT_TEST_RIGHT_PWM < -100 || ROBOT_TEST_RIGHT_PWM > 100
#error "台架测试PWM必须为-100..100整数。"
#endif

/**
 * @brief 清零双轮目标/PWM并复位PID，进入给定停车状态。
 * @param[in,out] c 控制器实例。
 * @param[in] state 停车原因或台架暂停/停车状态。
 */
static void Stop(Controller *c, RobotState state)
{
    c->state = state;
    c->left_target = c->right_target = 0.0f;
    c->left_pwm = c->right_pwm = 0.0f;
    Pid_Reset(&c->left_pid);
    Pid_Reset(&c->right_pid);
}

/**
 * @brief 初始化底盘状态机和左右速度PID。
 * @param[out] c 由empty.c持有的控制器实例。
 * @param[in] now_ms 启动等待计时起点，单位ms。
 * @note 只在统一初始化调用；不会启动定时器或操作电机GPIO。
 */
void Controller_Init(Controller *c, uint32_t now_ms)
{
    memset(c, 0, sizeof(*c));
    c->state = ROBOT_WAIT;
    c->start_ms = c->last_step_ms = now_ms;
    Pid_Init(&c->left_pid, PID_LEFT_PARAMETERS.kp, PID_LEFT_PARAMETERS.ki, PID_LEFT_PARAMETERS.kd,
             ROBOT_PWM_LIMIT, PID_LEFT_PARAMETERS.integral_limit, PID_LEFT_PARAMETERS.separation);
    Pid_Init(&c->right_pid, PID_RIGHT_PARAMETERS.kp, PID_RIGHT_PARAMETERS.ki, PID_RIGHT_PARAMETERS.kd,
             ROBOT_PWM_LIMIT, PID_RIGHT_PARAMETERS.integral_limit, PID_RIGHT_PARAMETERS.separation);
}

#if !ROBOT_MOTOR_TEST_MODE
/**
 * @brief 累计有驱动却无编码器增量的时间，到门限后保持饱和。
 * @param[in] previous 上一周期累计ms。
 * @param[in] pwm 本轮PWM命令百分比。
 * @param[in] delta 本轮10ms编码器增量。
 * @retval 累计诊断时间；是否停车由全局开关决定。
 */
static uint32_t StallTime(uint32_t previous, float pwm, int32_t delta)
{
    if (fabsf(pwm) < ROBOT_STALL_PWM || delta != 0) return 0U;
    /* 持续运行时只统计到门限，避免长时间无反馈使累计毫秒溢出。 */
    return previous < ROBOT_STALL_MS ? previous + ROBOT_CONTROL_MS : ROBOT_STALL_MS;
}
#endif

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
            c->left_target = c->right_target = ROBOT_BASE_SPEED * ROBOT_LINE_FALLBACK_RATIO;
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

/**
 * @brief 取得用于OLED的ASCII状态名。
 * @param[in] state 底盘状态。
 * @retval 静态字符串指针，调用者不释放。
 */
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
