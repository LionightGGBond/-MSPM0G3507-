/**
 * @file    Motion/pid.c
 * @brief   固定10ms离散速度PID：积分分离、限幅与条件积分抗饱和。
 * @details 顶部集中设置左右轮Kp/Ki/Kd、积分限幅与分离阈值。
 *          本文件只计算PWM命令，不读GPIO、不写电机、不安排任务周期。
 */
#include "pid.h"
#include <math.h>

/* ======================== 本模块参数（修改后重新编译） ======================== */
#define ROBOT_LEFT_KP             1.413f /* 左轮比例增益，影响误差响应强度。 */
#define ROBOT_LEFT_KI             0.79f /* 左轮离散积分增益，修正稳态误差。 */
#define ROBOT_LEFT_KD             0.03f /* 左轮离散微分增益，抑制误差突变。 */
#define ROBOT_RIGHT_KP            1.40f /* 右轮比例增益。 */
#define ROBOT_RIGHT_KI            0.81f /* 右轮离散积分增益。 */
#define ROBOT_RIGHT_KD            0.06f /* 右轮离散微分增益。 */
#define ROBOT_INTEGRAL_LIMIT      30.0f /* 积分累加限幅，避免长期误差积累。 */
#define ROBOT_I_SEPARATE          30.0f /* 积分分离阈值，单位计数/10ms。 */

/* Ki/Kd已经包含10ms离散周期；不要在主循环重复执行速度PID。
 * 左右电机单独整定；积分限幅和积分分离暂共用，结构体支持后续分别配置。 */
const PidParameters PID_LEFT_PARAMETERS = {
    ROBOT_LEFT_KP, ROBOT_LEFT_KI, ROBOT_LEFT_KD,
    ROBOT_INTEGRAL_LIMIT, ROBOT_I_SEPARATE
};
const PidParameters PID_RIGHT_PARAMETERS = {
    ROBOT_RIGHT_KP, ROBOT_RIGHT_KI, ROBOT_RIGHT_KD,
    ROBOT_INTEGRAL_LIMIT, ROBOT_I_SEPARATE
};

/**
 * @brief 将数值限制在对称区间[-limit,+limit]。
 * @param[in] value 待限幅数值。
 * @param[in] limit 非负幅值上限。
 */
static float Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/**
 * @brief 清除积分、上一拍误差和首拍标志，保留整定参数。
 * @param[in,out] pid PID实例。
 */
void Pid_Reset(Pid *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->has_previous = 0U;
}

/**
 * @brief 初始化一个离散PID实例。
 * @param[out] pid PID实例。
 * @param[in] kp 比例系数。
 * @param[in] ki 已包含10ms周期的积分系数。
 * @param[in] kd 已包含10ms周期的微分系数。
 * @param[in] output_limit PWM输出幅值上限，单位%。
 * @param[in] integral_limit 误差累加上限。
 * @param[in] separation 积分分离阈值，单位计数/10ms。
 */
void Pid_Init(Pid *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit, float separation)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = fabsf(output_limit);
    pid->integral_limit = fabsf(integral_limit);
    pid->separation = fabsf(separation);
    Pid_Reset(pid);
}

/**
 * @brief PID计算共用内核，支持上限和不同下限的条件积分抗饱和。
 * @param[in,out] pid PID历史。
 * @param[in] target 目标编码器计数/10ms。
 * @param[in] measured 本拍实测计数/10ms。
 * @param[in] minimum_output 允许的输出下限，单位%。
 * @retval PWM命令；零目标或非有限数输入返回0并清历史。
 */
static float Step(Pid *pid, float target, float measured, float minimum_output)
{
    float error = target - measured;
    float candidate, derivative, output;
    if (!isfinite(error) || !isfinite(pid->integral)) {
        Pid_Reset(pid);
        return 0.0f;
    }
    /* 零目标直接停车并清积分，避免停车期间继续积累误差。 */
    if (target == 0.0f) {
        Pid_Reset(pid);
        return 0.0f;
    }
    derivative = pid->has_previous ? error - pid->previous_error : 0.0f;
    candidate = (fabsf(error) > pid->separation) ? 0.0f :
                Clamp(pid->integral + error, pid->integral_limit);
    output = pid->kp * error + pid->ki * candidate + pid->kd * derivative;
    /* 条件积分：饱和且积分会进一步推向饱和时，不接受本次累加。
     * 已有积分仍允许在反向误差作用下退出饱和。 */
    if ((output > pid->output_limit && pid->ki * error > 0.0f) ||
        (output < minimum_output && pid->ki * error < 0.0f)) {
        if (fabsf(error) > pid->separation) pid->integral = 0.0f;
        output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    } else {
        pid->integral = candidate;
    }
    pid->previous_error = error;
    pid->has_previous = 1U;
    if (!isfinite(output)) {
        Pid_Reset(pid);
        return 0.0f;
    }
    if (output < minimum_output) return minimum_output;
    return output > pid->output_limit ? pid->output_limit : output;
}

/**
 * @brief 计算可正反转的PID输出。
 * @param[in,out] pid PID实例。
 * @param[in] target 目标计数/10ms。
 * @param[in] measured 实测计数/10ms。
 * @retval -output_limit..+output_limit的PWM百分比。
 */
float Pid_Step(Pid *pid, float target, float measured)
{
    return Step(pid, target, measured, -pid->output_limit);
}

/**
 * @brief 计算只前进且保持最低PWM的速度PID输出。
 * @param[in,out] pid PID实例。
 * @param[in] target 正的目标计数/10ms。
 * @param[in] measured 实测计数/10ms。
 * @param[in] minimum_output 最低正向PWM%，当前由controller.c设置为10。
 * @note 下限参与抗积分饱和；它是PWM下限，不是实际机械转速保证。
 */
float Pid_StepForward(Pid *pid, float target, float measured, float minimum_output)
{
    if (!isfinite(minimum_output)) return 0.0f;
    if (minimum_output < 0.0f) minimum_output = 0.0f;
    if (minimum_output > pid->output_limit) minimum_output = pid->output_limit;
    return Step(pid, target, measured, minimum_output);
}
