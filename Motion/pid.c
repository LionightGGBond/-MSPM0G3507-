#include "pid.h"
#include <math.h>

static float Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void Pid_Reset(Pid *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->has_previous = 0U;
}

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

float Pid_Step(Pid *pid, float target, float measured)
{
    return Step(pid, target, measured, -pid->output_limit);
}

float Pid_StepForward(Pid *pid, float target, float measured, float minimum_output)
{
    if (!isfinite(minimum_output)) return 0.0f;
    if (minimum_output < 0.0f) minimum_output = 0.0f;
    if (minimum_output > pid->output_limit) minimum_output = pid->output_limit;
    return Step(pid, target, measured, minimum_output);
}
