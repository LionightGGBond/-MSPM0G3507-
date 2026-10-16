#ifndef ROBOT_PID_H
#define ROBOT_PID_H

typedef struct {
    float kp, ki, kd;
    float integral, previous_error;
    float output_limit, integral_limit, separation;
    unsigned char has_previous;
} Pid;

void Pid_Init(Pid *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit, float separation);
void Pid_Reset(Pid *pid);
/* 必须以固定 10 ms 周期调用；输入或内部状态非有限数时返回零并复位。 */
float Pid_Step(Pid *pid, float target, float measured);
/* 正向连续运行：输出范围minimum_output..output_limit，包含下限抗积分饱和。
 * target=0和非有限数仍返回0；调用者正常循迹时须给正的速度目标。 */
float Pid_StepForward(Pid *pid, float target, float measured, float minimum_output);
#endif
