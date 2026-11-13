/**
 * @file    Motion/pid.h
 * @brief   PID参数结构、内部历史与计算接口声明。
 * @details Pid_Step允许双向输出；Pid_StepForward用于当前持续前进模式。
 */
#ifndef ROBOT_PID_H
#define ROBOT_PID_H

/* 参数定义在pid.c顶部；这里仅声明类型和跨模块只读访问入口。 */
typedef struct {
    float kp;             /**< 比例增益：增大可加快响应，过大可能振荡。 */
    float ki;             /**< 积分增益：修正稳态误差，已包含10ms周期。 */
    float kd;             /**< 微分增益：抑制误差突变，已包含10ms周期。 */
    float integral_limit; /**< 误差累加限幅，不是PWM百分比。 */
    float separation;     /**< 积分分离阈值，单位为编码器计数/10ms。 */
} PidParameters;
extern const PidParameters PID_LEFT_PARAMETERS;
extern const PidParameters PID_RIGHT_PARAMETERS;

typedef struct {
    float kp, ki, kd;                    /**< 从顶部只读参数复制的当前增益。 */
    float integral, previous_error;      /**< 误差累加值、上一拍误差。 */
    float output_limit, integral_limit, separation; /**< 输出上限%、积分限幅、分离阈值。 */
    unsigned char has_previous;          /**< 首拍为0，避免初始化时微分突跳。 */
} Pid;

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
              float output_limit, float integral_limit, float separation);
/**
 * @brief 清除积分、上一拍误差和首拍标志，保留整定参数。
 * @param[in,out] pid PID实例。
 */
void Pid_Reset(Pid *pid);
/* 必须以固定 10 ms 周期调用；输入或内部状态非有限数时返回零并复位。 */
/**
 * @brief 计算可正反转的PID输出。
 * @param[in,out] pid PID实例。
 * @param[in] target 目标计数/10ms。
 * @param[in] measured 实测计数/10ms。
 * @retval -output_limit..+output_limit的PWM百分比。
 */
float Pid_Step(Pid *pid, float target, float measured);
/* 正向连续运行：输出范围minimum_output..output_limit，包含下限抗积分饱和。
 * target=0和非有限数仍返回0；调用者正常循迹时须给正的速度目标。 */
/**
 * @brief 计算只前进且保持最低PWM的速度PID输出。
 * @param[in,out] pid PID实例。
 * @param[in] target 正的目标计数/10ms。
 * @param[in] measured 实测计数/10ms。
 * @param[in] minimum_output 最低正向PWM%，当前由controller.c设置为10。
 * @note 下限参与抗积分饱和；它是PWM下限，不是实际机械转速保证。
 */
float Pid_StepForward(Pid *pid, float target, float measured, float minimum_output);
#endif
