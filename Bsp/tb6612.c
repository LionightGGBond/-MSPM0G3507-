#include "tb6612.h"
#include "board.h"
#include "../robot_config.h"
#include <math.h>

static int s_left_sign, s_right_sign;

static void Pin(GPIO_Regs *port, uint32_t pin, bool high)
{
    if (high) DL_GPIO_setPins(port, pin);
    else DL_GPIO_clearPins(port, pin);
}

/* 四个方向脚的唯一硬件写入位置；转动电平由robot_config.h集中设置。
 * 只由本驱动调用，避免主循环手动写GPIO后被10ms控制中断覆盖。 */
static void SetInputLevels(bool left, bool in1_high, bool in2_high)
{
    if (left) {
        Pin(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN, in1_high);
        Pin(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN, in2_high);
    } else {
        Pin(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN, in1_high);
        Pin(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN, in2_high);
    }
}

static void SetOne(bool left, float percent)
{
    DL_TIMER_CC_INDEX channel = left ? GPIO_MOTOR_PWM_C1_IDX : GPIO_MOTOR_PWM_C0_IDX;
    int *last_sign = left ? &s_left_sign : &s_right_sign;
    percent *= left ? ROBOT_LEFT_MOTOR_SIGN : ROBOT_RIGHT_MOTOR_SIGN;
    float magnitude = fabsf(percent);
    int sign = percent >= 0.0f ? 1 : -1;
    uint32_t load = DL_TimerG_getLoadValue(MOTOR_PWM_INST);
    /* 方向改变前先维持至少一个控制周期的短路制动。 */
    if (!isfinite(percent) || magnitude <= ROBOT_PWM_DEADBAND ||
        (*last_sign != 0 && *last_sign != sign)) {
        SetInputLevels(left, true, true);
        DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, load, channel);
        *last_sign = 0;
        return;
    }
    if (magnitude > ROBOT_PWM_LIMIT) magnitude = ROBOT_PWM_LIMIT;
    if (magnitude > 100.0f) magnitude = 100.0f;
    uint32_t high_ticks = (uint32_t)((float)(load + 1U) * magnitude / 100.0f + 0.5f);
    /* SDK 下数 PWM 的比较值越小，占空比越大。
     * 非零输出限制在 1..LOAD 个高电平计数；最大约99.94%。
     * 停车由 IN1=IN2=1 的短路制动保证，不依赖 PWM 边界事件优先级。 */
    if (high_ticks > load) high_ticks = load;
    if (high_ticks == 0U) high_ticks = 1U;
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, load - high_ticks, channel);
    bool forward_in1 = (left ? ROBOT_TB6612_BIN1_FORWARD_LEVEL : ROBOT_TB6612_AIN1_FORWARD_LEVEL) != 0U;
    bool forward_in2 = (left ? ROBOT_TB6612_BIN2_FORWARD_LEVEL : ROBOT_TB6612_AIN2_FORWARD_LEVEL) != 0U;
    /* 反转时交换一对电平；不改变占空比计算和换向先刹车的逻辑。 */
    SetInputLevels(left, sign > 0 ? forward_in1 : forward_in2,
                         sign > 0 ? forward_in2 : forward_in1);
    *last_sign = sign;
}

void TB6612_Init(void)
{
    s_left_sign = s_right_sign = 0;
    TB6612_Brake();
    DL_TimerG_startCounter(MOTOR_PWM_INST);
}

void TB6612_Set(float left_percent, float right_percent)
{
    SetOne(true, left_percent);
    SetOne(false, right_percent);
}

void TB6612_Brake(void)
{
    TB6612_Set(0.0f, 0.0f);
}

void TB6612_ReadInputLevels(TB6612_InputLevels *levels)
{
    /* 读取DOUT输出锁存，避免将输出脚的输入缓冲状态误认为输出值。 */
    levels->ain1_high = (GPIO_MOTOR_AIN1_PORT->DOUT31_0 & GPIO_MOTOR_AIN1_PIN) != 0U;
    levels->ain2_high = (GPIO_MOTOR_AIN2_PORT->DOUT31_0 & GPIO_MOTOR_AIN2_PIN) != 0U;
    levels->bin1_high = (GPIO_MOTOR_BIN1_PORT->DOUT31_0 & GPIO_MOTOR_BIN1_PIN) != 0U;
    levels->bin2_high = (GPIO_MOTOR_BIN2_PORT->DOUT31_0 & GPIO_MOTOR_BIN2_PIN) != 0U;
}
