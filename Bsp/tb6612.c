/**
 * @file    Bsp/tb6612.c
 * @brief   TB6612双轮PWM和方向驱动，左轮B通道、右轮A通道。
 * @details 顶部统一设置方向符号、AIN/BIN正向电平、PWM死区和驱动输出上限。
 *          软件正值代表前进；停车为IN1=IN2=1短路制动；STBY按现有接线固定高。
 */
#include "tb6612.h"
#include "board.h"
#include <math.h>

/* ======================== 本模块参数（修改后重新编译） ======================== */
/* A通道=右轮；B通道=左轮。方向错误时只改对应MOTOR_SIGN。
 * 正向IN电平在此统一设置：0=低，1=高；停车仍固定11短路制动。
 * PWM死区必须小于controller.c配置的最低运行PWM（当前10%）。 */
#define ROBOT_LEFT_MOTOR_SIGN     1 /* 左轮正命令物理方向校正：+1或-1。 */
#define ROBOT_RIGHT_MOTOR_SIGN    1 /* 右轮正命令物理方向校正：+1或-1。 */
#define ROBOT_TB6612_AIN1_FORWARD_LEVEL  0U  /* PA15，右轮AIN1 */
#define ROBOT_TB6612_AIN2_FORWARD_LEVEL  1U  /* PA16，右轮AIN2 */
#define ROBOT_TB6612_BIN1_FORWARD_LEVEL  0U  /* PB0， 左轮BIN1 */
#define ROBOT_TB6612_BIN2_FORWARD_LEVEL  1U  /* PB1， 左轮BIN2 */
#define ROBOT_PWM_DEADBAND        5.0f /* 绝对PWM不超过此值则制动，单位%。 */
#define ROBOT_PWM_LIMIT           100.0f /* 本驱动允许的PWM幅值上限，单位%。 */

#if (ROBOT_LEFT_MOTOR_SIGN != 1 && ROBOT_LEFT_MOTOR_SIGN != -1) || \
    (ROBOT_RIGHT_MOTOR_SIGN != 1 && ROBOT_RIGHT_MOTOR_SIGN != -1)
#error "电机方向极性只能为+1或-1。"
#endif
#if ROBOT_TB6612_AIN1_FORWARD_LEVEL > 1U || ROBOT_TB6612_AIN2_FORWARD_LEVEL > 1U || \
    ROBOT_TB6612_BIN1_FORWARD_LEVEL > 1U || ROBOT_TB6612_BIN2_FORWARD_LEVEL > 1U
#error "方向输入电平只能为0或1。"
#endif

static int s_left_sign, s_right_sign;

/**
 * @brief 写入TB6612一个方向输入的高低电平。
 * @param[in] port GPIO端口。
 * @param[in] pin 引脚掩码。
 * @param[in] high true高电平，false低电平。
 */
static void Pin(GPIO_Regs *port, uint32_t pin, bool high)
{
    if (high) DL_GPIO_setPins(port, pin);
    else DL_GPIO_clearPins(port, pin);
}

/* 四个方向脚的唯一硬件写入位置；转动电平由本文件顶部参数集中设置。
 * 只由本驱动调用，避免主循环手动写GPIO后被10ms控制中断覆盖。 */
/**
 * @brief 统一设置单个电机通道的两个方向输入。
 * @param[in] left true选择左轮B通道，false选择右轮A通道。
 * @param[in] in1_high IN1输出电平。
 * @param[in] in2_high IN2输出电平。
 * @note 仅驱动内部调用；不要在主循环另写方向脚后与控制IRQ争用。
 */
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

/**
 * @brief 把单轮有符号PWM百分比转换为方向脚和下数PWM比较值。
 * @param[in] left true左轮B通道，false右轮A通道。
 * @param[in] percent 正值前进、负值反转、死区内制动。
 * @note 检测到换向时先刹车一拍；物理方向由顶部MOTOR_SIGN和电平表修正。
 */
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

/**
 * @brief 先建立两轮制动，再启动电机PWM定时器。
 * @note 在控制中断启用之前调用。
 */
void TB6612_Init(void)
{
    s_left_sign = s_right_sign = 0;
    TB6612_Brake();
    DL_TimerG_startCounter(MOTOR_PWM_INST);
}

/**
 * @brief 设置两轮有符号PWM命令。
 * @param[in] left_percent 左轮命令%，左轮使用B/PWMB。
 * @param[in] right_percent 右轮命令%，右轮使用A/PWMA。
 * @note 正常运行只由empty.c的控制IRQ调用；函数内部进行死区和限幅处理。
 */
void TB6612_Set(float left_percent, float right_percent)
{
    SetOne(true, left_percent);
    SetOne(false, right_percent);
}

/**
 * @brief 两轮执行短路制动，复位方向切换历史。
 * @note 不使用STBY脚；电机机械停止需要实际减速时间。
 */
void TB6612_Brake(void)
{
    TB6612_Set(0.0f, 0.0f);
}

/**
 * @brief 读取四个方向GPIO的输出锁存值。
 * @param[out] levels AIN1/AIN2/BIN1/BIN2的逻辑高低状态。
 * @note 这是软件输出寄存器，不等同于万用表实测电压；主循环在临界区中读取。
 */
void TB6612_ReadInputLevels(TB6612_InputLevels *levels)
{
    /* 读取DOUT输出锁存，避免将输出脚的输入缓冲状态误认为输出值。 */
    levels->ain1_high = (GPIO_MOTOR_AIN1_PORT->DOUT31_0 & GPIO_MOTOR_AIN1_PIN) != 0U;
    levels->ain2_high = (GPIO_MOTOR_AIN2_PORT->DOUT31_0 & GPIO_MOTOR_AIN2_PIN) != 0U;
    levels->bin1_high = (GPIO_MOTOR_BIN1_PORT->DOUT31_0 & GPIO_MOTOR_BIN1_PIN) != 0U;
    levels->bin2_high = (GPIO_MOTOR_BIN2_PORT->DOUT31_0 & GPIO_MOTOR_BIN2_PIN) != 0U;
}
