#include "board.h"
#include "../robot_config.h"

static volatile uint32_t s_milliseconds;

uint32_t Board_Lock(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}

void Board_Unlock(uint32_t saved)
{
    __set_PRIMASK(saved);
}

static void Output(GPIO_Regs *port, uint32_t pin, bool high)
{
    if (high) DL_GPIO_setPins(port, pin);
    else DL_GPIO_clearPins(port, pin);
}

void Board_Leds(bool heart, bool left, bool right)
{
    Output(GPIO_LED_HEART_PORT, GPIO_LED_HEART_PIN, heart == (ROBOT_LED_ACTIVE_LEVEL != 0U));
    Output(GPIO_LED_LEFT_PORT, GPIO_LED_LEFT_PIN, left == (ROBOT_LED_ACTIVE_LEVEL != 0U));
    Output(GPIO_LED_RIGHT_PORT, GPIO_LED_RIGHT_PIN, right == (ROBOT_LED_ACTIVE_LEVEL != 0U));
}

void Board_Buzzer(bool on)
{
    Output(GPIO_BUZZER_PORT, GPIO_BUZZER_OUT_PIN, on == (ROBOT_BUZZER_ACTIVE_LEVEL != 0U));
}

void Board_Init(void)
{
    SYSCFG_DL_init();
    s_milliseconds = 0U;
    Board_Leds(false, false, false);
    Board_Buzzer(false);
    /* GPIO 编码器最高优先级，SysTick 其次，控制中断再次。
     * 主循环不进入 STOP/STANDBY，避免未保存的定时器配置丢失。 */
    NVIC_SetPriority(GPIO_ENCODER_INT_IRQN, 0U);
    NVIC_SetPriority(SysTick_IRQn, 1U);
    NVIC_SetPriority(CONTROL_TIMER_INST_INT_IRQN, 2U);
    (void)SysTick_Config(CPUCLK_FREQ / 1000U);
    /* SysTick_Config 会改优先级，故随后再设置一次。 */
    NVIC_SetPriority(SysTick_IRQn, 1U);
}

bool Board_ValidateTimers(void)
{
    BoardTimerDiagnostics diagnostics;
    Board_ReadTimerDiagnostics(&diagnostics);
    return diagnostics.errors == 0U;
}

void Board_ReadTimerDiagnostics(BoardTimerDiagnostics *diagnostics)
{
    DL_TimerG_ClockConfig pwm_clock, control_clock;
    DL_TimerG_getClockConfig(MOTOR_PWM_INST, &pwm_clock);
    DL_TimerG_getClockConfig(CONTROL_TIMER_INST, &control_clock);
    diagnostics->errors = 0U;
    diagnostics->pwm_load = DL_TimerG_getLoadValue(MOTOR_PWM_INST);
    diagnostics->control_load = DL_TimerG_getLoadValue(CONTROL_TIMER_INST);
    if (pwm_clock.clockSel != DL_TIMER_CLOCK_BUSCLK ||
        pwm_clock.divideRatio != DL_TIMER_CLOCK_DIVIDE_1 || pwm_clock.prescale != 0U) {
        diagnostics->errors |= BOARD_TIMER_PWM_CLOCK_ERROR;
    }
    if (diagnostics->pwm_load != 1599U) diagnostics->errors |= BOARD_TIMER_PWM_LOAD_ERROR;
    if (control_clock.clockSel != DL_TIMER_CLOCK_BUSCLK ||
        control_clock.divideRatio != DL_TIMER_CLOCK_DIVIDE_1 || control_clock.prescale != 31U) {
        diagnostics->errors |= BOARD_TIMER_CTRL_CLOCK_ERROR;
    }
    if (diagnostics->control_load != 9999U) diagnostics->errors |= BOARD_TIMER_CTRL_LOAD_ERROR;
}

void SysTick_Handler(void)
{
    ++s_milliseconds;
}

uint32_t Board_Millis(void)
{
    return s_milliseconds;
}

void Board_DelayMs(uint32_t duration)
{
    uint32_t start = Board_Millis();
    while ((uint32_t)(Board_Millis() - start) < duration) { __NOP(); }
}

uint8_t Board_LineBits(void)
{
    static const uint32_t pins[5] = {GPIO_LINE_L2_PIN, GPIO_LINE_L1_PIN,
        GPIO_LINE_MID_PIN, GPIO_LINE_R1_PIN, GPIO_LINE_R2_PIN};
    /* 同一端口一次读取，避免五次读操作造成采样时刻不一致。 */
    uint32_t inputs = DL_GPIO_readPins(GPIO_LINE_PORT, GPIO_LINE_L2_PIN | GPIO_LINE_L1_PIN |
                                     GPIO_LINE_MID_PIN | GPIO_LINE_R1_PIN | GPIO_LINE_R2_PIN);
    uint8_t result = 0U;
    for (unsigned int i = 0U; i < 5U; ++i) {
        bool high = (inputs & pins[i]) != 0U;
        if (high == (ROBOT_LINE_ACTIVE_LEVEL != 0U)) result |= (uint8_t)(1U << i);
    }
    return result;
}
