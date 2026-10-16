#include "encoder.h"
#include "board.h"
#include "../robot_config.h"
#include "../Motion/quadrature.h"

static volatile uint32_t s_right_count, s_invalid;
static uint8_t s_previous_ab;
static uint16_t s_left_previous;
static uint32_t s_right_previous;
static int64_t s_left_position, s_right_position;

static uint8_t ReadRightAB(void)
{
    uint32_t pins = DL_GPIO_readPins(GPIO_ENCODER_PORT, BOARD_RIGHT_ENCODER_MASK);
    return (uint8_t)(((pins & GPIO_ENCODER_RIGHT_A_PIN) ? 2U : 0U) |
                     ((pins & GPIO_ENCODER_RIGHT_B_PIN) ? 1U : 0U));
}

void Encoder_Init(void)
{
    s_right_count = s_invalid = 0U;
    s_right_previous = 0U;
    s_left_position = s_right_position = 0;
    /* 初始化期间车辆静止。相序校正通过 robot_config.h 完成。 */
    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, BOARD_RIGHT_ENCODER_MASK);
    s_previous_ab = ReadRightAB();
    DL_TimerG_startCounter(ENCODER_LEFT_INST);
    s_left_previous = (uint16_t)DL_TimerG_getTimerCount(ENCODER_LEFT_INST);
    NVIC_ClearPendingIRQ(GPIO_ENCODER_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);
}

void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODER_PORT, BOARD_RIGHT_ENCODER_MASK);
    if (pending != 0U) {
        /* 先清已观察到的标志，再读 AB；新到的边沿可再次触发中断。
         * GPIO 标志不会排队：多边沿在 ISR 前合并仍可能漏计，须实测上限。 */
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, pending);
        uint8_t ab = ReadRightAB();
        int8_t delta = Quadrature_Delta(s_previous_ab, ab);
        if ((s_previous_ab ^ ab) == 3U) ++s_invalid;
        s_right_count += (uint32_t)(int32_t)delta;
        s_previous_ab = ab;
    }
}

void Encoder_Sample(EncoderSample *sample)
{
    uint32_t saved = Board_Lock();
    uint16_t left_now = (uint16_t)DL_TimerG_getTimerCount(ENCODER_LEFT_INST);
    uint32_t right_now = s_right_count;
    sample->right_invalid_transitions = s_invalid;
    Board_Unlock(saved);
    /* 无符号回绕求差，不清零硬件计数器，不丢失读/清之间的脉冲。
     * 前提：单个周期左轮绝对计数 <32768，右轮 <2^31。 */
    uint32_t left_wrapped = (uint16_t)(left_now - s_left_previous);
    uint32_t right_wrapped = right_now - s_right_previous;
    int32_t left_delta = left_wrapped < 32768U ? (int32_t)left_wrapped : (int32_t)left_wrapped - 65536;
    int32_t right_delta = right_wrapped <= 0x7FFFFFFFU ? (int32_t)right_wrapped :
                         -1 - (int32_t)(0xFFFFFFFFU - right_wrapped);
    sample->left_delta = left_delta * ROBOT_LEFT_ENCODER_SIGN;
    sample->right_delta = right_delta * ROBOT_RIGHT_ENCODER_SIGN;
    s_left_position += sample->left_delta;
    s_right_position += sample->right_delta;
    sample->left_position = s_left_position;
    sample->right_position = s_right_position;
    s_left_previous = left_now;
    s_right_previous = right_now;
}
