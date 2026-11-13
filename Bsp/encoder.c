/**
 * @file    Bsp/encoder.c
 * @brief   双轮编码器采集：左轮TIMG8 QEI，右轮GPIO AB双边沿。
 * @details 顶部修改左右编码器相序符号，物理前进应累计正计数。
 *          GROUP1中断只解码右轮边沿；10ms控制中断调用Encoder_Sample取差分。
 */
#include "encoder.h"
#include "board.h"
#include "../Motion/quadrature.h"

/* ======================== 本模块参数（修改后重新编译） ======================== */
#define ROBOT_LEFT_ENCODER_SIGN   1 /* 左轮物理前进应为正；相序相反改-1。 */
#define ROBOT_RIGHT_ENCODER_SIGN  1 /* 右轮物理前进应为正；与电机方向符号独立。 */

#if (ROBOT_LEFT_ENCODER_SIGN != 1 && ROBOT_LEFT_ENCODER_SIGN != -1) || \
    (ROBOT_RIGHT_ENCODER_SIGN != 1 && ROBOT_RIGHT_ENCODER_SIGN != -1)
#error "编码器相序极性只能为+1或-1。"
#endif

static volatile uint32_t s_right_count, s_invalid;
static uint8_t s_previous_ab;
static uint16_t s_left_previous;
static uint32_t s_right_previous;
static int64_t s_left_position, s_right_position;

/**
 * @brief 同时读取右轮A/B输入并编码成两位状态。
 * @retval (A<<1)|B，范围0..3。
 */
static uint8_t ReadRightAB(void)
{
    uint32_t pins = DL_GPIO_readPins(GPIO_ENCODER_PORT, BOARD_RIGHT_ENCODER_MASK);
    return (uint8_t)(((pins & GPIO_ENCODER_RIGHT_A_PIN) ? 2U : 0U) |
                     ((pins & GPIO_ENCODER_RIGHT_B_PIN) ? 1U : 0U));
}

/**
 * @brief 初始化左右计数历史，启动左QEI并开放右轮GPIO边沿中断。
 * @note 统一初始化时调用；不清零运行中的QEI来获得速度。
 */
void Encoder_Init(void)
{
    s_right_count = s_invalid = 0U;
    s_right_previous = 0U;
    s_left_position = s_right_position = 0;
    /* 初始化期间车辆静止。相序校正通过本文件顶部ENCODER_SIGN完成。 */
    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, BOARD_RIGHT_ENCODER_MASK);
    s_previous_ab = ReadRightAB();
    DL_TimerG_startCounter(ENCODER_LEFT_INST);
    s_left_previous = (uint16_t)DL_TimerG_getTimerCount(ENCODER_LEFT_INST);
    NVIC_ClearPendingIRQ(GPIO_ENCODER_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);
}

/**
 * @brief 右轮GPIO AB边沿中断：解码四倍频计数并记录非法跳变。
 * @note 中断优先级高于控制IRQ，不能在此打印或刷新屏幕。
 */
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

/**
 * @brief 读取两轮相邻采样差值和累计位置。
 * @param[out] sample 反馈样本，增量单位为计数/本次10ms窗口。
 * @note 只在控制IRQ调用；内部短临界区保证读取右轮计数一致。
 */
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
