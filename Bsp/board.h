/**
 * @file    Bsp/board.h
 * @brief   板级接口和生成配置约束。
 * @details 仅约定32MHz、外设实例与GPIO组织；具体引脚由ti_msp_dl_config.h提供。
 */
#ifndef ROBOT_BOARD_H
#define ROBOT_BOARD_H
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#if !defined(MOTOR_PWM_INST) || !defined(CONTROL_TIMER_INST) || \
    !defined(ENCODER_LEFT_INST) || !defined(OLED_I2C_INST) || !defined(IMU_I2C_INST)
#error "请先按 docs/SysConfig配置清单.md 配置并生成 SysConfig。"
#endif
#if CPUCLK_FREQ != 32000000
#error "本工程验证时钟为 32 MHz，请按配置清单设置 SYSCTL。"
#endif

/* 单一硬件映射入口。组名称固定后，在 SysConfig 中改引脚即可。
 * 同一 GPIO 组必须保持文档约定的端口组织，否则生成的宏名也会改变。 */
#define BOARD_RIGHT_ENCODER_MASK (GPIO_ENCODER_RIGHT_A_PIN | GPIO_ENCODER_RIGHT_B_PIN)

#define BOARD_TIMER_PWM_CLOCK_ERROR  (1U << 0)
#define BOARD_TIMER_PWM_LOAD_ERROR   (1U << 1)
#define BOARD_TIMER_CTRL_CLOCK_ERROR (1U << 2)
#define BOARD_TIMER_CTRL_LOAD_ERROR  (1U << 3)
typedef struct {
    uint32_t errors;
    uint32_t pwm_load, control_load;
} BoardTimerDiagnostics;

/**
 * @brief 调用SysConfig初始化，建立1ms时基并设置中断优先级。
 * @note 在empty.c的System_Init第一步调用；不在此启动10ms底盘控制定时器。
 */
void Board_Init(void);
/**
 * @brief 检查PWM与控制定时器的实际时钟和LOAD是否符合约定。
 * @retval true全部匹配；false存在配置错误。
 */
bool Board_ValidateTimers(void);
/**
 * @brief 读取定时器实际参数并生成错误位。
 * @param[out] diagnostics 检查结果及两个LOAD；非空。
 * @note 对照SysConfig清单核对PWM LOAD=1599、控制LOAD=9999。
 */
void Board_ReadTimerDiagnostics(BoardTimerDiagnostics *diagnostics);
/**
 * @brief 读取当前毫秒计数。
 * @retval uint32毫秒时间，约49.7天回绕；时间差须无符号相减。
 */
uint32_t Board_Millis(void);
/**
 * @brief 忙等指定毫秒数，供设备复位等初始化阶段使用。
 * @param[in] duration 等待时间ms。
 * @note 依赖SysTick可执行，禁止在控制IRQ或关中断期间调用。
 */
void Board_DelayMs(uint32_t duration);
/**
 * @brief 同一端口一次采样五路循迹GPIO并归一化黑线电平。
 * @retval bit0..4依次为最左到最右，黑线为1。
 */
uint8_t Board_LineBits(void);
/**
 * @brief 更新心跳、左侧和右侧LED的逻辑亮灭状态。
 * @param[in] heart 心跳灯是否亮。
 * @param[in] left 左灯是否亮。
 * @param[in] right 右灯是否亮。
 * @note 自动按本文件顶部的有效电平转换。
 */
void Board_Leds(bool heart, bool left, bool right);
/**
 * @brief 控制有源蜂鸣器的逻辑开关。
 * @param[in] on true响，false关闭。
 */
void Board_Buzzer(bool on);
/* 用于极短的共享快照；恢复进入函数前的 PRIMASK，不擅自打开中断。 */
/**
 * @brief 保存当前PRIMASK并暂时关闭可屏蔽中断。
 * @retval 原中断屏蔽值，必须传回Board_Unlock。
 */
uint32_t Board_Lock(void);
/**
 * @brief 恢复进入临界区之前的中断屏蔽状态。
 * @param[in] saved 对应Board_Lock返回值。
 */
void Board_Unlock(uint32_t saved);
#endif
