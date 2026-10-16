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

void Board_Init(void);
bool Board_ValidateTimers(void);
void Board_ReadTimerDiagnostics(BoardTimerDiagnostics *diagnostics);
uint32_t Board_Millis(void);
void Board_DelayMs(uint32_t duration);
uint8_t Board_LineBits(void);
void Board_Leds(bool heart, bool left, bool right);
void Board_Buzzer(bool on);
/* 用于极短的共享快照；恢复进入函数前的 PRIMASK，不擅自打开中断。 */
uint32_t Board_Lock(void);
void Board_Unlock(uint32_t saved);
#endif
