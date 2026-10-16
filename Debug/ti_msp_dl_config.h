/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for MOTOR_PWM */
#define MOTOR_PWM_INST                                                     TIMG7
#define MOTOR_PWM_INST_IRQHandler                               TIMG7_IRQHandler
#define MOTOR_PWM_INST_INT_IRQN                                 (TIMG7_INT_IRQn)
#define MOTOR_PWM_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_MOTOR_PWM_C0_PORT                                             GPIOB
#define GPIO_MOTOR_PWM_C0_PIN                                     DL_GPIO_PIN_15
#define GPIO_MOTOR_PWM_C0_IOMUX                                  (IOMUX_PINCM32)
#define GPIO_MOTOR_PWM_C0_IOMUX_FUNC                 IOMUX_PINCM32_PF_TIMG7_CCP0
#define GPIO_MOTOR_PWM_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_MOTOR_PWM_C1_PORT                                             GPIOB
#define GPIO_MOTOR_PWM_C1_PIN                                     DL_GPIO_PIN_16
#define GPIO_MOTOR_PWM_C1_IOMUX                                  (IOMUX_PINCM33)
#define GPIO_MOTOR_PWM_C1_IOMUX_FUNC                 IOMUX_PINCM33_PF_TIMG7_CCP1
#define GPIO_MOTOR_PWM_C1_IDX                                DL_TIMER_CC_1_INDEX




/* Defines for ENCODER_LEFT */
#define ENCODER_LEFT_INST                                                  TIMG8
#define ENCODER_LEFT_INST_IRQHandler                            TIMG8_IRQHandler
#define ENCODER_LEFT_INST_INT_IRQN                              (TIMG8_INT_IRQn)
/* Pin configuration defines for ENCODER_LEFT PHA Pin */
#define GPIO_ENCODER_LEFT_PHA_PORT                                         GPIOB
#define GPIO_ENCODER_LEFT_PHA_PIN                                  DL_GPIO_PIN_6
#define GPIO_ENCODER_LEFT_PHA_IOMUX                              (IOMUX_PINCM23)
#define GPIO_ENCODER_LEFT_PHA_IOMUX_FUNC             IOMUX_PINCM23_PF_TIMG8_CCP0
/* Pin configuration defines for ENCODER_LEFT PHB Pin */
#define GPIO_ENCODER_LEFT_PHB_PORT                                         GPIOB
#define GPIO_ENCODER_LEFT_PHB_PIN                                  DL_GPIO_PIN_7
#define GPIO_ENCODER_LEFT_PHB_IOMUX                              (IOMUX_PINCM24)
#define GPIO_ENCODER_LEFT_PHB_IOMUX_FUNC             IOMUX_PINCM24_PF_TIMG8_CCP1


/* Defines for CONTROL_TIMER */
#define CONTROL_TIMER_INST                                               (TIMG0)
#define CONTROL_TIMER_INST_IRQHandler                           TIMG0_IRQHandler
#define CONTROL_TIMER_INST_INT_IRQN                             (TIMG0_INT_IRQn)
#define CONTROL_TIMER_INST_LOAD_VALUE                                    (9999U)




/* Defines for OLED_I2C */
#define OLED_I2C_INST                                                       I2C1
#define OLED_I2C_INST_IRQHandler                                 I2C1_IRQHandler
#define OLED_I2C_INST_INT_IRQN                                     I2C1_INT_IRQn
#define OLED_I2C_BUS_SPEED_HZ                                             400000
#define GPIO_OLED_I2C_SDA_PORT                                             GPIOB
#define GPIO_OLED_I2C_SDA_PIN                                      DL_GPIO_PIN_3
#define GPIO_OLED_I2C_IOMUX_SDA                                  (IOMUX_PINCM16)
#define GPIO_OLED_I2C_IOMUX_SDA_FUNC                   IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_OLED_I2C_SCL_PORT                                             GPIOB
#define GPIO_OLED_I2C_SCL_PIN                                      DL_GPIO_PIN_2
#define GPIO_OLED_I2C_IOMUX_SCL                                  (IOMUX_PINCM15)
#define GPIO_OLED_I2C_IOMUX_SCL_FUNC                   IOMUX_PINCM15_PF_I2C1_SCL

/* Defines for IMU_I2C */
#define IMU_I2C_INST                                                        I2C0
#define IMU_I2C_INST_IRQHandler                                  I2C0_IRQHandler
#define IMU_I2C_INST_INT_IRQN                                      I2C0_INT_IRQn
#define IMU_I2C_BUS_SPEED_HZ                                              400000
#define GPIO_IMU_I2C_SDA_PORT                                              GPIOA
#define GPIO_IMU_I2C_SDA_PIN                                       DL_GPIO_PIN_0
#define GPIO_IMU_I2C_IOMUX_SDA                                    (IOMUX_PINCM1)
#define GPIO_IMU_I2C_IOMUX_SDA_FUNC                     IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_IMU_I2C_SCL_PORT                                              GPIOA
#define GPIO_IMU_I2C_SCL_PIN                                       DL_GPIO_PIN_1
#define GPIO_IMU_I2C_IOMUX_SCL                                    (IOMUX_PINCM2)
#define GPIO_IMU_I2C_IOMUX_SCL_FUNC                     IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for DEBUG_UART */
#define DEBUG_UART_INST                                                    UART0
#define DEBUG_UART_INST_FREQUENCY                                       32000000
#define DEBUG_UART_INST_IRQHandler                              UART0_IRQHandler
#define DEBUG_UART_INST_INT_IRQN                                  UART0_INT_IRQn
#define GPIO_DEBUG_UART_RX_PORT                                            GPIOA
#define GPIO_DEBUG_UART_TX_PORT                                            GPIOA
#define GPIO_DEBUG_UART_RX_PIN                                    DL_GPIO_PIN_11
#define GPIO_DEBUG_UART_TX_PIN                                    DL_GPIO_PIN_10
#define GPIO_DEBUG_UART_IOMUX_RX                                 (IOMUX_PINCM22)
#define GPIO_DEBUG_UART_IOMUX_TX                                 (IOMUX_PINCM21)
#define GPIO_DEBUG_UART_IOMUX_RX_FUNC                  IOMUX_PINCM22_PF_UART0_RX
#define GPIO_DEBUG_UART_IOMUX_TX_FUNC                  IOMUX_PINCM21_PF_UART0_TX
#define DEBUG_UART_BAUD_RATE                                            (115200)
#define DEBUG_UART_IBRD_32_MHZ_115200_BAUD                                  (17)
#define DEBUG_UART_FBRD_32_MHZ_115200_BAUD                                  (23)





/* Port definition for Pin Group GPIO_BUZZER */
#define GPIO_BUZZER_PORT                                                 (GPIOA)

/* Defines for OUT: GPIOA.21 with pinCMx 46 on package pin 17 */
#define GPIO_BUZZER_OUT_PIN                                     (DL_GPIO_PIN_21)
#define GPIO_BUZZER_OUT_IOMUX                                    (IOMUX_PINCM46)
/* Defines for AIN1: GPIOA.15 with pinCMx 37 on package pin 8 */
#define GPIO_MOTOR_AIN1_PORT                                             (GPIOA)
#define GPIO_MOTOR_AIN1_PIN                                     (DL_GPIO_PIN_15)
#define GPIO_MOTOR_AIN1_IOMUX                                    (IOMUX_PINCM37)
/* Defines for AIN2: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_MOTOR_AIN2_PORT                                             (GPIOA)
#define GPIO_MOTOR_AIN2_PIN                                     (DL_GPIO_PIN_16)
#define GPIO_MOTOR_AIN2_IOMUX                                    (IOMUX_PINCM38)
/* Defines for BIN1: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_MOTOR_BIN1_PORT                                             (GPIOB)
#define GPIO_MOTOR_BIN1_PIN                                      (DL_GPIO_PIN_0)
#define GPIO_MOTOR_BIN1_IOMUX                                    (IOMUX_PINCM12)
/* Defines for BIN2: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_MOTOR_BIN2_PORT                                             (GPIOB)
#define GPIO_MOTOR_BIN2_PIN                                      (DL_GPIO_PIN_1)
#define GPIO_MOTOR_BIN2_IOMUX                                    (IOMUX_PINCM13)
/* Defines for HEART: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GPIO_LED_HEART_PORT                                              (GPIOA)
#define GPIO_LED_HEART_PIN                                      (DL_GPIO_PIN_22)
#define GPIO_LED_HEART_IOMUX                                     (IOMUX_PINCM47)
/* Defines for LEFT: GPIOB.14 with pinCMx 31 on package pin 2 */
#define GPIO_LED_LEFT_PORT                                               (GPIOB)
#define GPIO_LED_LEFT_PIN                                       (DL_GPIO_PIN_14)
#define GPIO_LED_LEFT_IOMUX                                      (IOMUX_PINCM31)
/* Defines for RIGHT: GPIOB.18 with pinCMx 44 on package pin 15 */
#define GPIO_LED_RIGHT_PORT                                              (GPIOB)
#define GPIO_LED_RIGHT_PIN                                      (DL_GPIO_PIN_18)
#define GPIO_LED_RIGHT_IOMUX                                     (IOMUX_PINCM44)
/* Port definition for Pin Group GPIO_LINE */
#define GPIO_LINE_PORT                                                   (GPIOB)

/* Defines for L2: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_LINE_L2_PIN                                        (DL_GPIO_PIN_13)
#define GPIO_LINE_L2_IOMUX                                       (IOMUX_PINCM30)
/* Defines for L1: GPIOB.12 with pinCMx 29 on package pin 64 */
#define GPIO_LINE_L1_PIN                                        (DL_GPIO_PIN_12)
#define GPIO_LINE_L1_IOMUX                                       (IOMUX_PINCM29)
/* Defines for MID: GPIOB.11 with pinCMx 28 on package pin 63 */
#define GPIO_LINE_MID_PIN                                       (DL_GPIO_PIN_11)
#define GPIO_LINE_MID_IOMUX                                      (IOMUX_PINCM28)
/* Defines for R1: GPIOB.10 with pinCMx 27 on package pin 62 */
#define GPIO_LINE_R1_PIN                                        (DL_GPIO_PIN_10)
#define GPIO_LINE_R1_IOMUX                                       (IOMUX_PINCM27)
/* Defines for R2: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_LINE_R2_PIN                                         (DL_GPIO_PIN_9)
#define GPIO_LINE_R2_IOMUX                                       (IOMUX_PINCM26)
/* Port definition for Pin Group GPIO_ENCODER */
#define GPIO_ENCODER_PORT                                                (GPIOA)

/* Defines for RIGHT_A: GPIOA.24 with pinCMx 54 on package pin 25 */
// pins affected by this interrupt request:["RIGHT_A","RIGHT_B"]
#define GPIO_ENCODER_INT_IRQN                                   (GPIOA_INT_IRQn)
#define GPIO_ENCODER_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_ENCODER_RIGHT_A_IIDX                           (DL_GPIO_IIDX_DIO24)
#define GPIO_ENCODER_RIGHT_A_PIN                                (DL_GPIO_PIN_24)
#define GPIO_ENCODER_RIGHT_A_IOMUX                               (IOMUX_PINCM54)
/* Defines for RIGHT_B: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_ENCODER_RIGHT_B_IIDX                           (DL_GPIO_IIDX_DIO25)
#define GPIO_ENCODER_RIGHT_B_PIN                                (DL_GPIO_PIN_25)
#define GPIO_ENCODER_RIGHT_B_IOMUX                               (IOMUX_PINCM55)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_MOTOR_PWM_init(void);
void SYSCFG_DL_ENCODER_LEFT_init(void);
void SYSCFG_DL_CONTROL_TIMER_init(void);
void SYSCFG_DL_OLED_I2C_init(void);
void SYSCFG_DL_IMU_I2C_init(void);
void SYSCFG_DL_DEBUG_UART_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
