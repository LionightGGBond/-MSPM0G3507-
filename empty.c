/**
 * @file    empty.c
 * @brief   程序入口、统一初始化、前台主循环和10ms底盘控制中断。
 * @details 阅读顺序：main的while(1)→System_Init→CONTROL_TIMER_INST_IRQHandler。
 *          主循环负责串口、心跳、IMU、重试和OLED；中断负责固定周期电机控制。
 *          在顶部修改前台任务节拍；底盘参数在Motion/，硬件参数在Bsp/与Devices/。
 */
/*
 * Copyright (c) 2021, Texas Instruments Incorporated
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

#include "robot_config.h"
#include "Bsp/board.h"
#include "Bsp/tb6612.h"
#include "Bsp/encoder.h"
#include "Bsp/debug_uart.h"
#include "Devices/oled.h"
#include "Devices/mpu6050.h"
#include "App/robot_app.h"

/* ======================== 主循环任务节拍（单位ms） ======================== */
#define ROBOT_DISPLAY_MS          100U   /* OLED整帧绘制周期，I2C仍逐页发送。 */
#define ROBOT_DEBUG_REPORT_MS      500U   /* 串口诊断输出周期。 */
#define ROBOT_DEVICE_RETRY_MS      1000U  /* 停车时缺失设备重试周期。 */
#define ROBOT_START_BEEP_MS        100U   /* 启动提示音时长。 */
#define ROBOT_FAULT_BEEP_PERIOD_MS 600U   /* 故障提示音周期。 */
#define ROBOT_FAULT_BEEP_ON_MS     100U   /* 每个故障提示音的响铃时长。 */
#define ROBOT_HEARTBEAT_ON_MS     500U   /* 每秒心跳灯亮起的时长，其余时间熄灭。 */

/* ======================== 应用运行变量 ======================== */
static Controller s_control;             /* 控制ISR维护，前台经快照读取。 */
static EncoderSample s_encoder;          /* 本次10ms增量与累计位置。 */
static volatile uint32_t s_main_alive_ms; /* 前台写入，控制ISR检查是否超时。 */
static bool s_oled_ready, s_imu_ready;    /* 设备初始化结果。 */
static uint32_t s_last_imu, s_last_display, s_start_ms, s_retry_ms; /* ms时间戳。 */
static uint8_t s_page = 8U;               /* 0..7待发送页；8表示本帧完成。 */
static uint32_t s_heartbeat_seconds, s_heartbeat_epoch_ms; /* 秒计数、当前秒起点。 */
static bool s_heartbeat_led_on;           /* LED与OLED星号共用相位。 */
static volatile uint32_t s_control_irq_count, s_control_dt_ms; /* 中断诊断。 */
static BoardTimerDiagnostics s_timer_diagnostics; /* 初始化时检查结果。 */
static uint32_t s_last_report;            /* 最近一次串口报告时间，单位ms。 */
static bool s_report_requested;          /* 收到?命令时立即请求一组报告。 */

/* 私有函数声明：初始化集中在这里，主循环和控制中断见本文件下方。 */
static void System_Init(void);
static void Robot_GetSnapshot(RobotSnapshot *snapshot);
static void UpdateHeartbeat(uint32_t now_ms);
static bool MotorsDriven(RobotState state);
static void SerialCommands(void);

/**
 * @brief 程序入口：统一初始化后进入前台循环。
 * @note  这里只执行低优先级任务；电机控制由下方10ms中断执行。
 *        后续K230解析、云台指令可在串口任务后增加，长计算不可放进控制IRQ。
 */
int main(void)
{
    System_Init();

    while (1) {
        /* 1. 主循环存活标记与串口收发：不在控制中断里等待整包。 */
        uint32_t now = Board_Millis();
        s_main_alive_ms = now;
        DebugUart_Poll();
        SerialCommands();
        now = Board_Millis();
        s_main_alive_ms = now;
        /* 2. 心跳、快照、LED和蜂鸣器；快照短暂关中断保护64位位置。 */
        UpdateHeartbeat(now);
        RobotSnapshot snapshot;
        Robot_GetSnapshot(&snapshot);
        bool fault = snapshot.state >= ROBOT_LINE_LOST;
        Board_Leds(snapshot.heartbeat_led_on,
            (snapshot.line_bits & 1U) != 0U, (snapshot.line_bits & 16U) != 0U);
        Board_Buzzer((uint32_t)(now - s_start_ms) < ROBOT_START_BEEP_MS ||
            (fault && now % ROBOT_FAULT_BEEP_PERIOD_MS < ROBOT_FAULT_BEEP_ON_MS));
        /* 3. IMU按模块采样周期读取；姿态目前不参与底盘控制。 */
        if (s_imu_ready && (uint32_t)(now - s_last_imu) >= g_mpu6050_sample_period_ms) {
            s_last_imu = now;
            bool stationary_stop = !MotorsDriven(snapshot.state) &&
                                   snapshot.encoder.left_delta == 0 && snapshot.encoder.right_delta == 0;
            (void)Mpu6050_Update(now, stationary_stop);
        }
        /* 4. 设备缺失不影响底盘固定周期。只有停车时尝试重新初始化缺失设备。 */
        bool retry_allowed = !MotorsDriven(snapshot.state) && snapshot.state != ROBOT_TEST_STOPPED;
        if (retry_allowed && (uint32_t)(now - s_retry_ms) >= ROBOT_DEVICE_RETRY_MS) {
            s_retry_ms = now;
            if (!s_oled_ready) {
                s_main_alive_ms = Board_Millis();
                s_oled_ready = Oled_Init();
                s_page = 8U;
            }
            if (!s_imu_ready) {
                s_main_alive_ms = Board_Millis();
                s_imu_ready = Mpu6050_Init();
            }
        }
        /* 重试可能改变IMU就绪状态，绘制本帧前同步；控制快照无需重新读取。 */
        snapshot.imu_ready = s_imu_ready;

        /* 5. 中文串口诊断，默认500ms一组。 */
        if (DebugUart_Enabled() && (s_report_requested ||
            (uint32_t)(now - s_last_report) >= ROBOT_DEBUG_REPORT_MS)) {
            s_report_requested = false;
            s_last_report = now;
            RobotApp_Report(&snapshot);
        }
        /* 6. OLED默认100ms绘制新帧，每圈只传一页，避免长时间阻塞。 */
        if (s_oled_ready && s_page >= 8U && (uint32_t)(now - s_last_display) >= ROBOT_DISPLAY_MS) {
            s_last_display = now;
            RobotApp_Render(&snapshot);
            s_page = 0U;
        }
        if (s_oled_ready && s_page < 8U) {
            if (Oled_FlushPage(s_page)) ++s_page;
            else { s_oled_ready = false; s_page = 8U; }
        }
    }
}

/* ======================== 统一初始化 ======================== */
/**
 * @brief 统一初始化全部硬件、软件运行数据，最后启动控制定时器。
 * @note 无返回值；OLED/IMU初始化结果保存在就绪标志中。定时器配置错误时保持刹车。
 */
static void System_Init(void)
{
    /* 1. SysConfig外设、32MHz时钟、GPIO及1ms SysTick。 */
    Board_Init();
    /* 2. 建立制动状态后再启动PWM；编码器在控制定时器之前启动。 */
    TB6612_Init();
    Encoder_Init();
    /* 3. 设备初始化：失败只影响显示/诊断，不阻止底盘循迹。 */
    s_oled_ready = Oled_Init();
    s_imu_ready = Mpu6050_Init();

    /* 4. 初始化应用变量、PID/状态机和界面节拍。 */
    s_main_alive_ms = s_start_ms = Board_Millis();
    s_last_imu = s_last_display = s_retry_ms = s_start_ms;
    s_heartbeat_epoch_ms = s_start_ms;
    s_heartbeat_seconds = 0U;
    s_heartbeat_led_on = true;
    s_last_report = s_start_ms;
    s_report_requested = true;
    (void)DebugUart_Write("\r\n小车串口诊断：115200，8N1，接收编码UTF-8\r\n"
        "?：状态/帮助；s：暂停测试；r：继续测试；i：暂停后重新初始化IMU\r\n");
    s_control_irq_count = s_control_dt_ms = 0U;
    Controller_Init(&s_control, s_start_ms);
    /* 5. 核对GUI生成的定时器参数，最后开放10ms控制中断。 */
    Board_ReadTimerDiagnostics(&s_timer_diagnostics);
    if (s_timer_diagnostics.errors != 0U) {
        s_control.state = ROBOT_TIMING_ERROR;
        TB6612_Brake();
        return;
    }
    DL_TimerG_clearInterruptStatus(CONTROL_TIMER_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);
    DL_TimerG_startCounter(CONTROL_TIMER_INST);
}

/* ======================== 固定周期电机控制 ======================== */
/**
 * @brief CONTROL_TIMER的10ms零事件中断：反馈采样→循迹/PID→电机输出。
 * @note  由SysConfig映射为TIMG0_IRQHandler。此处不能刷屏、打印或等待I2C。
 *        GPIO编码器和SysTick仍由各自BSP文件处理，不重复定义中断入口。
 */
void CONTROL_TIMER_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(CONTROL_TIMER_INST) == DL_TIMERG_IIDX_ZERO) {
        Encoder_Sample(&s_encoder);
        uint32_t now_ms = Board_Millis();
        ++s_control_irq_count;
        s_control_dt_ms = now_ms - s_control.last_step_ms;
        Controller_Step(&s_control, now_ms, s_main_alive_ms, Board_LineBits(),
                         s_encoder.left_delta, s_encoder.right_delta);
        TB6612_Set(s_control.left_pwm, s_control.right_pwm);
    }
}

/* ======================== 主循环辅助函数 ======================== */
/**
 * @brief 按真实时间更新每秒计数及与PA22同相的闪烁标志。
 * @param[in] now_ms 1ms SysTick毫秒时间，允许无符号回绕。
 */
static void UpdateHeartbeat(uint32_t now_ms)
{
    /* 与控制ISR解耦：故障停车时主循环仍能计数。延迟时按经过的整秒补计，
     * 不用now直接除1000，避免毫秒计数回绕时心跳数字突然归零。 */
    uint32_t elapsed = now_ms - s_heartbeat_epoch_ms;
    uint32_t whole_seconds = elapsed / 1000U;
    s_heartbeat_seconds += whole_seconds;
    s_heartbeat_epoch_ms += whole_seconds * 1000U;
    /* 同一时间基准：亮500ms、灭500ms；每次进入亮相位时数字加一。 */
    s_heartbeat_led_on = (uint32_t)(now_ms - s_heartbeat_epoch_ms) < ROBOT_HEARTBEAT_ON_MS;
}

/**
 * @brief 原子读取控制数据，形成显示与日志使用的一致性快照。
 * @param[out] snapshot 非空输出结构体，包含64位位置和当前GPIO线型。
 * @note 仅主循环调用；函数保存并恢复进入时的中断屏蔽状态。
 */
static void Robot_GetSnapshot(RobotSnapshot *snapshot)
{
    /* 不将 volatile 当作互斥锁；快照期间禁止中断，尤其保证64位位置一致。 */
    uint32_t saved = Board_Lock();
    snapshot->state = s_control.state;
    snapshot->timer_diagnostics = s_timer_diagnostics;
    snapshot->imu_ready = s_imu_ready;
    snapshot->line_fallback = s_control.line_fallback;
    snapshot->left_target = s_control.left_target;
    snapshot->right_target = s_control.right_target;
    snapshot->left_speed = s_control.left_speed;
    snapshot->right_speed = s_control.right_speed;
    snapshot->left_pwm = s_control.left_pwm;
    snapshot->right_pwm = s_control.right_pwm;
    snapshot->encoder = s_encoder;
    TB6612_ReadInputLevels(&snapshot->motor_inputs);
    snapshot->now_ms = Board_Millis();
    snapshot->heartbeat_seconds = s_heartbeat_seconds;
    snapshot->heartbeat_led_on = s_heartbeat_led_on;
    snapshot->control_irq_count = s_control_irq_count;
    snapshot->control_dt_ms = s_control_dt_ms;
    Board_Unlock(saved);
    /* 配置错误时控制ISR尚未启动，显示仍需反映真实五路GPIO，不能一直显示初值0。 */
    snapshot->line_bits = Board_LineBits();
}

/**
 * @brief 判断当前是否处于主动电机运行状态。
 * @param[in] state 底盘状态。
 * @retval true 循迹运行或台架转动；false 等待、停车、暂停或故障。
 */
static bool MotorsDriven(RobotState state)
{
    return state == ROBOT_RUNNING || state == ROBOT_TEST_RUNNING;
}

/**
 * @brief 解析串口单字符命令并发出请求，避免在接收驱动中写入业务逻辑。
 * @note 每圈最多处理16字节；s/r仅用于台架测试，i只允许在明确停车时重置IMU。
 */
static void SerialCommands(void)
{
    char character;
    for (unsigned int n = 0U; n < 16U && DebugUart_Read(&character); ++n) {
        if (character == '?' || character == 'h') {
            s_report_requested = true;
            (void)DebugUart_Write("?：状态/帮助；s：暂停测试；r：继续测试；i：暂停或故障停车后重新初始化IMU\r\n");
        } else if (character == 's' || character == 'r') {
#if ROBOT_MOTOR_TEST_MODE
            uint32_t saved = Board_Lock();
            s_control.test_pause_requested = character == 's';
            Board_Unlock(saved);
            (void)DebugUart_Write(character == 's' ? "已请求暂停：下一次10ms控制中断刹车\r\n" :
                "已请求继续测试：故障锁存仍需修正后复位\r\n");
#else
            (void)DebugUart_Write("s/r命令仅在电机测试模式下有效\r\n");
#endif
            s_report_requested = true;
        } else if (character == 'i') {
            RobotSnapshot snapshot;
            Robot_GetSnapshot(&snapshot);
            /* 自动2秒停车很快会再次起转；手动重置IMU必须先暂停或已锁存停车。 */
            if (snapshot.state == ROBOT_TEST_PAUSED || snapshot.state == ROBOT_FINISHED ||
                snapshot.state >= ROBOT_LINE_LOST) {
                s_main_alive_ms = Board_Millis();
                s_imu_ready = Mpu6050_Init();
                s_main_alive_ms = Board_Millis();
                (void)DebugUart_Write(s_imu_ready ? "IMU初始化成功，请保持静止完成零偏标定\r\n" :
                                                       "IMU初始化失败，请查看后续IMU诊断\r\n");
            } else {
                (void)DebugUart_Write("当前不能重置IMU：先发s，确认状态为测试暂停，再发i\r\n");
            }
            s_report_requested = true;
        }
    }
}
