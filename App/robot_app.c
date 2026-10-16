#include "robot_app.h"
#include "../Bsp/board.h"
#include "../Bsp/tb6612.h"
#include "../Bsp/debug_uart.h"
#include "../Bsp/i2c_bus.h"
#include "../Devices/oled.h"
#include "../Devices/mpu6050.h"
#include "../robot_config.h"
#include <math.h>
#include <stdio.h>

static Controller s_control;
static EncoderSample s_encoder;
static volatile uint32_t s_main_alive_ms;
static bool s_oled_ready, s_imu_ready;
static uint32_t s_last_imu, s_last_display, s_start_ms, s_retry_ms;
static uint8_t s_page = 8U;
static uint32_t s_heartbeat_seconds, s_heartbeat_epoch_ms;
static bool s_heartbeat_led_on;
static volatile uint32_t s_control_irq_count, s_control_dt_ms;
static BoardTimerDiagnostics s_timer_diagnostics;
static uint32_t s_last_report;
static bool s_report_requested;

static void UpdateHeartbeat(uint32_t now_ms)
{
    /* 与控制ISR解耦：故障停车时主循环仍能计数。延迟时按经过的整秒补计，
     * 不用now直接除1000，避免毫秒计数回绕时心跳数字突然归零。 */
    uint32_t elapsed = now_ms - s_heartbeat_epoch_ms;
    uint32_t whole_seconds = elapsed / 1000U;
    s_heartbeat_seconds += whole_seconds;
    s_heartbeat_epoch_ms += whole_seconds * 1000U;
    /* 同一时间基准：亮500ms、灭500ms；每次进入亮相位时数字加一。 */
    s_heartbeat_led_on = (uint32_t)(now_ms - s_heartbeat_epoch_ms) < 500U;
}

void RobotApp_Snapshot(RobotSnapshot *snapshot)
{
    /* 不将 volatile 当作互斥锁；快照期间禁止中断，尤其保证64位位置一致。 */
    uint32_t saved = Board_Lock();
    snapshot->state = s_control.state;
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

void RobotApp_Init(bool oled_ready, bool imu_ready)
{
    s_oled_ready = oled_ready;
    s_imu_ready = imu_ready;
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

static void FormatAngle(char *text, size_t size, const char *label, float angle, bool valid)
{
    if (!valid || !isfinite(angle)) {
        (void)snprintf(text, size, "%s:  ---.- deg", label);
        return;
    }
    /* 保留一位小数而不启用printf浮点格式，避免截断小角度和负零显示。 */
    unsigned int tenths = (unsigned int)(fabsf(angle) * 10.0f + 0.5f);
    char sign = angle < 0.0f && tenths != 0U ? '-' : '+';
    (void)snprintf(text, size, "%s:%c%3u.%u deg", label, sign, tenths / 10U, tenths % 10U);
}

static void Render(const RobotSnapshot *s)
{
    char text[32];
    const ImuData *imu = Mpu6050_Data();
    Oled_Clear();
    const char *state_name = Controller_StateName(s->state);
    if (s_timer_diagnostics.errors != 0U) {
        uint32_t pwm_errors = BOARD_TIMER_PWM_CLOCK_ERROR | BOARD_TIMER_PWM_LOAD_ERROR;
        state_name = (s_timer_diagnostics.errors & pwm_errors) != 0U ? "PWM CONFIG" : "CTRL CONFIG";
    } else if (s->state == ROBOT_LINE_AMBIGUOUS) {
        state_name = "CROSS/INV";
    } else if (s->state == ROBOT_MAIN_TIMEOUT) {
        state_name = "MAIN STALL";
    }
    Oled_Text(0U, 0U, state_name);
    /* 右上角六位十进制计数，*与PA22心跳灯同相；屏幕六位数满后回绕。 */
    (void)snprintf(text, sizeof(text), "H:%06lu%c",
        (unsigned long)(s->heartbeat_seconds % 1000000U), s->heartbeat_led_on ? '*' : ' ');
    Oled_Text(0U, 12U, text);
    bool valid = s_imu_ready && imu->online && imu->attitude_valid;
    FormatAngle(text, sizeof(text), "PITCH", imu->pitch_deg, valid);
    Oled_Text(1U, 0U, text);
    FormatAngle(text, sizeof(text), "ROLL ", imu->roll_deg, valid);
    Oled_Text(2U, 0U, text);
    FormatAngle(text, sizeof(text), "YAW  ", imu->yaw_deg, valid && imu->calibrated);
    Oled_Text(3U, 0U, text);
    if (!s_imu_ready || !imu->online) {
        if (imu->fault == IMU_FAULT_ID) {
            (void)snprintf(text, sizeof(text), "IMU:%02X BAD ID:%02X", (unsigned)imu->device_address,
                           (unsigned)imu->who_am_i);
        } else {
            (void)snprintf(text, sizeof(text), "IMU:%02X %s E:%lu", (unsigned)imu->device_address,
                           Mpu6050_FaultName(), (unsigned long)imu->errors);
        }
    } else if (!imu->calibrated) {
        (void)snprintf(text, sizeof(text), "CAL:%lu/%u HOLD STILL", (unsigned long)imu->calibration_samples,
                       (unsigned)ROBOT_IMU_CALIBRATION_SAMPLES);
    } else if (!imu->attitude_valid) {
        (void)snprintf(text, sizeof(text), "IMU:WAIT GRAVITY");
    } else {
        (void)snprintf(text, sizeof(text), "IMU:OK %s YAW:REL",
                       imu->accel_correction_used ? "ACC" : "GYR");
    }
    Oled_Text(4U, 0U, text);
    (void)snprintf(text, sizeof(text), "LINE:%u%u%u%u%u HOLD:%u",
        (unsigned)((s->line_bits >> 0) & 1U), (unsigned)((s->line_bits >> 1) & 1U),
        (unsigned)((s->line_bits >> 2) & 1U), (unsigned)((s->line_bits >> 3) & 1U),
        (unsigned)((s->line_bits >> 4) & 1U), (unsigned)s->line_fallback);
    if (ROBOT_MOTOR_TEST_MODE) {
        Oled_Text(5U, 0U, DebugUart_Enabled() ? "TEST UART:ON  s=STOP" : "TEST UART:OFF      ");
    } else {
        Oled_Text(5U, 0U, text);
    }
    /* 使用整数格式，避免引入 printf 浮点格式化带来的栈与 Flash 开销。 */
    if (s->state == ROBOT_TIMING_ERROR) {
        (void)snprintf(text, sizeof(text), "PWM:%lu CT:%lu", (unsigned long)s_timer_diagnostics.pwm_load,
                       (unsigned long)s_timer_diagnostics.control_load);
        Oled_Text(6U, 0U, text);
        (void)snprintf(text, sizeof(text), "IRQ:%lu DT:%lums", (unsigned long)s->control_irq_count,
                       (unsigned long)s->control_dt_ms);
    } else if (ROBOT_MOTOR_TEST_MODE) {
        (void)snprintf(text, sizeof(text), "L PWM:%d ENC:%ld", (int)s->left_pwm,
                       (long)s->encoder.left_delta);
        Oled_Text(6U, 0U, text);
        (void)snprintf(text, sizeof(text), "R PWM:%d ENC:%ld", (int)s->right_pwm,
                       (long)s->encoder.right_delta);
    } else {
        (void)snprintf(text, sizeof(text), "L T:%d V:%d P:%d", (int)s->left_target,
                       (int)s->left_speed, (int)s->left_pwm);
        Oled_Text(6U, 0U, text);
        (void)snprintf(text, sizeof(text), "R T:%d V:%d P:%d", (int)s->right_target,
                       (int)s->right_speed, (int)s->right_pwm);
    }
    Oled_Text(7U, 0U, text);
}

static bool MotorsDriven(RobotState state)
{
    return state == ROBOT_RUNNING || state == ROBOT_TEST_RUNNING;
}

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
            RobotApp_Snapshot(&snapshot);
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

/* 中文名称只用于串口，OLED继续使用现有ASCII字库。源码和串口文本均为UTF-8。 */
static const char *SerialStateName(RobotState state)
{
    if (s_timer_diagnostics.errors != 0U) {
        uint32_t pwm_errors = BOARD_TIMER_PWM_CLOCK_ERROR | BOARD_TIMER_PWM_LOAD_ERROR;
        return (s_timer_diagnostics.errors & pwm_errors) != 0U ? "PWM配置错误" : "控制定时器配置错误";
    }
    switch (state) {
        case ROBOT_WAIT: return "启动等待";
        case ROBOT_RUNNING: return "循迹运行";
        case ROBOT_FINISHED: return "运行结束";
        case ROBOT_TEST_RUNNING: return "测试转动";
        case ROBOT_TEST_STOPPED: return "测试停车";
        case ROBOT_TEST_PAUSED: return "测试暂停";
        case ROBOT_LINE_LOST: return "丢线停车";
        case ROBOT_LINE_AMBIGUOUS: return "多线或无效线型停车";
        case ROBOT_ENCODER_STALL: return "编码器无反馈停车";
        case ROBOT_MAIN_TIMEOUT: return "主循环超时停车";
        case ROBOT_TIMING_ERROR: return "控制周期异常停车";
        default: return "未知状态";
    }
}

static const char *SerialImuFaultName(const ImuData *imu)
{
    if (imu->fault != IMU_FAULT_NONE && imu->last_bus_result == (uint8_t)I2C_TIMEOUT)
        return "通信超时";
    switch (imu->fault) {
        case IMU_FAULT_NONE: return "正常";
        case IMU_FAULT_PROBE: return "器件未应答";
        case IMU_FAULT_ID: return "芯片标识不符";
        case IMU_FAULT_RESET: return "复位失败";
        case IMU_FAULT_CONFIG: return "寄存器配置失败";
        case IMU_FAULT_READ: return "数据读取失败";
        default: return "未知故障";
    }
}

static const char *SerialBusName(uint8_t result)
{
    switch ((I2cResult)result) {
        case I2C_OK: return "成功";
        case I2C_TIMEOUT: return "超时";
        case I2C_DEVICE_ERROR: return "硬件通信错误";
        case I2C_ARGUMENT_ERROR: return "参数错误";
        default: return "未知错误";
    }
}

static const char *SerialI2cStageName(uint8_t stage)
{
    switch ((I2cStage)stage) {
        case I2C_STAGE_IDLE: return "等待总线空闲";
        case I2C_STAGE_FILL_TX: return "填充发送FIFO";
        case I2C_STAGE_TX: return "发送寄存器/配置";
        case I2C_STAGE_RX: return "接收寄存器数据";
        default: return "未传输";
    }
}

static void SerialReport(const RobotSnapshot *s)
{
    /* 中文UTF-8每个汉字通常占3字节，分行保持单行缓冲和2KB发送队列余量。
     * 每行独立入队，队列满时整行丢弃并累计；不使用阻塞printf发送。 */
    char text[448];
    const ImuData *imu = Mpu6050_Data();
    (void)snprintf(text, sizeof(text),
        "系统 时间=%lums 模式=%s 状态=%s 心跳=%lu 配置错误位=0x%lX "
        "PWM装载值=%lu 控制定时器装载值=%lu 中断次数=%lu 控制间隔=%lums\r\n",
        (unsigned long)s->now_ms, ROBOT_MOTOR_TEST_MODE ? "电机测试" : "循迹",
        SerialStateName(s->state), (unsigned long)s->heartbeat_seconds,
        (unsigned long)s_timer_diagnostics.errors, (unsigned long)s_timer_diagnostics.pwm_load,
        (unsigned long)s_timer_diagnostics.control_load, (unsigned long)s->control_irq_count,
        (unsigned long)s->control_dt_ms);
    (void)DebugUart_Write(text);
    (void)snprintf(text, sizeof(text),
        "底盘 循迹位=0x%02X 保持差速=%u 左轮(PWM=%d%%,计数=%ld) 右轮(PWM=%d%%,计数=%ld) "
        "方向输出(AIN1/2=%u%u,BIN1/2=%u%u) 右编码器非法跳变=%lu 发送丢行=%lu 接收错误=%lu\r\n",
        (unsigned)s->line_bits, (unsigned)s->line_fallback, (int)s->left_pwm,
        (long)s->encoder.left_delta, (int)s->right_pwm, (long)s->encoder.right_delta,
        (unsigned)s->motor_inputs.ain1_high, (unsigned)s->motor_inputs.ain2_high,
        (unsigned)s->motor_inputs.bin1_high, (unsigned)s->motor_inputs.bin2_high,
        (unsigned long)s->encoder.right_invalid_transitions,
        (unsigned long)DebugUart_Dropped(), (unsigned long)DebugUart_RxErrors());
    (void)DebugUart_Write(text);
    (void)snprintf(text, sizeof(text),
        "IMU 地址=0x%02X 芯片标识=0x%02X 在线=%s 故障=%s 总线=%s(%u) "
        "累计错误=%lu 标定=%lu/%u 标定完成=%s 姿态有效=%s 末次采样=%lums SCL=%u SDA=%u\r\n",
        (unsigned)imu->device_address, (unsigned)imu->who_am_i, imu->online ? "是" : "否",
        SerialImuFaultName(imu), SerialBusName(imu->last_bus_result),
        (unsigned)imu->last_bus_result, (unsigned long)imu->errors,
        (unsigned long)imu->calibration_samples, (unsigned)ROBOT_IMU_CALIBRATION_SAMPLES,
        imu->calibrated ? "是" : "否", imu->attitude_valid ? "是" : "否",
        (unsigned long)imu->sample_ms,
        /* 引脚已复用为I2C，读MBMON总线监视值，不用普通GPIO的DIN推断。 */
        (unsigned)(DL_I2C_getSCLStatus(IMU_I2C_INST) == DL_I2C_CONTROLLER_SCL_HIGH),
        (unsigned)(DL_I2C_getSDAStatus(IMU_I2C_INST) == DL_I2C_CONTROLLER_SDA_HIGH));
    (void)DebugUart_Write(text);
    /* 地址分别记录，0x69是最后尝试地址，不代表它确实有器件应答。
     * ADRACK/DATACK置1表示NACK，名称中的ACK不能误读为应答成功。 */
    (void)snprintf(text, sizeof(text),
        "IMU探测 68(已试=%u,结果=%u,标识=%02X,阶段=%s,MSR=%08lX) "
        "69(已试=%u,结果=%u,标识=%02X,阶段=%s,MSR=%08lX) "
        "最近MSR=%08lX 地址未应答=%u 数据未应答=%u 仲裁丢失=%u\r\n",
        (unsigned)imu->probe[0].attempted, (unsigned)imu->probe[0].bus_result,
        (unsigned)imu->probe[0].who_am_i, SerialI2cStageName(imu->probe[0].stage),
        (unsigned long)imu->probe[0].status,
        (unsigned)imu->probe[1].attempted, (unsigned)imu->probe[1].bus_result,
        (unsigned)imu->probe[1].who_am_i, SerialI2cStageName(imu->probe[1].stage),
        (unsigned long)imu->probe[1].status, (unsigned long)imu->last_i2c_status,
        (unsigned)((imu->last_i2c_status & DL_I2C_CONTROLLER_STATUS_ADDR_ACK) != 0U),
        (unsigned)((imu->last_i2c_status & DL_I2C_CONTROLLER_STATUS_DATA_ACK) != 0U),
        (unsigned)((imu->last_i2c_status & DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) != 0U));
    (void)DebugUart_Write(text);
    (void)snprintf(text, sizeof(text),
        "IMU数据 加速度原始=%d,%d,%d 陀螺仪原始=%d,%d,%d 温度原始=%d "
        "角度单位=0.1度 俯仰=%ld 横滚=%ld 偏航=%ld\r\n",
        (int)imu->accel_raw[0], (int)imu->accel_raw[1], (int)imu->accel_raw[2],
        (int)imu->gyro_raw[0], (int)imu->gyro_raw[1], (int)imu->gyro_raw[2], (int)imu->temperature_raw,
        (long)(imu->pitch_deg * 10.0f), (long)(imu->roll_deg * 10.0f), (long)(imu->yaw_deg * 10.0f));
    (void)DebugUart_Write(text);
}

void RobotApp_Poll(void)
{
    uint32_t now = Board_Millis();
    s_main_alive_ms = now;
    DebugUart_Poll();
    SerialCommands();
    now = Board_Millis();
    s_main_alive_ms = now;
    UpdateHeartbeat(now);
    RobotSnapshot snapshot;
    RobotApp_Snapshot(&snapshot);
    bool fault = snapshot.state >= ROBOT_LINE_LOST;
    Board_Leds(snapshot.heartbeat_led_on,
        (snapshot.line_bits & 1U) != 0U, (snapshot.line_bits & 16U) != 0U);
    Board_Buzzer((uint32_t)(now - s_start_ms) < 100U || (fault && now % 600U < 100U));
    if (s_imu_ready && (uint32_t)(now - s_last_imu) >= ROBOT_IMU_MS) {
        s_last_imu = now;
        bool stationary_stop = !MotorsDriven(snapshot.state) &&
                               snapshot.encoder.left_delta == 0 && snapshot.encoder.right_delta == 0;
        (void)Mpu6050_Update(now, stationary_stop);
    }
    /* 设备缺失不影响底盘固定周期。只有停车时尝试重新初始化缺失设备。 */
    bool retry_allowed = !MotorsDriven(snapshot.state) && snapshot.state != ROBOT_TEST_STOPPED;
    if (retry_allowed && (uint32_t)(now - s_retry_ms) >= 1000U) {
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
    if (DebugUart_Enabled() && (s_report_requested ||
        (uint32_t)(now - s_last_report) >= ROBOT_DEBUG_REPORT_MS)) {
        s_report_requested = false;
        s_last_report = now;
        SerialReport(&snapshot);
    }
    if (s_oled_ready && s_page >= 8U && (uint32_t)(now - s_last_display) >= ROBOT_DISPLAY_MS) {
        s_last_display = now;
        Render(&snapshot);
        s_page = 0U;
    }
    if (s_oled_ready && s_page < 8U) {
        if (Oled_FlushPage(s_page)) ++s_page;
        else { s_oled_ready = false; s_page = 8U; }
    }
}
