#include "target_track.h"
#include "f32c_motor.h"

/* USART1 接收 K230 靶心偏差。主循环依次完成零点校准、滤波、双轴速度计算，
 * 再由 USART2 向 F32C 发送目标转速。电机内部速度环负责跟随该转速。 */

/* ==================== 控制周期与视觉滤波 ==================== */
#define TRACK_FILTER_ALPHA        0.60f   /* 新帧权重，范围 (0, 1]。越小越平滑，响应越慢。 */
#define TRACK_PERIOD_MS           20U     /* 转速计算和发送周期，单位 ms，20 ms 对应 50 Hz。 */
#define TRACK_LOST_TIMEOUT_MS     200U    /* 有效视觉帧超时，单位 ms，达到后发零速。 */
#define TRACK_MOTOR_BOOT_MS       1000U   /* 上电后等待电机启动的时间，单位 ms。 */
#define TRACK_SETUP_RETRY_MS      250U    /* 电机初始化帧发送失败后的重试间隔，单位 ms。 */

/* ==================== 两轴调参区 ==================== */
typedef struct
{
    uint8_t address;              /* F32C 的串口地址，与上位机保存值一致。 */
    int8_t direction;             /* +1 或 -1，选能让本轴误差趋近零的方向。 */
    int16_t zero_px;               /* 激光点偏移，Y 轴沿用 K230 已乘 3/4 的坐标。 */
    float kp_rpm_per_px;           /* 视觉外环比例系数，单位 RPM/像素。 */
    float max_rpm;                /* 目标转速上限，单位 RPM。 */
    float start_px;               /* 停住后，误差达到此值才再次追踪。 */
    float stop_px;                /* 追踪中，误差进入此范围后减速至零。 */
    float min_accel_rpm_s;         /* 小误差下的加速斜率，单位 RPM/s。 */
    float max_accel_rpm_s;         /* 大误差下的加速斜率，单位 RPM/s。 */
    float full_accel_px;           /* 误差达到此值时用最大加速斜率。 */
    float brake_rpm_s;             /* 减速及换向前归零的斜率，单位 RPM/s。 */
} AxisTuning;

/* 水平轴参数。先确认方向和零点，再调速度、加速斜率及启停阈值。 */
static const AxisTuning x_tuning = {
    .address = 1U,                /* 水平电机在上位机保存的地址。 */
    .direction = 1,               /* +1 时转速与误差同号，-1 时反号。越追越偏就翻转符号。 */
    .zero_px = -5,                 /* 激光点相对图像中心的水平偏移，向左为正，单位 px。 */
    .kp_rpm_per_px = 0.14f,       /* 单位 RPM/px。调大后，同样的偏差会请求更高转速。 */
    .max_rpm = 15.0f,              /* 正反转速指令的绝对值上限，单位 RPM。 */
    .start_px = 6.0f,             /* 重新追踪的误差阈值，单位 px。调大可减少误启动。 */
    .stop_px = 3.0f,              /* 误差绝对值降到此值时开始停车，单位 px，应小于 start_px。 */
    .min_accel_rpm_s = 4.0f,      /* 加速斜率的下限，单位 RPM/s。调小后，小误差时起步更柔。 */
    .max_accel_rpm_s = 24.0f,     /* 加速斜率的上限，单位 RPM/s。调小后，大误差时提速更慢。 */
    .full_accel_px = 80.0f,       /* 达到最大加速斜率所需的误差，单位 px。调大后斜率随误差增长得更慢。 */
    .brake_rpm_s = 24.0f,         /* 减速和换向前的归零斜率，单位 RPM/s。调大后收速更快。 */
};

/* 俯仰轴参数。误差、标定偏移和启停阈值都使用 K230 缩放后的 Y 坐标。 */
static const AxisTuning y_tuning = {
    .address = 2U,                /* 俯仰电机在上位机保存的地址。 */
    .direction = -1,              /* +1 时转速与误差同号，-1 时反号。越追越偏就翻转符号。 */
    .zero_px = 0,                 /* 激光点相对图像中心的垂直偏移，向下为正，单位 px。 */
    .kp_rpm_per_px = 0.14f,       /* 单位 RPM/px。调大后，同样的偏差会请求更高转速。 */
    .max_rpm = 6.0f,              /* 正反转速指令的绝对值上限，单位 RPM。 */
    .start_px = 8.0f,             /* 重新追踪的误差阈值，单位 px。调大可减少误启动。 */
    .stop_px = 4.0f,              /* 误差绝对值降到此值时开始停车，单位 px，应小于 start_px。 */
    .min_accel_rpm_s = 1.0f,      /* 加速斜率的下限，单位 RPM/s。调小后，小误差时起步更柔。 */
    .max_accel_rpm_s = 6.0f,      /* 加速斜率的上限，单位 RPM/s。调小后，大误差时提速更慢。 */
    .full_accel_px = 60.0f,       /* 达到最大加速斜率所需的误差，单位 px。调大后斜率随误差增长得更慢。 */
    .brake_rpm_s = 8.0f,          /* 减速和换向前的归零斜率，单位 RPM/s。调大后收速更快。 */
};

/* ==================== K230 串口格式 ==================== */
/* 坐标帧为 @ex,ey#，丢靶帧为 @L#。 */
#define VISION_X_MAX_PX           160     /* X 偏差绝对值上限，来自 320 像素的识别画面。 */
#define VISION_Y_MAX_PX           120     /* Y 偏差绝对值上限，来自 K230 的 3/4 缩放。 */
#define VISION_FRAME_TIMEOUT_MS   15U     /* 从收到 @ 开始等待 # 的时限，单位 ms。 */
#define VISION_PAYLOAD_CAPACITY   12U     /* @ 和 # 之间的缓冲区容量，单位字节。 */

typedef struct
{
    uint32_t sequence;            /* 每次发布时递增，用于判断主循环是否读到新帧。 */
    uint32_t tick_ms;             /* 发布时刻，用于检查样本是否超时。 */
    int16_t x;
    int16_t y;
    uint8_t valid;                /* 1 表示检测到靶，0 表示丢靶或接收报错。 */
} VisionSample;

typedef struct
{
    float command_rpm;            /* 保留小数，允许多个周期累积不足 1 RPM 的变化。 */
    uint8_t tracking;             /* 记录启停状态，误差在两道阈值之间时沿用该状态。 */
} AxisState;

/* 串口和帧接收状态。 */
static UART_HandleTypeDef *vision_port;
static UART_HandleTypeDef *motor_port;
static volatile VisionSample latest_sample;   /* ISR 写，主循环按整帧读取。 */
static uint8_t rx_byte;
static char payload[VISION_PAYLOAD_CAPACITY];
static uint8_t payload_length;
static uint8_t receiving_frame;
static uint32_t frame_start_ms;

/* 主循环的电机初始化、滤波和转速斜坡状态。 */
static uint32_t last_update_ms;
static uint32_t setup_wait_start_ms;
static uint32_t setup_wait_ms;
static uint32_t filtered_sequence;
static float filtered_x;
static float filtered_y;
static AxisState x_axis;
static AxisState y_axis;
static uint8_t setup_sent;        /* 两轴的初始化命令已从 STM32 发出。 */
static uint8_t filter_ready;      /* 已有误差作为低通滤波的初值。 */
static uint8_t previous_tx_failed; /* 上一周期任一轴发送失败时，下一周期发零速。 */

/* ==================== 视觉帧解析 ==================== */
static void PublishSample(uint8_t valid, int16_t x, int16_t y)
{
    latest_sample.tick_ms = HAL_GetTick();
    latest_sample.x = x;
    latest_sample.y = y;
    latest_sample.valid = valid;
    ++latest_sample.sequence;
}

/* 读取一个带可选负号的十进制整数，成功后将游标移到下一个字符。 */
static uint8_t ParseCoordinate(const char **cursor, const char *end,
                               int16_t limit, int16_t *result)
{
    const char *p = *cursor;
    uint8_t negative = (p < end && *p == '-');
    if (negative != 0U)
    {
        ++p;
    }
    if (p == end || *p < '0' || *p > '9')
    {
        return 0U;
    }

    int32_t value = 0;
    while (p < end && *p >= '0' && *p <= '9')
    {
        value = value * 10 + (*p++ - '0');
        if (value > limit)
        {
            return 0U;
        }
    }

    *result = (int16_t)(negative != 0U ? -value : value);
    *cursor = p;
    return 1U;
}

/* 仅在两项坐标及分隔符全部有效时发布坐标。坏帧不会刷新有效数据的时间戳。 */
static void ParseFrame(void)
{
    if (payload_length == 1U && payload[0] == 'L')
    {
        PublishSample(0U, 0, 0);
        return;
    }

    const char *cursor = payload;
    const char *end = payload + payload_length;
    int16_t x;
    int16_t y;
    if (ParseCoordinate(&cursor, end, VISION_X_MAX_PX, &x) != 0U &&
        cursor < end && *cursor++ == ',' &&
        ParseCoordinate(&cursor, end, VISION_Y_MAX_PX, &y) != 0U && cursor == end)
    {
        PublishSample(1U, x, y);
    }
}

/* 遇到新的 @ 就从头收帧，便于从半帧、超时和缓冲区溢出中恢复。 */
static void ReceiveByte(uint8_t byte)
{
    uint32_t now = HAL_GetTick();
    if (byte == '@')
    {
        frame_start_ms = now;
        payload_length = 0U;
        receiving_frame = 1U;
    }
    else if (receiving_frame != 0U)
    {
        if (now - frame_start_ms >= VISION_FRAME_TIMEOUT_MS)
        {
            receiving_frame = 0U;
        }
        else if (byte == '#')
        {
            receiving_frame = 0U;
            ParseFrame();
        }
        else if (payload_length < sizeof(payload))
        {
            payload[payload_length++] = (char)byte;
        }
        else
        {
            receiving_frame = 0U;
        }
    }
}

/* ==================== USART1 中断回调 ==================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == vision_port && uart->ErrorCode == HAL_UART_ERROR_NONE)
    {
        /* 先保存当前字节并重新挂接接收，再解析已保存的字节。 */
        uint8_t byte = rx_byte;
        HAL_UART_Receive_IT(uart, &rx_byte, 1U);
        ReceiveByte(byte);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == vision_port)
    {
        receiving_frame = 0U;
        PublishSample(0U, 0, 0);
        /* HAL 终止了接收时重新挂接。仍在接收时 HAL_BUSY 表示无需重挂。 */
        HAL_UART_Receive_IT(uart, &rx_byte, 1U);
    }
}

/* ==================== 视觉外环 ==================== */
static VisionSample ReadLatestSample(void)
{
    /* 短暂屏蔽中断，避免复制过程中被串口中断换成另一帧。 */
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    VisionSample sample = latest_sample;
    __set_PRIMASK(irq_state);
    return sample;
}

static float Magnitude(float value)
{
    return value < 0.0f ? -value : value;
}

/* 按启停阈值、比例限速和速度斜坡依次计算一轴的转速指令。 */
static int16_t UpdateAxis(AxisState *axis, const AxisTuning *tuning, float error)
{
    float abs_error = Magnitude(error);

    /* 启动阈值比停止阈值大，靶心附近的识别抖动不易反复启停。 */
    if (abs_error <= tuning->stop_px)
    {
        axis->tracking = 0U;
    }
    else if (abs_error >= tuning->start_px)
    {
        axis->tracking = 1U;
    }

    /* 有效追踪时，速度大小由误差乘比例系数得到，符号由误差和电机方向确定。 */
    float target_rpm = 0.0f;
    if (axis->tracking != 0U)
    {
        float speed = abs_error * tuning->kp_rpm_per_px;
        if (speed > tuning->max_rpm)
        {
            speed = tuning->max_rpm;
        }
        target_rpm = error < 0.0f ? -speed : speed;
        target_rpm *= tuning->direction;
    }

    /* 需要反转时先将指令减到零，归零后才允许向反方向加速。 */
    if (target_rpm * axis->command_rpm < 0.0f)
    {
        target_rpm = 0.0f;
    }

    float rate_rpm_s = tuning->brake_rpm_s;
    if (Magnitude(target_rpm) > Magnitude(axis->command_rpm))
    {
        /* 加速斜率随误差绝对值线性增加，超过全加速误差后保持上限。 */
        float ratio = abs_error / tuning->full_accel_px;
        if (ratio > 1.0f)
        {
            ratio = 1.0f;
        }
        rate_rpm_s = tuning->min_accel_rpm_s +
            ratio * (tuning->max_accel_rpm_s - tuning->min_accel_rpm_s);
    }

    /* 斜率乘以控制周期的秒数，得到本周期允许的最大 RPM 变化量。 */
    float max_step = rate_rpm_s * (TRACK_PERIOD_MS / 1000.0f);
    float delta = target_rpm - axis->command_rpm;
    if (delta > max_step)
    {
        axis->command_rpm += max_step;
    }
    else if (delta < -max_step)
    {
        axis->command_rpm -= max_step;
    }
    else
    {
        axis->command_rpm = target_rpm;
    }

    /* F32C 的串口速度字段使用整数 RPM，在发送前才取整。 */
    float rpm = axis->command_rpm;
    return (int16_t)(rpm >= 0.0f ? rpm + 0.5f : rpm - 0.5f);
}

/* 两路串口初始化后调用。先启动视觉接收，电机初始化由主循环延时执行。 */
HAL_StatusTypeDef TargetTrack_Init(UART_HandleTypeDef *vision_uart,
                                   UART_HandleTypeDef *motor_uart)
{
    vision_port = vision_uart;
    motor_port = motor_uart;
    setup_wait_start_ms = HAL_GetTick();
    setup_wait_ms = TRACK_MOTOR_BOOT_MS;
    __HAL_UART_CLEAR_OREFLAG(vision_port);
    return HAL_UART_Receive_IT(vision_port, &rx_byte, 1U);
}

/* 主循环持续调用。等待电机上电，随后按控制周期更新两轴。 */
void TargetTrack_Task(void)
{
    uint32_t now = HAL_GetTick();
    if (setup_sent == 0U)
    {
        if (now - setup_wait_start_ms >= setup_wait_ms)
        {
            HAL_StatusTypeDef x_status = F32C_StartSpeedMode(motor_port, x_tuning.address);
            HAL_StatusTypeDef y_status = F32C_StartSpeedMode(motor_port, y_tuning.address);
            setup_sent = (x_status == HAL_OK && y_status == HAL_OK);
            setup_wait_start_ms = HAL_GetTick();
            setup_wait_ms = TRACK_SETUP_RETRY_MS;
            last_update_ms = setup_wait_start_ms;
        }
        return;
    }

    if (now - last_update_ms < TRACK_PERIOD_MS)
    {
        return;
    }
    last_update_ms = now;

    VisionSample sample = ReadLatestSample();
    int16_t x_rpm = 0;
    int16_t y_rpm = 0;
    if (sample.valid != 0U && HAL_GetTick() - sample.tick_ms < TRACK_LOST_TIMEOUT_MS &&
        previous_tx_failed == 0U)
    {
        /* K230 坐标减去激光点偏移即为控制误差。只对新帧滤波，重新捕获时用当前误差初始化。 */
        if (filter_ready == 0U)
        {
            filtered_x = (float)(sample.x - x_tuning.zero_px);
            filtered_y = (float)(sample.y - y_tuning.zero_px);
            filter_ready = 1U;
        }
        else if (sample.sequence != filtered_sequence)
        {
            filtered_x += TRACK_FILTER_ALPHA * ((float)(sample.x - x_tuning.zero_px) - filtered_x);
            filtered_y += TRACK_FILTER_ALPHA * ((float)(sample.y - y_tuning.zero_px) - filtered_y);
        }
        filtered_sequence = sample.sequence;

        x_rpm = UpdateAxis(&x_axis, &x_tuning, filtered_x);
        y_rpm = UpdateAxis(&y_axis, &y_tuning, filtered_y);
    }
    else
    {
        /* 丢靶、串口超时或上一周期发送失败时，两轴都发送零速。 */
        filter_ready = 0U;
        x_axis = (AxisState){0};
        y_axis = (AxisState){0};
    }

    HAL_StatusTypeDef x_status = F32C_SetSpeed(motor_port, x_tuning.address, x_rpm);
    HAL_StatusTypeDef y_status = F32C_SetSpeed(motor_port, y_tuning.address, y_rpm);
    previous_tx_failed = (x_status != HAL_OK || y_status != HAL_OK);
}
