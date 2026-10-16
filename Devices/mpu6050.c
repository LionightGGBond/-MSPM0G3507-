#include "mpu6050.h"
#include "../Bsp/i2c_bus.h"
#include "../robot_config.h"
#include <math.h>
#include <string.h>

/* 六轴四元数显式互补滤波：陀螺仪预测 + 重力误差PI校正。
 * 无磁力计，不能校正绝对航向；参数和使用范围见docs/IMU角度与互补滤波.md。 */
static ImuData s_data;
static float s_bias[3], s_sum[3], s_sum_square[3];
static float s_q[4], s_feedback_i[3];
static uint32_t s_samples, s_last_ms;
static bool s_have_time;

#define DEG_TO_RAD 0.01745329252f
#define RAD_TO_DEG 57.29577951f

static float Clamp(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void ResetCalibrationWindow(void)
{
    s_samples = 0U;
    s_data.calibration_samples = 0U;
    memset(s_sum, 0, sizeof(s_sum));
    memset(s_sum_square, 0, sizeof(s_sum_square));
}

static void UpdateCalibration(const float gyro[3], float norm2, bool stationary_wait)
{
    if (s_data.calibrated) return;
    bool still = stationary_wait && norm2 > 0.9f * 0.9f && norm2 < 1.1f * 1.1f;
    for (unsigned int i = 0U; i < 3U; ++i) {
        if (fabsf(gyro[i]) > ROBOT_IMU_CAL_MAX_DPS) still = false;
    }
    if (!still) {
        ResetCalibrationWindow();
        return;
    }
    for (unsigned int i = 0U; i < 3U; ++i) {
        s_sum[i] += gyro[i];
        s_sum_square[i] += gyro[i] * gyro[i];
    }
    s_data.calibration_samples = ++s_samples;
    if (s_samples < ROBOT_IMU_CALIBRATION_SAMPLES) return;
    for (unsigned int i = 0U; i < 3U; ++i) {
        float mean = s_sum[i] / (float)s_samples;
        float variance = s_sum_square[i] / (float)s_samples - mean * mean;
        if (variance > ROBOT_IMU_CAL_STD_DPS * ROBOT_IMU_CAL_STD_DPS) {
            ResetCalibrationWindow();
            return;
        }
    }
    for (unsigned int i = 0U; i < 3U; ++i) s_bias[i] = s_sum[i] / (float)s_samples;
    memset(s_feedback_i, 0, sizeof(s_feedback_i));
    s_data.calibrated = true;
}

static void PublishAngles(void)
{
    float w = s_q[0], x = s_q[1], y = s_q[2], z = s_q[3];
    s_data.roll_deg = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD_TO_DEG;
    /* 浮点舍入可能使asin参数略超±1，先限幅避免NaN。 */
    s_data.pitch_deg = asinf(Clamp(2.0f * (w * y - z * x), -1.0f, 1.0f)) * RAD_TO_DEG;
    s_data.yaw_deg = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD_TO_DEG;
}

static void InitializeAttitude(float ax, float ay, float az)
{
    float roll_half = 0.5f * atan2f(ay, az);
    float pitch_half = 0.5f * atan2f(-ax, sqrtf(ay * ay + az * az));
    /* 首次yaw置零；断线/长间隔恢复时只重建倾角，不伪造丢失期间的转角。 */
    float yaw_half = s_data.attitude_valid ? 0.5f * s_data.yaw_deg * DEG_TO_RAD : 0.0f;
    float cr = cosf(roll_half), sr = sinf(roll_half);
    float cp = cosf(pitch_half), sp = sinf(pitch_half);
    float cy = cosf(yaw_half), sy = sinf(yaw_half);
    s_q[0] = cr * cp * cy + sr * sp * sy;
    s_q[1] = sr * cp * cy - cr * sp * sy;
    s_q[2] = cr * sp * cy + sr * cp * sy;
    s_q[3] = cr * cp * sy - sr * sp * cy;
    s_data.attitude_valid = true;
    memset(s_feedback_i, 0, sizeof(s_feedback_i));
    PublishAngles();
}

static void UpdateAttitude(float ax, float ay, float az, float norm2,
                           float dt, bool reinitialize)
{
    bool gravity_valid = norm2 >= ROBOT_IMU_ACCEL_MIN_G * ROBOT_IMU_ACCEL_MIN_G &&
                         norm2 <= ROBOT_IMU_ACCEL_MAX_G * ROBOT_IMU_ACCEL_MAX_G;
    s_data.accel_correction_used = false;
    if (!s_data.attitude_valid || reinitialize) {
        if (gravity_valid) InitializeAttitude(ax, ay, az);
        return;
    }
    if (dt <= 0.0f) return;
    /* 零偏标定完成前只显示加速度倾角，yaw保持初始参考，不积分未校准陀螺。 */
    if (!s_data.calibrated) {
        if (gravity_valid) InitializeAttitude(ax, ay, az);
        return;
    }
    float rate[3] = {s_data.gyro_x_dps * DEG_TO_RAD, s_data.gyro_y_dps * DEG_TO_RAD,
                     s_data.gyro_z_dps * DEG_TO_RAD};
    float w = s_q[0], x = s_q[1], y = s_q[2], z = s_q[3];
    if (gravity_valid) {
        float inverse_norm = 1.0f / sqrtf(norm2);
        ax *= inverse_norm;
        ay *= inverse_norm;
        az *= inverse_norm;
        /* 将参考重力旋转到机体坐标，实测方向×预测方向作为校正误差。
         * 只约束倾角，重力不能提供绝对yaw参考。 */
        float vx = 2.0f * (x * z - w * y);
        float vy = 2.0f * (w * x + y * z);
        float vz = 1.0f - 2.0f * (x * x + y * y);
        float error[3] = {ay * vz - az * vy, az * vx - ax * vz, ax * vy - ay * vx};
        float limit = ROBOT_IMU_CF_I_LIMIT_DPS * DEG_TO_RAD;
        for (unsigned int i = 0U; i < 3U; ++i) {
            if (ROBOT_IMU_CF_KI > 0.0f) {
                s_feedback_i[i] = Clamp(s_feedback_i[i] + ROBOT_IMU_CF_KI * error[i] * dt, -limit, limit);
            } else {
                s_feedback_i[i] = 0.0f;
            }
            rate[i] += ROBOT_IMU_CF_KP * error[i];
        }
        s_data.accel_correction_used = true;
    }
    for (unsigned int i = 0U; i < 3U; ++i) rate[i] += s_feedback_i[i];
    /* q表示机体到参考坐标的旋转。使用同一旧四元数计算四个增量。 */
    float half_dt = 0.5f * dt;
    s_q[0] += (-x * rate[0] - y * rate[1] - z * rate[2]) * half_dt;
    s_q[1] += ( w * rate[0] + y * rate[2] - z * rate[1]) * half_dt;
    s_q[2] += ( w * rate[1] - x * rate[2] + z * rate[0]) * half_dt;
    s_q[3] += ( w * rate[2] + x * rate[1] - y * rate[0]) * half_dt;
    float q_norm2 = s_q[0] * s_q[0] + s_q[1] * s_q[1] + s_q[2] * s_q[2] + s_q[3] * s_q[3];
    if (!isfinite(q_norm2) || q_norm2 < 0.01f) {
        s_data.attitude_valid = false;
        return;
    }
    float inverse_q_norm = 1.0f / sqrtf(q_norm2);
    for (unsigned int i = 0U; i < 4U; ++i) s_q[i] *= inverse_q_norm;
    PublishAngles();
}

static void SaveBusResult(I2cResult result)
{
    I2cDiagnostics diagnostics;
    I2c_ReadDiagnostics(IMU_I2C_INST, &diagnostics);
    s_data.last_bus_result = (uint8_t)result;
    s_data.last_i2c_stage = (uint8_t)diagnostics.stage;
    s_data.last_i2c_status = diagnostics.status;
}

static bool Write(uint8_t reg, uint8_t value)
{
    uint8_t packet[2] = {reg, value};
    I2cResult result = I2c_Write(IMU_I2C_INST, s_data.device_address, packet, 2U);
    SaveBusResult(result);
    return result == I2C_OK;
}

static bool InitializationFailed(ImuFault fault)
{
    s_data.online = false;
    s_data.fault = fault;
    ++s_data.errors;
    return false;
}

static bool Probe(uint8_t address)
{
    s_data.device_address = address;
    s_data.who_am_i = 0xFFU;
    I2cResult result = I2c_ReadRegister(IMU_I2C_INST, address, 0x75U, &s_data.who_am_i, 1U);
    SaveBusResult(result);
    if (address == 0x68U || address == 0x69U) {
        ImuProbeDiagnostics *probe = &s_data.probe[address & 1U];
        probe->attempted = true;
        probe->who_am_i = s_data.who_am_i;
        probe->bus_result = (uint8_t)result;
        probe->stage = s_data.last_i2c_stage;
        probe->status = s_data.last_i2c_status;
    }
    return result == I2C_OK;
}

bool Mpu6050_Init(void)
{
    /* 保留累计错误数；此前每次重试清零，导致初始化失败也一直显示E:0。 */
    uint32_t previous_errors = s_data.errors;
    memset(&s_data, 0, sizeof(s_data));
    s_data.probe[0].who_am_i = s_data.probe[1].who_am_i = 0xFFU;
    s_data.errors = previous_errors;
    memset(s_bias, 0, sizeof(s_bias));
    ResetCalibrationWindow();
    memset(s_q, 0, sizeof(s_q));
    s_q[0] = 1.0f;
    memset(s_feedback_i, 0, sizeof(s_feedback_i));
    s_have_time = false;
    bool replied = Probe(ROBOT_MPU_ADDRESS);
#if ROBOT_IMU_AUTO_ADDRESS
    if (!replied) replied = Probe(ROBOT_MPU_ADDRESS == 0x68U ? 0x69U : 0x68U);
#endif
    if (!replied) return InitializationFailed(IMU_FAULT_PROBE);
    /* WHO_AM_I仍应为0x68，与AD0选择的总线地址无关；不把其他IMU当6050配置。 */
    if (s_data.who_am_i != 0x68U) return InitializationFailed(IMU_FAULT_ID);
    if (!Write(0x6BU, 0x80U)) return InitializationFailed(IMU_FAULT_RESET);
    Board_DelayMs(100U);
    /* 唤醒，陀螺X轴PLL；DLPF=3，采样100Hz，±500°/s，±2g。 */
    if (!Write(0x6BU, 0x01U) || !Write(0x1AU, 0x03U) ||
        !Write(0x19U, 9U) || !Write(0x1BU, 0x08U) || !Write(0x1CU, 0x00U)) {
        return InitializationFailed(IMU_FAULT_CONFIG);
    }
    Board_DelayMs(10U);
    s_data.online = true;
    s_data.fault = IMU_FAULT_NONE;
    return true;
}

static int16_t Signed16(uint8_t high, uint8_t low)
{
    uint16_t value = (uint16_t)(((uint16_t)high << 8) | low);
    return (int16_t)(value < 32768U ? (int32_t)value : (int32_t)value - 65536);
}

bool Mpu6050_Update(uint32_t now_ms, bool stationary_wait)
{
    uint8_t raw[14];
    I2cResult result = I2c_ReadRegister(IMU_I2C_INST, s_data.device_address, 0x3BU, raw, 14U);
    SaveBusResult(result);
    if (result != I2C_OK) {
        s_data.online = false;
        s_data.accel_correction_used = false;
        if (!s_data.calibrated) ResetCalibrationWindow();
        ++s_data.errors;
        s_data.fault = IMU_FAULT_READ;
        s_have_time = false;
        return false;
    }
    for (unsigned int i = 0U; i < 3U; ++i) {
        s_data.accel_raw[i] = Signed16(raw[2U * i], raw[2U * i + 1U]);
        s_data.gyro_raw[i] = Signed16(raw[8U + 2U * i], raw[9U + 2U * i]);
    }
    s_data.temperature_raw = Signed16(raw[6], raw[7]);
    s_data.sample_ms = now_ms;
    float ax = (float)s_data.accel_raw[0] / 16384.0f;
    float ay = (float)s_data.accel_raw[1] / 16384.0f;
    float az = (float)s_data.accel_raw[2] / 16384.0f;
    float gyro[3] = {(float)s_data.gyro_raw[0] / 65.5f,
                     (float)s_data.gyro_raw[1] / 65.5f,
                     (float)s_data.gyro_raw[2] / 65.5f};
    s_data.online = true;
    s_data.fault = IMU_FAULT_NONE;
    s_data.temperature_c = (float)s_data.temperature_raw / 340.0f + 36.53f;
    float norm2 = ax * ax + ay * ay + az * az;
    UpdateCalibration(gyro, norm2, stationary_wait);
    s_data.gyro_x_dps = gyro[0] - s_bias[0];
    s_data.gyro_y_dps = gyro[1] - s_bias[1];
    s_data.gyro_z_dps = gyro[2] - s_bias[2];
    uint32_t elapsed = now_ms - s_last_ms;
    UpdateAttitude(ax, ay, az, norm2, (float)elapsed * 0.001f,
                   !s_have_time || elapsed > ROBOT_IMU_MAX_DT_MS);
    s_last_ms = now_ms;
    s_have_time = true;
    return true;
}

const ImuData *Mpu6050_Data(void)
{
    return &s_data;
}

const char *Mpu6050_FaultName(void)
{
    if (s_data.fault != IMU_FAULT_NONE && s_data.last_bus_result == (uint8_t)I2C_TIMEOUT)
        return "TIMEOUT";
    switch (s_data.fault) {
        case IMU_FAULT_NONE: return "OK";
        case IMU_FAULT_PROBE: return "NO REPLY";
        case IMU_FAULT_ID: return "BAD ID";
        case IMU_FAULT_RESET: return "RESET";
        case IMU_FAULT_CONFIG: return "CONFIG";
        case IMU_FAULT_READ: return "READ";
        default: return "UNKNOWN";
    }
}
