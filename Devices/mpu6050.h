/**
 * @file    Devices/mpu6050.h
 * @brief   IMU数据、探测结果、采样周期与姿态更新接口。
 * @details 角度单位为度；Yaw为相对角，会漂移；在线/标定/姿态有效必须分别判断。
 */
#ifndef ROBOT_MPU6050_H
#define ROBOT_MPU6050_H
#include <stdbool.h>
#include <stdint.h>

/* 定义与调参均在mpu6050.c顶部；主循环和显示读取这些值。 */
extern const uint32_t g_mpu6050_sample_period_ms;
extern const uint32_t g_mpu6050_calibration_sample_count;
typedef enum {
    IMU_FAULT_NONE = 0, IMU_FAULT_PROBE, IMU_FAULT_ID,
    IMU_FAULT_RESET, IMU_FAULT_CONFIG, IMU_FAULT_READ
} ImuFault;

typedef struct {
    bool attempted;
    uint8_t who_am_i, bus_result, stage;
    uint32_t status;
} ImuProbeDiagnostics;
typedef struct {
    bool online, calibrated;
    bool attitude_valid, accel_correction_used;
    uint32_t calibration_samples;
    float pitch_deg, roll_deg, yaw_deg;
    float temperature_c;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    uint32_t errors;
    ImuFault fault;
    uint8_t device_address, who_am_i, last_bus_result;
    uint8_t last_i2c_stage;
    uint32_t last_i2c_status;
    ImuProbeDiagnostics probe[2]; /* 下标0=0x68，下标1=0x69。 */
    /* 最近一次成功读取的寄存器值，失败时保留，结合online和sample_ms判断新鲜度。 */
    int16_t accel_raw[3], gyro_raw[3], temperature_raw;
    uint32_t sample_ms;
} ImuData;

/**
 * @brief 探测地址、校验芯片ID、复位并配置MPU6050。
 * @retval true完成初始化；false失败，错误计数跨重试保留。
 * @note 有设备复位等待，只在统一初始化或明确停车时调用。
 */
bool Mpu6050_Init(void);
/* 主循环约每10ms调用，传真实时间。静止停车阶段采100个样本标定陀螺零偏。
 * 四元数互补滤波：陀螺仪预测，可信加速度计重力方向校正，输出ZYX欧拉角。
 * 轴向以MPU芯片坐标为准；默认水平静置时az约+1g，不能仅凭模块丝印猜安装轴。
 * 只有加速度计和陀螺仪，yaw是相对积分角，会漂移，不能当绝对航向。 */
/**
 * @brief 读取六轴/温度样本，更新标定、四元数与欧拉角。
 * @param[in] now_ms 当前真实毫秒时间。
 * @param[in] stationary_wait 标定允许标志，运行中必须为false。
 * @retval true本次读取成功；false失败，旧样本不可当新数据。
 * @note 由主循环按g_mpu6050_sample_period_ms调用，不参与底盘转向。
 */
bool Mpu6050_Update(uint32_t now_ms, bool stationary_wait);
/**
 * @brief 取得最近IMU数据和诊断的只读视图。
 * @retval 模块静态数据指针，不释放；结合online/sample_ms判断新鲜度。
 */
const ImuData *Mpu6050_Data(void);
/**
 * @brief 取得用于OLED的ASCII故障说明。
 * @retval 静态字符串；中文翻译由界面模块完成。
 */
const char *Mpu6050_FaultName(void);
#endif
