#ifndef ROBOT_MPU6050_H
#define ROBOT_MPU6050_H
#include <stdbool.h>
#include <stdint.h>
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

bool Mpu6050_Init(void);
/* 主循环约每10ms调用，传真实时间。静止停车阶段采100个样本标定陀螺零偏。
 * 四元数互补滤波：陀螺仪预测，可信加速度计重力方向校正，输出ZYX欧拉角。
 * 轴向以MPU芯片坐标为准；默认水平静置时az约+1g，不能仅凭模块丝印猜安装轴。
 * 只有加速度计和陀螺仪，yaw是相对积分角，会漂移，不能当绝对航向。 */
bool Mpu6050_Update(uint32_t now_ms, bool stationary_wait);
const ImuData *Mpu6050_Data(void);
const char *Mpu6050_FaultName(void);
#endif
