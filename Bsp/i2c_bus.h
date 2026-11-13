/**
 * @file    Bsp/i2c_bus.h
 * @brief   I2C传输结果、失败阶段、诊断和寄存器读写接口。
 * @details 接口内部有超时；禁止在10ms电机控制中断调用。
 */
#ifndef ROBOT_I2C_BUS_H
#define ROBOT_I2C_BUS_H
#include "board.h"
typedef enum { I2C_OK = 0, I2C_TIMEOUT, I2C_DEVICE_ERROR, I2C_ARGUMENT_ERROR } I2cResult;
typedef enum {
    I2C_STAGE_NONE = 0, I2C_STAGE_IDLE, I2C_STAGE_FILL_TX,
    I2C_STAGE_TX, I2C_STAGE_RX
} I2cStage;
typedef struct {
    uint32_t status; /* 保留恢复控制器之前的MSR，避免错误位被清除后无法定位。 */
    I2cStage stage;
} I2cDiagnostics;
/**
 * @brief 复制最近一次传输的阶段和恢复前MSR。
 * @param[in] bus I2C0或I2C1。
 * @param[out] diagnostics 非空诊断输出。
 */
void I2c_ReadDiagnostics(I2C_Regs *bus, I2cDiagnostics *diagnostics);
/* 仅主循环调用，包长 1..8（硬件 FIFO 深度）；所有等待有时间上限。 */
/**
 * @brief 通过FIFO发送1..8字节，所有等待均有超时。
 * @param[in] bus I2C0或I2C1。
 * @param[in] address 7位地址，不得左移。
 * @param[in] bytes 非空发送缓冲。
 * @param[in] length 1..8。
 * @retval I2cResult枚举中的结果码。
 */
I2cResult I2c_Write(I2C_Regs *bus, uint8_t address, const uint8_t *bytes, uint8_t length);
/* 寄存器读用 STOP 后再 START；MPU6050 支持此方式。读取过程持续排空 RX FIFO。 */
/**
 * @brief 先发送寄存器地址，再读取指定字节数并持续排空RX FIFO。
 * @param[in] bus I2C0或I2C1。
 * @param[in] address 7位器件地址。
 * @param[in] reg 首寄存器地址。
 * @param[out] bytes 接收缓冲。
 * @param[in] length 接收字节数。
 * @retval I2cResult结果码；失败时可读取诊断。
 */
I2cResult I2c_ReadRegister(I2C_Regs *bus, uint8_t address, uint8_t reg,
                         uint8_t *bytes, uint8_t length);
#endif
