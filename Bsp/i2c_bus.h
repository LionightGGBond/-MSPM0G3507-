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
void I2c_ReadDiagnostics(I2C_Regs *bus, I2cDiagnostics *diagnostics);
/* 仅主循环调用，包长 1..8（硬件 FIFO 深度）；所有等待有时间上限。 */
I2cResult I2c_Write(I2C_Regs *bus, uint8_t address, const uint8_t *bytes, uint8_t length);
/* 寄存器读用 STOP 后再 START；MPU6050 支持此方式。读取过程持续排空 RX FIFO。 */
I2cResult I2c_ReadRegister(I2C_Regs *bus, uint8_t address, uint8_t reg,
                         uint8_t *bytes, uint8_t length);
#endif
