#include "i2c_bus.h"
#include "../robot_config.h"

/* 两条总线分别保存，OLED传输不会覆盖IMU诊断；仅主循环调用。 */
static I2cDiagnostics s_diagnostics[2];

static I2cDiagnostics *Diagnostics(I2C_Regs *bus)
{
    return &s_diagnostics[bus == I2C1 ? 1U : 0U];
}

void I2c_ReadDiagnostics(I2C_Regs *bus, I2cDiagnostics *diagnostics)
{
    *diagnostics = *Diagnostics(bus);
}

static bool Expired(uint32_t start)
{
    return (uint32_t)(Board_Millis() - start) >= ROBOT_I2C_TIMEOUT_MS;
}

static I2cResult Abort(I2C_Regs *bus, I2cResult error)
{
    Diagnostics(bus)->status = DL_I2C_getControllerStatus(bus);
    DL_I2C_resetControllerTransfer(bus);
    DL_I2C_disableController(bus);
    DL_I2C_flushControllerTXFIFO(bus);
    DL_I2C_flushControllerRXFIFO(bus);
    DL_I2C_enableController(bus);
    return error;
}

static I2cResult Idle(I2C_Regs *bus, uint32_t start)
{
    while (true) {
        uint32_t status = DL_I2C_getControllerStatus(bus);
        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U &&
            (status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0U) return I2C_OK;
        if (Expired(start)) return Abort(bus, I2C_TIMEOUT);
    }
}

static I2cResult Done(I2C_Regs *bus, uint32_t start)
{
    while ((DL_I2C_getControllerStatus(bus) & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (Expired(start)) return Abort(bus, I2C_TIMEOUT);
    }
    if ((DL_I2C_getControllerStatus(bus) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        return Abort(bus, I2C_DEVICE_ERROR);
    }
    I2cResult result = Idle(bus, start);
    if (result == I2C_OK) Diagnostics(bus)->status = DL_I2C_getControllerStatus(bus);
    return result;
}

static void Start(I2C_Regs *bus, uint8_t address, DL_I2C_CONTROLLER_DIRECTION direction, uint8_t length)
{
    DL_I2C_startControllerTransfer(bus, address, direction, length);
    /* 按 TI 官方例程处理 I2C_ERR_13：启动后至少等 3 个功能时钟周期再读状态。
     * 配置要求 BUSCLK=32 MHz，I2C clock divider=1；64 CPU 周期有足够余量。 */
    delay_cycles(64U);
}

I2cResult I2c_Write(I2C_Regs *bus, uint8_t address, const uint8_t *bytes, uint8_t length)
{
    if (bytes == 0 || length == 0U || length > 8U || address > 0x7FU) return I2C_ARGUMENT_ERROR;
    Diagnostics(bus)->stage = I2C_STAGE_IDLE;
    Diagnostics(bus)->status = 0U;
    uint32_t start = Board_Millis();
    I2cResult result = Idle(bus, start);
    if (result != I2C_OK) return result;
    Diagnostics(bus)->stage = I2C_STAGE_FILL_TX;
    DL_I2C_flushControllerTXFIFO(bus);
    if (DL_I2C_fillControllerTXFIFO(bus, bytes, length) != length) return Abort(bus, I2C_DEVICE_ERROR);
    Diagnostics(bus)->stage = I2C_STAGE_TX;
    Start(bus, address, DL_I2C_CONTROLLER_DIRECTION_TX, length);
    return Done(bus, start);
}

I2cResult I2c_ReadRegister(I2C_Regs *bus, uint8_t address, uint8_t reg, uint8_t *bytes, uint8_t length)
{
    if (bytes == 0 || length == 0U || address > 0x7FU) return I2C_ARGUMENT_ERROR;
    I2cResult result = I2c_Write(bus, address, &reg, 1U);
    if (result != I2C_OK) return result;
    uint32_t start = Board_Millis();
    DL_I2C_flushControllerRXFIFO(bus);
    Diagnostics(bus)->stage = I2C_STAGE_RX;
    Start(bus, address, DL_I2C_CONTROLLER_DIRECTION_RX, length);
    for (uint8_t i = 0U; i < length; ++i) {
        while (DL_I2C_isControllerRXFIFOEmpty(bus)) {
            if ((DL_I2C_getControllerStatus(bus) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)
                return Abort(bus, I2C_DEVICE_ERROR);
            if (Expired(start)) return Abort(bus, I2C_TIMEOUT);
        }
        bytes[i] = DL_I2C_receiveControllerData(bus);
    }
    return Done(bus, start);
}
