/**
 * @file    Bsp/i2c_bus.c
 * @brief   硬件I2C FIFO轮询传输，含超时恢复和两总线独立诊断。
 * @details 仅主循环调用；顶部设置单次传输超时。
 *          失败前保存MSR，设备层负责解释地址、寄存器和芯片类型。
 */
#include "i2c_bus.h"

/* ======================== 本模块参数（修改后重新编译） ======================== */
#define ROBOT_I2C_TIMEOUT_MS       5U /* 单次事务等待上限，单位ms；不能设为无限等待。 */

/* 两条总线分别保存，OLED传输不会覆盖IMU诊断；仅主循环调用。 */
static I2cDiagnostics s_diagnostics[2];

/**
 * @brief 选择当前总线独立的诊断保存区。
 * @param[in] bus I2C0或I2C1寄存器。
 * @retval 对应诊断结构指针。
 */
static I2cDiagnostics *Diagnostics(I2C_Regs *bus)
{
    return &s_diagnostics[bus == I2C1 ? 1U : 0U];
}

/**
 * @brief 复制最近一次传输的阶段和恢复前MSR。
 * @param[in] bus I2C0或I2C1。
 * @param[out] diagnostics 非空诊断输出。
 */
void I2c_ReadDiagnostics(I2C_Regs *bus, I2cDiagnostics *diagnostics)
{
    *diagnostics = *Diagnostics(bus);
}

/**
 * @brief 检查本次传输是否达到顶部设置的超时时间。
 * @param[in] start 事务开始的毫秒时间。
 */
static bool Expired(uint32_t start)
{
    return (uint32_t)(Board_Millis() - start) >= ROBOT_I2C_TIMEOUT_MS;
}

/**
 * @brief 保存失败现场，再复位传输、清FIFO并重新使能控制器。
 * @param[in] bus I2C0或I2C1。
 * @param[in] error 返回给上层的错误分类。
 * @retval 原错误码；不会假装事务成功。
 */
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

/**
 * @brief 有时间上限地等待控制器空闲且总线不忙。
 * @param[in] bus I2C总线。
 * @param[in] start 本阶段开始时间。
 * @retval I2C_OK或I2C_TIMEOUT。
 */
static I2cResult Idle(I2C_Regs *bus, uint32_t start)
{
    while (true) {
        uint32_t status = DL_I2C_getControllerStatus(bus);
        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U &&
            (status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0U) return I2C_OK;
        if (Expired(start)) return Abort(bus, I2C_TIMEOUT);
    }
}

/**
 * @brief 有时间上限地等待事务结束，并检查硬件错误位。
 * @param[in] bus I2C总线。
 * @param[in] start 事务开始时间。
 * @retval 完成结果。
 */
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

/**
 * @brief 启动带START/STOP的硬件事务并满足TI勘误要求的状态读取延迟。
 * @param[in] bus I2C总线。
 * @param[in] address 7位地址。
 * @param[in] direction TX发送或RX接收。
 * @param[in] length 字节数。
 */
static void Start(I2C_Regs *bus, uint8_t address, DL_I2C_CONTROLLER_DIRECTION direction, uint8_t length)
{
    DL_I2C_startControllerTransfer(bus, address, direction, length);
    /* 按 TI 官方例程处理 I2C_ERR_13：启动后至少等 3 个功能时钟周期再读状态。
     * 配置要求 BUSCLK=32 MHz，I2C clock divider=1；64 CPU 周期有足够余量。 */
    delay_cycles(64U);
}

/**
 * @brief 通过FIFO发送1..8字节，所有等待均有超时。
 * @param[in] bus I2C0或I2C1。
 * @param[in] address 7位地址，不得左移。
 * @param[in] bytes 非空发送缓冲。
 * @param[in] length 1..8。
 * @retval I2cResult枚举中的结果码。
 */
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

/**
 * @brief 先发送寄存器地址，再读取指定字节数并持续排空RX FIFO。
 * @param[in] bus I2C0或I2C1。
 * @param[in] address 7位器件地址。
 * @param[in] reg 首寄存器地址。
 * @param[out] bytes 接收缓冲。
 * @param[in] length 接收字节数。
 * @retval I2cResult结果码；失败时可读取诊断。
 */
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
