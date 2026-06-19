/**
 * @file    i2c_bridge.c
 * @brief   MSPM0G3507 I2C 桥接层 — 实现 lsm6dsr_io_t 回调
 *
 * 将 LSM6DSR 驱动层的 lsm6dsr_io_t 接口桥接到 MSPM0G3507 DriverLib I2C API。
 *
 * 硬件配置：
 *   - I2C 实例：I2C_0_INST (I2C0)
 *   - 时钟速率：400 kHz (Fast Mode)
 *   - 7-bit 地址：0x6A (SDO/SA0 接 GND)
 *   - 引脚：SCL=PB6, SDA=PB7（由 SYSCFG_DL_init() 配置）
 *
 * 注意事项：
 *   - MSPM0 I2C 使用 7-bit 地址，lsm6dsr_io_t 已左移 1 位（0xD4）
 *   - DriverLib I2C API 需要 7-bit 地址，因此需要右移 1 位
 *   - 读操作：先写寄存器地址，再读数据（Restart 条件）
 */

#include "platform.h"
#include "lsm6dsr.h"
#include "ti_msp_dl_config.h"

/* I2C busy-wait timeout: ~50ms at 80MHz (80M * 0.05 = 4M cycles) */
#define I2C_TIMEOUT_CYCLES  4000000UL

/* ============================================================
 * I2C 读写实现
 * ============================================================ */

/**
 * @brief I2C 多字节读取
 *
 * @param ctx  I2C 实例指针（I2C_Regs*）
 * @param reg  起始寄存器地址
 * @param buf  读取数据缓冲区
 * @param len  读取字节数
 * @return 0=成功, -1=失败
 *
 * 时序：
 *   START → [ADDR+W] → [REG] → RESTART → [ADDR+R] → [DATA0]...[DATAn] → STOP
 */
static int8_t mspm0_i2c_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len)
{
    I2C_Regs *i2c = (I2C_Regs *)ctx;
    uint32_t timeout;

    if (!i2c || !buf || len == 0) return -1;

    /* LSM6DSR I2C 地址已左移 1 位（0xD4），DriverLib 需要 7-bit（0x6A） */
    uint16_t addr = LSM6DSR_I2C_ADDR >> 1;

    /* Step 1: 写寄存器地址（带 Restart 条件，不发 Stop） */
    DL_I2C_startControllerTransfer(i2c, addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    DL_I2C_transmitControllerData(i2c, reg, DL_I2C_CONTROLLER_SEND_RESTART);

    /* 等待寄存器地址发送完成 */
    timeout = I2C_TIMEOUT_CYCLES;
    while (DL_I2C_getControllerBusy(i2c) && --timeout) { }
    if (!timeout) return -1;

    /* Step 2: 读取数据（Restart → Read → Stop） */
    DL_I2C_startControllerTransfer(i2c, addr, DL_I2C_CONTROLLER_DIRECTION_RX, len);

    for (uint16_t i = 0; i < len; i++) {
        /* 等待数据就绪 */
        timeout = I2C_TIMEOUT_CYCLES;
        while (DL_I2C_getControllerRXFIFOEmpty(i2c) && --timeout) { }
        if (!timeout) return -1;
        buf[i] = DL_I2C_receiveControllerData(i2c);
    }

    /* 等待传输完成 */
    timeout = I2C_TIMEOUT_CYCLES;
    while (DL_I2C_getControllerBusy(i2c) && --timeout) { }
    if (!timeout) return -1;

    return 0;
}

/**
 * @brief I2C 多字节写入
 *
 * @param ctx  I2C 实例指针（I2C_Regs*）
 * @param reg  起始寄存器地址
 * @param buf  写入数据缓冲区
 * @param len  写入字节数
 * @return 0=成功, -1=失败
 *
 * 时序：
 *   START → [ADDR+W] → [REG] → [DATA0]...[DATAn] → STOP
 */
static int8_t mspm0_i2c_write(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    I2C_Regs *i2c = (I2C_Regs *)ctx;
    uint32_t timeout;

    if (!i2c || !buf || len == 0) return -1;

    /* LSM6DSR I2C 地址已左移 1 位（0xD4），DriverLib 需要 7-bit（0x6A） */
    uint16_t addr = LSM6DSR_I2C_ADDR >> 1;

    /* 准备发送缓冲区：寄存器地址 + 数据 */
    uint8_t tx_buf[32];  /* 最大单次写入 */
    if (len + 1 > sizeof(tx_buf)) return -1;

    tx_buf[0] = reg;
    for (uint16_t i = 0; i < len; i++) {
        tx_buf[i + 1] = buf[i];
    }

    /* 启动写传输 */
    DL_I2C_startControllerTransfer(i2c, addr, DL_I2C_CONTROLLER_DIRECTION_TX, len + 1);

    /* 发送数据 */
    for (uint16_t i = 0; i < len + 1; i++) {
        timeout = I2C_TIMEOUT_CYCLES;
        while (!DL_I2C_getControllerTXFIFOEmpty(i2c) && --timeout) { }
        if (!timeout) return -1;
        /* 最后一个字节发送 Stop 条件 */
        DL_I2C_transmitControllerData(i2c, tx_buf[i],
            (i == len) ? DL_I2C_CONTROLLER_SEND_STOP : DL_I2C_CONTROLLER_SEND_NEXT);
    }

    /* 等待传输完成 */
    timeout = I2C_TIMEOUT_CYCLES;
    while (DL_I2C_getControllerBusy(i2c) && --timeout) { }
    if (!timeout) return -1;

    return 0;
}

/* ============================================================
 * I/O 实例
 * ============================================================ */

/**
 * @brief LSM6DSR I2C I/O 实例
 *
 * 全局实例，供 bsp_lsm6dsr.c 使用。
 * ctx 指向 I2C0 外设基地址。
 */
lsm6dsr_io_t lsm6dsr_io = {
    .read  = mspm0_i2c_read,
    .write = mspm0_i2c_write,
    .ctx   = I2C_0_INST,
};
