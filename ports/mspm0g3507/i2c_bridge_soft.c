/**
 * @file    i2c_bridge_soft.c
 * @brief   MSPM0G3507 软件 I2C 桥接层 — 基于 GPIO 位操作实现 lsm6dsr_io_t
 *
 * 与 i2c_bridge.c（硬件 I2C）并存，通过编译选项切换：
 *   - USE_SOFT_I2C 定义时使用本文件
 *   - 否则使用 i2c_bridge.c（硬件 I2C）
 *
 * 硬件配置：
 *   - SCL: PA0 (PINCM1) — 与 MPU6050 共用引脚
 *   - SDA: PA1 (PINCM2) — 与 MPU6050 共用引脚
 *   - 速率: 100kHz (标准模式)
 *   - 超时: 50ms
 *
 * 注意事项：
 *   - LSM6DSR 和 MPU6050 不能同时使用同一组引脚
 *   - 需要外部 4.7kΩ 上拉电阻
 *   - 软件 I2C 是阻塞操作，在 FreeRTOS 任务中需注意优先级
 *
 * 依赖：
 *   - platform.h: delay_us() 微秒延时
 *   - ti_msp_dl_config.h: GPIO 操作
 */

#include "platform.h"
#include "lsm6dsr.h"
#include "ti_msp_dl_config.h"

/* ============================================================
 * 引脚配置
 * ============================================================ */

/** SCL 引脚配置 */
#define SOFT_I2C_SCL_PORT    GPIOA
#define SOFT_I2C_SCL_PIN     DL_GPIO_PIN_0
#define SOFT_I2C_SCL_IOMUX   IOMUX_PINCM1

/** SDA 引脚配置 */
#define SOFT_I2C_SDA_PORT    GPIOA
#define SOFT_I2C_SDA_PIN     DL_GPIO_PIN_1
#define SOFT_I2C_SDA_IOMUX   IOMUX_PINCM2

/* ============================================================
 * I2C 时序参数
 * ============================================================ */

/** 半周期延时 (100kHz → 5us) */
#define I2C_HALF_PERIOD_US   5
/** 数据建立时间 */
#define I2C_SETUP_US         1
/** 超时计数 (~50ms at 80MHz) */
#define I2C_TIMEOUT_CYCLES   50000UL

/* ============================================================
 * LSM6DSR I2C 地址
 * ============================================================ */

/** LSM6DSR 7-bit I2C 地址 (0x6A) */
#define LSM6DSR_7BIT_ADDR   (LSM6DSR_I2C_ADDR >> 1)

/* ============================================================
 * GPIO 操作宏
 * ============================================================ */

/** SCL 输出高/低 */
#define SCL_HIGH()  DL_GPIO_setPins(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN)
#define SCL_LOW()   DL_GPIO_clearPins(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN)

/** SDA 输出高/低 */
#define SDA_HIGH()  DL_GPIO_setPins(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN)
#define SDA_LOW()   DL_GPIO_clearPins(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN)

/** SDA 读取 (返回 0 或 1) */
#define SDA_READ()  (DL_GPIO_readPins(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN) & SOFT_I2C_SDA_PIN ? 1 : 0)

/** SDA 方向切换 */
#define SDA_OUT()   DL_GPIO_enableOutput(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN)
#define SDA_IN()    DL_GPIO_disableOutput(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN)

/* ============================================================
 * 软件 I2C 底层操作
 * ============================================================ */

/**
 * @brief 初始化软件 I2C 引脚
 *
 * 配置 SCL/SDA 为开漏输出模式，外部上拉
 */
static void soft_i2c_init_pins(void)
{
    /* SCL: 输出模式 */
    DL_GPIO_initDigitalOutput(SOFT_I2C_SCL_IOMUX);
    DL_GPIO_setPins(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN);
    DL_GPIO_enableOutput(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN);

    /* SDA: 输出模式 */
    DL_GPIO_initDigitalOutput(SOFT_I2C_SDA_IOMUX);
    DL_GPIO_setPins(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN);
    DL_GPIO_enableOutput(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN);

    /* 确保总线空闲 (SCL=HIGH, SDA=HIGH) */
    SCL_HIGH();
    SDA_HIGH();
    g_platform->delay_us(I2C_HALF_PERIOD_US);
}

/**
 * @brief 产生 I2C START 条件
 *
 * 时序: SDA: HIGH→LOW (SCL=HIGH 期间)
 */
static void soft_i2c_start(void)
{
    SDA_OUT();
    SDA_HIGH();
    SCL_HIGH();
    g_platform->delay_us(I2C_HALF_PERIOD_US);
    SDA_LOW();
    g_platform->delay_us(I2C_HALF_PERIOD_US);
    SCL_LOW();
    g_platform->delay_us(I2C_HALF_PERIOD_US);
}

/**
 * @brief 产生 I2C STOP 条件
 *
 * 时序: SDA: LOW→HIGH (SCL=HIGH 期间)
 */
static void soft_i2c_stop(void)
{
    SDA_OUT();
    SDA_LOW();
    g_platform->delay_us(I2C_HALF_PERIOD_US);
    SCL_HIGH();
    g_platform->delay_us(I2C_HALF_PERIOD_US);
    SDA_HIGH();
    g_platform->delay_us(I2C_HALF_PERIOD_US);
}

/**
 * @brief 发送一个字节并检查 ACK
 *
 * @param data 发送的字节
 * @return 0=ACK收到, -1=NACK或超时
 */
static int8_t soft_i2c_write_byte(uint8_t data)
{
    uint32_t timeout;

    SDA_OUT();
    for (int i = 7; i >= 0; i--) {
        if (data & (1 << i)) {
            SDA_HIGH();
        } else {
            SDA_LOW();
        }
        g_platform->delay_us(I2C_SETUP_US);
        SCL_HIGH();
        g_platform->delay_us(I2C_HALF_PERIOD_US);
        SCL_LOW();
        g_platform->delay_us(I2C_HALF_PERIOD_US);
    }

    /* 读取 ACK */
    SDA_IN();
    SCL_HIGH();
    g_platform->delay_us(I2C_HALF_PERIOD_US);

    timeout = I2C_TIMEOUT_CYCLES;
    while (SDA_READ() && --timeout) { }

    SCL_LOW();
    g_platform->delay_us(I2C_HALF_PERIOD_US);

    return (timeout > 0) ? 0 : -1;
}

/**
 * @brief 读取一个字节并发送 ACK/NACK
 *
 * @param ack 1=发送ACK, 0=发送NACK
 * @return 读取的字节
 */
static uint8_t soft_i2c_read_byte(int ack)
{
    uint8_t data = 0;

    SDA_IN();
    for (int i = 7; i >= 0; i--) {
        SCL_HIGH();
        g_platform->delay_us(I2C_HALF_PERIOD_US);
        if (SDA_READ()) {
            data |= (1 << i);
        }
        SCL_LOW();
        g_platform->delay_us(I2C_HALF_PERIOD_US);
    }

    /* 发送 ACK/NACK */
    SDA_OUT();
    if (ack) {
        SDA_LOW();  /* ACK */
    } else {
        SDA_HIGH(); /* NACK */
    }
    g_platform->delay_us(I2C_SETUP_US);
    SCL_HIGH();
    g_platform->delay_us(I2C_HALF_PERIOD_US);
    SCL_LOW();
    g_platform->delay_us(I2C_HALF_PERIOD_US);

    return data;
}

/* ============================================================
 * lsm6dsr_io_t 接口实现
 * ============================================================ */

/**
 * @brief 软件 I2C 多字节读取
 *
 * @param ctx  未使用（传 NULL）
 * @param reg  起始寄存器地址
 * @param buf  读取数据缓冲区
 * @param len  读取字节数
 * @return 0=成功, -1=失败
 *
 * 时序:
 *   START → [ADDR+W] → [REG] → RESTART → [ADDR+R] → [DATA0]...[DATAn] → STOP
 */
static int8_t soft_i2c_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len)
{
    (void)ctx;

    if (!buf || len == 0) return -1;

    /* START → ADDR+W → REG */
    soft_i2c_start();
    if (soft_i2c_write_byte((uint8_t)((LSM6DSR_7BIT_ADDR << 1) | 0)) != 0) {
        soft_i2c_stop();
        return -1;
    }
    if (soft_i2c_write_byte(reg) != 0) {
        soft_i2c_stop();
        return -1;
    }

    /* Repeated START → ADDR+R */
    soft_i2c_start();
    if (soft_i2c_write_byte((uint8_t)((LSM6DSR_7BIT_ADDR << 1) | 1)) != 0) {
        soft_i2c_stop();
        return -1;
    }

    /* 读取数据 (最后一个字节发 NACK) */
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = soft_i2c_read_byte(i < len - 1 ? 1 : 0);
    }

    soft_i2c_stop();
    return 0;
}

/**
 * @brief 软件 I2C 多字节写入
 *
 * @param ctx  未使用（传 NULL）
 * @param reg  起始寄存器地址
 * @param buf  写入数据缓冲区
 * @param len  写入字节数
 * @return 0=成功, -1=失败
 *
 * 时序:
 *   START → [ADDR+W] → [REG] → [DATA0]...[DATAn] → STOP
 */
static int8_t soft_i2c_write(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    (void)ctx;

    if (!buf || len == 0) return -1;

    /* START → ADDR+W → REG */
    soft_i2c_start();
    if (soft_i2c_write_byte((uint8_t)((LSM6DSR_7BIT_ADDR << 1) | 0)) != 0) {
        soft_i2c_stop();
        return -1;
    }
    if (soft_i2c_write_byte(reg) != 0) {
        soft_i2c_stop();
        return -1;
    }

    /* 写入数据 */
    for (uint16_t i = 0; i < len; i++) {
        if (soft_i2c_write_byte(buf[i]) != 0) {
            soft_i2c_stop();
            return -1;
        }
    }

    soft_i2c_stop();
    return 0;
}

/* ============================================================
 * I/O 实例
 * ============================================================ */

/**
 * @brief LSM6DSR 软件 I2C I/O 实例
 *
 * 供 bsp_lsm6dsr.c 使用。
 * 使用方式:
 * @code
 *   #ifdef USE_SOFT_I2C
 *     extern lsm6dsr_io_t lsm6dsr_io_soft;
 *     #define lsm6dsr_io  lsm6dsr_io_soft
 *   #endif
 * @endcode
 */
lsm6dsr_io_t lsm6dsr_io_soft = {
    .read  = soft_i2c_read,
    .write = soft_i2c_write,
    .ctx   = NULL,
};

/**
 * @brief 初始化软件 I2C 桥接层
 *
 * 在 bsp_lsm6dsr_init() 之前调用
 */
void soft_i2c_bridge_init(void)
{
    soft_i2c_init_pins();
}
