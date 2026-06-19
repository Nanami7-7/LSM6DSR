/**
 * @file    spi_bridge.c
 * @brief   MSPM0G3507 硬件 SPI 桥接层 — 实现 lsm6dsr_io_t 回调
 *
 * 参考: lsm6dsr_STdC/example/read_data_simple.c (ST 官方 SPI 实现)
 *
 * 硬件配置（通过宏定义，方便修改）：
 *   - SPI 实例：SPI1 (MSPM0G3507)
 *   - 时钟速率：10 MHz
 *   - 模式：CPOL=0, CPHA=0 (Mode 0, Motorola 4-Wire)
 *   - 数据位：8-bit, MSB First
 *   - CS：GPIO 手动控制（PB17）
 *
 * LSM6DSR SPI 协议：
 *   - 读操作：CS↓ → [REG|0x80] → [DUMMY]×len → CS↑
 *   - 写操作：CS↓ → [REG&0x7F] → [DATA]×len → CS↑
 *
 * 使用方式：
 * @code
 *   extern lsm6dsr_io_t lsm6dsr_io_spi;
 *   extern void spi_bridge_init(void);
 *
 *   // 初始化
 *   spi_bridge_init();
 *
 *   // 使用（通过 lsm6dsr_io_t 接口）
 *   lsm6dsr_io = lsm6dsr_io_spi;
 * @endcode
 */

#include "platform.h"
#include "lsm6dsr.h"
#include "ti_msp_dl_config.h"

/* ============================================================
 * 硬件配置宏（修改引脚/实例只需改这里）
 * ============================================================ */

/** @name SPI 实例 */
/**@{*/
#define LSM6DSR_SPI_INST         SPI1          /**< SPI 外设实例 */
/**@}*/

/** @name CS 引脚（GPIO 手动控制） */
/**@{*/
#define LSM6DSR_CS_PORT          GPIOB         /**< CS 端口 */
#define LSM6DSR_CS_PIN           DL_GPIO_PIN_17 /**< CS 引脚 (PB17) */
#define LSM6DSR_CS_IOMUX         IOMUX_PINCM44  /**< CS 引脚复用 */
/**@}*/

/** @name SPI 时序参数 */
/**@{*/
/* 注意：DL_SPI_transmitDataBlocking8/receiveDataBlocking8 内部已有超时机制 */
/**@}*/

/** @name LSM6DSR SPI 协议常量 */
/**@{*/
#define LSM6DSR_SPI_READ_FLAG    0x80          /**< 读标志：bit7=1 */
#define LSM6DSR_SPI_WRITE_FLAG   0x7F          /**< 写掩码：bit7=0 */
/**@}*/

/* ============================================================
 * CS 控制宏
 * ============================================================ */

/** CS 拉低（选中） */
#define CS_LOW()   DL_GPIO_clearPins(LSM6DSR_CS_PORT, LSM6DSR_CS_PIN)

/** CS 拉高（释放） */
#define CS_HIGH()  DL_GPIO_setPins(LSM6DSR_CS_PORT, LSM6DSR_CS_PIN)

/* ============================================================
 * SPI 读写实现（参考 ST 官方 read_data_simple.c）
 * ============================================================ */

/**
 * @brief SPI 多字节读取
 *
 * 时序（参考 ST 官方 platform_read, line 288-308）：
 *   CS↓ → [REG|0x80] → 排空RX → [DUMMY]×len → 读取RX → CS↑
 *
 * SPI 全双工特性：每发送1字节，RX FIFO 也收到1字节。
 * 发送地址字节后 RX FIFO 中有1字节垃圾，必须先排空。
 *
 * @param ctx  未使用（传 NULL）
 * @param reg  起始寄存器地址
 * @param buf  读取数据缓冲区
 * @param len  读取字节数
 * @return 0=成功, -1=失败
 */
static int8_t mspm0_spi_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len)
{
    (void)ctx;
    uint8_t dummy;

    if (!buf || len == 0) return -1;

    /* CS 拉低（开始传输） */
    CS_LOW();

    /* 发送寄存器地址（bit7=1 表示读） */
    DL_SPI_transmitDataBlocking8(LSM6DSR_SPI_INST,
                                  (uint8_t)(reg | LSM6DSR_SPI_READ_FLAG));

    /* 排空 RX FIFO：地址字节发送时产生的垃圾字节 */
    dummy = DL_SPI_receiveDataBlocking8(LSM6DSR_SPI_INST);
    (void)dummy;

    /* 读取数据：发送 dummy 字节，接收返回数据 */
    for (uint16_t i = 0; i < len; i++) {
        DL_SPI_transmitDataBlocking8(LSM6DSR_SPI_INST, 0xFF);
        buf[i] = DL_SPI_receiveDataBlocking8(LSM6DSR_SPI_INST);
    }

    /* CS 拉高（结束传输） */
    CS_HIGH();

    return 0;
}

/**
 * @brief SPI 多字节写入
 *
 * 时序（参考 ST 官方 platform_write, line 258-276）：
 *   CS↓ → [REG&0x7F] → 排空RX → [DATA]×len → 排空RX → CS↑
 *
 * SPI 全双工特性：每发送1字节，RX FIFO 也收到1字节。
 * MSPM0 RX FIFO 只有4字节，长写入会溢出。必须排空。
 *
 * @param ctx  未使用（传 NULL）
 * @param reg  起始寄存器地址
 * @param buf  写入数据缓冲区
 * @param len  写入字节数
 * @return 0=成功, -1=失败
 */
static int8_t mspm0_spi_write(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    (void)ctx;
    uint8_t dummy;

    if (!buf || len == 0) return -1;

    /* CS 拉低（开始传输） */
    CS_LOW();

    /* 发送寄存器地址（bit7=0 表示写） */
    DL_SPI_transmitDataBlocking8(LSM6DSR_SPI_INST,
                                  (uint8_t)(reg & LSM6DSR_SPI_WRITE_FLAG));

    /* 排空 RX FIFO：地址字节发送时产生的垃圾字节 */
    dummy = DL_SPI_receiveDataBlocking8(LSM6DSR_SPI_INST);
    (void)dummy;

    /* 发送数据，每发一字节排空一次 RX FIFO 防止溢出 */
    for (uint16_t i = 0; i < len; i++) {
        DL_SPI_transmitDataBlocking8(LSM6DSR_SPI_INST, buf[i]);
        dummy = DL_SPI_receiveDataBlocking8(LSM6DSR_SPI_INST);
        (void)dummy;
    }

    /* CS 拉高（结束传输） */
    CS_HIGH();

    return 0;
}

/* ============================================================
 * I/O 实例
 * ============================================================ */

/**
 * @brief LSM6DSR SPI I/O 实例
 *
 * 供 bsp_lsm6dsr.c 使用。
 * ctx 设为 NULL（CS 通过宏直接控制，无需上下文）。
 */
lsm6dsr_io_t lsm6dsr_io_spi = {
    .read  = mspm0_spi_read,
    .write = mspm0_spi_write,
    .ctx   = NULL,
};

/* ============================================================
 * SPI 初始化
 * ============================================================ */

/**
 * @brief 初始化 SPI 桥接层
 *
 * 配置 CS 引脚为 GPIO 输出模式。
 * SPI 外设由 SysConfig 配置（ti_msp_dl_config.c）。
 *
 * @note 在 bsp_lsm6dsr_init() 之前调用
 */
void spi_bridge_init(void)
{
    /* CS 引脚初始化为输出高电平（未选中） */
    DL_GPIO_initDigitalOutput(LSM6DSR_CS_IOMUX);
    DL_GPIO_setPins(LSM6DSR_CS_PORT, LSM6DSR_CS_PIN);
    DL_GPIO_enableOutput(LSM6DSR_CS_PORT, LSM6DSR_CS_PIN);

    /* 确保 CS 初始状态为高 */
    CS_HIGH();
}
