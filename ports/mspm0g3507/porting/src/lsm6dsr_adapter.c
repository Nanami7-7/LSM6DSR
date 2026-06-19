/**
 * @file    lsm6dsr_adapter.c
 * @brief   LSM6DSR 官方驱动适配器实现
 *
 * 将 lsm6dsr_io_t 接口适配到官方 ST 驱动的 stmdev_ctx_t 接口。
 *
 * 适配原理：
 *   lsm6dsr_io_t.read(ctx, reg, buf, len)  →  stmdev_ctx_t.read_reg(handle, reg, buf, len)
 *   lsm6dsr_io_t.write(ctx, reg, buf, len) →  stmdev_ctx_t.write_reg(handle, reg, buf, len)
 *
 *   回调函数将 stmdev_ctx_t.handle (指向 lsm6dsr_io_t) 转换后调用对应的 read/write。
 */

#include "lsm6dsr_adapter.h"

/* ============================================================
 * 适配回调函数
 * ============================================================ */

/**
 * @brief 写寄存器回调（stmdev_ctx_t.write_reg）
 *
 * 将调用转发到 lsm6dsr_io_t.write()
 */
static int32_t adapter_write_reg(void *handle, uint8_t reg,
                                  uint8_t *bufp, uint16_t len)
{
    lsm6dsr_io_t *io = (lsm6dsr_io_t *)handle;
    if (!io || !io->write) return -1;
    return (int32_t)io->write(io->ctx, reg, (const uint8_t *)bufp, len);
}

/**
 * @brief 读寄存器回调（stmdev_ctx_t.read_reg）
 *
 * 将调用转发到 lsm6dsr_io_t.read()
 */
static int32_t adapter_read_reg(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
    lsm6dsr_io_t *io = (lsm6dsr_io_t *)handle;
    if (!io || !io->read) return -1;
    return (int32_t)io->read(io->ctx, reg, bufp, len);
}

/* ============================================================
 * 适配器接口实现
 * ============================================================ */

int lsm6dsr_adapter_init(lsm6dsr_io_t *io, stmdev_ctx_t *ctx)
{
    if (!io || !ctx) return -1;

    ctx->write_reg = adapter_write_reg;
    ctx->read_reg  = adapter_read_reg;
    ctx->handle    = io;  /* 回调函数通过 handle 访问 lsm6dsr_io_t */

    return 0;
}

int lsm6dsr_adapter_who_am_i(stmdev_ctx_t *ctx)
{
    uint8_t id = 0;
    if (!ctx) return -1;
    if (lsm6dsr_device_id_get(ctx, &id) != 0) return -1;
    return (int)id;
}

int lsm6dsr_adapter_read_accel_raw(stmdev_ctx_t *ctx,
                                     int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    if (!ctx || !ax || !ay || !az) return -1;
    if (lsm6dsr_acceleration_raw_get(ctx, buf) != 0) return -1;

    /* LSM6DSR 输出小端格式：[XL, XH, YL, YH, ZL, ZH] */
    *ax = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    *ay = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    *az = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

    return 0;
}

int lsm6dsr_adapter_read_gyro_raw(stmdev_ctx_t *ctx,
                                    int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];
    if (!ctx || !gx || !gy || !gz) return -1;
    if (lsm6dsr_angular_rate_raw_get(ctx, buf) != 0) return -1;

    /* LSM6DSR 输出小端格式：[XL, XH, YL, YH, ZL, ZH] */
    *gx = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    *gy = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    *gz = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

    return 0;
}

int lsm6dsr_adapter_read_temp_raw(stmdev_ctx_t *ctx, int16_t *temp)
{
    uint8_t buf[2];
    if (!ctx || !temp) return -1;
    if (lsm6dsr_temperature_raw_get(ctx, buf) != 0) return -1;

    /* LSM6DSR 温度小端格式：[TL, TH] */
    *temp = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);

    return 0;
}
