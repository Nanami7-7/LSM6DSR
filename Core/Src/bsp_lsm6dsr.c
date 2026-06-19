/**
 * @file    bsp_lsm6dsr.c
 * @brief   BSP 业务层 — 互补滤波器与姿态估计算法实现
 *
 * 核心算法与功能:
 *   - 互补滤波器 (Complementary Filter)：融合 ACC 低频姿态 + GYRO 积分
 *   - 自适应 α：运动时 α=0.99 (近纯 GYRO)，静止时 α=0.30 (ACC 主导收敛)
 *   - 三重静止检测：
 *       1. ACC 方差滑动窗口 (抗振动干扰)
 *       2. ACC 幅值校验 (|mag² - 1G| < tol)
 *       3. GYRO 幅值校验 (|gyro| > threshold → 运动)
 *   - Runtime 偏置跟踪：静止时 bg += rate × fg (X/Y vs Z 独立速率)
 *   - 平台抽象计时：通过 platform_t 接口获取微秒级时间戳
 *   - VOFA+ 格式化：10 通道 FireWater 协议
 *
 * 平台依赖：
 *   - g_platform->delay_ms()   — 毫秒延时
 *   - g_platform->get_tick_us() — 微秒级时间戳
 *   - g_platform->debug_printf() — 调试输出
 */
#include "bsp_lsm6dsr.h"
#include "lsm6dsr.h"
#include "platform.h"
#include "log.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/** @brief 平台 I/O 实例 (定义于 test_lsm6dsr.c) */
extern lsm6dsr_io_t lsm6dsr_io;

/* ---- 默认全局上下文（用于向后兼容） ---- */
static bsp_lsm6dsr_ctx_t default_ctx;

/**
 * @brief  计算 ACC 3 轴方差总和（内部函数）
 * @param[in,out] ctx  上下文结构体指针
 * @return 方差总和 (mg²)，窗口未填满时返回 0
 */
static float compute_acc_variance(bsp_lsm6dsr_ctx_t *ctx)
{
    if (ctx->var_samples < BSP_ACC_VAR_WINDOW) return 0.0f;

    float mx = 0, my = 0, mz = 0;
    for (int i = 0; i < BSP_ACC_VAR_WINDOW; i++) {
        mx += ctx->ax_buf[i]; my += ctx->ay_buf[i]; mz += ctx->az_buf[i];
    }
    mx /= BSP_ACC_VAR_WINDOW;
    my /= BSP_ACC_VAR_WINDOW;
    mz /= BSP_ACC_VAR_WINDOW;

    float vx = 0, vy = 0, vz = 0;
    for (int i = 0; i < BSP_ACC_VAR_WINDOW; i++) {
        float dx = ctx->ax_buf[i] - mx; vx += dx * dx;
        float dy = ctx->ay_buf[i] - my; vy += dy * dy;
        float dz = ctx->az_buf[i] - mz; vz += dz * dz;
    }
    vx /= BSP_ACC_VAR_WINDOW;
    vy /= BSP_ACC_VAR_WINDOW;
    vz /= BSP_ACC_VAR_WINDOW;

    ctx->last_variance = vx + vy + vz;
    return ctx->last_variance;
}

/**
 * @brief  初始化滤波器状态（内部函数）
 * @param[out] ctx  上下文结构体指针
 * @param[in]  ax0  初始加速度 X (g)
 * @param[in]  ay0  初始加速度 Y (g)
 * @param[in]  az0  初始加速度 Z (g)
 */
static void init_filter_state(bsp_lsm6dsr_ctx_t *ctx, float ax0, float ay0, float az0)
{
    ctx->pitch = atan2(-ax0, sqrt(ay0*ay0 + az0*az0)) * 180.0 / M_PI;
    ctx->roll  = atan2( ay0, sqrt(ax0*ax0 + az0*az0)) * 180.0 / M_PI;
    ctx->yaw   = 0.0;

    ctx->var_buf_idx = 0;
    ctx->var_samples = BSP_ACC_VAR_WINDOW;
    for (int i = 0; i < BSP_ACC_VAR_WINDOW; i++) {
        ctx->ax_buf[i] = ax0; ctx->ay_buf[i] = ay0; ctx->az_buf[i] = az0;
    }
    ctx->alpha = BSP_ALPHA_STATIONARY;
    ctx->last_variance = 0.0f;
}

/* ------------------------------------------------------------------ */
/**
 * @brief  初始化 BSP 上下文
 * @param[out] ctx  上下文结构体指针（调用者分配）
 * @return 0=成功, -1=失败
 */
int bsp_lsm6dsr_init_ctx(bsp_lsm6dsr_ctx_t *ctx)
{
    if (!ctx) {
        LOG_ERR("Context pointer is NULL");
        return -1;
    }

    /* 清零所有状态 */
    memset(ctx, 0, sizeof(bsp_lsm6dsr_ctx_t));

    /* Full reset + wait */
    lsm6dsr_reset(&lsm6dsr_io);
    g_platform->delay_ms(100);

    /* Debug: hardware info */
    {
        uint8_t whoami = 0;
        lsm6dsr_read_reg(&lsm6dsr_io, LSM6DSR_REG_WHO_AM_I, &whoami);
        LOG_INDENT("I2C: DevAddr=0x%04X  WHO_AM_I=0x%02X",
                   (unsigned)LSM6DSR_I2C_ADDR, (unsigned)whoami);
    }

    /* Sensor configuration */
    lsm6dsr_i3c_disable(&lsm6dsr_io);
    lsm6dsr_set_if_inc(&lsm6dsr_io, 1);
    lsm6dsr_set_bdu(&lsm6dsr_io, 1);
    lsm6dsr_accel_config(&lsm6dsr_io,
        LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_gyro_config(&lsm6dsr_io,
        LSM6DSR_GYRO_ODR_104HZ, LSM6DSR_GYRO_FS_250DPS);
    g_platform->delay_ms(BSP_CALIB_SETTLE_MS);

    LOG_INDENT("ACC ODR=104Hz FS=4G  GYRO ODR=104Hz FS=250dps");

    /* Initialize filter state */
    {
        float ax0, ay0, az0;
        lsm6dsr_read_accel_float(&lsm6dsr_io, &ax0, &ay0, &az0,
                                 LSM6DSR_ACCEL_FS_4G);
        init_filter_state(ctx, ax0, ay0, az0);
    }

    /* Create filter instance */
    ctx->current_filter_type = FILTER_TYPE_COMPLEMENTARY;
    ctx->active_filter = filter_create(ctx->current_filter_type);
    if (!ctx->active_filter) {
        LOG_ERR("Failed to create filter");
        return -1;
    }
    LOG_INDENT("Filter: %s", filter_type_name(ctx->current_filter_type));

    LOG_INDENT("Initial pitch=%.2f  roll=%.2f",
               ctx->pitch, ctx->roll);

    /* Initialize timing */
    ctx->last_tick_us = g_platform->get_tick_us();
    ctx->is_stationary = 1;
    ctx->initialized = 1;

    /* Calibrate */
    bsp_lsm6dsr_calibrate_ctx(ctx);

    LOG_INDENT("BSP init done  (cal=%s, bias=%.4f,%.4f,%.4f)  alpha=%.2f",
               ctx->cal_ok ? "OK" : "FAIL",
               (double)ctx->bgx, (double)ctx->bgy, (double)ctx->bgz,
               ctx->alpha);

    return 0;
}

/* ------------------------------------------------------------------ */
/**
 * @brief  传感器初始化（向后兼容版本）
 * @details 使用默认全局上下文，不支持多实例
 */
void bsp_lsm6dsr_init(void)
{
    if (bsp_lsm6dsr_init_ctx(&default_ctx) != 0) {
        LOG_ERR("Failed to initialize default context");
        while (1);  /* 阻塞报错 */
    }
}

/* ------------------------------------------------------------------ */
/**
 * @brief  陀螺零偏校准（上下文版本）
 * @param[in,out] ctx  上下文结构体指针
 * @return 0=成功, -1=失败
 */
int bsp_lsm6dsr_calibrate_ctx(bsp_lsm6dsr_ctx_t *ctx)
{
    if (!ctx || !ctx->initialized) {
        return -1;
    }

    ctx->bgx = 0.0f; ctx->bgy = 0.0f; ctx->bgz = 0.0f;
    ctx->cal_ok = 0;
    {
        int   n_valid = 0;
        float pax, pay, paz;

        lsm6dsr_read_accel_float(&lsm6dsr_io, &pax, &pay, &paz,
                                 LSM6DSR_ACCEL_FS_4G);

        LOG_INFO("Calibrating gyro bias (%d samples, keep still)...",
                 BSP_CALIB_SAMPLES);

        for (int i = 0; i < BSP_CALIB_SAMPLES; i++)
        {
            float tax, tay, taz, tgx, tgy, tgz;
            lsm6dsr_read_accel_float(&lsm6dsr_io, &tax, &tay, &taz,
                                     LSM6DSR_ACCEL_FS_4G);
            lsm6dsr_read_gyro_float(&lsm6dsr_io, &tgx, &tgy, &tgz,
                                    LSM6DSR_GYRO_FS_250DPS);

            float mag2 = tax*tax + tay*tay + taz*taz;
            if (fabsf(mag2 - BSP_CALIB_ACC_MAG_REF) < BSP_CALIB_ACC_MAG_TOL
                && fabsf(tax - pax) < BSP_CALIB_ACC_DELTA_MAX
                && fabsf(tay - pay) < BSP_CALIB_ACC_DELTA_MAX
                && fabsf(taz - paz) < BSP_CALIB_ACC_DELTA_MAX)
            {
                ctx->bgx += tgx; ctx->bgy += tgy; ctx->bgz += tgz;
                n_valid++;
            }
            pax = tax; pay = tay; paz = taz;
            g_platform->delay_ms(BSP_CALIB_SAMPLE_DELAY_MS);
        }

        if (n_valid >= BSP_CALIB_SAMPLES / 2)
        {
            ctx->bgx /= (float)n_valid;
            ctx->bgy /= (float)n_valid;
            ctx->bgz /= (float)n_valid;
            ctx->cal_ok = 1;
            LOG_INFO("Gyro bias: X=%.4f  Y=%.4f  Z=%.4f dps  (%d/%d OK)",
                     (double)ctx->bgx, (double)ctx->bgy, (double)ctx->bgz,
                     n_valid, BSP_CALIB_SAMPLES);
        }
        else
        {
            ctx->bgx = ctx->bgy = ctx->bgz = 0.0f;
            LOG_WARN("Too few stationary samples (%d/%d), bias=0",
                     n_valid, BSP_CALIB_SAMPLES);
        }
    }

    /* read one frame to print residual */
    {
        float gx0, gy0, gz0;
        lsm6dsr_read_gyro_float(&lsm6dsr_io, &gx0, &gy0, &gz0,
                                LSM6DSR_GYRO_FS_250DPS);
        if (ctx->cal_ok) { gx0 -= ctx->bgx; gy0 -= ctx->bgy; gz0 -= ctx->bgz; }
        LOG_INDENT("GYRO residual after cal: X=%.4f  Y=%.4f  Z=%.4f dps",
                   (double)gx0, (double)gy0, (double)gz0);
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/**
 * @brief  陀螺零偏校准（向后兼容版本）
 */
void bsp_lsm6dsr_calibrate(void)
{
    bsp_lsm6dsr_calibrate_ctx(&default_ctx);
}

/* ------------------------------------------------------------------ */
/**
 * @brief  姿态更新（上下文版本）
 * @param[in,out] ctx  上下文结构体指针
 * @param[out]    data 输出数据结构体指针
 * @return 0=成功, <0=错误码
 *
 * @details 每帧依次执行:
 *          1. 时间戳计算（使用 64 位防溢出）
 *          2. 传感器读取
 *          3. 方差滑动窗口更新
 *          4. 三重静止检测
 *          5. 偏置校正与跟踪
 *          6. 滤波器更新
 *          7. 结果填充
 */
int bsp_lsm6dsr_update_ctx(bsp_lsm6dsr_ctx_t *ctx, bsp_lsm6dsr_data_t *data)
{
    if (!ctx || !ctx->initialized) {
        return -1;
    }
    if (!data) {
        return -2;
    }

    /* ---- timing (platform tick, microsecond resolution) ---- */
    uint64_t now = g_platform->get_tick_us();
    double dt;

    if (ctx->last_tick_us == 0) {
        /* 首帧或重置后 */
        dt = 0.01;  /* 默认 10ms */
    } else {
        dt = (double)(now - ctx->last_tick_us) * 1e-6;  /* us to seconds */

        /* 检测时间戳回绕（理论上 64 位不会发生，但保险起见） */
        if (now < ctx->last_tick_us) {
            dt = 0.01;
        }

        /* 检测异常的时间间隔 */
        if (dt > 0.5 || dt <= 0.0) {
            dt = 0.01;
        }
    }
    ctx->last_tick_us = now;

    /* ---- read sensors ---- */
    float fax, fay, faz, fgx, fgy, fgz;
    lsm6dsr_read_accel_float(&lsm6dsr_io, &fax, &fay, &faz,
                             LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_read_gyro_float(&lsm6dsr_io, &fgx, &fgy, &fgz,
                            LSM6DSR_GYRO_FS_250DPS);
    lsm6dsr_read_temp(&lsm6dsr_io, &data->temperature);

    /* ---- update sliding window ---- */
    ctx->ax_buf[ctx->var_buf_idx] = fax;
    ctx->ay_buf[ctx->var_buf_idx] = fay;
    ctx->az_buf[ctx->var_buf_idx] = faz;
    ctx->var_buf_idx = (ctx->var_buf_idx + 1) % BSP_ACC_VAR_WINDOW;
    if (ctx->var_samples < BSP_ACC_VAR_WINDOW) ctx->var_samples++;

    /* ---- variance-based stationary detection ---- */
    float var_sum = compute_acc_variance(ctx);
    int stationary = (var_sum < BSP_ACC_VAR_THRESHOLD);
    /* dual-check: accel magnitude must be near 1G */
    float mag2 = fax*fax + fay*fay + faz*faz;
    if (fabsf(mag2 - BSP_CALIB_ACC_MAG_REF) >= BSP_CALIB_ACC_MAG_TOL) {
        stationary = 0;
    }

    /* ---- bias correction ---- */
    if (ctx->cal_ok) {
        fgx -= ctx->bgx; fgy -= ctx->bgy; fgz -= ctx->bgz;

        /* triple-check: gyro magnitude rejects bias-eating pure rotation */
        float gyro_mag2 = fgx*fgx + fgy*fgy + fgz*fgz;
        if (gyro_mag2 > (BSP_GYRO_MOTION_THRESHOLD * BSP_GYRO_MOTION_THRESHOLD)) {
            stationary = 0;
        }

        /* ---- runtime bias tracking (only when stationary) ---- */
        if (stationary) {
            ctx->bgx += BSP_BIAS_STATIONARY_RATE * fgx;
            ctx->bgy += BSP_BIAS_STATIONARY_RATE * fgy;
            ctx->bgz += BSP_BIAS_STATIONARY_RATE_Z * fgz;
        }
    }

    /* ---- update filter ---- */
    if (ctx->active_filter) {
        filter_input_t fin = {
            .ax = fax, .ay = fay, .az = faz,
            .gx = fgx, .gy = fgy, .gz = fgz,
            .dt = (float)dt
        };
        filter_output_t fout;
        ctx->active_filter->update(ctx->active_filter, &fin, &fout);

        data->pitch = fout.pitch;
        data->roll  = fout.roll;
        data->yaw   = fout.yaw;
    } else {
        /* fallback: 互补滤波器 */
        double acc_pitch = atan2(-fax, sqrt(fay*fay + faz*faz)) * 180.0 / M_PI;
        double acc_roll  = atan2( fay, sqrt(fax*fax + faz*faz)) * 180.0 / M_PI;
        ctx->pitch = 0.98 * (ctx->pitch + fgy * dt) + 0.02 * acc_pitch;
        ctx->roll  = 0.98 * (ctx->roll  - fgx * dt) + 0.02 * acc_roll;
        ctx->yaw  += fgz * dt;
        data->pitch = (float)ctx->pitch;
        data->roll  = (float)ctx->roll;
        data->yaw   = (float)ctx->yaw;
    }

    /* ---- fill result struct ---- */
    data->ax    = fax * 0.00980665f;
    data->ay    = fay * 0.00980665f;
    data->az    = faz * 0.00980665f;
    data->gx    = fgx;
    data->gy    = fgy;
    data->gz    = fgz;

    /* ---- cache for production API ---- */
    ctx->last_data = *data;
    ctx->is_stationary = stationary;
    ctx->update_count++;

    return 0;
}

/* ------------------------------------------------------------------ */
/**
 * @brief  姿态更新（向后兼容版本）
 */
void bsp_lsm6dsr_update(bsp_lsm6dsr_data_t *data)
{
    bsp_lsm6dsr_update_ctx(&default_ctx, data);
}

/* ------------------------------------------------------------------ */
/**
 * @brief  获取陀螺零偏（上下文版本）
 */
void bsp_lsm6dsr_get_bias_ctx(const bsp_lsm6dsr_ctx_t *ctx, float *bx, float *by, float *bz)
{
    if (!ctx) return;
    if (bx) *bx = ctx->bgx;
    if (by) *by = ctx->bgy;
    if (bz) *bz = ctx->bgz;
}

/**
 * @brief  获取陀螺零偏（向后兼容版本）
 */
void bsp_lsm6dsr_get_bias(float *bx, float *by, float *bz)
{
    bsp_lsm6dsr_get_bias_ctx(&default_ctx, bx, by, bz);
}

/**
 * @brief  查询静止状态（上下文版本）
 */
int bsp_lsm6dsr_is_stationary_ctx(const bsp_lsm6dsr_ctx_t *ctx)
{
    if (!ctx) return 0;
    return ctx->is_stationary;
}

/**
 * @brief  查询静止状态（向后兼容版本）
 */
int bsp_lsm6dsr_is_stationary(void)
{
    return bsp_lsm6dsr_is_stationary_ctx(&default_ctx);
}

/**
 * @brief  获取上一帧 ACC 方差总和（向后兼容版本）
 */
float bsp_lsm6dsr_get_last_variance(void)
{
    return default_ctx.last_variance;
}

/**
 * @brief  获取最新缓存数据（向后兼容版本）
 */
const bsp_lsm6dsr_data_t* bsp_lsm6dsr_get_data(void)
{
    return &default_ctx.last_data;
}

/* ------------------------------------------------------------------ */
/**
 * @brief  销毁上下文，释放滤波器资源
 */
void bsp_lsm6dsr_destroy_ctx(bsp_lsm6dsr_ctx_t *ctx)
{
    if (!ctx) return;

    if (ctx->active_filter) {
        filter_destroy_safe(ctx->active_filter);
        ctx->active_filter = NULL;
    }

    ctx->initialized = 0;
}

/* ------------------------------------------------------------------ */
/**
 * @brief  切换滤波器类型（上下文版本）
 */
int bsp_lsm6dsr_set_filter_ctx(bsp_lsm6dsr_ctx_t *ctx, filter_type_t type)
{
    if (!ctx || !ctx->initialized) {
        return -1;
    }

    if (type < 0 || type >= FILTER_TYPE_COUNT) {
        LOG_ERR("Invalid filter type %d", type);
        return -1;
    }

    /* 销毁旧滤波器 */
    if (ctx->active_filter) {
        filter_destroy_safe(ctx->active_filter);
        ctx->active_filter = NULL;
    }

    /* 创建新滤波器 */
    ctx->active_filter = filter_create(type);
    if (!ctx->active_filter) {
        LOG_ERR("Failed to create filter type %d", type);
        return -1;
    }

    ctx->current_filter_type = type;
    LOG_INFO("Filter switched to: %s", filter_type_name(type));
    return 0;
}

/**
 * @brief  切换滤波器类型（向后兼容版本）
 */
int bsp_lsm6dsr_set_filter(filter_type_t type)
{
    return bsp_lsm6dsr_set_filter_ctx(&default_ctx, type);
}

/**
 * @brief  获取当前滤波器类型（向后兼容版本）
 */
filter_type_t bsp_lsm6dsr_get_filter_type(void)
{
    return default_ctx.current_filter_type;
}

/**
 * @brief  设置滤波器参数（上下文版本）
 */
int bsp_lsm6dsr_set_filter_param_ctx(bsp_lsm6dsr_ctx_t *ctx, filter_param_t param, float value)
{
    if (!ctx || !ctx->initialized || !ctx->active_filter) {
        return -1;
    }

    if (param < 0 || param >= FILTER_PARAM_COUNT) {
        LOG_ERR("Invalid filter param %d", param);
        return -1;
    }

    ctx->active_filter->set_param(ctx->active_filter, param, value);
    LOG_DBG("Filter param %d set to %.4f", param, (double)value);
    return 0;
}

/**
 * @brief  设置滤波器参数（向后兼容版本）
 */
int bsp_lsm6dsr_set_filter_param(filter_param_t param, float value)
{
    return bsp_lsm6dsr_set_filter_param_ctx(&default_ctx, param, value);
}

/**
 * @brief  获取滤波器类型名称（向后兼容版本）
 */
const char* bsp_lsm6dsr_get_filter_name(void)
{
    return filter_type_name(default_ctx.current_filter_type);
}

/* ------------------------------------------------------------------ */
/**
 * @brief  VOFA+ FireWater 10 通道格式化
 * @param[out] buf     输出缓冲区
 * @param[in]  buf_size 缓冲区大小
 * @param[in]  data     IMU 数据指针
 * @return 写入缓冲区的字符数
 *
 * @details 格式: "ax,ay,az,gx,gy,gz,pitch,roll,yaw,temp\r\n"
 *          - ax/ay/az: m/s² (%.3f)
 *          - gx/gy/gz: dps (%.3f)
 *          - pitch/roll/yaw: deg (%.2f)
 *          - temp: °C (%.1f)
 */
int bsp_lsm6dsr_vofa_format(char *buf, int buf_size, const bsp_lsm6dsr_data_t *data)
{
    if (!buf || !data || buf_size <= 0) {
        return 0;
    }

    return snprintf(buf, buf_size,
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f\r\n",
        (double)data->ax, (double)data->ay, (double)data->az,
        (double)data->gx, (double)data->gy, (double)data->gz,
        data->pitch, data->roll, data->yaw, data->temperature);
}
