/**
 * @file    filter_math.h
 * @brief   滤波器库共享数学常量与内联辅助函数
 *
 * 设计目标：
 *   - 全部 float 精度，避免 M4F 上 double promotion 性能损失
 *     （未加 f 后缀的常量如 180.0 会被提升为 double，C89/C99 默认行为）
 *   - 内联函数消除函数调用开销，便于 FPU pipeline
 *   - 集中管理数学常量，便于后续 Phase 2 切换到 fp_* 钩子
 *
 * MCU 端考虑：
 *   - 全部 static inline，零调用开销
 *   - 无动态分配，无栈占用（除参数本身）
 *   - FPU 友好：单次运算，无循环
 *
 * 注意：本文件目前直接调用 libm（sqrtf/atan2f/asinf/sinf/cosf）。
 * Phase 2 将引入 filter_platform.h 的 fp_* 钩子后，本文件可整体
 * 改为转发到 fp_*，保持调用方代码不变。
 */

#ifndef FILTER_MATH_H
#define FILTER_MATH_H

#include "filter_platform.h"
#include <math.h>   /* fmodf（wrap_deg_180 用，无 CMSIS-DSP 等价，保留 libm） */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 数学常量（float 精度）
 * ============================================================ */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/** π 的 float 版本，避免 3.14159...f 字面量散落各处 */
#define M_PI_F       3.14159265358979323846f

/** 2π */
#define M_TWO_PI_F   6.28318530717958647692f

/** π/180，度→弧度 */
#define DEG2RAD_F    (M_PI_F / 180.0f)

/** 180/π，弧度→度 */
#define RAD2DEG_F    (180.0f / M_PI_F)

/** 重力加速度 (m/s²)，LSM6DSR 默认单位转换用 */
#define GRAVITY_MSS  9.80665f

/* ============================================================
 * 内联辅助函数
 * ============================================================ */

/**
 * @brief 钳位 float 到 [lo, hi]
 * @note  比 fmaxf/fminf 组合更直接，FPU 单周期比较-选择
 */
static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

/**
 * @brief 安全 asin：钳位输入到 [-1, 1] 避免 NaN
 * @note  转发到 fp_asin（默认 libm，分支可覆盖为 CMSIS-DSP）
 */
static inline float safe_asinf(float x)
{
    return fp_asin(clampf(x, -1.0f, 1.0f));
}

/**
 * @brief 安全除法：分母接近 0 时返回 0
 * @note  用于 ACC 归一化等场景，避免除零产生 Inf
 */
static inline float safe_divf(float num, float den, float eps)
{
    if (fp_fabs(den) < eps) return 0.0f;
    return num / den;
}

/**
 * @brief 角度 wrap 到 [-180, 180)
 * @note  互补/LKF 的 yaw 累积用，避免 ±180 跳变
 *        O(1) 实现，优于 while 循环版本
 */
static inline float wrap_deg_180(float deg)
{
    /* fmodf 让结果符号与被除数一致，需后续修正到 [-180, 180) */
    float r = fmodf(deg + 180.0f, 360.0f);
    if (r < 0.0f) r += 360.0f;
    return r - 180.0f;
}

/**
 * @brief 快速平方倒数 1/sqrtf(x)，带 NaN 防护
 * @note  转发到 fp_sqrt，分支可覆盖为 arm_sqrt_f32 + 1/x
 */
static inline float fast_rsqrtf(float x)
{
    if (x <= 0.0f) return 0.0f;  /* 防止 NaN/Inf 传播 */
    return 1.0f / fp_sqrt(x);
}

#ifdef __cplusplus
}
#endif

#endif /* FILTER_MATH_H */
