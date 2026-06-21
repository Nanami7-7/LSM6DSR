/**
 * @file    filter_platform_default.c
 * @brief   filter_platform.h 的默认实现（libm 包装，可移植）
 *
 * 设计：
 *   - 所有 fp_* 直接转发到 libm（sqrtf/atan2f 等），保证数值与原 filter.c 一致
 *   - fp_get_cycles 返回 0（PC 无 DWT），fp_get_dt 返回固定 0.01f
 *   - fp_delay_ms 空（PC 测试不需要阻塞）
 *   - fp_malloc/fp_free 在 FILTER_STATIC_ONLY 下返回 NULL/空
 *
 * 分支覆盖：
 *   - stm32f407 排除本文件，提供 platform_stm32f407.c（CMSIS-DSP + DWT）
 *   - mspm0g3507 排除本文件，提供 platform_mspm0.c（多项式 + SysTick）
 *
 * 数值合同验证：见 test_platform_smoke.c
 */

#include "filter_platform.h"
#include <math.h>
#include <stdlib.h>

/* ============================================================
 * 数学钩子 — libm 包装
 * ============================================================
 *
 * 注意：filter_math.h 的内联函数（clampf/safe_asinf/wrap_deg_180）
 * 不经 fp_* 转发，因为它们是纯比较/算术，无 libm 依赖。
 * 只有用到 sqrtf/atan2f/asinf/sinf/cosf 的才走 fp_*。
 */

float fp_sqrt(float x)
{
    if (x < 0.0f) return 0.0f;   /* 防止 NaN 传播 */
    return sqrtf(x);
}

float fp_atan2(float y, float x)
{
    return atan2f(y, x);
}

float fp_asin(float x)
{
    /* 钳位到 [-1, 1] 防 NaN */
    if (x < -1.0f) x = -1.0f;
    else if (x > 1.0f) x = 1.0f;
    return asinf(x);
}

float fp_sin(float x)
{
    return sinf(x);
}

float fp_cos(float x)
{
    return cosf(x);
}

float fp_fabs(float x)
{
    return fabsf(x);
}

int fp_isnan(float x)
{
    return isnan(x);
}

int fp_isinf(float x)
{
    return isinf(x);
}

/* ============================================================
 * 时序钩子 — PC/可移植桩
 * ============================================================ */

void fp_init_timing(void)
{
    /* PC 无 DWT，空实现。stm32f407 分支此处使能 DWT->CTRL |= CYCCNTENA */
}

uint32_t fp_get_cycles(void)
{
    /* PC 无周期计数器，返回 0。
     * BSP 层若依赖此值做 dt 计算，会走 fp_get_dt 回退路径。
     * stm32f407 分支返回 DWT->CYCCNT。 */
    return 0u;
}

float fp_get_dt(void)
{
    /* 默认 100Hz 采样假设。BSP 层应优先用 fp_get_cycles 差值算 dt，
     * 此函数仅作 fallback。 */
    return 0.01f;
}

void fp_delay_ms(uint32_t ms)
{
    /* PC 测试不需要阻塞延时。stm32f407 分支调 HAL_Delay(ms)。 */
    (void)ms;
}

/* ============================================================
 * 内存钩子
 * ============================================================ */

void* fp_malloc(size_t n)
{
#if FILTER_STATIC_ONLY
    (void)n;
    return NULL;   /* MCU 静态分配模式，禁用 malloc */
#else
    return malloc(n);
#endif
}

void fp_free(void *p)
{
#if FILTER_STATIC_ONLY
    (void)p;
#else
    free(p);
#endif
}

/* ============================================================
 * 矩阵钩子 — 朴素 C 实现（可移植）
 * ============================================================
 *
 * 当前 EKF/LKF 不调用这些（用手写三重循环）。
 * 保留供 Phase 5 stm32f407 分支的 filter_ekf_cmsis.c 重写时验证接口。
 */

void fp_mat_mult(float *dst, const float *a, const float *b,
                 int ar, int ac, int bc)
{
    /* 行主序：dst[i][j] = Σ_k a[i][k] * b[k][j] */
    for (int i = 0; i < ar; i++) {
        for (int j = 0; j < bc; j++) {
            float sum = 0.0f;
            for (int k = 0; k < ac; k++) {
                sum += a[i * ac + k] * b[k * bc + j];
            }
            dst[i * bc + j] = sum;
        }
    }
}

int fp_mat_inverse_3x3(float *dst, const float *src)
{
    /* 3×3 解析求逆，与 EKF 内的 S_inv 公式一致 */
    float a = src[0], b = src[1], c = src[2];
    float d = src[3], e = src[4], f = src[5];
    float g = src[6], h = src[7], k = src[8];
    float det = a * (e * k - f * h) - b * (d * k - f * g) + c * (d * h - e * g);
    if (fabsf(det) < 1e-10f) return -1;   /* 奇异矩阵 */
    float inv_det = 1.0f / det;
    dst[0] = (e * k - f * h) * inv_det;
    dst[1] = (c * h - b * k) * inv_det;
    dst[2] = (b * f - c * e) * inv_det;
    dst[3] = (f * g - d * k) * inv_det;
    dst[4] = (a * k - c * g) * inv_det;
    dst[5] = (c * d - a * f) * inv_det;
    dst[6] = (d * h - e * g) * inv_det;
    dst[7] = (b * g - a * h) * inv_det;
    dst[8] = (a * e - b * d) * inv_det;
    return 0;
}
