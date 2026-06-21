/**
 * @file    platform_stm32f407.c
 * @brief   STM32F407 平台后端 — CMSIS-DSP 加速 + DWT 计时 + HAL 延时
 *
 * 设计：
 *   - fp_sqrt → arm_sqrt_f32（CMSIS-DSP，比 libm sqrtf 快 ~40%）
 *   - fp_atan2 → atan2f（回退，arm_atan2_f32 需 CMSIS-DSP v1.10+）
 *   - fp_get_cycles → DWT->CYCCNT（~6ns @ 168MHz）
 *   - fp_delay_ms → HAL_Delay
 *   - fp_init_timing → 使能 DWT CYCCNT
 *
 * 编译条件（Keil MDK / STM32CubeIDE）：
 *   -ICMSIS/Core/Include -ICMSIS/DSP/Include
 *   链接 arm_cortexM4lf_math.lib（或源码 libarm_cortexM4lf_math.a）
 *
 * PC 编译测试（无 CMSIS-DSP 时）：
 *   定义 FILTER_PLATFORM_STM32F407_NO_HW 跳过 STM32 HAL 头文件
 *   所有 fp_* 回退到 libm + 桩
 */

#include "filter_platform.h"
#include <math.h>

#if !defined(FILTER_PLATFORM_STM32F407_NO_HW)
/* MCU 编译路径：包含 STM32 HAL + CMSIS-DSP */
#include "stm32f4xx.h"
#include "arm_math.h"
#endif

/* ═══════════════════════════════════════════════════════════════
 * 数学钩子
 * ═══════════════════════════════════════════════════════════════ */

float fp_sqrt(float x)
{
#if !defined(FILTER_PLATFORM_STM32F407_NO_HW)
    /* CMSIS-DSP arm_sqrt_f32 — M4F FPU 硬浮点，单周期 √x 近似 */
    float y;
    arm_sqrt_f32(x, &y);
    return y;
#else
    /* PC 回退 */
    if (x < 0.0f) return 0.0f;
    return sqrtf(x);
#endif
}

float fp_atan2(float y, float x)
{
#if !defined(FILTER_PLATFORM_STM32F407_NO_HW) && defined(ARM_MATH_ATAN2)
    /* CMSIS-DSP v1.10+ 提供 arm_atan2_f32 */
    return arm_atan2_f32(y, x);
#else
    /* 回退 libm atan2f（arm_atan2_f32 不是所有 CMSIS-DSP 版本都有） */
    return atan2f(y, x);
#endif
}

float fp_asin(float x)
{
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

/* ═══════════════════════════════════════════════════════════════
 * 时序钩子
 * ═══════════════════════════════════════════════════════════════ */

void fp_init_timing(void)
{
#if !defined(FILTER_PLATFORM_STM32F407_NO_HW)
    /* 使能 DWT 周期计数器（调试模式需 CK_DEBUGEN=1） */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#else
    /* PC 桩：无 DWT */
#endif
}

uint32_t fp_get_cycles(void)
{
#if !defined(FILTER_PLATFORM_STM32F407_NO_HW)
    return DWT->CYCCNT;
#else
    return 0u;
#endif
}

float fp_get_dt(void)
{
    /* stm32f407 用 DWT 周期计差 / SystemCoreClock 在 BSP 中计算，
     * 此函数仅作回退（BSP 应优先用 fp_get_cycles） */
    return 0.01f;
}

void fp_delay_ms(uint32_t ms)
{
#if !defined(FILTER_PLATFORM_STM32F407_NO_HW)
    HAL_Delay(ms);
#else
    (void)ms;
#endif
}

/* ═══════════════════════════════════════════════════════════════
 * 内存钩子
 * ═══════════════════════════════════════════════════════════════ */

void* fp_malloc(size_t n)
{
    (void)n;
    return NULL;  /* STATIC_ONLY 模式 */
}

void fp_free(void *p)
{
    (void)p;
    /* STATIC_ONLY 模式 */
}

/* ═══════════════════════════════════════════════════════════════
 * 矩阵钩子 — CMSIS-DSP 加速
 * ═══════════════════════════════════════════════════════════════ */

void fp_mat_mult(float *dst, const float *a, const float *b,
                 int ar, int ac, int bc)
{
#if !defined(FILTER_PLATFORM_STM32F407_NO_HW)
    arm_matrix_instance_f32 ma = {ar, ac, (float*)a};
    arm_matrix_instance_f32 mb = {ac, bc, (float*)b};
    arm_matrix_instance_f32 md = {ar, bc, dst};
    arm_mat_mult_f32(&ma, &mb, &md);
#else
    /* 朴素 C 实现（PC 回退） */
    for (int i = 0; i < ar; i++) {
        for (int j = 0; j < bc; j++) {
            float sum = 0.0f;
            for (int k = 0; k < ac; k++) {
                sum += a[i * ac + k] * b[k * bc + j];
            }
            dst[i * bc + j] = sum;
        }
    }
#endif
}

int fp_mat_inverse_3x3(float *dst, const float *src)
{
    /* 3×3 解析求逆（CMSIS-DSP 无 3×3 直接求逆函数） */
    float a = src[0], b = src[1], c = src[2];
    float d = src[3], e = src[4], f = src[5];
    float g = src[6], h = src[7], k = src[8];
    float det = a * (e * k - f * h) - b * (d * k - f * g) + c * (d * h - e * g);
    if (fabsf(det) < 1e-10f) return -1;
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
