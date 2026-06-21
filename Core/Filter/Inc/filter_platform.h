/**
 * @file    filter_platform.h
 * @brief   滤波器平台抽象层 — 钩子契约（frozen on branches）
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 设计动机                                                      │
 * └──────────────────────────────────────────────────────────────┘
 *   原 6 个滤波器直接调用 libm（atan2f/sqrtf 等），导致：
 *     - 无法在 STM32F407 切换到更快的 CMSIS-DSP（arm_sqrt_f32 等）
 *     - 无法在 MSPM0G3507（M0+ 无 FPU）切换到定点/多项式逼近
 *     - BSP 层硬编码 DWT/HAL_Delay，阻塞非 ARM 平台
 *
 *   本头文件定义一组 fp_* 钩子，滤波器只调 fp_*，具体后端由分支选择：
 *     - master 默认：filter_platform_default.c（libm 包装，可移植）
 *     - stm32f407：  opt_stm32f407/platform_stm32f407.c（CMSIS-DSP + DWT）
 *     - mspm0g3507： opt_mspm0/platform_mspm0.c（多项式逼近 + SysTick）
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 冻结契约（分支不得修改本文件）                                │
 * └──────────────────────────────────────────────────────────────┘
 *   - 所有 fp_* 函数签名固定
 *   - 所有 FILTER_USE_* / FILTER_DISABLE_* 宏名固定
 *   - 新增钩子只能 append，不得修改现有签名
 *
 *   分支提供 fp_* 实现的方式：
 *     1. 编译期排除 filter_platform_default.c
 *     2. 在 opt_<mcu>/platform_<mcu>.c 提供同名 fp_* 定义
 *     3. 链接器优先选分支实现（filter_platform_default.c 中的 fp_* 标 weak，
 *        Phase 3 完成）
 */

#ifndef FILTER_PLATFORM_H
#define FILTER_PLATFORM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 后端选择宏（编译期由分支 -D 指定）
 * ============================================================
 *
 * 优先级（高→低）：
 *   FILTER_USE_CMSIS_DSP    STM32F4/F7/H7 + CMSIS-DSP 库
 *   FILTER_USE_FIXED_POINT  M0+ 无 FPU，Q16.16 定点
 *   FILTER_USE_FAST_MATH    通用快速多项式逼近（精度略损）
 *   （未定义）               libm 默认实现
 *
 * FILTER_STATIC_ONLY       默认定义，禁用 malloc/fp_malloc（MCU 推荐）
 * FILTER_ALLOW_DYNAMIC     允许 filter_create 动态分配（仅 PC 测试）
 *
 * FILTER_DISABLE_<TYPE>    禁用某滤波器（见 Phase 3）
 */

#ifndef FILTER_STATIC_ONLY
#define FILTER_STATIC_ONLY 1
#endif

/* ============================================================
 * 滤波器按需禁用（编译期 -D 控制）
 * ============================================================
 *
 * 定义 FILTER_DISABLE_<TYPE> 后，对应滤波器从工厂表中移除：
 *   - filter_get_static_size(<TYPE>) 返回 0
 *   - filter_create_static(<TYPE>, ...) 返回 NULL
 *   - 滤波器 .c 文件仍编译（weak 符号存在），仅工厂不注册
 *
 * 典型用法（M0+ 小 RAM MCU 禁用重滤波器）：
 *   gcc -DFILTER_DISABLE_EKF -DFILTER_DISABLE_LKF ...
 */

/* 以下宏默认未定义（= 启用）。定义后禁用对应滤波器。 */

/* #define FILTER_DISABLE_COMPLEMENTARY  -- 已注释，启用 */
/* #define FILTER_DISABLE_LPF            -- 已注释，启用 */
/* #define FILTER_DISABLE_EKF            -- 已注释，启用 */
/* #define FILTER_DISABLE_LKF            -- 已注释，启用 */
/* #define FILTER_DISABLE_MAHONY         -- 已注释，启用 */
/* #define FILTER_DISABLE_MADGWICK       -- 已注释，启用 */

/* ============================================================
 * 覆盖机制选择
 * ============================================================
 *
 * FILTER_WEAK: 在滤波器函数前标注，使分支可提供非 weak 覆盖
 *   默认 __attribute__((weak))，适用于 GCC/Clang/ArmClang v6
 *   ARMCC v5 用 __weak（需分支在 platform_<mcu>.h 重新定义）
 *
 * FILTER_OVERRIDE_MECHANISM:
 *   "weak"         — weak 符号（默认，推荐）
 *   "include_swap" — 分支替换整个 .c 文件（工具链不支持 weak 时回退）
 */

#ifndef FILTER_OVERRIDE_MECHANISM
#define FILTER_OVERRIDE_MECHANISM "weak"
#endif

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION < 6000000)
  /* ARMCC v5 使用 __weak */
  #define FILTER_WEAK __weak
#elif defined(__MINGW32__) || defined(__MINGW64__)
  /* MinGW PE 格式不支持函数 weak 符号（与 ELF 不同）
   * PC 测试不需要覆盖，用普通强符号即可。
   * 分支覆盖通过 include_swap 机制（替换整个 .c 文件）实现。 */
  #define FILTER_WEAK
#else
  /* GCC / Clang / ArmClang v6+ 在 ELF/COFF 上支持 */
  #define FILTER_WEAK __attribute__((weak))
#endif

/* ============================================================
 * 动态分配开关（仅 PC 测试，MCU 永不启用）
 * ============================================================
 *
 * FILTER_ALLOW_DYNAMIC: 启用 filter_create(type) 动态分配包装
 *   必须显式 -D 定义（默认不启用），MCU 分支永不定义
 *   启用后 filter_dynamic.c 被编译，其中 filter_create 调用 fp_malloc
 */

/* #define FILTER_ALLOW_DYNAMIC  -- 已注释，仅 PC 测试用 */

/* ============================================================
 * 数学钩子
 * ============================================================
 *
 * 滤波器内所有数学运算必须通过 fp_* 调用，禁止直接调 libm。
 * 默认实现见 filter_platform_default.c，分支可在 platform_<mcu>.c 覆盖。
 *
 * 语义合同（所有后端必须保证）：
 *   - fp_sqrt(x<0) 返回 0（不返回 NaN，避免污染滤波器状态）
 *   - fp_atan2(0,0) 返回 0
 *   - fp_asin(|x|>1) 钳位到 ±π/2（不返回 NaN）
 *   - fp_isnan / fp_isinf 返回 int（0/1）
 */

float   fp_sqrt(float x);
float   fp_atan2(float y, float x);
float   fp_asin(float x);
float   fp_sin(float x);
float   fp_cos(float x);
float   fp_fabs(float x);
int     fp_isnan(float x);
int     fp_isinf(float x);

/* ============================================================
 * 时序钩子（BSP 层用）
 * ============================================================
 *
 * fp_init_timing：  初始化高精度计时器（如 DWT CYCCNT），仅初始化一次
 * fp_get_cycles：   返回单调递增周期计数值（精度 ~1/SystemCoreClock 秒）
 *                   平台无周期计数器时返回 0（BSP 应回退到 fp_get_dt）
 * fp_get_dt：       返回距上次调用的时间差（秒）
 *                   默认实现返回固定 0.01f（100Hz 假设）
 * fp_delay_ms：     阻塞延时（毫秒），用于 BSP 启动/校准
 */

void    fp_init_timing(void);
uint32_t fp_get_cycles(void);
float   fp_get_dt(void);
void    fp_delay_ms(uint32_t ms);

/* ============================================================
 * 内存钩子（PC 测试用，MCU 静态分配时不调用）
 * ============================================================
 *
 * FILTER_STATIC_ONLY 定义时，fp_malloc 返回 NULL，fp_free 空操作。
 * 仅 FILTER_ALLOW_DYNAMIC 时 filter_dynamic.c 会调 fp_malloc。
 */

void*   fp_malloc(size_t n);
void    fp_free(void *p);

/* ============================================================
 * 矩阵钩子（EKF/LKF 可选加速，Phase 5 CMSIS-DSP override 用）
 * ============================================================
 *
 * 当前 EKF/LKF 用手写三重循环，不调这些钩子。
 * Phase 5 stm32f407 分支可重写 ekf_update 用 fp_mat_mult 调 arm_mat_mult_f32。
 *
 * 行主序：a 是 ar×ac 矩阵，b 是 ac×bc，dst 是 ar×bc
 */

void    fp_mat_mult(float *dst, const float *a, const float *b,
                    int ar, int ac, int bc);
int     fp_mat_inverse_3x3(float *dst, const float *src);  /* 返回 0=成功, -1=奇异 */

#ifdef __cplusplus
}
#endif

#endif /* FILTER_PLATFORM_H */
