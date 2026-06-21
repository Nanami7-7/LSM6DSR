/**
 * @file    test_platform_smoke.c
 * @brief   filter_platform.h 钩子函数烟雾测试
 *
 * 验证 fp_* 默认实现（filter_platform_default.c）的数值合同：
 *   - fp_sqrt:  非负，x<0 返回 0
 *   - fp_atan2: atan2(1,0) = π/2，atan2(0,0) = 0
 *   - fp_asin:  钳位到 [-π/2, π/2]，|x|>1 不返回 NaN
 *   - fp_sin:   sin(0) = 0
 *   - fp_cos:   cos(0) = 1
 *   - fp_fabs:  正确取绝对值
 *   - fp_isnan: NaN 检测
 *   - fp_isinf: Inf 检测
 *
 * 编译：
 *   gcc -o test_platform_smoke test_platform_smoke.c \
 *       ../Core/Filter/Platform/filter_platform_default.c \
 *       -I../Core/Filter/Inc -lm -Wall -Wextra
 */

#include <stdio.h>
#include <math.h>
#include "filter_platform.h"
#include "filter_math.h"

static int tests_total = 0;
static int tests_passed = 0;

static void test_assert(int cond, const char *msg)
{
    tests_total++;
    if (cond) {
        tests_passed++;
        printf("  [PASS] %s\n", msg);
    } else {
        printf("  [FAIL] %s\n", msg);
    }
}

static int approx_eq(float a, float b, float eps)
{
    return fabsf(a - b) < eps;
}

int main(void)
{
    printf("\n=== filter_platform 烟雾测试 ===\n\n");

    /* ---- fp_sqrt ---- */
    printf("[fp_sqrt]\n");
    test_assert(approx_eq(fp_sqrt(4.0f), 2.0f, 1e-6f), "fp_sqrt(4) = 2");
    test_assert(approx_eq(fp_sqrt(0.0f), 0.0f, 1e-6f), "fp_sqrt(0) = 0");
    test_assert(fp_sqrt(-1.0f) == 0.0f, "fp_sqrt(-1) = 0 (不返回 NaN)");

    /* ---- fp_atan2 ---- */
    printf("\n[fp_atan2]\n");
    test_assert(approx_eq(fp_atan2(1.0f, 0.0f), M_PI_F / 2.0f, 1e-6f),
                "fp_atan2(1,0) = π/2");
    test_assert(approx_eq(fp_atan2(0.0f, 0.0f), 0.0f, 1e-6f),
                "fp_atan2(0,0) = 0");
    test_assert(approx_eq(fp_atan2(1.0f, 1.0f), M_PI_F / 4.0f, 1e-6f),
                "fp_atan2(1,1) = π/4");

    /* ---- fp_asin ---- */
    printf("\n[fp_asin]\n");
    test_assert(approx_eq(fp_asin(0.0f), 0.0f, 1e-6f), "fp_asin(0) = 0");
    test_assert(approx_eq(fp_asin(1.0f), M_PI_F / 2.0f, 1e-6f), "fp_asin(1) = π/2");
    test_assert(approx_eq(fp_asin(-1.0f), -M_PI_F / 2.0f, 1e-6f), "fp_asin(-1) = -π/2");
    test_assert(!fp_isnan(fp_asin(1.5f)), "fp_asin(1.5) 钳位不返回 NaN");
    test_assert(!fp_isnan(fp_asin(-1.5f)), "fp_asin(-1.5) 钳位不返回 NaN");

    /* ---- fp_sin / fp_cos ---- */
    printf("\n[fp_sin/fp_cos]\n");
    test_assert(approx_eq(fp_sin(0.0f), 0.0f, 1e-6f), "fp_sin(0) = 0");
    test_assert(approx_eq(fp_cos(0.0f), 1.0f, 1e-6f), "fp_cos(0) = 1");
    test_assert(approx_eq(fp_sin(M_PI_F / 2.0f), 1.0f, 1e-6f), "fp_sin(π/2) = 1");
    test_assert(approx_eq(fp_cos(M_PI_F), -1.0f, 1e-5f), "fp_cos(π) = -1");

    /* ---- fp_fabs ---- */
    printf("\n[fp_fabs]\n");
    test_assert(approx_eq(fp_fabs(-3.5f), 3.5f, 1e-6f), "fp_fabs(-3.5) = 3.5");
    test_assert(approx_eq(fp_fabs(3.5f), 3.5f, 1e-6f), "fp_fabs(3.5) = 3.5");
    test_assert(approx_eq(fp_fabs(0.0f), 0.0f, 1e-6f), "fp_fabs(0) = 0");

    /* ---- fp_isnan / fp_isinf ---- */
    printf("\n[fp_isnan/fp_isinf]\n");
    test_assert(fp_isnan(0.0f / 0.0f) == 1, "fp_isnan(NaN) = 1");
    test_assert(fp_isnan(1.0f) == 0, "fp_isnan(1.0) = 0");
    test_assert(fp_isinf(1.0f / 0.0f) == 1, "fp_isinf(Inf) = 1");
    test_assert(fp_isinf(1.0f) == 0, "fp_isinf(1.0) = 0");

    /* ---- 时序钩子（默认实现） ---- */
    printf("\n[时序钩子默认实现]\n");
    fp_init_timing();
    test_assert(fp_get_cycles() == 0u, "fp_get_cycles 默认返回 0");
    test_assert(approx_eq(fp_get_dt(), 0.01f, 1e-6f), "fp_get_dt 默认返回 0.01f");
    fp_delay_ms(1);  /* 不应阻塞 */
    test_assert(1, "fp_delay_ms 默认空操作");

    /* ---- 矩阵钩子 ---- */
    printf("\n[矩阵钩子]\n");
    {
        /* 2×3 × 3×2 = 2×2 */
        float a[6] = {1, 2, 3, 4, 5, 6};   /* [[1,2,3],[4,5,6]] */
        float b[6] = {7, 8, 9, 10, 11, 12}; /* [[7,8],[9,10],[11,12]] */
        float dst[4];
        fp_mat_mult(dst, a, b, 2, 3, 2);
        /* 期望：[[1*7+2*9+3*11, 1*8+2*10+3*12], [4*7+5*9+6*11, 4*8+5*10+6*12]]
         *      = [[58, 64], [139, 154]] */
        test_assert(approx_eq(dst[0], 58.0f, 1e-4f) && approx_eq(dst[1], 64.0f, 1e-4f) &&
                    approx_eq(dst[2], 139.0f, 1e-4f) && approx_eq(dst[3], 154.0f, 1e-4f),
                    "fp_mat_mult 2×3 × 3×2 正确");
    }
    {
        /* 3×3 求逆：单位阵的逆 = 单位阵 */
        float I[9] = {1,0,0, 0,1,0, 0,0,1};
        float inv[9];
        int ret = fp_mat_inverse_3x3(inv, I);
        test_assert(ret == 0, "fp_mat_inverse_3x3(单位阵) 返回 0");
        test_assert(approx_eq(inv[0], 1.0f, 1e-4f) && approx_eq(inv[4], 1.0f, 1e-4f) &&
                    approx_eq(inv[8], 1.0f, 1e-4f),
                    "fp_mat_inverse_3x3(单位阵) = 单位阵");
    }

    /* ---- 内存钩子（FILTER_STATIC_ONLY 模式） ---- */
    printf("\n[内存钩子 FILTER_STATIC_ONLY]\n");
    test_assert(fp_malloc(100) == NULL, "fp_malloc 在 STATIC_ONLY 下返回 NULL");
    fp_free(NULL);  /* 不应崩溃 */
    test_assert(1, "fp_free 空操作");

    /* ---- 总结 ---- */
    printf("\n========================================\n");
    printf("  总测试数: %d\n", tests_total);
    printf("  通过: %d\n", tests_passed);
    printf("  失败: %d\n", tests_total - tests_passed);
    printf("  通过率: %.1f%%\n", 100.0f * tests_passed / tests_total);
    printf("========================================\n");
    return (tests_passed == tests_total) ? 0 : 1;
}
