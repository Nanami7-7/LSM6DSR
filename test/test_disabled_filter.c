/**
 * @file    test_disabled_filter.c
 * @brief   编译期滤波器禁用测试
 *
 * 编译：
 *   gcc -DFILTER_DISABLE_EKF -DFILTER_DISABLE_LKF -DFILTER_STATIC_ONLY \
 *       -o test_disabled test_disabled_filter.c \
 *       ../Core/Filter/Src/filter_factory.c ../Core/Filter/Src/filter_common.c \
 *       ../Core/Filter/Src/filter_complementary.c ../Core/Filter/Src/filter_lpf.c \
 *       ../Core/Filter/Src/filter_mahony.c ../Core/Filter/Src/filter_madgwick.c \
 *       ../Core/Filter/Src/filter_config.c \
 *       ../Core/Filter/Platform/filter_platform_default.c \
 *       -I../Core/Filter/Inc -lm -Wall -Wextra
 */

#include <stdio.h>
#include <string.h>
#include "filter.h"
#include "filter_math.h"

#define FILTER_BUF_SIZE 512
static uint8_t filter_buf[FILTER_BUF_SIZE] __attribute__((aligned(4)));

static int passed = 0;
static int total = 0;

#define TEST_ASSERT(cond, msg) do { \
    total++; \
    if (cond) { passed++; printf("  [PASS] %s\n", msg); } \
    else { printf("  [FAIL] %s\n", msg); } \
} while(0)

int main(void)
{
    printf("\n=== 滤波器禁用测试 (FILTER_DISABLE_EKF + FILTER_DISABLE_LKF) ===\n\n");

    /* EKF 禁用后创建返回 NULL */
    TEST_ASSERT(filter_get_static_size(FILTER_TYPE_EKF) == 0,
                "EKF get_static_size = 0 (已禁用)");
    TEST_ASSERT(filter_create_static(FILTER_TYPE_EKF, filter_buf, FILTER_BUF_SIZE) == NULL,
                "EKF filter_create_static = NULL (已禁用)");

    /* LKF 禁用后创建返回 NULL */
    TEST_ASSERT(filter_get_static_size(FILTER_TYPE_LKF) == 0,
                "LKF get_static_size = 0 (已禁用)");
    TEST_ASSERT(filter_create_static(FILTER_TYPE_LKF, filter_buf, FILTER_BUF_SIZE) == NULL,
                "LKF filter_create_static = NULL (已禁用)");

    /* 未禁用的互补滤波器仍可用 */
    memset(filter_buf, 0, FILTER_BUF_SIZE);
    filter_t *f = filter_create_static(FILTER_TYPE_COMPLEMENTARY, filter_buf, FILTER_BUF_SIZE);
    TEST_ASSERT(f != NULL, "Complementary 创建成功（未禁用）");
    TEST_ASSERT(f->update != NULL, "Complementary update 非空");

    /* 运行一帧 */
    filter_input_t in = { .ax=0,.ay=0,.az=1.0f,.gx=0,.gy=0,.gz=0,.dt=0.01f };
    filter_output_t out;
    f->update(f, &in, &out);
    TEST_ASSERT(out.pitch < 0.5f && out.pitch > -0.5f,
                "Complementary 静止输出有效");

    /* Mahony 未禁用 */
    memset(filter_buf, 0, FILTER_BUF_SIZE);
    f = filter_create_static(FILTER_TYPE_MAHONY, filter_buf, FILTER_BUF_SIZE);
    TEST_ASSERT(f != NULL, "Mahony 创建成功（未禁用）");

    /* FILTER_TYPE_COUNT 不变（禁用不改变枚举大小） */
    TEST_ASSERT(FILTER_TYPE_COUNT == 6, "FILTER_TYPE_COUNT = 6 (禁用不改变枚举)");

    printf("\n========================================\n");
    printf("  总测试数: %d  通过: %d  失败: %d\n", total, passed, total - passed);
    return (passed == total) ? 0 : 1;
}
