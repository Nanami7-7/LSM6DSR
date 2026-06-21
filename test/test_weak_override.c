/**
 * @file    test_weak_override.c
 * @brief   FILTER_WEAK 覆盖机制验证
 *
 * 定义非 weak 的 complementary_update，写哨兵值 42.0f 到 out->pitch。
 * 验证 filter_create_static(COMPLEMENTARY) → update 后 out->pitch == 42.0f，
 * 证明链接器优先选了非 weak 版本（而非 filter_complementary.c 的 weak 版本）。
 */

#include <stdio.h>
#include <string.h>
#include "filter.h"

/* 静态缓冲区 */
static uint8_t filter_buf[512] __attribute__((aligned(4)));

/* 非 weak complementary_update — 覆盖 filter_complementary.c 的 weak 版本 */
void complementary_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    /* 哨兵值验证 */
    out->pitch = 42.0f;
    out->roll  = 42.0f;
    out->yaw   = 42.0f;
    out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
    /* 其他覆盖函数（非 weak）不重新定义，链接器用 weak 版本 */
}

static int passed = 0;
static int total = 0;

#define TEST_ASSERT(cond, msg) do { \
    total++; \
    if (cond) { passed++; printf("  [PASS] %s\n", msg); } \
    else { printf("  [FAIL] %s\n", msg); } \
} while(0)

int main(void)
{
    printf("\n=== FILTER_WEAK 覆盖测试 ===\n\n");

    /* 创建 Complementary（工厂表指向 complementary_update，链接器应选非 weak 版本） */
    memset(filter_buf, 0, sizeof(filter_buf));
    filter_t *f = filter_create_static(FILTER_TYPE_COMPLEMENTARY, filter_buf, sizeof(filter_buf));
    TEST_ASSERT(f != NULL, "Complementary filter_create 成功");

    /* 调用 update，应被覆盖的 complementary_update 处理，写哨兵值 */
    filter_input_t in = { .ax=0,.ay=0,.az=1.0f,.gx=0,.gy=0,.gz=0,.dt=0.01f };
    filter_output_t out;
    f->update(f, &in, &out);

    TEST_ASSERT(out.pitch == 42.0f, "pitch 哨兵值 = 42.0 (覆盖生效)");
    TEST_ASSERT(out.roll  == 42.0f, "roll  哨兵值 = 42.0 (覆盖生效)");
    TEST_ASSERT(out.yaw   == 42.0f, "yaw   哨兵值 = 42.0 (覆盖生效)");

    /* 不覆盖 lpf_update → 仍走 weak 版本（正常行为） */
    memset(filter_buf, 0, sizeof(filter_buf));
    f = filter_create_static(FILTER_TYPE_LPF, filter_buf, sizeof(filter_buf));
    TEST_ASSERT(f != NULL, "LPF filter_create 成功");
    f->update(f, &in, &out);
    TEST_ASSERT(out.pitch != 42.0f, "LPF 未被覆盖（走 weak 版本）");

    printf("\n========================================\n");
    printf("  总测试数: %d  通过: %d  失败: %d\n", total, passed, total - passed);
    return (passed == total) ? 0 : 1;
}
