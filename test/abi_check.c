/**
 * @file    abi_check.c
 * @brief   ABI 完整性检查 — 验证分支未修改冻结接口
 *
 * 编译：
 *   gcc -o abi_check abi_check.c \
 *       ../Core/Filter/Platform/filter_platform_default.c \
 *       -I../Core/Filter/Inc -lm
 *
 * 运行：
 *   ./abi_check && echo "ABI OK" || echo "ABI BROKEN"
 *
 * 分支必须此测试通过 → Phase 6 sync 脚本阻断不合规分支
 */

#include <stdio.h>
/* 关键：abi_expected.h 必须在 filter.h 之前 include，
 * 使 filter.h 内的 _Static_assert 校验 sizeof(filter_t) */
#include "abi_expected.h"
#include "filter.h"

static int ok = 1;
static int total = 0;

#define CHECK(cond, msg) do { \
    total++; \
    if (!(cond)) { printf("  [ABI BREAK] %s\n", msg); ok = 0; } \
} while(0)

int main(void)
{
    printf("\n=== ABI 完整性检查 ===\n\n");

    /* sizeof 检查 */
    CHECK(sizeof(filter_t) == FILTER_ABI_SIZEOF_FILTER_T,
          "sizeof(filter_t) 匹配");
    CHECK(sizeof(filter_input_t) == FILTER_ABI_SIZEOF_FILTER_INPUT_T,
          "sizeof(filter_input_t) 匹配");
    CHECK(sizeof(filter_output_t) == FILTER_ABI_SIZEOF_FILTER_OUTPUT_T,
          "sizeof(filter_output_t) 匹配");
    CHECK(sizeof(filter_safety_config_t) == FILTER_ABI_SIZEOF_FILTER_SAFETY_CONFIG_T,
          "sizeof(filter_safety_config_t) 匹配");

    /* offsetof 检查 */
    CHECK(offsetof(filter_t, update) == FILTER_ABI_OFF_UPDATE,
          "offsetof(update) 匹配");
    CHECK(offsetof(filter_t, reset) == FILTER_ABI_OFF_RESET,
          "offsetof(reset) 匹配");
    CHECK(offsetof(filter_t, set_param) == FILTER_ABI_OFF_SET_PARAM,
          "offsetof(set_param) 匹配");
    CHECK(offsetof(filter_t, type) == FILTER_ABI_OFF_TYPE,
          "offsetof(type) 匹配");
    CHECK(offsetof(filter_t, degrade) == FILTER_ABI_OFF_DEGRADE,
          "offsetof(degrade) 匹配");
    CHECK(offsetof(filter_t, priv) == FILTER_ABI_OFF_PRIV,
          "offsetof(priv) 匹配");
    CHECK(offsetof(filter_t, safety_config) == FILTER_ABI_OFF_SAFETY_CONFIG,
          "offsetof(safety_config) 匹配");

    /* 枚举值检查 */
    CHECK(FILTER_TYPE_COUNT == FILTER_ABI_FILTER_TYPE_COUNT,
          "FILTER_TYPE_COUNT 匹配");
    CHECK(FILTER_DEGRADE_COUNT == FILTER_ABI_FILTER_DEGRADE_COUNT,
          "FILTER_DEGRADE_COUNT 匹配");
    CHECK(FILTER_PARAM_COUNT == FILTER_ABI_FILTER_PARAM_COUNT,
          "FILTER_PARAM_COUNT 匹配");

    /* API 版本检查 */
    CHECK(FILTER_API_VERSION == FILTER_ABI_API_VERSION,
          "FILTER_API_VERSION 匹配");

    /* 函数签名存在性由 filter.h 声明保证，
     * 编译错误 = 签名漂移，无需运行时检查。 */

    printf("\n========================================\n");
    printf("  ABI integrity: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
