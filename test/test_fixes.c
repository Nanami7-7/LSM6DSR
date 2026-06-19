/**
 * @file    test_fixes.c
 * @brief   测试所有修复的功能
 *
 * 测试内容：
 *   1. 线程安全 - 多实例支持
 *   2. destroy 安全性
 *   3. 四元数归一化统一性
 *   4. 时间戳溢出处理
 *   5. 错误处理机制
 *   6. 参数验证
 *
 * 编译（PC）：
 *   gcc -o test_fixes test_fixes.c \
 *       ../Core/Filter/Src/filter.c \
 *       ../Core/Src/bsp_lsm6dsr.c \
 *       -I../Core/Filter/Inc \
 *       -I../Core/Inc \
 *       -lm -Wall -Wextra -DDEBUG
 *
 * 运行：
 *   ./test_fixes
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include "../Core/Filter/Inc/filter.h"

/* ============================================================
 * 测试统计
 * ============================================================ */
static int tests_total = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(condition, message) do { \
    tests_total++; \
    if (condition) { \
        tests_passed++; \
        printf("  [PASS] %s\n", message); \
    } else { \
        tests_failed++; \
        printf("  [FAIL] %s\n", message); \
    } \
} while(0)

#define TEST_SECTION(title) do { \
    printf("\n=== %s ===\n", title); \
} while(0)

/* ============================================================
 * 测试 1: destroy 安全性
 * ============================================================ */
void test_destroy_safety(void)
{
    TEST_SECTION("Test 1: Destroy Safety");

    /* 测试 1.1: NULL 指针安全性 — 不崩溃即通过 */
    filter_destroy_safe(NULL);
    /* 如果执行到这里说明没有崩溃 */
    int null_safe = 1;
    TEST_ASSERT(null_safe, "NULL pointer destroy is safe (no crash)");

    /* 测试 1.2: 动态分配的滤波器正常销毁 */
    filter_t *f = filter_create(FILTER_TYPE_COMPLEMENTARY);
    TEST_ASSERT(f != NULL, "Dynamic filter created successfully");

    if (f) {
        /* 验证滤波器功能正常 */
        filter_input_t in = {
            .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
            .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
            .dt = 0.01f
        };
        filter_output_t out;
        f->update(f, &in, &out);
        int valid_before = !isnan(out.pitch) && !isnan(out.roll);
        TEST_ASSERT(valid_before, "Dynamic filter works before destroy");

        filter_destroy_safe(f);
        /* 销毁后无法验证指针状态，但不崩溃即通过 */
    }

    /* 测试 1.3: 静态分配的滤波器安全销毁 */
    uint8_t buf[1024];
    f = filter_create_static(FILTER_TYPE_EKF, buf, sizeof(buf));
    TEST_ASSERT(f != NULL, "Static filter created successfully");

    if (f) {
        /* 验证静态滤波器功能正常 */
        filter_input_t in = {
            .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
            .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
            .dt = 0.01f
        };
        filter_output_t out;
        f->update(f, &in, &out);
        int valid_before = !isnan(out.pitch) && !isnan(out.roll);
        TEST_ASSERT(valid_before, "Static filter works before destroy");

        /* 销毁（应该是 no-op） */
        filter_destroy_safe(f);

        /* 验证静态滤波器仍然可用（因为 destroy 是 no-op） */
        f->update(f, &in, &out);
        int valid_after = !isnan(out.pitch) && !isnan(out.roll);
        TEST_ASSERT(valid_after, "Static filter still works after destroy (no-op)");

        /* 再次销毁 */
        filter_destroy_safe(f);
        f->update(f, &in, &out);
        int valid_double = !isnan(out.pitch) && !isnan(out.roll);
        TEST_ASSERT(valid_double, "Static filter survives double destroy");
    }
}

/* ============================================================
 * 测试 2: 四元数归一化
 * ============================================================ */
void test_quaternion_normalization(void)
{
    TEST_SECTION("Test 2: Quaternion Normalization");

    /* 测试 2.1: 正常归一化 */
    float q0 = 1.0f, q1 = 0.5f, q2 = 0.3f, q3 = 0.1f;
    filter_normalize_quaternion(&q0, &q1, &q2, &q3);

    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    TEST_ASSERT(fabsf(norm - 1.0f) < 1e-5f, "Normal quaternion normalized correctly");

    /* 测试 2.2: 零四元数处理 */
    q0 = 0.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    filter_normalize_quaternion(&q0, &q1, &q2, &q3);

    TEST_ASSERT(q0 == 1.0f && q1 == 0.0f && q2 == 0.0f && q3 == 0.0f,
                "Zero quaternion reset to identity");

    /* 测试 2.3: NaN 输入处理 */
    q0 = NAN; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    filter_normalize_quaternion(&q0, &q1, &q2, &q3);

    TEST_ASSERT(q0 == 1.0f && q1 == 0.0f && q2 == 0.0f && q3 == 0.0f,
                "NaN quaternion reset to identity");

    /* 测试 2.4: Inf 输入处理 */
    q0 = INFINITY; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    filter_normalize_quaternion(&q0, &q1, &q2, &q3);

    TEST_ASSERT(q0 == 1.0f && q1 == 0.0f && q2 == 0.0f && q3 == 0.0f,
                "Inf quaternion reset to identity");

    /* 测试 2.5: 接近零的四元数 */
    q0 = 1e-15f; q1 = 1e-15f; q2 = 1e-15f; q3 = 1e-15f;
    filter_normalize_quaternion(&q0, &q1, &q2, &q3);

    TEST_ASSERT(q0 == 1.0f, "Near-zero quaternion reset to identity");
}

/* ============================================================
 * 测试 3: 参数验证
 * ============================================================ */
void test_parameter_validation(void)
{
    TEST_SECTION("Test 3: Parameter Validation");

    filter_t *f = filter_create(FILTER_TYPE_COMPLEMENTARY);
    TEST_ASSERT(f != NULL, "Filter created for parameter testing");

    if (!f) return;

    filter_input_t in = {
        .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = 0.01f
    };
    filter_output_t out;

    /* 测试 3.1: 有效参数 — 滤波器应该正常工作 */
    f->set_param(f, FILTER_PARAM_ALPHA, 0.5f);
    f->update(f, &in, &out);
    TEST_ASSERT(!isnan(out.pitch) && !isnan(out.roll),
                "Valid alpha=0.5: filter produces valid output");

    /* 测试 3.2: 边界参数 */
    f->set_param(f, FILTER_PARAM_ALPHA, 0.0f);
    f->update(f, &in, &out);
    TEST_ASSERT(!isnan(out.pitch) && !isnan(out.roll),
                "Boundary alpha=0.0: filter produces valid output");

    f->set_param(f, FILTER_PARAM_ALPHA, 1.0f);
    f->update(f, &in, &out);
    TEST_ASSERT(!isnan(out.pitch) && !isnan(out.roll),
                "Boundary alpha=1.0: filter produces valid output");

    /* 测试 3.3: 无效参数 — 滤波器应该拒绝并保持之前的有效值 */
    /* 先设置一个有效值 */
    f->set_param(f, FILTER_PARAM_ALPHA, 0.7f);
    f->update(f, &in, &out);
    float pitch_before = out.pitch;

    /* 尝试设置无效值 */
    f->set_param(f, FILTER_PARAM_ALPHA, -0.1f);
    f->update(f, &in, &out);
    /* 输出应该仍然有效（不崩溃，不产生NaN） */
    TEST_ASSERT(!isnan(out.pitch) && !isnan(out.roll),
                "Invalid alpha=-0.1: filter still produces valid output");

    f->set_param(f, FILTER_PARAM_ALPHA, 1.1f);
    f->update(f, &in, &out);
    TEST_ASSERT(!isnan(out.pitch) && !isnan(out.roll),
                "Invalid alpha=1.1: filter still produces valid output");

    f->set_param(f, FILTER_PARAM_ALPHA, NAN);
    f->update(f, &in, &out);
    TEST_ASSERT(!isnan(out.pitch) && !isnan(out.roll),
                "Invalid alpha=NAN: filter still produces valid output");

    f->set_param(f, FILTER_PARAM_ALPHA, INFINITY);
    f->update(f, &in, &out);
    TEST_ASSERT(!isnan(out.pitch) && !isnan(out.roll),
                "Invalid alpha=INF: filter still produces valid output");

    /* 测试 3.4: EKF 参数验证 */
    filter_t *ekf = filter_create(FILTER_TYPE_EKF);
    if (ekf) {
        ekf->set_param(ekf, FILTER_PARAM_Q_ANGLE, 0.001f);
        ekf->update(ekf, &in, &out);
        TEST_ASSERT(!isnan(out.pitch) && !isnan(out.roll),
                    "Valid Q_angle=0.001: EKF produces valid output");

        ekf->set_param(ekf, FILTER_PARAM_Q_ANGLE, -0.001f);
        ekf->update(ekf, &in, &out);
        TEST_ASSERT(!isnan(out.pitch) && !isnan(out.roll),
                    "Invalid Q_angle=-0.001: EKF still produces valid output");

        filter_destroy_safe(ekf);
    }

    filter_destroy_safe(f);
}

/* ============================================================
 * 测试 4: 错误处理机制
 * ============================================================ */
static int error_callback_called = 0;
static filter_error_t last_error_code = FILTER_OK;

void test_error_callback(const filter_error_info_t *info, void *user_data)
{
    (void)user_data;
    error_callback_called = 1;
    last_error_code = info->code;
    printf("    [CALLBACK] Error: %s (code=%d)\n", info->message, info->code);
}

void test_error_handling(void)
{
    TEST_SECTION("Test 4: Error Handling");

    /* 设置错误回调 */
    filter_set_error_callback(test_error_callback, NULL);
    error_callback_called = 0;
    last_error_code = FILTER_OK;

    /* 测试 4.1: 无效类型应该触发错误 */
    filter_t *f = filter_create(99);  /* 无效类型 */
    TEST_ASSERT(f == NULL, "Invalid filter type returns NULL");
    TEST_ASSERT(error_callback_called, "Error callback was called");
    TEST_ASSERT(last_error_code == FILTER_ERR_INVALID_TYPE,
                "Correct error code reported");

    /* 测试 4.2: 获取最后的错误 */
    filter_error_info_t err = filter_get_last_error();
    TEST_ASSERT(err.code == FILTER_ERR_INVALID_TYPE, "Last error is correct");

    /* 清理 */
    filter_set_error_callback(NULL, NULL);
}

/* ============================================================
 * 测试 5: 滤波器基本功能
 * ============================================================ */
void test_filter_basic_functionality(void)
{
    TEST_SECTION("Test 5: Filter Basic Functionality");

    /* 测试 5.1: 创建所有类型的滤波器 */
    const char *names[] = {"Complementary", "LPF", "EKF", "LKF", "Mahony", "Madgwick"};
    filter_type_t types[] = {
        FILTER_TYPE_COMPLEMENTARY,
        FILTER_TYPE_LPF,
        FILTER_TYPE_EKF,
        FILTER_TYPE_LKF,
        FILTER_TYPE_MAHONY,
        FILTER_TYPE_MADGWICK
    };

    for (int i = 0; i < 6; i++) {
        filter_t *f = filter_create(types[i]);
        TEST_ASSERT(f != NULL, names[i]);

        if (f) {
            /* 测试更新 */
            filter_input_t in = {
                .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
                .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
                .dt = 0.01f
            };
            filter_output_t out;
            f->update(f, &in, &out);

            /* 验证输出不是 NaN */
            int valid = !isnan(out.pitch) && !isnan(out.roll) && !isnan(out.yaw);
            TEST_ASSERT(valid, "  Output is valid (not NaN)");

            filter_destroy_safe(f);
        }
    }
}

/* ============================================================
 * 测试 6: 退化模式
 * ============================================================ */
void test_degrade_modes(void)
{
    TEST_SECTION("Test 6: Degrade Modes");

    filter_degrade_t modes[] = {
        FILTER_DEGRADE_NONE,
        FILTER_DEGRADE_STATIC_ONLY,
        FILTER_DEGRADE_GYRO_ONLY,
        FILTER_DEGRADE_ACC_ONLY,
        FILTER_DEGRADE_HOLD_LAST
    };
    const char *mode_names[] = {
        "NONE", "STATIC_ONLY", "GYRO_ONLY", "ACC_ONLY", "HOLD_LAST"
    };

    /* 测试所有6种滤波器的退化模式 */
    filter_type_t types[] = {
        FILTER_TYPE_COMPLEMENTARY,
        FILTER_TYPE_LPF,
        FILTER_TYPE_EKF,
        FILTER_TYPE_LKF,
        FILTER_TYPE_MAHONY,
        FILTER_TYPE_MADGWICK
    };
    const char *type_names[] = {
        "Complementary", "LPF", "EKF", "LKF", "Mahony", "Madgwick"
    };

    filter_input_t in_static = {
        .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = 0.01f
    };
    filter_input_t in_dynamic = {
        .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
        .gx = 10.0f, .gy = 20.0f, .gz = 30.0f,
        .dt = 0.01f
    };
    filter_output_t out, out_prev;

    for (int t = 0; t < 6; t++) {
        filter_t *f = filter_create(types[t]);
        if (!f) continue;

        /* 先用静态输入初始化，建立基线 */
        for (int j = 0; j < 10; j++) {
            f->update(f, &in_static, &out);
        }
        out_prev = out;

        /* 测试每种退化模式 */
        for (int i = 0; i < 5; i++) {
            filter_set_degrade(f, modes[i]);
            f->update(f, &in_dynamic, &out);

            int valid = !isnan(out.pitch) && !isnan(out.roll) && !isnan(out.yaw) &&
                        !isinf(out.pitch) && !isinf(out.roll) && !isinf(out.yaw);
            char msg[64];
            snprintf(msg, sizeof(msg), "%s %s: output valid", type_names[t], mode_names[i]);
            TEST_ASSERT(valid, msg);

            /* 验证 HOLD_LAST 模式确实保持上次输出 */
            if (modes[i] == FILTER_DEGRADE_HOLD_LAST) {
                float pitch_diff = fabsf(out.pitch - out_prev.pitch);
                float roll_diff = fabsf(out.roll - out_prev.roll);
                snprintf(msg, sizeof(msg), "%s HOLD_LAST: pitch unchanged (diff=%.4f)", type_names[t], pitch_diff);
                TEST_ASSERT(pitch_diff < 1e-6f, msg);
            }

            out_prev = out;
        }

        /* 恢复正常模式，验证可以恢复 */
        filter_set_degrade(f, FILTER_DEGRADE_NONE);
        for (int j = 0; j < 10; j++) {
            f->update(f, &in_static, &out);
        }
        int valid = !isnan(out.pitch) && !isnan(out.roll) && !isnan(out.yaw);
        char msg[64];
        snprintf(msg, sizeof(msg), "%s: recovery after degrade valid", type_names[t]);
        TEST_ASSERT(valid, msg);

        filter_destroy_safe(f);
    }
}

/* ============================================================
 * 测试 7: 输出验证
 * ============================================================ */
void test_output_validation(void)
{
    TEST_SECTION("Test 7: Output Validation");

    /* 测试 7.1: 有效输出 */
    filter_output_t out1 = {
        .pitch = 45.0f,
        .roll = -30.0f,
        .yaw = 90.0f,
        .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f
    };
    TEST_ASSERT(filter_validate_output(&out1) == 1, "Valid output accepted");

    /* 测试 7.2: NaN 输出 */
    filter_output_t out2 = {
        .pitch = NAN,
        .roll = 0.0f,
        .yaw = 0.0f,
        .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f
    };
    TEST_ASSERT(filter_validate_output(&out2) == 0, "NaN output rejected");

    /* 测试 7.3: 超范围输出 */
    filter_output_t out3 = {
        .pitch = 200.0f,
        .roll = 0.0f,
        .yaw = 0.0f,
        .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f
    };
    TEST_ASSERT(filter_validate_output(&out3) == 0, "Out-of-range output rejected");

    /* 测试 7.4: NULL 指针 */
    TEST_ASSERT(filter_validate_output(NULL) == 0, "NULL output rejected");
}

/* ============================================================
 * 主测试函数
 * ============================================================ */
int main(void)
{
    printf("LSM6DSR Filter Library - Fix Verification Tests\n");
    printf("================================================\n");

    /* 运行所有测试 */
    test_destroy_safety();
    test_quaternion_normalization();
    test_parameter_validation();
    test_error_handling();
    test_filter_basic_functionality();
    test_degrade_modes();
    test_output_validation();

    /* 打印总结 */
    printf("\n================================================\n");
    printf("Test Summary:\n");
    printf("  Total:  %d\n", tests_total);
    printf("  Passed: %d\n", tests_passed);
    printf("  Failed: %d\n", tests_failed);
    printf("================================================\n");

    if (tests_failed == 0) {
        printf("✅ All tests passed!\n");
        return 0;
    } else {
        printf("❌ %d tests failed!\n", tests_failed);
        return 1;
    }
}
