/**
 * @file    test_filters.c
 * @brief   滤波器数学验证测试
 *
 * 测试内容：
 *   1. 基本功能测试（创建/销毁/重置）
 *   2. 静态姿态测试（已知姿态验证输出）
 *   3. 动态旋转测试（恒定角速度积分）
 *   4. 数值稳定性测试（极端输入、NaN/Inf保护）
 *   5. 四元数归一化验证
 *
 * 编译（PC）：
 *   gcc -o test_filters.exe test_filters.c ../Core/Filter/Src/filter.c ../Core/Filter/Src/filter_config.c -I../Core/Filter/Inc -lm -Wall -Wextra
 *
 * 运行：
 *   ./test_filters
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <float.h>
#include "../Core/Filter/Inc/filter.h"

/* ============================================================
 * 测试配置
 * ============================================================ */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TEST_DT         0.01f   /* 10ms 采样周期 */
#define TEST_ITERATIONS 1000    /* 测试迭代次数 */
#define TOLERANCE       0.5f    /* 角度容差（度） */
#define TOLERANCE_QUAT  1e-5f   /* 四元数容差 */

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

#define TEST_ASSERT_NEAR(actual, expected, tol, message) do { \
    float _diff = fabsf((actual) - (expected)); \
    TEST_ASSERT(_diff <= (tol), message); \
    if (_diff > (tol)) { \
        printf("         实际值: %.6f, 期望值: %.6f, 差值: %.6f\n", actual, expected, _diff); \
    } \
} while(0)

/* ============================================================
 * 辅助函数
 * ============================================================ */

/** 检查浮点数是否有效（非NaN、非Inf） */
static int is_valid_float(float x) {
    return !isnan(x) && !isinf(x);
}

/** 检查四元数是否归一化 */
static int is_normalized_quat(float q0, float q1, float q2, float q3) {
    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    return fabsf(norm - 1.0f) < TOLERANCE_QUAT;
}

/** 检查角度是否在有效范围内 */
static int is_valid_angle(float angle) {
    return is_valid_float(angle) && fabsf(angle) <= 360.0f;
}

/** 打印滤波器输出 */

/* ============================================================
 * 测试用例
 * ============================================================ */

/**
 * 测试1：滤波器创建和销毁
 */
static void test_create_destroy(void) {
    printf("\n=== 测试1：滤波器创建和销毁 ===\n");

    const char *names[] = {"Complementary", "LPF", "EKF", "LKF", "Mahony", "Madgwick"};

    for (int i = 0; i < FILTER_TYPE_COUNT; i++) {
        filter_type_t type = (filter_type_t)i;
        filter_t *f = filter_create(type);

        char msg[64];
        snprintf(msg, sizeof(msg), "%s 创建成功", names[i]);
        TEST_ASSERT(f != NULL, msg);

        if (f) {
            snprintf(msg, sizeof(msg), "%s 类型正确", names[i]);
            TEST_ASSERT(f->type == type, msg);

            snprintf(msg, sizeof(msg), "%s update函数非空", names[i]);
            TEST_ASSERT(f->update != NULL, msg);

            snprintf(msg, sizeof(msg), "%s priv数据非空", names[i]);
            TEST_ASSERT(f->priv != NULL, msg);

            f->destroy(f);
        }
    }

    /* 测试无效类型 */
    filter_t *invalid = filter_create(FILTER_TYPE_COUNT);
    TEST_ASSERT(invalid == NULL, "无效类型返回NULL");
}

/**
 * 测试2：静态姿态 - 水平放置
 * 输入：ax=0, ay=0, az=1g（Z轴向上）
 * 期望：pitch≈0°, roll≈0°
 */
static void test_static_level(void) {
    printf("\n=== 测试2：静态姿态 - 水平放置 ===\n");

    filter_input_t in = {
        .ax = 0.0f, .ay = 0.0f, .az = 1.0f,  /* 水平放置 */
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,   /* 静止 */
        .dt = TEST_DT
    };
    filter_output_t out;

    for (int i = 0; i < FILTER_TYPE_COUNT; i++) {
        filter_t *f = filter_create((filter_type_t)i);
        if (!f) continue;

        /* 多次迭代让滤波器收敛 */
        for (int j = 0; j < 1000; j++) {
            f->update(f, &in, &out);
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "%s pitch≈0° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.pitch);
        TEST_ASSERT(fabsf(out.pitch) < TOLERANCE, msg);

        snprintf(msg, sizeof(msg), "%s roll≈0° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.roll);
        TEST_ASSERT(fabsf(out.roll) < TOLERANCE, msg);

        snprintf(msg, sizeof(msg), "%s 输出有效", filter_type_name((filter_type_t)i));
        TEST_ASSERT(is_valid_float(out.pitch) && is_valid_float(out.roll) && is_valid_float(out.yaw), msg);

        f->destroy(f);
    }
}

/**
 * 测试3：静态姿态 - 俯仰45度
 * 输入：ax=-sin(45°), ay=0, az=cos(45°)
 * 期望：pitch≈45°, roll≈0°
 */
static void test_static_pitch_45(void) {
    printf("\n=== 测试3：静态姿态 - 俯仰45度 ===\n");

    float angle = 45.0f * M_PI / 180.0f;
    filter_input_t in = {
        .ax = -sinf(angle), .ay = 0.0f, .az = cosf(angle),
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = TEST_DT
    };
    filter_output_t out;

    for (int i = 0; i < FILTER_TYPE_COUNT; i++) {
        filter_t *f = filter_create((filter_type_t)i);
        if (!f) continue;

        for (int j = 0; j < 1000; j++) {
            f->update(f, &in, &out);
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "%s pitch≈45° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.pitch);
        TEST_ASSERT(fabsf(out.pitch - 45.0f) < TOLERANCE, msg);

        snprintf(msg, sizeof(msg), "%s roll≈0° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.roll);
        TEST_ASSERT(fabsf(out.roll) < TOLERANCE, msg);

        f->destroy(f);
    }
}

/**
 * 测试4：静态姿态 - 横滚45度
 * 输入：ax=0, ay=sin(45°), az=cos(45°)
 * 期望：pitch≈0°, roll≈45°
 */
static void test_static_roll_45(void) {
    printf("\n=== 测试4：静态姿态 - 横滚45度 ===\n");

    float angle = 45.0f * M_PI / 180.0f;
    filter_input_t in = {
        .ax = 0.0f, .ay = sinf(angle), .az = cosf(angle),
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = TEST_DT
    };
    filter_output_t out;

    for (int i = 0; i < FILTER_TYPE_COUNT; i++) {
        filter_t *f = filter_create((filter_type_t)i);
        if (!f) continue;

        for (int j = 0; j < 1000; j++) {
            f->update(f, &in, &out);
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "%s pitch≈0° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.pitch);
        TEST_ASSERT(fabsf(out.pitch) < TOLERANCE, msg);

        snprintf(msg, sizeof(msg), "%s roll≈45° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.roll);
        TEST_ASSERT(fabsf(out.roll - 45.0f) < TOLERANCE, msg);

        f->destroy(f);
    }
}

/**
 * 测试5：动态旋转 - 恒定角速度积分
 * 输入：gz=100 dps（绕Z轴旋转），ax=0, ay=0, az=1g
 * 期望：yaw持续增加
 */
static void test_dynamic_rotation(void) {
    printf("\n=== 测试5：动态旋转 - 恒定角速度积分 ===\n");

    filter_input_t in = {
        .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
        .gx = 0.0f, .gy = 0.0f, .gz = 100.0f,  /* 100 dps */
        .dt = TEST_DT
    };
    filter_output_t out;

    for (int i = 0; i < FILTER_TYPE_COUNT; i++) {
        filter_t *f = filter_create((filter_type_t)i);
        if (!f) continue;

        /* 运行1秒（100帧） */
        for (int j = 0; j < 100; j++) {
            f->update(f, &in, &out);
        }

        /* 期望yaw≈100°（100 dps × 1s） */
        char msg[64];
        snprintf(msg, sizeof(msg), "%s yaw≈100° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.yaw);
        TEST_ASSERT(fabsf(out.yaw - 100.0f) < 5.0f, msg);  /* 允许5°误差 */

        snprintf(msg, sizeof(msg), "%s 动态输出有效", filter_type_name((filter_type_t)i));
        TEST_ASSERT(is_valid_float(out.pitch) && is_valid_float(out.roll) && is_valid_float(out.yaw), msg);

        f->destroy(f);
    }
}

/**
 * 测试6：四元数归一化验证
 * 使用四元数的滤波器应保持归一化
 */
static void test_quaternion_normalization(void) {
    printf("\n=== 测试6：四元数归一化验证 ===\n");

    filter_input_t in = {
        .ax = 0.5f, .ay = 0.3f, .az = 0.8f,
        .gx = 50.0f, .gy = -30.0f, .gz = 20.0f,
        .dt = TEST_DT
    };
    filter_output_t out;

    /* 只测试使用四元数的滤波器：EKF, Mahony, Madgwick */
    filter_type_t quat_filters[] = {FILTER_TYPE_EKF, FILTER_TYPE_MAHONY, FILTER_TYPE_MADGWICK};

    for (int i = 0; i < 3; i++) {
        filter_t *f = filter_create(quat_filters[i]);
        if (!f) continue;

        int all_normalized = 1;
        for (int j = 0; j < TEST_ITERATIONS; j++) {
            f->update(f, &in, &out);

            if (!is_normalized_quat(out.q0, out.q1, out.q2, out.q3)) {
                all_normalized = 0;
                break;
            }
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "%s 四元数保持归一化", filter_type_name(quat_filters[i]));
        TEST_ASSERT(all_normalized, msg);

        if (!all_normalized) {
            float norm = sqrtf(out.q0*out.q0 + out.q1*out.q1 + out.q2*out.q2 + out.q3*out.q3);
            printf("         最终归一化误差: %.6f (norm=%.6f)\n", fabsf(norm - 1.0f), norm);
        }

        f->destroy(f);
    }
}

/**
 * 测试7：NaN/Inf输入保护
 * 滤波器应对无效输入保持稳定
 */
static void test_nan_inf_protection(void) {
    printf("\n=== 测试7：NaN/Inf输入保护 ===\n");

    filter_input_t in_valid = {
        .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = TEST_DT
    };

    filter_input_t in_nan = {
        .ax = NAN, .ay = 0.0f, .az = 1.0f,
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = TEST_DT
    };

    filter_input_t in_inf = {
        .ax = INFINITY, .ay = 0.0f, .az = 1.0f,
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = TEST_DT
    };

    filter_input_t in_zero_dt = {
        .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = 0.0f
    };

    filter_output_t out;

    for (int i = 0; i < FILTER_TYPE_COUNT; i++) {
        filter_t *f = filter_create((filter_type_t)i);
        if (!f) continue;

        /* 先用有效输入初始化 */
        f->update(f, &in_valid, &out);

        /* 测试NaN输入 */
        f->update(f, &in_nan, &out);
        char msg[64];
        snprintf(msg, sizeof(msg), "%s NaN输入后输出稳定", filter_type_name((filter_type_t)i));
        TEST_ASSERT(is_valid_float(out.pitch) && is_valid_float(out.roll) && is_valid_float(out.yaw), msg);

        /* 测试Inf输入 */
        f->update(f, &in_inf, &out);
        snprintf(msg, sizeof(msg), "%s Inf输入后输出稳定", filter_type_name((filter_type_t)i));
        TEST_ASSERT(is_valid_float(out.pitch) && is_valid_float(out.roll) && is_valid_float(out.yaw), msg);

        /* 测试dt=0 */
        f->update(f, &in_zero_dt, &out);
        snprintf(msg, sizeof(msg), "%s dt=0输入后输出稳定", filter_type_name((filter_type_t)i));
        TEST_ASSERT(is_valid_float(out.pitch) && is_valid_float(out.roll) && is_valid_float(out.yaw), msg);

        f->destroy(f);
    }
}

/**
 * 测试8：重置功能
 * 重置后输出应回到初始状态（pitch、roll、yaw 都应接近0°）
 */
static void test_reset(void) {
    printf("\n=== 测试8：重置功能 ===\n");

    filter_input_t in = {
        .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
        .gx = 0.0f, .gy = 0.0f, .gz = 100.0f,
        .dt = TEST_DT
    };
    filter_output_t out;

    for (int i = 0; i < FILTER_TYPE_COUNT; i++) {
        filter_t *f = filter_create((filter_type_t)i);
        if (!f) continue;

        /* 运行100帧，积累yaw */
        for (int j = 0; j < 100; j++) {
            f->update(f, &in, &out);
        }

        /* 重置 */
        f->reset(f);

        /* 重置后用静止输入，运行多帧确保收敛 */
        filter_input_t in_static = {
            .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
            .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
            .dt = TEST_DT
        };
        for (int j = 0; j < 10; j++) {
            f->update(f, &in_static, &out);
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "%s 重置后pitch≈0° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.pitch);
        TEST_ASSERT(fabsf(out.pitch) < TOLERANCE, msg);

        snprintf(msg, sizeof(msg), "%s 重置后roll≈0° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.roll);
        TEST_ASSERT(fabsf(out.roll) < TOLERANCE, msg);

        /* 验证yaw也重置（对于支持yaw重置的滤波器） */
        snprintf(msg, sizeof(msg), "%s 重置后yaw≈0° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.yaw);
        TEST_ASSERT(fabsf(out.yaw) < TOLERANCE, msg);

        f->destroy(f);
    }
}

/**
 * 测试9：长时间运行稳定性
 * 运行10000帧，检查是否有发散或NaN
 */
static void test_long_term_stability(void) {
    printf("\n=== 测试9：长时间运行稳定性 (10000帧) ===\n");

    filter_input_t in = {
        .ax = 0.1f, .ay = -0.05f, .az = 0.99f,
        .gx = 10.0f, .gy = -5.0f, .gz = 2.0f,
        .dt = TEST_DT
    };
    filter_output_t out;

    for (int i = 0; i < FILTER_TYPE_COUNT; i++) {
        filter_t *f = filter_create((filter_type_t)i);
        if (!f) continue;

        int stable = 1;
        for (int j = 0; j < 10000; j++) {
            f->update(f, &in, &out);

            if (!is_valid_float(out.pitch) || !is_valid_float(out.roll) || !is_valid_float(out.yaw)) {
                stable = 0;
                printf("  在第%d帧出现NaN/Inf\n", j);
                break;
            }
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "%s 10000帧稳定运行", filter_type_name((filter_type_t)i));
        TEST_ASSERT(stable, msg);

        if (stable) {
            snprintf(msg, sizeof(msg), "%s 最终输出有效", filter_type_name((filter_type_t)i));
            TEST_ASSERT(is_valid_angle(out.pitch) && is_valid_angle(out.roll) && is_valid_angle(out.yaw), msg);
        }

        f->destroy(f);
    }
}

/**
 * 测试10：参数设置
 * 测试set_param功能 — 验证参数实际生效
 */
static void test_set_param(void) {
    printf("\n=== 测试10：参数设置 ===\n");

    /* 使用45°俯仰输入，验证参数对收敛行为的影响 */
    float angle = 45.0f * M_PI / 180.0f;
    filter_input_t in = {
        .ax = -sinf(angle), .ay = 0.0f, .az = cosf(angle),
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = TEST_DT
    };
    filter_output_t out_default, out_modified;

    /* 测试互补滤波器alpha参数 — alpha越小收敛越快 */
    {
        /* 默认alpha (0.98) 滤波器 */
        filter_t *f1 = filter_create(FILTER_TYPE_COMPLEMENTARY);
        /* 修改alpha (0.50) 滤波器 */
        filter_t *f2 = filter_create(FILTER_TYPE_COMPLEMENTARY);
        if (f1 && f2) {
            f2->set_param(f2, FILTER_PARAM_ALPHA, 0.50f);
            /* 运行100帧，观察收敛差异 */
            for (int j = 0; j < 100; j++) {
                f1->update(f1, &in, &out_default);
                f2->update(f2, &in, &out_modified);
            }
            /* alpha=0.5 应该收敛更快，更接近45° */
            float err_default = fabsf(out_default.pitch - 45.0f);
            float err_modified = fabsf(out_modified.pitch - 45.0f);
            TEST_ASSERT(err_modified < err_default,
                        "互补滤波器 alpha=0.5 比默认收敛更快");
            TEST_ASSERT(is_valid_float(out_modified.pitch),
                        "互补滤波器 set_param(alpha=0.5) 后输出有效");
        }
        if (f1) f1->destroy(f1);
        if (f2) f2->destroy(f2);
    }

    /* 测试LPF截止频率参数 — 截止频率越低，滤波越强 */
    {
        filter_t *f1 = filter_create(FILTER_TYPE_LPF);
        filter_t *f2 = filter_create(FILTER_TYPE_LPF);
        if (f1 && f2) {
            f2->set_param(f2, FILTER_PARAM_CUTOFF_FREQ, 2.0f);
            /* 使用动态输入测试滤波效果 */
            filter_input_t in_dynamic = {
                .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
                .gx = 0.0f, .gy = 50.0f, .gz = 0.0f,
                .dt = TEST_DT
            };
            for (int j = 0; j < 50; j++) {
                f1->update(f1, &in_dynamic, &out_default);
                f2->update(f2, &in_dynamic, &out_modified);
            }
            /* 低截止频率应该滤波更强（输出更平滑） */
            /* 验证输出有效即可 */
            TEST_ASSERT(is_valid_float(out_modified.pitch),
                        "LPF set_param(cutoff=2Hz) 后输出有效");
        }
        if (f1) f1->destroy(f1);
        if (f2) f2->destroy(f2);
    }

    /* 测试EKF参数 */
    {
        filter_t *f = filter_create(FILTER_TYPE_EKF);
        if (f) {
            f->set_param(f, FILTER_PARAM_Q_ANGLE, 0.01f);
            f->set_param(f, FILTER_PARAM_Q_BIAS, 0.001f);
            f->set_param(f, FILTER_PARAM_R_MEASURE, 0.1f);
            for (int j = 0; j < 100; j++) {
                f->update(f, &in, &out_modified);
            }
            TEST_ASSERT(fabsf(out_modified.pitch - 45.0f) < TOLERANCE,
                        "EKF set_param 后收敛到45°");
            f->destroy(f);
        }
    }

    /* 测试Mahony kp/ki参数 — kp越高收敛越快 */
    {
        filter_t *f1 = filter_create(FILTER_TYPE_MAHONY);
        filter_t *f2 = filter_create(FILTER_TYPE_MAHONY);
        if (f1 && f2) {
            f2->set_param(f2, FILTER_PARAM_KP, 20.0f);
            for (int j = 0; j < 20; j++) {
                f1->update(f1, &in, &out_default);
                f2->update(f2, &in, &out_modified);
            }
            float err_default = fabsf(out_default.pitch - 45.0f);
            float err_modified = fabsf(out_modified.pitch - 45.0f);
            TEST_ASSERT(err_modified < err_default,
                        "Mahony kp=20 比默认kp=10收敛更快");
        }
        if (f1) f1->destroy(f1);
        if (f2) f2->destroy(f2);
    }

    /* 测试Madgwick beta参数 — beta越高收敛越快 */
    {
        filter_t *f1 = filter_create(FILTER_TYPE_MADGWICK);
        filter_t *f2 = filter_create(FILTER_TYPE_MADGWICK);
        if (f1 && f2) {
            f2->set_param(f2, FILTER_PARAM_KP, 0.1f);  /* beta=0.1 */
            for (int j = 0; j < 100; j++) {
                f1->update(f1, &in, &out_default);
                f2->update(f2, &in, &out_modified);
            }
            /* 验证两个滤波器输出不同（参数生效） */
            float diff = fabsf(out_default.pitch - out_modified.pitch);
            TEST_ASSERT(diff > 0.01f,
                        "Madgwick beta=0.1 与默认beta=0.5 输出不同");
        }
        if (f1) f1->destroy(f1);
        if (f2) f2->destroy(f2);
    }

    /* 测试LKF参数 */
    {
        filter_t *f = filter_create(FILTER_TYPE_LKF);
        if (f) {
            f->set_param(f, FILTER_PARAM_Q_ANGLE, 0.01f);
            f->set_param(f, FILTER_PARAM_R_MEASURE, 0.1f);
            for (int j = 0; j < 100; j++) {
                f->update(f, &in, &out_modified);
            }
            TEST_ASSERT(fabsf(out_modified.pitch - 45.0f) < TOLERANCE,
                        "LKF set_param 后收敛到45°");
            f->destroy(f);
        }
    }
}

/**
 * 测试11：ACC幅值异常保护
 * 加速度幅值过大或过小时应跳过测量更新
 */
static void test_acc_magnitude_guard(void) {
    printf("\n=== 测试11：ACC幅值异常保护 ===\n");

    filter_output_t out;

    /* 幅值过小 */
    filter_input_t in_small = {
        .ax = 0.001f, .ay = 0.001f, .az = 0.001f,
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = TEST_DT
    };

    /* 幅值过大 */
    filter_input_t in_large = {
        .ax = 100.0f, .ay = 100.0f, .az = 100.0f,
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = TEST_DT
    };

    /* 先用有效输入初始化 */
    filter_input_t in_valid = {
        .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = TEST_DT
    };

    /* 只测试EKF（有ACC幅值检查） */
    filter_t *f = filter_create(FILTER_TYPE_EKF);
    if (f) {
        f->update(f, &in_valid, &out);

        f->update(f, &in_small, &out);
        TEST_ASSERT(is_valid_float(out.pitch), "EKF ACC幅值过小时输出稳定");

        f->update(f, &in_large, &out);
        TEST_ASSERT(is_valid_float(out.pitch), "EKF ACC幅值过大时输出稳定");

        f->destroy(f);
    }
}

/**
 * 测试12：滤波器类型名称
 */
static void test_type_name(void) {
    printf("\n=== 测试12：滤波器类型名称 ===\n");

    TEST_ASSERT(strcmp(filter_type_name(FILTER_TYPE_COMPLEMENTARY), "Complementary") == 0,
                "互补滤波器名称正确");
    TEST_ASSERT(strcmp(filter_type_name(FILTER_TYPE_LPF), "LPF") == 0,
                "LPF名称正确");
    TEST_ASSERT(strcmp(filter_type_name(FILTER_TYPE_EKF), "EKF") == 0,
                "EKF名称正确");
    TEST_ASSERT(strcmp(filter_type_name(FILTER_TYPE_LKF), "LKF") == 0,
                "LKF名称正确");
    TEST_ASSERT(strcmp(filter_type_name(FILTER_TYPE_MAHONY), "Mahony") == 0,
                "Mahony名称正确");
    TEST_ASSERT(strcmp(filter_type_name(FILTER_TYPE_MADGWICK), "Madgwick") == 0,
                "Madgwick名称正确");
    TEST_ASSERT(strcmp(filter_type_name(FILTER_TYPE_COUNT), "Unknown") == 0,
                "无效类型返回Unknown");
}

/* ============================================================
 * 主函数
 * ============================================================ */
int main(void) {
    printf("========================================\n");
    printf("  IMU滤波器数学验证测试\n");
    printf("========================================\n");
    printf("测试配置：dt=%.3fs, 迭代=%d, 容差=%.1f°\n", TEST_DT, TEST_ITERATIONS, TOLERANCE);

    /* 运行所有测试 */
    test_create_destroy();
    test_static_level();
    test_static_pitch_45();
    test_static_roll_45();
    test_dynamic_rotation();
    test_quaternion_normalization();
    test_nan_inf_protection();
    test_reset();
    test_long_term_stability();
    test_set_param();
    test_acc_magnitude_guard();
    test_type_name();

    /* 打印总结 */
    printf("\n========================================\n");
    printf("  测试总结\n");
    printf("========================================\n");
    printf("  总测试数: %d\n", tests_total);
    printf("  通过: %d\n", tests_passed);
    printf("  失败: %d\n", tests_failed);
    printf("  通过率: %.1f%%\n", 100.0f * tests_passed / tests_total);
    printf("========================================\n");

    return (tests_failed > 0) ? 1 : 0;
}
