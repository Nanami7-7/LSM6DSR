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
 *   gcc -o test_filters test_filters.c \
 *       ../Core/Filter/Src/filter_factory.c \
 *       ../Core/Filter/Src/filter_common.c \
 *       ../Core/Filter/Src/filter_complementary.c \
 *       ../Core/Filter/Src/filter_lpf.c \
 *       ../Core/Filter/Src/filter_ekf.c \
 *       ../Core/Filter/Src/filter_lkf.c \
 *       ../Core/Filter/Src/filter_mahony.c \
 *       ../Core/Filter/Src/filter_madgwick.c \
 *       ../Core/Filter/Src/filter_config.c \
 *       -I../Core/Filter/Inc -lm -Wall -Wextra
 *
 * 运行：
 *   ./test_filters
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <float.h>
#include "../Core/Filter/Inc/filter.h"

/* 静态分配辅助宏 */
#define FILTER_BUF_SIZE 512
static uint8_t filter_buf[FILTER_BUF_SIZE] __attribute__((aligned(4)));

static filter_t* test_create_filter(filter_type_t type) {
    memset(filter_buf, 0, FILTER_BUF_SIZE);
    return filter_create_static(type, filter_buf, FILTER_BUF_SIZE);
}

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
        filter_t *f = test_create_filter(type);

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

        }
    }

    /* 测试无效类型 */
    filter_t *invalid = test_create_filter(FILTER_TYPE_COUNT);
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
        filter_t *f = test_create_filter((filter_type_t)i);
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
        filter_t *f = test_create_filter((filter_type_t)i);
        if (!f) continue;

        for (int j = 0; j < 1000; j++) {
            f->update(f, &in, &out);
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "%s pitch≈45° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.pitch);
        TEST_ASSERT(fabsf(out.pitch - 45.0f) < TOLERANCE, msg);

        snprintf(msg, sizeof(msg), "%s roll≈0° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.roll);
        TEST_ASSERT(fabsf(out.roll) < TOLERANCE, msg);

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
        filter_t *f = test_create_filter((filter_type_t)i);
        if (!f) continue;

        for (int j = 0; j < 1000; j++) {
            f->update(f, &in, &out);
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "%s pitch≈0° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.pitch);
        TEST_ASSERT(fabsf(out.pitch) < TOLERANCE, msg);

        snprintf(msg, sizeof(msg), "%s roll≈45° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.roll);
        TEST_ASSERT(fabsf(out.roll - 45.0f) < TOLERANCE, msg);

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
        filter_t *f = test_create_filter((filter_type_t)i);
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
        filter_t *f = test_create_filter(quat_filters[i]);
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
        filter_t *f = test_create_filter((filter_type_t)i);
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

    }
}

/**
 * 测试8：重置功能
 * 重置后输出应回到初始状态
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
        filter_t *f = test_create_filter((filter_type_t)i);
        if (!f) continue;

        /* 运行一段时间 */
        for (int j = 0; j < 100; j++) {
            f->update(f, &in, &out);
        }

        /* 重置 */
        f->reset(f);

        /* 重置后用静止输入 */
        filter_input_t in_static = {
            .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
            .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
            .dt = TEST_DT
        };
        f->update(f, &in_static, &out);

        char msg[64];
        snprintf(msg, sizeof(msg), "%s 重置后pitch≈0° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.pitch);
        TEST_ASSERT(fabsf(out.pitch) < TOLERANCE, msg);

        snprintf(msg, sizeof(msg), "%s 重置后roll≈0° (实际=%.2f°)", filter_type_name((filter_type_t)i), out.roll);
        TEST_ASSERT(fabsf(out.roll) < TOLERANCE, msg);

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
        filter_t *f = test_create_filter((filter_type_t)i);
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

    }
}

/**
 * 测试10：参数设置
 * 测试set_param功能
 */
static void test_set_param(void) {
    printf("\n=== 测试10：参数设置 ===\n");

    filter_output_t out;
    filter_input_t in = {
        .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = TEST_DT
    };

    /* 测试互补滤波器alpha参数 */
    {
        filter_t *f = test_create_filter(FILTER_TYPE_COMPLEMENTARY);
        if (f) {
            f->set_param(f, FILTER_PARAM_ALPHA, 0.5f);
            f->update(f, &in, &out);
            TEST_ASSERT(is_valid_float(out.pitch), "互补滤波器 set_param(alpha=0.5) 后输出有效");
        }
    }

    /* 测试LPF截止频率参数 */
    {
        filter_t *f = test_create_filter(FILTER_TYPE_LPF);
        if (f) {
            f->set_param(f, FILTER_PARAM_CUTOFF_FREQ, 5.0f);
            f->update(f, &in, &out);
            TEST_ASSERT(is_valid_float(out.pitch), "LPF set_param(cutoff=5Hz) 后输出有效");
        }
    }

    /* 测试EKF参数 */
    {
        filter_t *f = test_create_filter(FILTER_TYPE_EKF);
        if (f) {
            f->set_param(f, FILTER_PARAM_Q_ANGLE, 0.01f);
            f->set_param(f, FILTER_PARAM_Q_BIAS, 0.001f);
            f->set_param(f, FILTER_PARAM_R_MEASURE, 0.1f);
            f->update(f, &in, &out);
            TEST_ASSERT(is_valid_float(out.pitch), "EKF set_param 后输出有效");
        }
    }

    /* 测试Mahony kp/ki参数 */
    {
        filter_t *f = test_create_filter(FILTER_TYPE_MAHONY);
        if (f) {
            f->set_param(f, FILTER_PARAM_KP, 10.0f);
            f->set_param(f, FILTER_PARAM_KI, 0.1f);
            f->update(f, &in, &out);
            TEST_ASSERT(is_valid_float(out.pitch), "Mahony set_param(kp=10, ki=0.1) 后输出有效");
        }
    }

    /* 测试Madgwick beta参数 */
    {
        filter_t *f = test_create_filter(FILTER_TYPE_MADGWICK);
        if (f) {
            f->set_param(f, FILTER_PARAM_KP, 0.5f);  /* beta映射到KP */
            f->update(f, &in, &out);
            TEST_ASSERT(is_valid_float(out.pitch), "Madgwick set_param(beta=0.5) 后输出有效");
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
    filter_t *f = test_create_filter(FILTER_TYPE_EKF);
    if (f) {
        f->update(f, &in_valid, &out);

        f->update(f, &in_small, &out);
        TEST_ASSERT(is_valid_float(out.pitch), "EKF ACC幅值过小时输出稳定");

        f->update(f, &in_large, &out);
        TEST_ASSERT(is_valid_float(out.pitch), "EKF ACC幅值过大时输出稳定");

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
    TEST_ASSERT(strcmp(filter_type_name(FILTER_TYPE_MAHONY), "Mahony") == 0,
                "Mahony名称正确");
    TEST_ASSERT(strcmp(filter_type_name(FILTER_TYPE_MADGWICK), "Madgwick") == 0,
                "Madgwick名称正确");
    TEST_ASSERT(strcmp(filter_type_name(FILTER_TYPE_COUNT), "Unknown") == 0,
                "无效类型返回Unknown");
}

static void test_safety_apis(void) {
    printf("\n=== 测试13：安全保护API ===\n");

    /* 测试 filter_validate_output */
    filter_output_t valid_out = { .pitch = 10.0f, .roll = 20.0f, .yaw = 30.0f,
                                  .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f };
    TEST_ASSERT(filter_validate_output(&valid_out) == 1, "有效输出通过验证");

    filter_output_t nan_out = { .pitch = NAN, .roll = 0.0f, .yaw = 0.0f,
                                .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f };
    TEST_ASSERT(filter_validate_output(&nan_out) == 0, "NaN输出被拒绝");

    filter_output_t inf_out = { .pitch = INFINITY, .roll = 0.0f, .yaw = 0.0f,
                                .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f };
    TEST_ASSERT(filter_validate_output(&inf_out) == 0, "Inf输出被拒绝");

    /* 测试 filter_normalize_quaternion */
    float q0 = 2.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    filter_normalize_quaternion(&q0, &q1, &q2, &q3);
    TEST_ASSERT(fabsf(q0 - 1.0f) < TOLERANCE_QUAT, "四元数归一化: q0=1.0");
    TEST_ASSERT(fabsf(q1) < TOLERANCE_QUAT, "四元数归一化: q1=0.0");

    /* 测试 filter_check_acc_quality */
    TEST_ASSERT(filter_check_acc_quality(0.0f, 0.0f, 1.0f) == 1, "正常ACC幅值(1g)");
    TEST_ASSERT(filter_check_acc_quality(0.0f, 0.0f, 0.1f) == 0, "异常ACC幅值(0.1g)");
    TEST_ASSERT(filter_check_acc_quality(0.0f, 0.0f, 5.0f) == 0, "异常ACC幅值(5g)");

    /* 测试 filter_check_gyro_quality */
    TEST_ASSERT(filter_check_gyro_quality(100.0f, 100.0f, 100.0f) == 1, "正常GYRO值");
    TEST_ASSERT(filter_check_gyro_quality(2000.0f, 0.0f, 0.0f) == 0, "GYRO饱和(2000dps)");

    /* 测试 filter_set_safety_config */
    filter_t *f = test_create_filter(FILTER_TYPE_COMPLEMENTARY);
    if (f) {
        filter_safety_config_t cfg = FILTER_SAFETY_DEFAULT;
        cfg.angle_min = -90.0f;
        cfg.angle_max = 90.0f;
        filter_set_safety_config(f, &cfg);
        TEST_ASSERT(f->safety_config.angle_min == -90.0f, "安全配置设置: angle_min");
        TEST_ASSERT(f->safety_config.angle_max == 90.0f, "安全配置设置: angle_max");
    }
}

static void test_degrade_apis(void) {
    printf("\n=== 测试14：退化模式API ===\n");

    /* 测试 filter_degrade_name */
    TEST_ASSERT(strcmp(filter_degrade_name(FILTER_DEGRADE_NONE), "None") == 0, "退化名称: None");
    TEST_ASSERT(strcmp(filter_degrade_name(FILTER_DEGRADE_GYRO_ONLY), "GyroOnly") == 0, "退化名称: GyroOnly");
    TEST_ASSERT(strcmp(filter_degrade_name(FILTER_DEGRADE_ACC_ONLY), "AccOnly") == 0, "退化名称: AccOnly");
    TEST_ASSERT(strcmp(filter_degrade_name(FILTER_DEGRADE_HOLD_LAST), "HoldLast") == 0, "退化名称: HoldLast");

    /* 测试 filter_set_degrade */
    filter_t *f = test_create_filter(FILTER_TYPE_COMPLEMENTARY);
    if (f) {
        filter_set_degrade(f, FILTER_DEGRADE_GYRO_ONLY);
        TEST_ASSERT(f->degrade == FILTER_DEGRADE_GYRO_ONLY, "退化模式设置: GYRO_ONLY");
        filter_set_degrade(f, FILTER_DEGRADE_NONE);
        TEST_ASSERT(f->degrade == FILTER_DEGRADE_NONE, "退化模式清除: NONE");
    }
}

static void test_static_allocation(void) {
    printf("\n=== 测试15：静态分配 ===\n");

    for (int i = 0; i < FILTER_TYPE_COUNT; i++) {
        filter_type_t type = (filter_type_t)i;
        size_t size = filter_get_static_size(type);
        TEST_ASSERT(size > 0, "静态大小>0");

        char buf[1024];
        if (size <= sizeof(buf)) {
            filter_t *f = filter_create_static(type, buf, sizeof(buf));
            if (f) {
                TEST_ASSERT(f->type == type, "静态分配类型正确");
                /* 静态分配不需要销毁，验证destroy为NULL即可 */
            }
        }
    }
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
    test_safety_apis();
    test_degrade_apis();
    test_static_allocation();

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
