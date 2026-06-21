/**
 * @file    filter_common.c
 * @brief   滤波器公共工具函数实现
 *
 * 包含：
 *   - 类型/退化模式名称查询
 *   - 退化模式设置
 *   - 传感器质量评估
 *   - 输出验证、四元数归一化、协方差正则化
 *   - 安全配置设置
 *
 * 从原 filter.c:1049-1097 + 1212-1276 拆出。零行为变化。
 */

#include "filter.h"
#include "filter_internal.h"
#include "filter_math.h"
#include <math.h>
#include <stddef.h>

/* ============================================================
 * 共享辅助函数实现
 * ============================================================
 *
 * 这些函数原本散落在 6 个滤波器中重复实现，现统一抽出到此。
 * 抽出后行为字节一致（运算顺序保持），但代码量减少 ~200 行，
 * 并保证所有滤波器对相同输入产生相同中间结果。
 */

void filter_acc_to_euler(float ax, float ay, float az,
                         float *pitch_deg, float *roll_deg)
{
    /* 注意运算顺序与原 filter.c 各处一致，保证 golden output 字节相同 */
    float pitch = fp_atan2(-ax, fp_sqrt(ay * ay + az * az)) * RAD2DEG_F;
    float roll  = fp_atan2( ay, fp_sqrt(ax * ax + az * az)) * RAD2DEG_F;
    if (pitch_deg) *pitch_deg = pitch;
    if (roll_deg)  *roll_deg  = roll;
}

void filter_quat_to_euler(float q0, float q1, float q2, float q3,
                          float *pitch_deg, float *roll_deg, float *yaw_deg)
{
    /* pitch: asin 项必须钳位，否则 |2(q1q3-q0q2)| > 1 时返回 NaN */
    float pitch = safe_asinf(-2.0f * (q1 * q3 - q0 * q2)) * RAD2DEG_F;
    /* roll: atan2 自带 [-π, π] 范围，无需钳位 */
    float roll  = fp_atan2(2.0f * (q0 * q1 + q2 * q3),
                           1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG_F;
    /* yaw: 同 roll */
    float yaw   = fp_atan2(2.0f * (q0 * q3 + q1 * q2),
                           1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG_F;
    if (pitch_deg) *pitch_deg = pitch;
    if (roll_deg)  *roll_deg  = roll;
    if (yaw_deg)   *yaw_deg   = yaw;
}

void filter_quat_normalize_inplace(float *q0, float *q1, float *q2, float *q3)
{
    if (!q0 || !q1 || !q2 || !q3) return;

    float n = fp_sqrt((*q0) * (*q0) + (*q1) * (*q1) +
                      (*q2) * (*q2) + (*q3) * (*q3));
    if (n < 1e-10f) {
        /* 退化：归零或溢出，重置为单位四元数 */
        *q0 = 1.0f; *q1 = 0.0f; *q2 = 0.0f; *q3 = 0.0f;
        return;
    }
    float inv = 1.0f / n;
    *q0 *= inv; *q1 *= inv; *q2 *= inv; *q3 *= inv;
}

void filter_acc_to_quat(float ax, float ay, float az,
                        float *q0, float *q1, float *q2, float *q3)
{
    float ax_norm = fp_sqrt(ax * ax + ay * ay + az * az);
    if (ax_norm <= 0.01f) {
        /* 自由落体或静止：ACC 不可靠，保留原四元数（不更新） */
        return;
    }
    float inv = 1.0f / ax_norm;
    float ax_n = ax * inv, ay_n = ay * inv, az_n = az * inv;

    /* 从 ACC 求 pitch/roll（弧度），转半角四元数（yaw=0） */
    float acc_pitch_rad = fp_atan2(-ax_n, fp_sqrt(ay_n * ay_n + az_n * az_n));
    float acc_roll_rad  = fp_atan2( ay_n, fp_sqrt(ax_n * ax_n + az_n * az_n));
    float cp = fp_cos(acc_pitch_rad * 0.5f);
    float sp = fp_sin(acc_pitch_rad * 0.5f);
    float cr = fp_cos(acc_roll_rad * 0.5f);
    float sr = fp_sin(acc_roll_rad * 0.5f);

    /* q = q_pitch ⊗ q_roll，展开后： */
    *q0 = cp * cr;
    *q1 = cp * sr;
    *q2 = sp * cr;
    *q3 = -sp * sr;  /* yaw=0 */
}

/* ============================================================
 * 类型与退化模式名称
 * ============================================================ */

const char* filter_type_name(filter_type_t type)
{
    static const char *names[] = {
        "Complementary",
        "LPF",
        "EKF",
        "LKF",
        "Mahony",
        "Madgwick"
    };
    if (type >= 0 && type < FILTER_TYPE_COUNT) {
        return names[type];
    }
    return "Unknown";
}

static const char *degrade_names[] = {
    "None",
    "StaticOnly",
    "GyroOnly",
    "AccOnly",
    "HoldLast"
};

void filter_set_degrade(filter_t *f, filter_degrade_t degrade) {
    if (f && degrade >= 0 && degrade < FILTER_DEGRADE_COUNT) {
        f->degrade = degrade;
    }
}

const char* filter_degrade_name(filter_degrade_t degrade) {
    if (degrade >= 0 && degrade < FILTER_DEGRADE_COUNT) {
        return degrade_names[degrade];
    }
    return "Unknown";
}

/* ============================================================
 * 传感器质量评估
 * ============================================================ */

int filter_check_acc_quality(float ax, float ay, float az) {
    float norm = fp_sqrt(ax*ax + ay*ay + az*az);
    return (norm >= 0.5f && norm <= 2.0f) ? 1 : 0;
}

int filter_check_gyro_quality(float gx, float gy, float gz) {
    return (fp_fabs(gx) <= 400.0f && fp_fabs(gy) <= 400.0f && fp_fabs(gz) <= 400.0f) ? 1 : 0;
}

/* ============================================================
 * 数值安全保护
 * ============================================================ */

int filter_validate_output(const filter_output_t *out) {
    if (!out) return 0;

    /* 检查NaN/Inf */
    if (fp_isnan(out->pitch) || fp_isinf(out->pitch) ||
        fp_isnan(out->roll)  || fp_isinf(out->roll)  ||
        fp_isnan(out->yaw)   || fp_isinf(out->yaw)) {
        return 0;
    }

    /* 检查角度范围 */
    if (out->pitch < -180.0f || out->pitch > 180.0f ||
        out->roll  < -180.0f || out->roll  > 180.0f ||
        out->yaw   < -180.0f || out->yaw   > 180.0f) {
        return 0;
    }

    return 1;
}

void filter_normalize_quaternion(float *q0, float *q1, float *q2, float *q3) {
    if (!q0 || !q1 || !q2 || !q3) return;

    float norm = fp_sqrt((*q0)*(*q0) + (*q1)*(*q1) + (*q2)*(*q2) + (*q3)*(*q3));
    if (norm < 1e-10f) {
        /* 退化为单位四元数 */
        *q0 = 1.0f; *q1 = *q2 = *q3 = 0.0f;
        return;
    }

    float inv_norm = 1.0f / norm;
    *q0 *= inv_norm;
    *q1 *= inv_norm;
    *q2 *= inv_norm;
    *q3 *= inv_norm;
}

void filter_regularize_covariance(float P[][7], int size, float factor) {
    if (!P || size <= 0) return;

    /* 确保对角线元素为正 */
    for (int i = 0; i < size; i++) {
        if (P[i][i] < factor) {
            P[i][i] = factor;
        }
    }

    /* 限制非对角线元素（防止数值爆炸） */
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (i != j) {
                float max_val = fp_sqrt(P[i][i] * P[j][j]);
                if (fp_fabs(P[i][j]) > max_val) {
                    P[i][j] = (P[i][j] > 0) ? max_val : -max_val;
                }
            }
        }
    }
}

void filter_set_safety_config(filter_t *f, const filter_safety_config_t *config) {
    if (!f || !config) return;
    f->safety_config = *config;
}
