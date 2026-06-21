/**
 * @file    filter_mahony.c
 * @brief   Mahony 滤波器 — PI 控制器互补滤波（SO(3) 上的无源互补）
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 1. 算法来源                                                  │
 * └──────────────────────────────────────────────────────────────┘
 *   论文：Mahony et al., 2008, "Nonlinear complementary filters
 *         on the special orthogonal group"
 *
 *   核心思想：在 SO(3) 上设计 PI 控制器，让估计姿态 R 追踪观测姿态 v，
 *   误差由 ACC 与估计重力方向的叉积给出。
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 2. 状态向量                                                  │
 * └──────────────────────────────────────────────────────────────┘
 *   x = [q0, q1, q2, q3, ix, iy, iz]
 *   q：单位四元数（姿态）
 *   ix,iy,iz：PI 控制器的积分项（估计陀螺偏置）
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 3. 误差计算                                                  │
 * └──────────────────────────────────────────────────────────────┘
 *   估计重力方向（地球坐标系 +Z 旋转到 body 系）：
 *     v = R(q)ᵀ · [0,0,1]ᵀ = [2(q1q3 - q0q2), 2(q0q1 + q2q3), 1 - 2(q1² + q2²)]
 *   观测重力方向：a = ACC / |ACC|
 *   误差（叉积，等价于 SO(3) 上的测地距离的微分）：
 *     e = a × v
 *
 *   ⚠ 叉积顺序：a × v（原代码用 ay*vz - az*vy 等）
 *     不同教材有 a×v 与 v×a 的差异，方向决定 PI 收敛方向，必须一致。
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 4. PI 控制器                                                 │
 * └──────────────────────────────────────────────────────────────┘
 *   ω_correction = kp·e + ki·∫e dt
 *     kp：比例增益（快速收敛）
 *     ki：积分增益（估计偏置，0 则不估偏置）
 *   积分抗饱和：|ix|,|iy|,|iz| ≤ INTEGRAL_LIMIT（防止累积过载）
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 5. 四元数积分                                                │
 * └──────────────────────────────────────────────────────────────┘
 *   ω_eff = ω_gyro + ω_correction   （单位 rad/s）
 *   q̇ = 0.5 · q ⊗ [0, ω_eff]
 *   q[k+1] = q[k] + q̇·dt
 *   归一化（防 ‖q‖ 漂移）
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 6. 退化模式                                                  │
 * └──────────────────────────────────────────────────────────────┘
 *   HOLD_LAST：返回上次四元数
 *   GYRO_ONLY：跳过 ACC 修正，仅四元数积分（ω_correction = 0）
 *   ACC_ONLY：从 ACC 重置四元数（filter_acc_to_quat），保留积分项
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 7. MCU 资源占用                                              │
 * └──────────────────────────────────────────────────────────────┘
 *   priv: 8·4 = 32 字节
 *   栈: ~80 字节
 *   周期: ~500 cycles @ 168MHz M4F（比 EKF 快 16×）
 *
 *   ⚠ INTEGRAL_LIMIT 硬编码为 0.5f（约 28.6°/s）。
 *     Phase 3 将提取到 mahony_priv_t，便于运行时调参。
 */

#include "filter.h"
#include "filter_internal.h"
#include "filter_math.h"
#include <math.h>

/* ============================================================
 * Mahony 滤波器
 * ============================================================ */

/* PI 积分抗饱和限幅（rad/s）
 * 0.5 rad/s ≈ 28.6 deg/s，覆盖机器狗正常偏置范围 */
#define MAHONY_INTEGRAL_LIMIT 0.5f

typedef struct {
    float kp;              /**< 比例增益 */
    float ki;              /**< 积分增益（0 = 不估偏置） */
    float q0, q1, q2, q3;  /**< 四元数 */
    float ix, iy, iz;      /**< 积分项（估计的陀螺偏置补偿） */
} mahony_priv_t;

FILTER_WEAK void mahony_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    mahony_priv_t *p = (mahony_priv_t *)self->priv;
    float dt = in->dt;

    /* 输入验证 */
    if (dt <= 0.0f || fp_isnan(in->ax) || fp_isinf(in->ax) ||
        fp_isnan(in->gx) || fp_isinf(in->gx)) {
        filter_quat_to_euler(p->q0, p->q1, p->q2, p->q3,
                             &out->pitch, &out->roll, &out->yaw);
        out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
        return;
    }

    /* HOLD_LAST */
    if (self->degrade == FILTER_DEGRADE_HOLD_LAST) {
        filter_quat_to_euler(p->q0, p->q1, p->q2, p->q3,
                             &out->pitch, &out->roll, &out->yaw);
        out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
        return;
    }

    /* GYRO_ONLY：跳过 ACC 修正，仅四元数积分 */
    if (self->degrade == FILTER_DEGRADE_GYRO_ONLY) {
        float gx = in->gx * DEG2RAD_F;
        float gy = in->gy * DEG2RAD_F;
        float gz = in->gz * DEG2RAD_F;
        float q0 = p->q0, q1 = p->q1, q2 = p->q2, q3 = p->q3;
        p->q0 += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
        p->q1 += 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * dt;
        p->q2 += 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * dt;
        p->q3 += 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * dt;
        filter_quat_normalize_inplace(&p->q0, &p->q1, &p->q2, &p->q3);
        filter_quat_to_euler(p->q0, p->q1, p->q2, p->q3,
                             &out->pitch, &out->roll, &out->yaw);
        out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
        return;
    }

    /* ACC_ONLY：从 ACC 重置四元数（保留积分项，不重置偏置估计） */
    if (self->degrade == FILTER_DEGRADE_ACC_ONLY) {
        filter_acc_to_quat(in->ax, in->ay, in->az,
                           &p->q0, &p->q1, &p->q2, &p->q3);
        filter_quat_to_euler(p->q0, p->q1, p->q2, p->q3,
                             &out->pitch, &out->roll, &out->yaw);
        out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
        return;
    }

    /* ── 正常模式 ── */
    float ax = in->ax, ay = in->ay, az = in->az;
    /* 陀螺度→弧度 */
    float gx = in->gx * DEG2RAD_F;
    float gy = in->gy * DEG2RAD_F;
    float gz = in->gz * DEG2RAD_F;

    /* ACC 归一化（防 |a|=0 除零） */
    float norm = fp_sqrt(ax * ax + ay * ay + az * az);
    if (norm > 0.0f) {
        float inv = 1.0f / norm;
        ax *= inv; ay *= inv; az *= inv;
    }

    /* 估计重力方向 v = R(q)ᵀ·[0,0,1] */
    float vx = 2.0f * (p->q1 * p->q3 - p->q0 * p->q2);
    float vy = 2.0f * (p->q0 * p->q1 + p->q2 * p->q3);
    float vz = 1.0f - 2.0f * (p->q1 * p->q1 + p->q2 * p->q2);

    /* 误差 e = a × v（叉积） */
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    /* PI 控制器 */
    if (p->ki > 0.0f) {
        p->ix += p->ki * ex * dt;
        p->iy += p->ki * ey * dt;
        p->iz += p->ki * ez * dt;
        /* 积分抗饱和 */
        p->ix = clampf(p->ix, -MAHONY_INTEGRAL_LIMIT, MAHONY_INTEGRAL_LIMIT);
        p->iy = clampf(p->iy, -MAHONY_INTEGRAL_LIMIT, MAHONY_INTEGRAL_LIMIT);
        p->iz = clampf(p->iz, -MAHONY_INTEGRAL_LIMIT, MAHONY_INTEGRAL_LIMIT);
        gx += p->ix;
        gy += p->iy;
        gz += p->iz;
    }

    /* 比例项叠加到 ω */
    gx += p->kp * ex;
    gy += p->kp * ey;
    gz += p->kp * ez;

    /* 四元数积分 q̇ = 0.5·q⊗ω */
    float q0 = p->q0, q1 = p->q1, q2 = p->q2, q3 = p->q3;
    p->q0 += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    p->q1 += 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * dt;
    p->q2 += 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * dt;
    p->q3 += 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * dt;

    filter_quat_normalize_inplace(&p->q0, &p->q1, &p->q2, &p->q3);

    /* 输出 */
    filter_quat_to_euler(p->q0, p->q1, p->q2, p->q3,
                         &out->pitch, &out->roll, &out->yaw);
    out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
}

FILTER_WEAK void mahony_reset(filter_t *self)
{
    mahony_priv_t *p = (mahony_priv_t *)self->priv;
    p->q0 = 1.0f; p->q1 = p->q2 = p->q3 = 0.0f;
    p->ix = p->iy = p->iz = 0.0f;
}

FILTER_WEAK void mahony_set_param(filter_t *self, filter_param_t param, float value)
{
    mahony_priv_t *p = (mahony_priv_t *)self->priv;
    if (param == FILTER_PARAM_KP) p->kp = value;
    if (param == FILTER_PARAM_KI) p->ki = value;
}

FILTER_WEAK size_t mahony_get_static_size(void)
{
    return sizeof(mahony_priv_t);
}

FILTER_WEAK void mahony_init(void *priv)
{
    mahony_priv_t *p = (mahony_priv_t *)priv;
    /* 默认参数（见 filter_config.h）：
     *   kp = 10.0  （论文 0.5~10，工程取高端求快速收敛）
     *   ki = 0.0   （默认不估偏置，BSP 层有独立偏置跟踪）
     *   q0 = 1     （单位四元数） */
    p->q0 = 1.0f;
    p->kp = 10.0f;
    p->ki = 0.0f;
}
