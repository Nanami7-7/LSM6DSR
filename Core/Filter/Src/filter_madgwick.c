/**
 * @file    filter_madgwick.c
 * @brief   Madgwick 滤波器 — 梯度下降法四元数姿态估计
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 1. 算法来源                                                  │
 * └──────────────────────────────────────────────────────────────┘
 *   论文：Madgwick, S.O.H., 2010, "An efficient orientation filter
 *         for inertial and inertial/magnetic sensor arrays"
 *
 *   核心思想：用梯度下降法最小化 |a - h(q)|²，
 *   h(q) = R(q)ᵀ·g 是预测重力方向，a 是观测加速度。
 *   梯度方向作为修正项叠加到四元数微分方程。
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 2. 状态向量                                                  │
 * └──────────────────────────────────────────────────────────────┘
 *   x = [q0, q1, q2, q3, β]
 *   q：单位四元数
 *   β：梯度下降步长（论文称 beta，类似 Mahony 的 kp）
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 3. 目标函数与梯度                                            │
 * └──────────────────────────────────────────────────────────────┘
 *   目标：f(q) = R(q)ᵀ·g - a = 0   （g=[0,0,1], a=ACC/|ACC|）
 *   f(q) = [2(q1q3 - q0q2) - ax,
 *           2(q0q1 + q2q3) - ay,
 *           1 - 2(q1² + q2²) - az]
 *   雅可比 J = ∂f/∂q (3×4)，解析式见代码
 *   梯度 ∇f = Jᵀ·f  （4 维向量 s0..s3）
 *   归一化 s /= |s|（防步长随误差大小波动）
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 4. 四元数微分方程                                            │
 * └──────────────────────────────────────────────────────────────┘
 *   q̇ = 0.5·q⊗ω - β·s
 *       ↑陀螺积分项    ↑梯度下降修正项
 *   离散：q[k+1] = q[k] + q̇·dt
 *
 *   权衡：
 *     - β 大：梯度下降主导，快速收敛到 ACC 解，但噪声大
 *     - β 小：陀螺积分主导，平滑但漂移大
 *     - 论文 β=0.033（IMU），工程取 0.5（快收敛，适合实时）
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 5. 退化模式                                                  │
 * └──────────────────────────────────────────────────────────────┘
 *   HOLD_LAST：返回上次四元数
 *   GYRO_ONLY：跳过梯度下降（β·s = 0），仅四元数积分
 *   ACC_ONLY：从 ACC 重置四元数
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 6. MCU 资源占用                                              │
 * └──────────────────────────────────────────────────────────────┘
 *   priv: 5·4 = 20 字节
 *   栈: ~100 字节（梯度中间量）
 *   周期: ~700 cycles @ 168MHz M4F（比 EKF 快 11×，比 Mahony 稍慢）
 *
 *   无偏置估计（priv 不含 ix/iy/iz），依赖 BSP 层外部偏置跟踪。
 */

#include "filter.h"
#include "filter_internal.h"
#include "filter_math.h"
#include <math.h>

/* ============================================================
 * Madgwick 滤波器
 * ============================================================ */

typedef struct {
    float beta;           /**< 梯度下降步长 */
    float q0, q1, q2, q3; /**< 四元数 */
} madgwick_priv_t;

FILTER_WEAK void madgwick_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    madgwick_priv_t *p = (madgwick_priv_t *)self->priv;
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

    /* GYRO_ONLY：仅四元数积分 */
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

    /* ACC_ONLY：从 ACC 重置四元数 */
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
    float gx = in->gx * DEG2RAD_F;
    float gy = in->gy * DEG2RAD_F;
    float gz = in->gz * DEG2RAD_F;

    float q0 = p->q0, q1 = p->q1, q2 = p->q2, q3 = p->q3;

    /* ACC 归一化 */
    float norm = fp_sqrt(ax * ax + ay * ay + az * az);
    if (norm > 0.0f) {
        float inv = 1.0f / norm;
        ax *= inv; ay *= inv; az *= inv;
    }

    /* ── 梯度下降步 ──
     * 目标函数 f(q) 与雅可比 J 的展开（Madgwick 论文 §3.5）：
     *   f = [2(q1q3 - q0q2) - ax,
     *        2(q0q1 + q2q3) - ay,
     *        1 - 2(q1² + q2²) - az]
     *   J = [[-2q2,  2q3, -2q0,  2q1],
     *        [ 2q1,  2q0,  2q3,  2q2],
     *        [   0, -4q1, -4q2,     0]]
     * 梯度 s = Jᵀ·f （4 维），展开后：
     */
    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    /* 归一化梯度（防步长波动） */
    norm = fp_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm > 0.0f) {
        float inv = 1.0f / norm;
        s0 *= inv; s1 *= inv; s2 *= inv; s3 *= inv;
    }

    /* 四元数微分方程 q̇ = 0.5·q⊗ω - β·s */
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - p->beta * s0;
    float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - p->beta * s1;
    float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - p->beta * s2;
    float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - p->beta * s3;

    /* 积分 */
    p->q0 += qDot0 * dt;
    p->q1 += qDot1 * dt;
    p->q2 += qDot2 * dt;
    p->q3 += qDot3 * dt;

    filter_quat_normalize_inplace(&p->q0, &p->q1, &p->q2, &p->q3);

    /* 输出 */
    filter_quat_to_euler(p->q0, p->q1, p->q2, p->q3,
                         &out->pitch, &out->roll, &out->yaw);
    out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
}

FILTER_WEAK void madgwick_reset(filter_t *self)
{
    madgwick_priv_t *p = (madgwick_priv_t *)self->priv;
    p->q0 = 1.0f; p->q1 = p->q2 = p->q3 = 0.0f;
}

FILTER_WEAK void madgwick_set_param(filter_t *self, filter_param_t param, float value)
{
    madgwick_priv_t *p = (madgwick_priv_t *)self->priv;
    /* Madgwick 用 beta，复用 KP 枚举（filter_param_t 无 BETA 项） */
    if (param == FILTER_PARAM_KP) p->beta = value;
}

FILTER_WEAK size_t madgwick_get_static_size(void)
{
    return sizeof(madgwick_priv_t);
}

FILTER_WEAK void madgwick_init(void *priv)
{
    madgwick_priv_t *p = (madgwick_priv_t *)priv;
    /* 默认 β = 0.5（论文 0.033，工程取大值求快收敛）
     * 见 filter_config.h 的 MADGWICK_BETA_DEFAULT */
    p->q0 = 1.0f;
    p->beta = 0.5f;
}
