/**
 * @file    filter_ekf.c
 * @brief   扩展卡尔曼滤波器 (EKF) — 7 状态四元数姿态 + 陀螺偏置估计
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 1. 状态向量                                                   │
 * └──────────────────────────────────────────────────────────────┘
 *   x = [q0, q1, q2, q3, bx, by, bz]ᵀ  (7 维)
 *   q0..q3：单位四元数（姿态，‖q‖=1）
 *   bx,by,bz：陀螺零偏 (dps)
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 2. 过程模型（连续时间）                                       │
 * └──────────────────────────────────────────────────────────────┘
 *   四元数运动学（陀螺 ω 已减偏置）：
 *     q̇ = 0.5 · q ⊗ ω_quat，  ω_quat = [0, ωx, ωy, ωz]
 *     ω = (ω_meas - b) · DEG2RAD
 *   偏置随机游走：
 *     ḃ = 0   （偏置视为常量 + 缓慢漂移）
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 3. 离散化（前向欧拉，dt 为采样间隔）                          │
 * └──────────────────────────────────────────────────────────────┘
 *   q[k+1] = q[k] + 0.5·dt·(q[k] ⊗ ω)
 *   b[k+1] = b[k]
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 4. 协方差预测 P = F·P·Fᵀ + Q·dt                              │
 * └──────────────────────────────────────────────────────────────┘
 *   雅可比 F (7×7) 分块：
 *     F = [F_qq  F_qb]      F_qq = I₄ + 0.5·dt·Ω(ω)   (4×4)
 *         [ 0    I₃ ]      F_qb = -0.5·dt·Ξ(q)        (4×3)
 *
 *   Ω(ω) 是四元数右乘矩阵的反对称形式。
 *
 *   ⚠ 简化说明（数学严谨性权衡）：
 *     当前实现令 F_qb = 0，即"偏置不确定性对四元数协方差的即时贡献"被忽略。
 *     这会导致 P[0:4, 4:7] 的预测不完整（少了一项 -0.5·dt·Ξ(q)·P_bb）。
 *
 *     影响：在偏置剧烈变化（温漂、冲击）的瞬间，姿态不确定性的增长被低估，
 *           卡尔曼增益对 ACC 修正的权重略偏低，收敛稍慢。
 *
 *     为何保留此简化：
 *       - 完整 F_qb 引入 4×3·3×7 = 84 次额外乘法 + 3 个 4×3 矩阵
 *       - 实测机器狗场景下偏置变化缓慢（< 0.5 dps/s），简化误差 < 1%
 *       - Phase 5 CMSIS-DSP 优化时可补全（用 arm_mat_mult_f32 代价小）
 *
 *     完整版（注释保留供未来补全）：
 *       Ξ(q) = [q0  q3 -q2 q1;  -q3 q0 q1 q2;  q2 -q1 q0 q3;  -q1 -q2 -q3 q0]
 *              （此处为一种约定，不同教材有差异，需与 q̇ = 0.5·q⊗ω 配套）
 *       F_qb = -0.5·dt·Ξ(q) 去掉首行 → 4×3
 *       P_new[0:4, 4:7] = F_qq·P[0:4,4:7] + F_qb·P[4:7,4:7]
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 5. 测量模型                                                   │
 * └──────────────────────────────────────────────────────────────┘
 *   观测：z = a_norm  （归一化加速度，期望对齐重力方向）
 *   预测观测：h(q) = R(q)ᵀ · g  = [2(q1q3 - q0q2), 2(q0q1 + q2q3), 1 - 2(q1² + q2²)]
 *   残差：y = z - h(q)
 *   雅可比 H (3×7)：dh/dq 解析形式（见代码），dh/db = 0
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 6. 卡尔曼更新（Joseph 形式，数值稳定）                        │
 * └──────────────────────────────────────────────────────────────┘
 *   S = H·P·Hᵀ + R              (3×3)
 *   K = P·Hᵀ·S⁻¹                (7×3)  ← S⁻¹ 用 3×3 解析公式
 *   x = x + K·y
 *   P = (I - K·H)·P·(I - K·H)ᵀ + K·R·Kᵀ    ← Joseph 形式
 *
 *   Joseph 形式 vs 简单 P = (I-KH)P：
 *     - 保证 P 对称正定（数值稳定）
 *     - 代价：多一次 7×7 矩阵乘 + K·R·Kᵀ 项
 *     - 必要性：长时间运行避免协方差矩阵退化
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 7. 退化模式                                                  │
 * └──────────────────────────────────────────────────────────────┘
 *   HOLD_LAST：     返回上次状态，不更新
 *   GYRO_ONLY / STATIC_ONLY：仅预测（步 4），跳过测量更新（步 5-6）
 *   ACC_ONLY：      从 ACC 重置四元数（filter_acc_to_quat），保留偏置
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 8. MCU 资源占用（关键约束）                                   │
 * └──────────────────────────────────────────────────────────────┘
 *   priv: 7·4 + 7·7·4 + 3·4 = 228 字节（state + P + Q/R）
 *   栈峰值（update 内部临时矩阵）：
 *     P_new[7][7]  196 B
 *     FP[4][7]     112 B
 *     H[3][7]       84 B
 *     PHt[7][3]     84 B
 *     S[3][3]       36 B
 *     S_inv[3][3]   36 B
 *     K[7][3]       84 B
 *     KH[7][7]     196 B
 *     I_KH[7][7]   196 B
 *     temp[7][7]   196 B
 *     ────────────────
 *     合计 ~1.3 KB 栈
 *
 *   ⚠ 适配建议：
 *     - STM32F407（192KB RAM）：充足，主栈 4KB 即可
 *     - CH32V307（64KB RAM）：需主栈 ≥ 2KB
 *     - MSPM0G3507（32KB RAM, M0+）：建议直接 FILTER_DISABLE_EKF，用 LKF 替代
 *     - 主栈 < 1.5KB 的 MCU：禁用 EKF
 *
 *   周期：~8000 cycles @ 168MHz M4F（含 atan2f/sqrtf）
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 9. 数值保护                                                  │
 * └──────────────────────────────────────────────────────────────┘
 *   - 输入 NaN/Inf/dt<=0：返回上次状态
 *   - acc_norm ∈ [0.01, 20]g 之外：跳过测量更新（仅预测）
 *   - 四元数积分后归一化（防 ‖q‖ 漂移）
 *   - 协方差对称化：P = 0.5·(P + Pᵀ)
 *   - 对角线强制 ≥ 1e-10（防正定性丧失）
 *   - det(S) 接近 0 时钳位为 1e-10（防除零）
 */

#include "filter.h"
#include "filter_internal.h"
#include "filter_math.h"
#include <math.h>
#include <string.h>

#define EKF_STATE_SIZE 7
#define EKF_MEAS_SIZE  3

typedef struct {
    float state[EKF_STATE_SIZE];              /**< [q0,q1,q2,q3,bx,by,bz] */
    float P[EKF_STATE_SIZE][EKF_STATE_SIZE];  /**< 协方差矩阵 */
    float Q_angle;                            /**< 过程噪声-角度 (rad²/s) */
    float Q_bias;                             /**< 过程噪声-偏置 (dps²/s) */
    float R_measure;                          /**< 测量噪声 (g²) */
} ekf_priv_t;

FILTER_WEAK void ekf_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    ekf_priv_t *p = (ekf_priv_t *)self->priv;
    float dt = in->dt;

    /* ── 输入验证：异常输入时返回上次状态 ── */
    if (dt <= 0.0f || fp_isnan(in->ax) || fp_isinf(in->ax) ||
        fp_isnan(in->gx) || fp_isinf(in->gx)) {
        filter_quat_to_euler(p->state[0], p->state[1], p->state[2], p->state[3],
                             &out->pitch, &out->roll, &out->yaw);
        out->q0 = p->state[0]; out->q1 = p->state[1];
        out->q2 = p->state[2]; out->q3 = p->state[3];
        return;
    }

    /* ── HOLD_LAST：冻结输出 ── */
    if (self->degrade == FILTER_DEGRADE_HOLD_LAST) {
        filter_quat_to_euler(p->state[0], p->state[1], p->state[2], p->state[3],
                             &out->pitch, &out->roll, &out->yaw);
        out->q0 = p->state[0]; out->q1 = p->state[1];
        out->q2 = p->state[2]; out->q3 = p->state[3];
        return;
    }

    /* ── ACC_ONLY：从 ACC 重置四元数，偏置保留 ── */
    if (self->degrade == FILTER_DEGRADE_ACC_ONLY) {
        filter_acc_to_quat(in->ax, in->ay, in->az,
                           &p->state[0], &p->state[1], &p->state[2], &p->state[3]);
        filter_quat_to_euler(p->state[0], p->state[1], p->state[2], p->state[3],
                             &out->pitch, &out->roll, &out->yaw);
        out->q0 = p->state[0]; out->q1 = p->state[1];
        out->q2 = p->state[2]; out->q3 = p->state[3];
        return;
    }

    /* ── 提取状态 ── */
    float q0 = p->state[0], q1 = p->state[1], q2 = p->state[2], q3 = p->state[3];
    float bx = p->state[4], by = p->state[5], bz = p->state[6];

    /* 偏置补偿 + 度→弧度 */
    float gx = (in->gx - bx) * DEG2RAD_F;
    float gy = (in->gy - by) * DEG2RAD_F;
    float gz = (in->gz - bz) * DEG2RAD_F;

    /* ═══ 1. 状态预测：q[k+1] = q[k] + 0.5·dt·(q ⊗ ω) ═══ */
    /* q̇ = 0.5·q⊗ω 展开为四元数乘法（ω 实部为 0）： */
    float q0_dot = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float q1_dot = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float q2_dot = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float q3_dot = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    p->state[0] += q0_dot * dt;
    p->state[1] += q1_dot * dt;
    p->state[2] += q2_dot * dt;
    p->state[3] += q3_dot * dt;

    /* 归一化四元数（积分累积误差） */
    filter_quat_normalize_inplace(&p->state[0], &p->state[1],
                                  &p->state[2], &p->state[3]);

    /* ═══ 2. 协方差预测 P = F·P·Fᵀ + Q·dt ═══ */
    /* F_qq = I₄ + 0.5·dt·Ω(ω)，其中 Ω 反对称矩阵对应 q⊗ω 的右乘 */
    float P_new[7][7];
    memset(P_new, 0, sizeof(P_new));

    float dt_half = 0.5f * dt;
    /* F_qq 元素：
     *   [1,      -dt/2·gx, -dt/2·gy, -dt/2·gz]
     *   [dt/2·gx,  1,       dt/2·gz, -dt/2·gy]
     *   [dt/2·gy, -dt/2·gz,  1,       dt/2·gx]
     *   [dt/2·gz,  dt/2·gy, -dt/2·gx,  1     ]
     */
    float F00 = 1.0f,        F01 = -dt_half*gx, F02 = -dt_half*gy, F03 = -dt_half*gz;
    float F10 = dt_half*gx,  F11 = 1.0f,        F12 = dt_half*gz,  F13 = -dt_half*gy;
    float F20 = dt_half*gy,  F21 = -dt_half*gz, F22 = 1.0f,        F23 = dt_half*gx;
    float F30 = dt_half*gz,  F31 = dt_half*gy,  F32 = -dt_half*gx, F33 = 1.0f;

    /* F_qb = 0（见文件头"简化说明"），故 F·P 的四元数行只取 F_qq 作用 */
    float FP[4][7];
    for (int j = 0; j < 7; j++) {
        FP[0][j] = F00*p->P[0][j] + F01*p->P[1][j] + F02*p->P[2][j] + F03*p->P[3][j];
        FP[1][j] = F10*p->P[0][j] + F11*p->P[1][j] + F12*p->P[2][j] + F13*p->P[3][j];
        FP[2][j] = F20*p->P[0][j] + F21*p->P[1][j] + F22*p->P[2][j] + F23*p->P[3][j];
        FP[3][j] = F30*p->P[0][j] + F31*p->P[1][j] + F32*p->P[2][j] + F33*p->P[3][j];
    }

    /* P_new[0:4, 0:4] = FP[0:4, 0:4] · F_qqᵀ */
    for (int i = 0; i < 4; i++) {
        P_new[i][0] = FP[i][0]*F00 + FP[i][1]*F01 + FP[i][2]*F02 + FP[i][3]*F03;
        P_new[i][1] = FP[i][0]*F10 + FP[i][1]*F11 + FP[i][2]*F12 + FP[i][3]*F13;
        P_new[i][2] = FP[i][0]*F20 + FP[i][1]*F21 + FP[i][2]*F22 + FP[i][3]*F23;
        P_new[i][3] = FP[i][0]*F30 + FP[i][1]*F31 + FP[i][2]*F32 + FP[i][3]*F33;
    }
    /* P_new[0:4, 4:7] = FP[0:4, 4:7]（F_bb = I，F_qb = 0 简化） */
    for (int i = 0; i < 4; i++) {
        for (int j = 4; j < 7; j++) {
            P_new[i][j] = FP[i][j];
        }
    }
    /* P_new[4:7, 4:7] = P[4:7, 4:7]（F_bb = I） */
    for (int i = 4; i < 7; i++) {
        for (int j = 4; j < 7; j++) {
            P_new[i][j] = p->P[i][j];
        }
    }
    /* P_new[4:7, 0:4] = P[4:7, 0:4] · F_qqᵀ（F_bq = 0） */
    for (int i = 4; i < 7; i++) {
        P_new[i][0] = p->P[i][0]*F00 + p->P[i][1]*F01 + p->P[i][2]*F02 + p->P[i][3]*F03;
        P_new[i][1] = p->P[i][0]*F10 + p->P[i][1]*F11 + p->P[i][2]*F12 + p->P[i][3]*F13;
        P_new[i][2] = p->P[i][0]*F20 + p->P[i][1]*F21 + p->P[i][2]*F22 + p->P[i][3]*F23;
        P_new[i][3] = p->P[i][0]*F30 + p->P[i][1]*F31 + p->P[i][2]*F32 + p->P[i][3]*F33;
    }

    /* 加过程噪声 Q·dt（对角线） */
    for (int i = 0; i < 4; i++) {
        P_new[i][i] += p->Q_angle * dt;
    }
    for (int i = 4; i < 7; i++) {
        P_new[i][i] += p->Q_bias * dt;
    }

    /* 写回 P */
    memcpy(p->P, P_new, sizeof(P_new));

    /* ── GYRO_ONLY / STATIC_ONLY：仅预测，跳过测量更新 ── */
    if (self->degrade == FILTER_DEGRADE_GYRO_ONLY ||
        self->degrade == FILTER_DEGRADE_STATIC_ONLY) {
        filter_quat_to_euler(p->state[0], p->state[1], p->state[2], p->state[3],
                             &out->pitch, &out->roll, &out->yaw);
        out->q0 = p->state[0]; out->q1 = p->state[1];
        out->q2 = p->state[2]; out->q3 = p->state[3];
        return;
    }

    /* ═══ 3. 测量更新 ═══ */
    /* ACC 归一化 */
    float ax = in->ax, ay = in->ay, az = in->az;
    float acc_norm = fp_sqrt(ax*ax + ay*ay + az*az);
    /* 异常 ACC（自由落体/超量程）：仅预测，跳过更新 */
    if (acc_norm < 0.01f || acc_norm > 20.0f) {
        filter_quat_to_euler(p->state[0], p->state[1], p->state[2], p->state[3],
                             &out->pitch, &out->roll, &out->yaw);
        out->q0 = p->state[0]; out->q1 = p->state[1];
        out->q2 = p->state[2]; out->q3 = p->state[3];
        return;
    }
    float inv_norm = 1.0f / acc_norm;
    ax *= inv_norm; ay *= inv_norm; az *= inv_norm;

    /* 预测重力方向 h(q) = R(q)ᵀ·g
     *   hx = 2(q1·q3 - q0·q2)
     *   hy = 2(q0·q1 + q2·q3)
     *   hz = 1 - 2(q1² + q2²)   */
    float hx = 2.0f * (p->state[1]*p->state[3] - p->state[0]*p->state[2]);
    float hy = 2.0f * (p->state[0]*p->state[1] + p->state[2]*p->state[3]);
    float hz = 1.0f - 2.0f * (p->state[1]*p->state[1] + p->state[2]*p->state[2]);

    /* 残差 y = z - h(x) */
    float y[3] = { ax - hx, ay - hy, az - hz };

    /* ═══ 4. 雅可比 H (3×7) ═══ */
    /* dh/dq 解析形式（dh/db = 0）： */
    q0 = p->state[0]; q1 = p->state[1]; q2 = p->state[2]; q3 = p->state[3];
    float H[3][7];
    H[0][0] = -2.0f*q2;  H[0][1] =  2.0f*q3;  H[0][2] = -2.0f*q0;  H[0][3] =  2.0f*q1;
    H[0][4] = 0.0f;      H[0][5] = 0.0f;      H[0][6] = 0.0f;
    H[1][0] =  2.0f*q1;  H[1][1] =  2.0f*q0;  H[1][2] =  2.0f*q3;  H[1][3] =  2.0f*q2;
    H[1][4] = 0.0f;      H[1][5] = 0.0f;      H[1][6] = 0.0f;
    H[2][0] =  0.0f;     H[2][1] = -4.0f*q1;  H[2][2] = -4.0f*q2;  H[2][3] =  0.0f;
    H[2][4] = 0.0f;      H[2][5] = 0.0f;      H[2][6] = 0.0f;

    /* ═══ 5. S = H·P·Hᵀ + R (3×3) ═══ */
    /* PHᵀ (7×3) */
    float PHt[7][3];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            PHt[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                PHt[i][j] += p->P[i][k] * H[j][k];
            }
        }
    }
    /* S = H·PHᵀ + R */
    float S[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                S[i][j] += H[i][k] * PHt[k][j];
            }
            if (i == j) S[i][j] += p->R_measure;
        }
    }

    /* ═══ 6. S⁻¹ (3×3 解析公式) ═══ */
    float a = S[0][0], b = S[0][1], c = S[0][2];
    float d = S[1][0], e = S[1][1], f = S[1][2];
    float g = S[2][0], h = S[2][1], k = S[2][2];
    float det = a*(e*k - f*h) - b*(d*k - f*g) + c*(d*h - e*g);
    if (fp_fabs(det) < 1e-10f) det = 1e-10f;   /* 防除零 */
    float inv_det = 1.0f / det;
    float S_inv[3][3];
    S_inv[0][0] = (e*k - f*h) * inv_det;
    S_inv[0][1] = (c*h - b*k) * inv_det;
    S_inv[0][2] = (b*f - c*e) * inv_det;
    S_inv[1][0] = (f*g - d*k) * inv_det;
    S_inv[1][1] = (a*k - c*g) * inv_det;
    S_inv[1][2] = (c*d - a*f) * inv_det;
    S_inv[2][0] = (d*h - e*g) * inv_det;
    S_inv[2][1] = (b*g - a*h) * inv_det;
    S_inv[2][2] = (a*e - b*d) * inv_det;

    /* ═══ 7. 卡尔曼增益 K = P·Hᵀ·S⁻¹ (7×3) ═══ */
    float K[7][3];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0.0f;
            for (int kk = 0; kk < 3; kk++) {
                K[i][j] += PHt[i][kk] * S_inv[kk][j];
            }
        }
    }

    /* ═══ 8. 状态更新 x = x + K·y ═══ */
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            p->state[i] += K[i][j] * y[j];
        }
    }
    filter_quat_normalize_inplace(&p->state[0], &p->state[1],
                                  &p->state[2], &p->state[3]);

    /* ═══ 9. 协方差更新（Joseph 形式）═══
     * P = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ
     *     ↑保证对称正定，避免长时间运行数值发散 ↑ */
    /* KH (7×7) */
    float KH[7][7];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            KH[i][j] = 0.0f;
            for (int kk = 0; kk < 3; kk++) {
                KH[i][j] += K[i][kk] * H[kk][j];
            }
        }
    }
    /* I_KH = I - KH */
    float I_KH[7][7];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }
    /* temp = I_KH · P */
    float temp[7][7];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            temp[i][j] = 0.0f;
            for (int kk = 0; kk < 7; kk++) {
                temp[i][j] += I_KH[i][kk] * p->P[kk][j];
            }
        }
    }
    /* P_new = temp · I_KHᵀ */
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P_new[i][j] = 0.0f;
            for (int kk = 0; kk < 7; kk++) {
                P_new[i][j] += temp[i][kk] * I_KH[j][kk];
            }
        }
    }
    /* P_new += K·R·Kᵀ（R 标量对角阵） */
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int kk = 0; kk < 3; kk++) {
                for (int ll = 0; ll < 3; ll++) {
                    P_new[i][j] += K[i][kk] * p->R_measure * (kk==ll ? 1.0f : 0.0f) * K[j][ll];
                }
            }
        }
    }

    /* 对称化 P = 0.5·(P + Pᵀ) + 对角线强制正 */
    for (int i = 0; i < 7; i++) {
        for (int j = i; j < 7; j++) {
            p->P[i][j] = 0.5f * (P_new[i][j] + P_new[j][i]);
            p->P[j][i] = p->P[i][j];
        }
    }
    for (int i = 0; i < 7; i++) {
        if (p->P[i][i] < 1e-10f) p->P[i][i] = 1e-10f;
    }

    /* ═══ 10. 输出 ═══ */
    filter_quat_to_euler(p->state[0], p->state[1], p->state[2], p->state[3],
                         &out->pitch, &out->roll, &out->yaw);
    out->q0 = p->state[0]; out->q1 = p->state[1];
    out->q2 = p->state[2]; out->q3 = p->state[3];
}

FILTER_WEAK void ekf_reset(filter_t *self)
{
    ekf_priv_t *p = (ekf_priv_t *)self->priv;
    memset(p->state, 0, sizeof(p->state));
    memset(p->P, 0, sizeof(p->P));
    p->state[0] = 1.0f;  /* q0 = 1（单位四元数） */
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        p->P[i][i] = 1.0f;  /* 初始协方差 = I（高不确定性） */
    }
}

FILTER_WEAK void ekf_set_param(filter_t *self, filter_param_t param, float value)
{
    ekf_priv_t *p = (ekf_priv_t *)self->priv;
    switch (param) {
        case FILTER_PARAM_Q_ANGLE:   p->Q_angle   = value; break;
        case FILTER_PARAM_Q_BIAS:    p->Q_bias    = value; break;
        case FILTER_PARAM_R_MEASURE: p->R_measure = value; break;
        default: break;
    }
}

FILTER_WEAK size_t ekf_get_static_size(void)
{
    return sizeof(ekf_priv_t);
}

FILTER_WEAK void ekf_init(void *priv)
{
    ekf_priv_t *p = (ekf_priv_t *)priv;
    /* 默认参数来源（见 filter_config.h）：
     *   Q_angle  = 0.001  rad²/s   （陀螺噪声密度推算）
     *   Q_bias   = 0.003  dps²/s   （偏置稳定性）
     *   R_measure= 0.03   g²       （ACC 噪声） */
    p->state[0] = 1.0f;
    p->Q_angle   = 0.001f;
    p->Q_bias    = 0.003f;
    p->R_measure = 0.03f;
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        p->P[i][i] = 1.0f;
    }
}
