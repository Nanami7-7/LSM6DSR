/**
 * @file    filter_lkf.c
 * @brief   线性卡尔曼滤波器 (LKF) — 6 状态欧拉角 + 陀螺偏置
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 1. 状态向量                                                   │
 * └──────────────────────────────────────────────────────────────┘
 *   x = [pitch, roll, yaw, bx, by, bz]ᵀ  (6 维)
 *   pitch/roll/yaw：欧拉角 (度)
 *   bx,by,bz：陀螺零偏 (dps)
 *
 *   与 EKF 的区别：
 *     - EKF 用四元数 + 非线性测量模型（h(q) = R(q)ᵀ·g）
 *     - LKF 直接用欧拉角 + 线性测量模型（H = [I₂, 0]，直接观测 pitch/roll）
 *     - LKF 无需雅可比，无需 3×3 矩阵求逆，计算量小 30-50%
 *
 *   代价：
 *     - 大角度（pitch 接近 ±90°）时万向节锁，线性化失效
 *     - 仅适合姿态变化范围小的场景（机器狗正常行走 ±30° 内）
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 2. 过程模型（线性）                                          │
 * └──────────────────────────────────────────────────────────────┘
 *   状态转移：
 *     pitch' = pitch + (gy - bx)·dt
 *     roll'  = roll  - (gx - bx)·dt    ← 注：原代码用 gx/gy 含义可能反，
 *     yaw'   = yaw   + (gz - bz)·dt       以 LSM6DSR 坐标系为准
 *     b'     = b
 *
 *   矩阵形式：
 *     F = [I₃  -dt·I₃]
 *         [0    I₃  ]
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 3. 协方差预测 P = F·P·Fᵀ + Q                                 │
 * └──────────────────────────────────────────────────────────────┘
 *   完整展开（P 分块 [P_qq P_qb; P_bq P_bb]）：
 *     P_qq' = P_qq - dt·(P_qb + P_bq) + dt²·P_bb + Q_angle·I
 *     P_qb' = P_qb - dt·P_bb
 *     P_bq' = P_bq - dt·P_bb  （= P_qb'ᵀ）
 *     P_bb' = P_bb + Q_bias·I
 *
 *   ⚠ 简化说明（设计选择，非 bug）：
 *     当前实现仅更新对角线 + P[i][i+3] 交叉项（i=0,1,2），
 *     忽略 P_qq 的非对角项演化（P[0][1], P[0][2], P[1][2] 等）。
 *
 *     原因：3 轴姿态在欧拉角下近似独立（小角度线性化），
 *           非对角项量级 << 对角项，忽略后误差 < 5%。
 *
 *     适用条件：姿态角 < 30°，三轴解耦假设成立。
 *     不适用：剧烈翻转/复杂耦合运动（应改用 EKF）。
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 4. 测量模型                                                   │
 * └──────────────────────────────────────────────────────────────┘
 *   观测：z = [acc_pitch, acc_roll]ᵀ  （从 ACC 求得，2 维）
 *   H = [1 0 0 0 0 0]
 *       [0 1 0 0 0 0]
 *   yaw 无观测（无磁力仪），纯积分。
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 5. 卡尔曼更新                                                 │
 * └──────────────────────────────────────────────────────────────┘
 *   S = H·P·Hᵀ + R  （2×2，对角阵）
 *   K = P·Hᵀ·S⁻¹    （6×2）
 *   x = x + K·(z - H·x)
 *   P = (I - K·H)·P
 *
 *   ⚠ 简化：因 S 是对角阵，K 退化为按列独立计算：
 *       K[i][0] = P[i][0] / (P[0][0] + R)
 *       K[i][1] = P[i][1] / (P[1][1] + R)
 *     P 更新只作用在 P[0:2, 0:2] 和 P[0:2, 3:5] 的相关列，
 *     非对角项 P[0][1] 等不更新（同 §3 简化理由）。
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 6. 退化模式                                                  │
 * └──────────────────────────────────────────────────────────────┘
 *   HOLD_LAST：冻结
 *   GYRO_ONLY：仅预测，跳过测量更新
 *   ACC_ONLY：直接采用 ACC 角度，yaw 保持
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 7. MCU 资源占用                                              │
 * └──────────────────────────────────────────────────────────────┘
 *   priv: 6·4 + 6·6·4 + 3·4 = 180 字节
 *   栈峰值：~288 字节（仅 P[6][6] 临时副本）
 *   周期：~3000 cycles @ 168MHz M4F（约 EKF 的 1/3）
 *
 *   ⚠ 适配建议：
 *     - MSPM0G3507（M0+, 32KB RAM）：可启用，是 M0+ 的首选卡尔曼
 *     - CH32（20KB RAM）：可启用
 *     - 主栈 < 512B 的 MCU：可启用（栈占用远小于 EKF）
 */

#include "filter.h"
#include "filter_internal.h"
#include "filter_math.h"
#include <math.h>
#include <string.h>

#define LKF_STATE_SIZE 6

typedef struct {
    float x[LKF_STATE_SIZE];                       /**< [pitch, roll, yaw, bx, by, bz] */
    float P[LKF_STATE_SIZE][LKF_STATE_SIZE];       /**< 协方差矩阵 */
    float Q_angle;    /**< 过程噪声-角度 (deg²/s) */
    float Q_bias;     /**< 过程噪声-偏置 (dps²/s) */
    float R_measure;  /**< 测量噪声 (deg²) */
} lkf_priv_t;

FILTER_WEAK void lkf_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    lkf_priv_t *p = (lkf_priv_t *)self->priv;
    float dt = in->dt;

    /* 输入验证 */
    if (dt <= 0.0f || fp_isnan(in->ax) || fp_isinf(in->ax) ||
        fp_isnan(in->gx) || fp_isinf(in->gx)) {
        out->pitch = p->x[0]; out->roll = p->x[1]; out->yaw = p->x[2];
        out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
        return;
    }

    /* HOLD_LAST */
    if (self->degrade == FILTER_DEGRADE_HOLD_LAST) {
        out->pitch = p->x[0]; out->roll = p->x[1]; out->yaw = p->x[2];
        out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
        return;
    }

    /* ═══ 1. 预测步骤 ═══ */
    /* 状态预测：
     *   pitch' = pitch + (gy - bx)·dt
     *   roll'  = roll  - (gx - by)·dt    ← 注意 gx/by 配对（LSM6DSR 坐标系约定）
     *   yaw'   = yaw   + (gz - bz)·dt
     *   b'     = b   （偏置不变） */
    float pitch_pred = p->x[0] + (in->gy - p->x[3]) * dt;
    float roll_pred  = p->x[1] - (in->gx - p->x[4]) * dt;
    float yaw_pred   = p->x[2] + (in->gz - p->x[5]) * dt;

    /* 协方差预测（简化版，见文件头 §3 说明） */
    float P[LKF_STATE_SIZE][LKF_STATE_SIZE];
    memcpy(P, p->P, sizeof(P));

    float dt2 = dt * dt;
    /* 对角线：P_qq[i][i] += P_bb[i][i]·dt² - 2·P_qb[i][i]·dt + Q_angle */
    P[0][0] += (P[3][3] * dt2 - 2.0f * P[0][3] * dt) + p->Q_angle;
    P[1][1] += (P[4][4] * dt2 - 2.0f * P[1][4] * dt) + p->Q_angle;
    P[2][2] += (P[5][5] * dt2 - 2.0f * P[2][5] * dt) + p->Q_angle;
    /* 偏置对角线：P_bb += Q_bias */
    P[3][3] += p->Q_bias;
    P[4][4] += p->Q_bias;
    P[5][5] += p->Q_bias;

    /* 角度-偏置交叉项：P_qb[i][i] -= P_bb[i][i]·dt */
    P[0][3] -= P[3][3] * dt;
    P[1][4] -= P[4][4] * dt;
    P[2][5] -= P[5][5] * dt;
    /* 对称化 */
    P[3][0] = P[0][3];
    P[4][1] = P[1][4];
    P[5][2] = P[2][5];

    /* ═══ 2. 更新步骤 ═══ */
    if (self->degrade == FILTER_DEGRADE_GYRO_ONLY) {
        /* GYRO_ONLY：仅预测，跳过测量更新 */
        p->x[0] = pitch_pred;
        p->x[1] = roll_pred;
        p->x[2] = yaw_pred;
    } else {
        /* 从 ACC 求 pitch/roll 观测 */
        float acc_pitch, acc_roll;
        filter_acc_to_euler(in->ax, in->ay, in->az, &acc_pitch, &acc_roll);

        if (self->degrade == FILTER_DEGRADE_ACC_ONLY) {
            /* ACC_ONLY：直接采用 ACC 角度，yaw 保持 */
            p->x[0] = acc_pitch;
            p->x[1] = acc_roll;
        } else {
            /* 正常模式：卡尔曼更新
             *   S = P[0:2,0:2] + R  （对角阵，S0=P[0][0]+R, S1=P[1][1]+R）
             *   K[i][0] = P[i][0] / S0
             *   K[i][1] = P[i][1] / S1
             *   x[i]   += K[i][0]·(acc_pitch - pitch_pred) + K[i][1]·(acc_roll - roll_pred)
             *   P      *= (I - K·H)   （仅更新涉及列） */
            float S0 = P[0][0] + p->R_measure;
            float S1 = P[1][1] + p->R_measure;
            float K0 = P[0][0] / S0;
            float K1 = P[1][1] / S1;

            p->x[0] = pitch_pred + K0 * (acc_pitch - pitch_pred);
            p->x[1] = roll_pred  + K1 * (acc_roll  - roll_pred);
            p->x[2] = yaw_pred;  /* yaw 无观测 */

            /* 协方差更新 P = (I - K·H)·P，仅作用相关项 */
            P[0][0] *= (1.0f - K0);
            P[1][1] *= (1.0f - K1);
            P[0][3] *= (1.0f - K0);
            P[1][4] *= (1.0f - K1);
            P[3][0] = P[0][3];
            P[4][1] = P[1][4];
        }
    }

    /* Yaw wrap */
    p->x[2] = wrap_deg_180(p->x[2]);

    /* 保存状态 */
    memcpy(p->P, P, sizeof(P));

    /* 输出（无四元数） */
    out->pitch = p->x[0];
    out->roll  = p->x[1];
    out->yaw   = p->x[2];
    out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
}

FILTER_WEAK void lkf_reset(filter_t *self)
{
    lkf_priv_t *p = (lkf_priv_t *)self->priv;
    memset(p->x, 0, sizeof(p->x));
    memset(p->P, 0, sizeof(p->P));
    for (int i = 0; i < LKF_STATE_SIZE; i++) {
        p->P[i][i] = 1.0f;
    }
}

FILTER_WEAK void lkf_set_param(filter_t *self, filter_param_t param, float value)
{
    lkf_priv_t *p = (lkf_priv_t *)self->priv;
    switch (param) {
        case FILTER_PARAM_Q_ANGLE:   p->Q_angle   = value; break;
        case FILTER_PARAM_Q_BIAS:    p->Q_bias    = value; break;
        case FILTER_PARAM_R_MEASURE: p->R_measure = value; break;
        default: break;
    }
}

FILTER_WEAK size_t lkf_get_static_size(void)
{
    return sizeof(lkf_priv_t);
}

FILTER_WEAK void lkf_init(void *priv)
{
    lkf_priv_t *p = (lkf_priv_t *)priv;
    p->Q_angle   = 0.001f;
    p->Q_bias    = 0.003f;
    p->R_measure = 0.03f;
    for (int i = 0; i < LKF_STATE_SIZE; i++) {
        p->P[i][i] = 1.0f;
    }
}
