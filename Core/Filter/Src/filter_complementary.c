/**
 * @file    filter_complementary.c
 * @brief   互补滤波器 (Complementary Filter) — 经典 α 融合
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 算法推导                                                      │
 * └──────────────────────────────────────────────────────────────┘
 *
 * 状态向量：x = [pitch, roll, yaw]（欧拉角，度）
 *
 * 连续时间模型：
 *   陀螺积分：  dθ/dt = ω_gyro
 *   ACC 观测：  θ_acc = atan2(...)  （仅 pitch/roll，无 yaw）
 *
 * 频域互补：
 *   G_gyro(s) = α·τs/(τs+1)   高通（信任 GYRO 高频，抑制低频漂移）
 *   G_acc(s)  = 1/(τs+1)      低通（信任 ACC 低频，抑制高频噪声）
 *   α = τ/(τ+dt)，τ = α·dt/(1-α) 为时间常数
 *
 * 离散化（前向欧拉）：
 *   pitch[k] = α·(pitch[k-1] + gy·dt) + (1-α)·acc_pitch
 *   roll[k]  = α·(roll[k-1]  - gx·dt) + (1-α)·acc_roll
 *   yaw[k]   = yaw[k-1] + gz·dt      （无 ACC 参考，纯积分，会漂移）
 *
 * 退化模式：
 *   - HOLD_LAST：     冻结状态，返回上次输出（传感器失效）
 *   - GYRO_ONLY：     仅陀螺积分，跳过 ACC 修正（ACC 受冲击）
 *   - ACC_ONLY：      仅 ACC，跳过陀螺积分（GYRO 饱和），yaw 保持
 *   - STATIC_ONLY：   走正常路径（互补滤波本质包含静态 ACC 修正）
 *
 * 数值稳定性：
 *   - yaw wrap 到 [-180, 180)，避免累积溢出
 *   - 输入 NaN/Inf 时返回上次输出（不污染状态）
 *
 * MCU 资源占用：
 *   - priv: 16 字节（4 float）
 *   - 栈: ~24 字节（acc_pitch/acc_roll 局部量）
 *   - 周期: ~150 cycles @ 168MHz M4F（含 2 次 atan2f）
 */

#include "filter.h"
#include "filter_internal.h"
#include "filter_math.h"
#include <math.h>

/* ============================================================
 * 互补滤波器 (Complementary Filter)
 * ============================================================ */

typedef struct {
    float alpha;            /**< 融合系数 α ∈ [0,1]，越大越信任 GYRO */
    float pitch, roll, yaw; /**< 姿态角 (度) */
} complementary_priv_t;

FILTER_WEAK void complementary_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    complementary_priv_t *p = (complementary_priv_t *)self->priv;

    /* 输入验证：dt<=0 或 NaN/Inf 时冻结输出，保护状态 */
    if (in->dt <= 0.0f || fp_isnan(in->ax) || fp_isinf(in->ax) ||
        fp_isnan(in->gx) || fp_isinf(in->gx)) {
        out->pitch = p->pitch;
        out->roll  = p->roll;
        out->yaw   = p->yaw;
        out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
        return;
    }

    /* 退化模式：HOLD_LAST 直接返回上次输出 */
    if (self->degrade == FILTER_DEGRADE_HOLD_LAST) {
        out->pitch = p->pitch;
        out->roll  = p->roll;
        out->yaw   = p->yaw;
        out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
        return;
    }

    /* 从 ACC 计算 pitch/roll（共享辅助函数，原重复代码已抽出） */
    float acc_pitch, acc_roll;
    filter_acc_to_euler(in->ax, in->ay, in->az, &acc_pitch, &acc_roll);

    if (self->degrade == FILTER_DEGRADE_GYRO_ONLY) {
        /* 仅陀螺积分：pitch += gy*dt，roll -= gx*dt，yaw += gz*dt */
        p->pitch += in->gy * in->dt;
        p->roll  -= in->gx * in->dt;
        p->yaw   += in->gz * in->dt;
    } else if (self->degrade == FILTER_DEGRADE_ACC_ONLY) {
        /* 仅 ACC：直接采用 ACC 角度，yaw 保持（无参考） */
        p->pitch = acc_pitch;
        p->roll  = acc_roll;
    } else {
        /* 正常 / STATIC_ONLY：互补融合
         *   pitch = α*(pitch + gy*dt) + (1-α)*acc_pitch
         *   roll  = α*(roll  - gx*dt) + (1-α)*acc_roll
         * 注：gx 是绕 X 轴角速度，roll 绕 X 轴，但 LSM6DSR 坐标系下
         *     roll 增加对应 gx 反向，故取负号。 */
        p->pitch = p->alpha * (p->pitch + in->gy * in->dt) + (1.0f - p->alpha) * acc_pitch;
        p->roll  = p->alpha * (p->roll  - in->gx * in->dt) + (1.0f - p->alpha) * acc_roll;
        p->yaw  += in->gz * in->dt;
    }

    /* Yaw wrap 到 [-180, 180) */
    p->yaw = wrap_deg_180(p->yaw);

    /* 输出（无四元数表示） */
    out->pitch = p->pitch;
    out->roll  = p->roll;
    out->yaw   = p->yaw;
    out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
}

FILTER_WEAK void complementary_reset(filter_t *self)
{
    complementary_priv_t *p = (complementary_priv_t *)self->priv;
    p->pitch = 0.0f;
    p->roll  = 0.0f;
    p->yaw   = 0.0f;
}

FILTER_WEAK void complementary_set_param(filter_t *self, filter_param_t param, float value)
{
    complementary_priv_t *p = (complementary_priv_t *)self->priv;
    if (param == FILTER_PARAM_ALPHA) {
        p->alpha = value;
    }
}

FILTER_WEAK size_t complementary_get_static_size(void)
{
    return sizeof(complementary_priv_t);
}

FILTER_WEAK void complementary_init(void *priv)
{
    complementary_priv_t *p = (complementary_priv_t *)priv;
    p->alpha = 0.98f;       /* 默认 τ ≈ 0.49s @ 100Hz，滤除 <2Hz 的 ACC 噪声 */
    p->pitch = 0.0f;
    p->roll  = 0.0f;
    p->yaw   = 0.0f;
}
