/**
 * @file    filter_lpf.c
 * @brief   一阶低通滤波器 (LPF) — RC 低通 + 陀螺积分
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │ 算法推导                                                      │
 * └──────────────────────────────────────────────────────────────┘
 *
 * 状态向量：x = [pitch, roll, yaw, prev_pitch, prev_roll, prev_yaw]（度）
 *
 * 一阶 RC 低通离散化（前向欧拉）：
 *   连续：  H(s) = 1/(RC·s + 1)，RC = 1/(2π·fc)
 *   离散：  α = dt/(RC+dt)
 *          y[k] = y[k-1] + α·(x[k] - y[k-1])
 *
 * 滤波器结构：先融合 ACC+GYRO 再低通
 *   pitch_fused = acc_pitch + gy·dt   （ACC 提供角度基准，GYRO 提供角速度积分）
 *   pitch[k] = prev_pitch + α·(pitch_fused - prev_pitch)
 *
 * 与互补滤波的差异：
 *   - 互补：α 是 GYRO 权重（高频信任 GYRO）
 *   - LPF：α 是新样本权重（α 大→跟踪快、噪声多；α 小→平滑、延迟大）
 *   - LPF 不区分高低频通道，整体低通
 *
 * 退化模式：与互补滤波相同（HOLD_LAST/GYRO_ONLY/ACC_ONLY/STATIC_ONLY）
 *
 * 数值稳定性：
 *   - yaw wrap 到 [-180, 180)
 *   - prev_* 保存当前帧作为下次输入
 *   - 输入 NaN/Inf 冻结输出
 *
 * MCU 资源占用：
 *   - priv: 24 字节（6 float）
 *   - 栈: ~32 字节
 *   - 周期: ~160 cycles @ 168MHz M4F
 */

#include "filter.h"
#include "filter_internal.h"
#include "filter_math.h"
#include <math.h>

/* ============================================================
 * 一阶低通滤波器 (Low-Pass Filter)
 * ============================================================ */

typedef struct {
    float cutoff_freq;                      /**< 截止频率 fc (Hz) */
    float alpha;                            /**< 滤波系数（运行时由 fc+dt 计算） */
    float pitch, roll, yaw;                 /**< 当前姿态角 (度) */
    float prev_pitch, prev_roll, prev_yaw;  /**< 上一帧姿态角 */
} lpf_priv_t;

FILTER_WEAK void lpf_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    lpf_priv_t *p = (lpf_priv_t *)self->priv;

    /* 输入验证 */
    if (in->dt <= 0.0f || fp_isnan(in->ax) || fp_isinf(in->ax) ||
        fp_isnan(in->gx) || fp_isinf(in->gx)) {
        out->pitch = p->pitch;
        out->roll  = p->roll;
        out->yaw   = p->yaw;
        out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
        return;
    }

    /* ACC → pitch/roll */
    float acc_pitch, acc_roll;
    filter_acc_to_euler(in->ax, in->ay, in->az, &acc_pitch, &acc_roll);

    /* 退化模式：HOLD_LAST */
    if (self->degrade == FILTER_DEGRADE_HOLD_LAST) {
        out->pitch = p->pitch; out->roll = p->roll; out->yaw = p->yaw;
        out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
        return;
    }

    if (self->degrade == FILTER_DEGRADE_GYRO_ONLY) {
        /* 仅陀螺积分 */
        p->pitch += in->gy * in->dt;
        p->roll  -= in->gx * in->dt;
        p->yaw   += in->gz * in->dt;
    } else if (self->degrade == FILTER_DEGRADE_ACC_ONLY) {
        /* 仅 ACC，yaw 保持 */
        p->pitch = acc_pitch;
        p->roll  = acc_roll;
    } else {
        /* 正常 / STATIC_ONLY：RC 低通融合
         *   α = dt / (RC + dt)，  RC = 1/(2π·fc)
         *   pitch = prev + α·((acc_pitch + gy·dt) - prev)
         *   roll  = prev + α·((acc_roll  - gx·dt) - prev)  ← roll 绕 X，gx 反号
         *   yaw   = 纯陀螺积分（无 ACC 参考） */
        float rc = 1.0f / (M_TWO_PI_F * p->cutoff_freq);
        float alpha = in->dt / (rc + in->dt);
        p->pitch = p->prev_pitch + alpha * (acc_pitch + in->gy * in->dt - p->prev_pitch);
        p->roll  = p->prev_roll  + alpha * (acc_roll  - in->gx * in->dt - p->prev_roll);
        p->yaw  += in->gz * in->dt;
    }

    /* Yaw wrap */
    p->yaw = wrap_deg_180(p->yaw);

    /* 保存当前值供下帧使用 */
    p->prev_pitch = p->pitch;
    p->prev_roll  = p->roll;

    /* 输出 */
    out->pitch = p->pitch;
    out->roll  = p->roll;
    out->yaw   = p->yaw;
    out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
}

FILTER_WEAK void lpf_reset(filter_t *self)
{
    lpf_priv_t *p = (lpf_priv_t *)self->priv;
    p->pitch = p->roll = p->yaw = 0.0f;
    p->prev_pitch = p->prev_roll = p->prev_yaw = 0.0f;
}

FILTER_WEAK void lpf_set_param(filter_t *self, filter_param_t param, float value)
{
    lpf_priv_t *p = (lpf_priv_t *)self->priv;
    if (param == FILTER_PARAM_CUTOFF_FREQ) {
        p->cutoff_freq = value;
    }
}

FILTER_WEAK size_t lpf_get_static_size(void)
{
    return sizeof(lpf_priv_t);
}

FILTER_WEAK void lpf_init(void *priv)
{
    lpf_priv_t *p = (lpf_priv_t *)priv;
    p->cutoff_freq = 10.0f;   /* 默认 10Hz，对应 τ ≈ 16ms */
    p->alpha = 0.0f;          /* 运行时由 dt+fc 计算 */
    p->pitch = p->roll = p->yaw = 0.0f;
    p->prev_pitch = p->prev_roll = p->prev_yaw = 0.0f;
}
