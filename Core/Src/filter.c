/**
 * @file    filter.c
 * @brief   滤波器统一接口实现
 *
 * 实现：
 *   - 互补滤波器 (Complementary Filter)
 *   - 一阶低通滤波器 (LPF)
 *   - 扩展卡尔曼滤波器 (EKF)
 *   - Mahony 滤波器
 *   - Madgwick 滤波器
 *
 * 设计理念：
 *   - 0成本抽象：函数指针调用
 *   - 可读性：每个滤波器独立实现
 *   - 内存安全：create/destroy 配对
 */

#include "filter.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ============================================================
 * 1. 互补滤波器 (Complementary Filter)
 * ============================================================ */

typedef struct {
    float alpha;        /**< 融合系数 */
    float pitch, roll, yaw;  /**< 姿态角 */
} complementary_priv_t;

static void complementary_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    complementary_priv_t *p = (complementary_priv_t *)self->priv;

    /* 输入验证 */
    if (in->dt <= 0.0f || isnan(in->ax) || isinf(in->ax) ||
        isnan(in->gx) || isinf(in->gx)) {
        out->pitch = p->pitch;
        out->roll  = p->roll;
        out->yaw   = p->yaw;
        out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
        return;
    }

    /* 退化模式检查 */
    if (self->degrade == FILTER_DEGRADE_HOLD_LAST) {
        /* HOLD_LAST: 直接返回上次输出，不更新状态 */
        out->pitch = p->pitch;
        out->roll  = p->roll;
        out->yaw   = p->yaw;
        out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
        return;
    }

    /* 计算 ACC 姿态角 */
    float acc_pitch = atan2f(-in->ax, sqrtf(in->ay * in->ay + in->az * in->az)) * 180.0f / M_PI;
    float acc_roll  = atan2f( in->ay, sqrtf(in->ax * in->ax + in->az * in->az)) * 180.0f / M_PI;

    /* 退化模式：陀螺仪仅积分（跳过ACC修正） */
    if (self->degrade == FILTER_DEGRADE_GYRO_ONLY) {
        p->pitch += in->gy * in->dt;
        p->roll  -= in->gx * in->dt;
        p->yaw   += in->gz * in->dt;
    } else if (self->degrade == FILTER_DEGRADE_ACC_ONLY) {
        /* 退化模式：仅加速度计（跳过GYRO积分） */
        p->pitch = acc_pitch;
        p->roll  = acc_roll;
    } else {
        /* 正常模式：互补滤波融合 */
        p->pitch = p->alpha * (p->pitch + in->gy * in->dt) + (1.0f - p->alpha) * acc_pitch;
        p->roll  = p->alpha * (p->roll  - in->gx * in->dt) + (1.0f - p->alpha) * acc_roll;
        p->yaw  += in->gz * in->dt;
    }

    /* Yaw wrap-around */
    while (p->yaw > 180.0f)  p->yaw -= 360.0f;
    while (p->yaw < -180.0f) p->yaw += 360.0f;

    /* 输出 */
    out->pitch = p->pitch;
    out->roll  = p->roll;
    out->yaw   = p->yaw;
    out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
}

static void complementary_reset(filter_t *self)
{
    complementary_priv_t *p = (complementary_priv_t *)self->priv;
    p->pitch = p->roll = p->yaw = 0.0f;
}

static void complementary_set_param(filter_t *self, filter_param_t param, float value)
{
    complementary_priv_t *p = (complementary_priv_t *)self->priv;
    if (param == FILTER_PARAM_ALPHA) {
        p->alpha = value;
    }
}

static void complementary_destroy(filter_t *self)
{
    free(self->priv);
    free(self);
}

filter_t* filter_create_complementary(float alpha)
{
    filter_t *f = (filter_t *)malloc(sizeof(filter_t));
    if (!f) return NULL;

    complementary_priv_t *p = (complementary_priv_t *)malloc(sizeof(complementary_priv_t));
    if (!p) { free(f); return NULL; }

    p->alpha = alpha;
    p->pitch = p->roll = p->yaw = 0.0f;

    f->update    = complementary_update;
    f->reset     = complementary_reset;
    f->set_param = complementary_set_param;
    f->destroy   = complementary_destroy;
    f->type      = FILTER_TYPE_COMPLEMENTARY;
    f->degrade   = FILTER_DEGRADE_NONE;
    f->priv      = p;

    return f;
}

/* ============================================================
 * 2. 一阶低通滤波器 (Low-Pass Filter)
 * ============================================================ */

typedef struct {
    float cutoff_freq;  /**< 截止频率 (Hz) */
    float alpha;        /**< 滤波系数 */
    float pitch, roll, yaw;  /**< 姿态角 */
    float prev_pitch, prev_roll, prev_yaw;  /**< 上一帧值 */
} lpf_priv_t;

static void lpf_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    lpf_priv_t *p = (lpf_priv_t *)self->priv;

    /* 输入验证 */
    if (in->dt <= 0.0f || isnan(in->ax) || isinf(in->ax) ||
        isnan(in->gx) || isinf(in->gx)) {
        out->pitch = p->pitch;
        out->roll  = p->roll;
        out->yaw   = p->yaw;
        out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
        return;
    }
    /* 计算 ACC 姿态角 */
    float acc_pitch = atan2f(-in->ax, sqrtf(in->ay * in->ay + in->az * in->az)) * 180.0f / M_PI;
    float acc_roll  = atan2f( in->ay, sqrtf(in->ax * in->ax + in->az * in->az)) * 180.0f / M_PI;

    /* 退化模式检查 */
    if (self->degrade == FILTER_DEGRADE_HOLD_LAST) {
        out->pitch = p->pitch; out->roll = p->roll; out->yaw = p->yaw;
        out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
        return;
    }

    if (self->degrade == FILTER_DEGRADE_GYRO_ONLY) {
        /* 仅陀螺仪积分，跳过ACC修正 */
        p->pitch += in->gy * in->dt;
        p->roll  -= in->gx * in->dt;
        p->yaw   += in->gz * in->dt;
    } else if (self->degrade == FILTER_DEGRADE_ACC_ONLY) {
        /* 仅加速度计，跳过GYRO积分，yaw保持不变 */
        p->pitch = acc_pitch;
        p->roll  = acc_roll;
    } else {
        /* 正常模式 + STATIC_ONLY（需外部静止检测） */
        float rc = 1.0f / (2.0f * M_PI * p->cutoff_freq);
        float alpha = in->dt / (rc + in->dt);
        p->pitch = p->prev_pitch + alpha * (acc_pitch + in->gy * in->dt - p->prev_pitch);
        p->roll  = p->prev_roll  + alpha * (acc_roll  - in->gx * in->dt - p->prev_roll);
        p->yaw  += in->gz * in->dt;
    }
    /* Yaw wrap-around */
    while (p->yaw > 180.0f)  p->yaw -= 360.0f;
    while (p->yaw < -180.0f) p->yaw += 360.0f;

    /* 保存当前值 */
    p->prev_pitch = p->pitch;
    p->prev_roll  = p->roll;

    /* 输出 */
    out->pitch = p->pitch;
    out->roll  = p->roll;
    out->yaw   = p->yaw;
    out->q0 = out->q1 = out->q2 = out->q3 = 0.0f;
}

static void lpf_reset(filter_t *self)
{
    lpf_priv_t *p = (lpf_priv_t *)self->priv;
    p->pitch = p->roll = p->yaw = 0.0f;
    p->prev_pitch = p->prev_roll = p->prev_yaw = 0.0f;
}

static void lpf_set_param(filter_t *self, filter_param_t param, float value)
{
    lpf_priv_t *p = (lpf_priv_t *)self->priv;
    if (param == FILTER_PARAM_CUTOFF_FREQ) {
        p->cutoff_freq = value;
    }
}

static void lpf_destroy(filter_t *self)
{
    free(self->priv);
    free(self);
}

filter_t* filter_create_lpf(float cutoff_freq)
{
    filter_t *f = (filter_t *)malloc(sizeof(filter_t));
    if (!f) return NULL;

    lpf_priv_t *p = (lpf_priv_t *)malloc(sizeof(lpf_priv_t));
    if (!p) { free(f); return NULL; }

    p->cutoff_freq = cutoff_freq;
    p->alpha = 0.0f;
    p->pitch = p->roll = p->yaw = 0.0f;
    p->prev_pitch = p->prev_roll = p->prev_yaw = 0.0f;

    f->update    = lpf_update;
    f->reset     = lpf_reset;
    f->set_param = lpf_set_param;
    f->destroy   = lpf_destroy;
    f->type      = FILTER_TYPE_LPF;
    f->degrade   = FILTER_DEGRADE_NONE;
    f->priv      = p;

    return f;
}

/* ============================================================
 * 3. 扩展卡尔曼滤波器 (EKF)
 * ============================================================ */

/* 状态向量: [q0, q1, q2, q3, bias_x, bias_y, bias_z] */
#define EKF_STATE_SIZE 7
#define EKF_MEAS_SIZE 3

typedef struct {
    float state[EKF_STATE_SIZE];        /**< 状态向量 */
    float P[EKF_STATE_SIZE][EKF_STATE_SIZE];  /**< 协方差矩阵 */
    float Q_angle;                      /**< 过程噪声-角度 */
    float Q_bias;                       /**< 过程噪声-偏置 */
    float R_measure;                    /**< 测量噪声 */
} ekf_priv_t;

/* 矩阵操作辅助函数 */


static void ekf_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    ekf_priv_t *p = (ekf_priv_t *)self->priv;
    float dt = in->dt;

    /* 输入验证 */
    if (dt <= 0.0f || isnan(in->ax) || isinf(in->ax) ||
        isnan(in->gx) || isinf(in->gx)) {
        /* 输出上次状态 */
        float q0 = p->state[0], q1 = p->state[1], q2 = p->state[2], q3 = p->state[3];
        out->q0 = q0; out->q1 = q1; out->q2 = q2; out->q3 = q3;
        out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (q1*q3 - q0*q2)))) * 180.0f / M_PI;
        out->roll  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * 180.0f / M_PI;
        out->yaw   = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 180.0f / M_PI;
        return;
    }

    /* 退化模式检查 */
    if (self->degrade == FILTER_DEGRADE_HOLD_LAST) {
        float q0 = p->state[0], q1 = p->state[1], q2 = p->state[2], q3 = p->state[3];
        out->q0 = q0; out->q1 = q1; out->q2 = q2; out->q3 = q3;
        out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (q1*q3 - q0*q2)))) * 180.0f / M_PI;
        out->roll  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * 180.0f / M_PI;
        out->yaw   = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 180.0f / M_PI;
        return;
    }

    if (self->degrade == FILTER_DEGRADE_ACC_ONLY) {
        /* 仅加速度计：从ACC计算姿态角并重置四元数 */
        float ax_norm = sqrtf(in->ax*in->ax + in->ay*in->ay + in->az*in->az);
        if (ax_norm > 0.01f) {
            float ax_n = in->ax/ax_norm, ay_n = in->ay/ax_norm, az_n = in->az/ax_norm;
            float acc_pitch_rad = atan2f(-ax_n, sqrtf(ay_n*ay_n + az_n*az_n));
            float acc_roll_rad  = atan2f( ay_n, sqrtf(ax_n*ax_n + az_n*az_n));
            float cp = cosf(acc_pitch_rad * 0.5f);
            float sp = sinf(acc_pitch_rad * 0.5f);
            float cr = cosf(acc_roll_rad * 0.5f);
            float sr = sinf(acc_roll_rad * 0.5f);
            p->state[0] = cp * cr;
            p->state[1] = cp * sr;
            p->state[2] = sp * cr;
            p->state[3] = -sp * sr;  /* yaw=0 */
            /* 偏置保持不变 */
        }
        float q0l = p->state[0], q1l = p->state[1], q2l = p->state[2], q3l = p->state[3];
        out->q0 = q0l; out->q1 = q1l; out->q2 = q2l; out->q3 = q3l;
        out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (q1l*q3l - q0l*q2l)))) * 180.0f / M_PI;
        out->roll  = atan2f(2.0f * (q0l*q1l + q2l*q3l), 1.0f - 2.0f * (q1l*q1l + q2l*q2l)) * 180.0f / M_PI;
        out->yaw   = atan2f(2.0f * (q0l*q3l + q1l*q2l), 1.0f - 2.0f * (q2l*q2l + q3l*q3l)) * 180.0f / M_PI;
        return;
    }

    /* 提取状态 */
    float q0 = p->state[0], q1 = p->state[1], q2 = p->state[2], q3 = p->state[3];
    float bx = p->state[4], by = p->state[5], bz = p->state[6];

    /* 偏置补偿并转换为 rad/s */
    float gx = (in->gx - bx) * M_PI / 180.0f;
    float gy = (in->gy - by) * M_PI / 180.0f;
    float gz = (in->gz - bz) * M_PI / 180.0f;

    /* ===== 1. 状态预测 ===== */
    float q0_dot = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float q1_dot = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float q2_dot = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float q3_dot = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    p->state[0] += q0_dot * dt;
    p->state[1] += q1_dot * dt;
    p->state[2] += q2_dot * dt;
    p->state[3] += q3_dot * dt;

    /* 四元数归一化 */
    float norm = sqrtf(p->state[0]*p->state[0] + p->state[1]*p->state[1] +
                       p->state[2]*p->state[2] + p->state[3]*p->state[3]);
    if (norm > 1e-10f) {
        p->state[0] /= norm;
        p->state[1] /= norm;
        p->state[2] /= norm;
        p->state[3] /= norm;
    }

    /* ===== 2. 协方差预测 P = F * P * F^T + Q ===== */
    /* F 矩阵 (7x7)：四元数部分有非对角元素 */
    /* F_qq = I + 0.5*dt*Omega(gx,gy,gz) */
    /* F_qb = -0.5*dt*Xi(q) */
    /* F_bq = 0, F_bb = I */

    /* 临时存储 P 的副本用于计算 F*P*F^T */
    float P_new[7][7];
    memset(P_new, 0, sizeof(P_new));

    /* F 矩阵元素（预计算） */
    float dt_half = 0.5f * dt;
    /* F_qq = I + 0.5*dt*Omega */
    /* Omega = [0, -gx, -gy, -gz; gx, 0, gz, -gy; gy, -gz, 0, gx; gz, gy, -gx, 0] */
    float F00 = 1.0f, F01 = -dt_half*gx, F02 = -dt_half*gy, F03 = -dt_half*gz;
    float F10 = dt_half*gx, F11 = 1.0f, F12 = dt_half*gz, F13 = -dt_half*gy;
    float F20 = dt_half*gy, F21 = -dt_half*gz, F22 = 1.0f, F23 = dt_half*gx;
    float F30 = dt_half*gz, F31 = dt_half*gy, F32 = -dt_half*gx, F33 = 1.0f;
    /* F_qb = -0.5*dt*Xi(q)，但实际计算中简化为 0（偏置影响小） */

    /* 计算 F*P 的四元数部分 (4x7) */
    float FP[4][7];
    for (int j = 0; j < 7; j++) {
        FP[0][j] = F00*p->P[0][j] + F01*p->P[1][j] + F02*p->P[2][j] + F03*p->P[3][j];
        FP[1][j] = F10*p->P[0][j] + F11*p->P[1][j] + F12*p->P[2][j] + F13*p->P[3][j];
        FP[2][j] = F20*p->P[0][j] + F21*p->P[1][j] + F22*p->P[2][j] + F23*p->P[3][j];
        FP[3][j] = F30*p->P[0][j] + F31*p->P[1][j] + F32*p->P[2][j] + F33*p->P[3][j];
    }
    /* 偏置部分：P[4:7][:] 不变（F_bb = I, F_bq = 0） */

    /* 计算 (F*P)*F^T 的四元数部分 (4x4) */
    for (int i = 0; i < 4; i++) {
        P_new[i][0] = FP[i][0]*F00 + FP[i][1]*F01 + FP[i][2]*F02 + FP[i][3]*F03;
        P_new[i][1] = FP[i][0]*F10 + FP[i][1]*F11 + FP[i][2]*F12 + FP[i][3]*F13;
        P_new[i][2] = FP[i][0]*F20 + FP[i][1]*F21 + FP[i][2]*F22 + FP[i][3]*F23;
        P_new[i][3] = FP[i][0]*F30 + FP[i][1]*F31 + FP[i][2]*F32 + FP[i][3]*F33;
    }
    /* 偏置交叉项：P_new[0:4][4:7] = FP[0:4][4:7] * I = FP[0:4][4:7] */
    for (int i = 0; i < 4; i++) {
        for (int j = 4; j < 7; j++) {
            P_new[i][j] = FP[i][j];
        }
    }
    /* 偏置自项：P_new[4:7][4:7] = P[4:7][4:7] */
    for (int i = 4; i < 7; i++) {
        for (int j = 4; j < 7; j++) {
            P_new[i][j] = p->P[i][j];
        }
    }
    /* 偏置-四元数交叉项：P_new[4:7][0:4] = P[4:7][0:4] * F_qq^T */
    /* P_new[i][j] = sum_{k=0}^{3} P[i][k] * F_qq[j][k] */
    for (int i = 4; i < 7; i++) {
        P_new[i][0] = p->P[i][0]*F00 + p->P[i][1]*F01 + p->P[i][2]*F02 + p->P[i][3]*F03;
        P_new[i][1] = p->P[i][0]*F10 + p->P[i][1]*F11 + p->P[i][2]*F12 + p->P[i][3]*F13;
        P_new[i][2] = p->P[i][0]*F20 + p->P[i][1]*F21 + p->P[i][2]*F22 + p->P[i][3]*F23;
        P_new[i][3] = p->P[i][0]*F30 + p->P[i][1]*F31 + p->P[i][2]*F32 + p->P[i][3]*F33;
    }

    /* 加过程噪声 Q */
    for (int i = 0; i < 4; i++) {
        P_new[i][i] += p->Q_angle * dt;
    }
    for (int i = 4; i < 7; i++) {
        P_new[i][i] += p->Q_bias * dt;
    }

    /* 复制回 P */
    memcpy(p->P, P_new, sizeof(P_new));

    /* 退化模式：GYRO_ONLY/STATIC_ONLY - 跳过测量更新 */
    if (self->degrade == FILTER_DEGRADE_GYRO_ONLY ||
        self->degrade == FILTER_DEGRADE_STATIC_ONLY) {
        q0 = p->state[0]; q1 = p->state[1]; q2 = p->state[2]; q3 = p->state[3];
        out->q0 = q0; out->q1 = q1; out->q2 = q2; out->q3 = q3;
        out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (q1*q3 - q0*q2)))) * 180.0f / M_PI;
        out->roll  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * 180.0f / M_PI;
        out->yaw   = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 180.0f / M_PI;
        return;
    }
    /* ===== 3. 测量更新 ===== */
    /* 加速度计归一化 */
    float ax = in->ax, ay = in->ay, az = in->az;
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    /* 跳过异常读数 */
    if (acc_norm < 0.01f || acc_norm > 20.0f) {
        /* 只做预测，不做测量更新 */
        q0 = p->state[0]; q1 = p->state[1]; q2 = p->state[2]; q3 = p->state[3];
        out->q0 = q0; out->q1 = q1; out->q2 = q2; out->q3 = q3;
        out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (q1*q3 - q0*q2)))) * 180.0f / M_PI;
        out->roll  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * 180.0f / M_PI;
        out->yaw   = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 180.0f / M_PI;
        return;
    }
    ax /= acc_norm; ay /= acc_norm; az /= acc_norm;

    /* 预测的重力方向 h(q) */
    float hx = 2.0f * (p->state[1]*p->state[3] - p->state[0]*p->state[2]);
    float hy = 2.0f * (p->state[0]*p->state[1] + p->state[2]*p->state[3]);
    float hz = 1.0f - 2.0f * (p->state[1]*p->state[1] + p->state[2]*p->state[2]);

    /* 测量残差 y = z - h(x) */
    float y[3] = { ax - hx, ay - hy, az - hz };

    /* ===== 4. 计算 H 矩阵 (3x7) ===== */
    /* h(q) = [2(q1*q3 - q0*q2), 2(q0*q1 + q2*q3), 1 - 2(q1^2 + q2^2)] */
    /* dh/dq = [[-2q2, 2q3, -2q0, 2q1], */
    /*          [ 2q1, 2q0,  2q3, 2q2], */
    /*          [   0, -4q1, -4q2,   0]] */
    /* dh/db = 0 (3x3) */
    q0 = p->state[0]; q1 = p->state[1]; q2 = p->state[2]; q3 = p->state[3];
    float H[3][7];
    H[0][0] = -2.0f*q2;  H[0][1] = 2.0f*q3;   H[0][2] = -2.0f*q0;  H[0][3] = 2.0f*q1;
    H[0][4] = 0.0f;      H[0][5] = 0.0f;      H[0][6] = 0.0f;
    H[1][0] = 2.0f*q1;   H[1][1] = 2.0f*q0;   H[1][2] = 2.0f*q3;   H[1][3] = 2.0f*q2;
    H[1][4] = 0.0f;      H[1][5] = 0.0f;      H[1][6] = 0.0f;
    H[2][0] = 0.0f;      H[2][1] = -4.0f*q1;  H[2][2] = -4.0f*q2;  H[2][3] = 0.0f;
    H[2][4] = 0.0f;      H[2][5] = 0.0f;      H[2][6] = 0.0f;

    /* ===== 5. 计算 S = H * P * H^T + R (3x3) ===== */
    /* 先计算 P * H^T (7x3) */
    float PHt[7][3];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            PHt[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                PHt[i][j] += p->P[i][k] * H[j][k];
            }
        }
    }

    /* S = H * (P * H^T) + R */
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

    /* ===== 6. 计算 S 的逆 (3x3 解析公式) ===== */
    float a = S[0][0], b = S[0][1], c = S[0][2];
    float d = S[1][0], e = S[1][1], f = S[1][2];
    float g = S[2][0], h = S[2][1], k = S[2][2];
    float det = a*(e*k - f*h) - b*(d*k - f*g) + c*(d*h - e*g);
    if (fabsf(det) < 1e-10f) det = 1e-10f;  /* 防止除零 */
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

    /* ===== 7. 卡尔曼增益 K = P * H^T * S^-1 (7x3) ===== */
    float K[7][3];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0.0f;
            for (int kk = 0; kk < 3; kk++) {
                K[i][j] += PHt[i][kk] * S_inv[kk][j];
            }
        }
    }

    /* ===== 8. 状态更新 x = x + K * y ===== */
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            p->state[i] += K[i][j] * y[j];
        }
    }

    /* 四元数归一化 */
    norm = sqrtf(p->state[0]*p->state[0] + p->state[1]*p->state[1] +
                 p->state[2]*p->state[2] + p->state[3]*p->state[3]);
    if (norm > 1e-10f) {
        p->state[0] /= norm;
        p->state[1] /= norm;
        p->state[2] /= norm;
        p->state[3] /= norm;
    }

    /* ===== 9. 协方差更新 P = (I - K*H) * P ===== */
    /* 计算 K*H (7x7) */
    float KH[7][7];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            KH[i][j] = 0.0f;
            for (int kk = 0; kk < 3; kk++) {
                KH[i][j] += K[i][kk] * H[kk][j];
            }
        }
    }

    /* P = (I - K*H) * P，使用 Joseph 形式保证数值稳定 */
    /* P = (I-KH) * P * (I-KH)^T + K*R*K^T */
    float I_KH[7][7];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }

    /* 临时矩阵存储 */
    float temp[7][7];
    /* temp = I_KH * P */
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            temp[i][j] = 0.0f;
            for (int kk = 0; kk < 7; kk++) {
                temp[i][j] += I_KH[i][kk] * p->P[kk][j];
            }
        }
    }
    /* P_new = temp * I_KH^T */
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P_new[i][j] = 0.0f;
            for (int kk = 0; kk < 7; kk++) {
                P_new[i][j] += temp[i][kk] * I_KH[j][kk];  /* I_KH^T */
            }
        }
    }
    /* 加 K*R*K^T */
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int kk = 0; kk < 3; kk++) {
                for (int ll = 0; ll < 3; ll++) {
                    P_new[i][j] += K[i][kk] * p->R_measure * (kk==ll ? 1.0f : 0.0f) * K[j][ll];
                }
            }
        }
    }

    /* 确保协方差矩阵对称且正定 */
    for (int i = 0; i < 7; i++) {
        for (int j = i; j < 7; j++) {
            p->P[i][j] = 0.5f * (P_new[i][j] + P_new[j][i]);
            p->P[j][i] = p->P[i][j];
        }
    }
    /* 对角线强制为正 */
    for (int i = 0; i < 7; i++) {
        if (p->P[i][i] < 1e-10f) p->P[i][i] = 1e-10f;
    }

    /* ===== 10. 输出 ===== */
    q0 = p->state[0]; q1 = p->state[1]; q2 = p->state[2]; q3 = p->state[3];
    out->q0 = q0; out->q1 = q1; out->q2 = q2; out->q3 = q3;
    out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (q1*q3 - q0*q2)))) * 180.0f / M_PI;
    out->roll  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * 180.0f / M_PI;
    out->yaw   = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 180.0f / M_PI;
}

static void ekf_reset(filter_t *self)
{
    ekf_priv_t *p = (ekf_priv_t *)self->priv;
    memset(p->state, 0, sizeof(p->state));
    memset(p->P, 0, sizeof(p->P));
    p->state[0] = 1.0f;  /* q0 = 1 */
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        p->P[i][i] = 1.0f;
    }
}

static void ekf_set_param(filter_t *self, filter_param_t param, float value)
{
    ekf_priv_t *p = (ekf_priv_t *)self->priv;
    switch (param) {
        case FILTER_PARAM_Q_ANGLE:  p->Q_angle  = value; break;
        case FILTER_PARAM_Q_BIAS:   p->Q_bias   = value; break;
        case FILTER_PARAM_R_MEASURE: p->R_measure = value; break;
        default: break;
    }
}

static void ekf_destroy(filter_t *self)
{
    free(self->priv);
    free(self);
}

filter_t* filter_create_ekf(float q_angle, float q_bias, float r_measure)
{
    filter_t *f = (filter_t *)malloc(sizeof(filter_t));
    if (!f) return NULL;

    ekf_priv_t *p = (ekf_priv_t *)malloc(sizeof(ekf_priv_t));
    if (!p) { free(f); return NULL; }

    p->Q_angle = q_angle;
    p->Q_bias = q_bias;
    p->R_measure = r_measure;

    /* 初始化状态 */
    memset(p->state, 0, sizeof(p->state));
    p->state[0] = 1.0f;  /* q0 = 1 */

    /* 初始化协方差矩阵 */
    memset(p->P, 0, sizeof(p->P));
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        p->P[i][i] = 1.0f;
    }

    f->update    = ekf_update;
    f->reset     = ekf_reset;
    f->set_param = ekf_set_param;
    f->destroy   = ekf_destroy;
    f->type      = FILTER_TYPE_EKF;
    f->degrade   = FILTER_DEGRADE_NONE;
    f->priv      = p;

    return f;
}

/* ============================================================
 * 4. Mahony 滤波器
 * ============================================================ */

typedef struct {
    float kp;           /**< 比例增益 */
    float ki;           /**< 积分增益 */
    float q0, q1, q2, q3;  /**< 四元数 */
    float ix, iy, iz;   /**< 积分误差 */
} mahony_priv_t;

static void mahony_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    mahony_priv_t *p = (mahony_priv_t *)self->priv;
    float dt = in->dt;

    /* 输入验证 */
    if (dt <= 0.0f || isnan(in->ax) || isinf(in->ax) ||
        isnan(in->gx) || isinf(in->gx)) {
        out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
        out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (p->q1*p->q3 - p->q0*p->q2)))) * 180.0f / M_PI;
        out->roll  = atan2f(2.0f * (p->q0*p->q1 + p->q2*p->q3), 1.0f - 2.0f * (p->q1*p->q1 + p->q2*p->q2)) * 180.0f / M_PI;
        out->yaw   = atan2f(2.0f * (p->q0*p->q3 + p->q1*p->q2), 1.0f - 2.0f * (p->q2*p->q2 + p->q3*p->q3)) * 180.0f / M_PI;
        return;
    }
    /* 退化模式检查 */
    if (self->degrade == FILTER_DEGRADE_HOLD_LAST) {
        out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
        out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (p->q1*p->q3 - p->q0*p->q2)))) * 180.0f / M_PI;
        out->roll  = atan2f(2.0f * (p->q0*p->q1 + p->q2*p->q3), 1.0f - 2.0f * (p->q1*p->q1 + p->q2*p->q2)) * 180.0f / M_PI;
        out->yaw   = atan2f(2.0f * (p->q0*p->q3 + p->q1*p->q2), 1.0f - 2.0f * (p->q2*p->q2 + p->q3*p->q3)) * 180.0f / M_PI;
        return;
    }

    if (self->degrade == FILTER_DEGRADE_GYRO_ONLY) {
        /* 仅陀螺仪：跳过ACC修正，只做四元数积分 */
        float gx = in->gx * M_PI / 180.0f;
        float gy = in->gy * M_PI / 180.0f;
        float gz = in->gz * M_PI / 180.0f;
        float q0 = p->q0, q1 = p->q1, q2 = p->q2, q3 = p->q3;
        p->q0 += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
        p->q1 += 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * dt;
        p->q2 += 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * dt;
        p->q3 += 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * dt;
        float norm = sqrtf(p->q0*p->q0 + p->q1*p->q1 + p->q2*p->q2 + p->q3*p->q3);
        if (norm > 0.0f) {
            p->q0 /= norm; p->q1 /= norm; p->q2 /= norm; p->q3 /= norm;
        }
        out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
        out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (p->q1*p->q3 - p->q0*p->q2)))) * 180.0f / M_PI;
        out->roll  = atan2f(2.0f * (p->q0*p->q1 + p->q2*p->q3), 1.0f - 2.0f * (p->q1*p->q1 + p->q2*p->q2)) * 180.0f / M_PI;
        out->yaw   = atan2f(2.0f * (p->q0*p->q3 + p->q1*p->q2), 1.0f - 2.0f * (p->q2*p->q2 + p->q3*p->q3)) * 180.0f / M_PI;
        return;
    }

    if (self->degrade == FILTER_DEGRADE_ACC_ONLY) {
        /* 仅加速度计：从ACC计算姿态角并重置四元数 */
        float ax_norm = sqrtf(in->ax*in->ax + in->ay*in->ay + in->az*in->az);
        if (ax_norm > 0.01f) {
            float ax_n = in->ax/ax_norm, ay_n = in->ay/ax_norm, az_n = in->az/ax_norm;
            float acc_pitch_rad = atan2f(-ax_n, sqrtf(ay_n*ay_n + az_n*az_n));
            float acc_roll_rad  = atan2f( ay_n, sqrtf(ax_n*ax_n + az_n*az_n));
            float cp = cosf(acc_pitch_rad * 0.5f);
            float sp = sinf(acc_pitch_rad * 0.5f);
            float cr = cosf(acc_roll_rad * 0.5f);
            float sr = sinf(acc_roll_rad * 0.5f);
            p->q0 = cp * cr;
            p->q1 = cp * sr;
            p->q2 = sp * cr;
            p->q3 = -sp * sr;  /* yaw=0 */
        }
        out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
        out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (p->q1*p->q3 - p->q0*p->q2)))) * 180.0f / M_PI;
        out->roll  = atan2f(2.0f * (p->q0*p->q1 + p->q2*p->q3), 1.0f - 2.0f * (p->q1*p->q1 + p->q2*p->q2)) * 180.0f / M_PI;
        out->yaw   = atan2f(2.0f * (p->q0*p->q3 + p->q1*p->q2), 1.0f - 2.0f * (p->q2*p->q2 + p->q3*p->q3)) * 180.0f / M_PI;
        return;
    }
    float ax = in->ax, ay = in->ay, az = in->az;
    float gx = in->gx * M_PI / 180.0f;  /* 转换为 rad/s */
    float gy = in->gy * M_PI / 180.0f;
    float gz = in->gz * M_PI / 180.0f;

    /* 归一化加速度 */
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 0.0f) {
        ax /= norm; ay /= norm; az /= norm;
    }

    /* 估计重力方向 */
    float vx = 2.0f * (p->q1 * p->q3 - p->q0 * p->q2);
    float vy = 2.0f * (p->q0 * p->q1 + p->q2 * p->q3);
    float vz = 1.0f - 2.0f * (p->q1 * p->q1 + p->q2 * p->q2);

    /* 误差（叉积） */
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    /* PI 控制器 */
    if (p->ki > 0.0f) {
        p->ix += p->ki * ex * dt;
        p->iy += p->ki * ey * dt;
        p->iz += p->ki * ez * dt;
        /* 积分抗饱和限幅 */
        #define INTEGRAL_LIMIT 0.5f  /* 约 28.6°/s */
        p->ix = fmaxf(-INTEGRAL_LIMIT, fminf(INTEGRAL_LIMIT, p->ix));
        p->iy = fmaxf(-INTEGRAL_LIMIT, fminf(INTEGRAL_LIMIT, p->iy));
        p->iz = fmaxf(-INTEGRAL_LIMIT, fminf(INTEGRAL_LIMIT, p->iz));
        #undef INTEGRAL_LIMIT
        gx += p->ix;
        gy += p->iy;
        gz += p->iz;
    }

    gx += p->kp * ex;
    gy += p->kp * ey;
    gz += p->kp * ez;

    /* 四元数积分 */
    float q0 = p->q0, q1 = p->q1, q2 = p->q2, q3 = p->q3;
    p->q0 += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    p->q1 += 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * dt;
    p->q2 += 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * dt;
    p->q3 += 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * dt;

    /* 归一化 */
    norm = sqrtf(p->q0 * p->q0 + p->q1 * p->q1 + p->q2 * p->q2 + p->q3 * p->q3);
    if (norm > 0.0f) {
        p->q0 /= norm; p->q1 /= norm; p->q2 /= norm; p->q3 /= norm;
    }

    /* 输出 */
    out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
    out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (p->q1 * p->q3 - p->q0 * p->q2)))) * 180.0f / M_PI;
    out->roll  = atan2f(2.0f * (p->q0 * p->q1 + p->q2 * p->q3),
                        1.0f - 2.0f * (p->q1 * p->q1 + p->q2 * p->q2)) * 180.0f / M_PI;
    out->yaw   = atan2f(2.0f * (p->q0 * p->q3 + p->q1 * p->q2),
                        1.0f - 2.0f * (p->q2 * p->q2 + p->q3 * p->q3)) * 180.0f / M_PI;
}

static void mahony_reset(filter_t *self)
{
    mahony_priv_t *p = (mahony_priv_t *)self->priv;
    p->q0 = 1.0f; p->q1 = p->q2 = p->q3 = 0.0f;
    p->ix = p->iy = p->iz = 0.0f;
}

static void mahony_set_param(filter_t *self, filter_param_t param, float value)
{
    mahony_priv_t *p = (mahony_priv_t *)self->priv;
    if (param == FILTER_PARAM_KP) p->kp = value;
    if (param == FILTER_PARAM_KI) p->ki = value;
}

static void mahony_destroy(filter_t *self)
{
    free(self->priv);
    free(self);
}

filter_t* filter_create_mahony(float kp, float ki)
{
    filter_t *f = (filter_t *)malloc(sizeof(filter_t));
    if (!f) return NULL;

    mahony_priv_t *p = (mahony_priv_t *)malloc(sizeof(mahony_priv_t));
    if (!p) { free(f); return NULL; }

    p->kp = kp;
    p->ki = ki;
    p->q0 = 1.0f; p->q1 = p->q2 = p->q3 = 0.0f;
    p->ix = p->iy = p->iz = 0.0f;

    f->update    = mahony_update;
    f->reset     = mahony_reset;
    f->set_param = mahony_set_param;
    f->destroy   = mahony_destroy;
    f->type      = FILTER_TYPE_MAHONY;
    f->degrade   = FILTER_DEGRADE_NONE;
    f->priv      = p;

    return f;
}

/* ============================================================
 * 5. Madgwick 滤波器
 * ============================================================ */

typedef struct {
    float beta;         /**< 梯度下降步长 */
    float q0, q1, q2, q3;  /**< 四元数 */
} madgwick_priv_t;

static void madgwick_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    madgwick_priv_t *p = (madgwick_priv_t *)self->priv;
    float dt = in->dt;

    /* 输入验证 */
    if (dt <= 0.0f || isnan(in->ax) || isinf(in->ax) ||
        isnan(in->gx) || isinf(in->gx)) {
        out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
        out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (p->q1*p->q3 - p->q0*p->q2)))) * 180.0f / M_PI;
        out->roll  = atan2f(2.0f * (p->q0*p->q1 + p->q2*p->q3), 1.0f - 2.0f * (p->q1*p->q1 + p->q2*p->q2)) * 180.0f / M_PI;
        out->yaw   = atan2f(2.0f * (p->q0*p->q3 + p->q1*p->q2), 1.0f - 2.0f * (p->q2*p->q2 + p->q3*p->q3)) * 180.0f / M_PI;
        return;
    }
    /* 退化模式检查 */
    if (self->degrade == FILTER_DEGRADE_HOLD_LAST) {
        out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
        out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (p->q1*p->q3 - p->q0*p->q2)))) * 180.0f / M_PI;
        out->roll  = atan2f(2.0f * (p->q0*p->q1 + p->q2*p->q3), 1.0f - 2.0f * (p->q1*p->q1 + p->q2*p->q2)) * 180.0f / M_PI;
        out->yaw   = atan2f(2.0f * (p->q0*p->q3 + p->q1*p->q2), 1.0f - 2.0f * (p->q2*p->q2 + p->q3*p->q3)) * 180.0f / M_PI;
        return;
    }

    if (self->degrade == FILTER_DEGRADE_GYRO_ONLY) {
        /* 仅陀螺仪：跳过梯度下降修正，只做四元数积分 */
        float gx = in->gx * M_PI / 180.0f;
        float gy = in->gy * M_PI / 180.0f;
        float gz = in->gz * M_PI / 180.0f;
        float q0 = p->q0, q1 = p->q1, q2 = p->q2, q3 = p->q3;
        p->q0 += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
        p->q1 += 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * dt;
        p->q2 += 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * dt;
        p->q3 += 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * dt;
        float norm = sqrtf(p->q0*p->q0 + p->q1*p->q1 + p->q2*p->q2 + p->q3*p->q3);
        if (norm > 0.0f) {
            p->q0 /= norm; p->q1 /= norm; p->q2 /= norm; p->q3 /= norm;
        }
        out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
        out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (p->q1*p->q3 - p->q0*p->q2)))) * 180.0f / M_PI;
        out->roll  = atan2f(2.0f * (p->q0*p->q1 + p->q2*p->q3), 1.0f - 2.0f * (p->q1*p->q1 + p->q2*p->q2)) * 180.0f / M_PI;
        out->yaw   = atan2f(2.0f * (p->q0*p->q3 + p->q1*p->q2), 1.0f - 2.0f * (p->q2*p->q2 + p->q3*p->q3)) * 180.0f / M_PI;
        return;
    }

    if (self->degrade == FILTER_DEGRADE_ACC_ONLY) {
        /* 仅加速度计：从ACC计算姿态角并重置四元数 */
        float ax_norm = sqrtf(in->ax*in->ax + in->ay*in->ay + in->az*in->az);
        if (ax_norm > 0.01f) {
            float ax_n = in->ax/ax_norm, ay_n = in->ay/ax_norm, az_n = in->az/ax_norm;
            float acc_pitch_rad = atan2f(-ax_n, sqrtf(ay_n*ay_n + az_n*az_n));
            float acc_roll_rad  = atan2f( ay_n, sqrtf(ax_n*ax_n + az_n*az_n));
            float cp = cosf(acc_pitch_rad * 0.5f);
            float sp = sinf(acc_pitch_rad * 0.5f);
            float cr = cosf(acc_roll_rad * 0.5f);
            float sr = sinf(acc_roll_rad * 0.5f);
            p->q0 = cp * cr;
            p->q1 = cp * sr;
            p->q2 = sp * cr;
            p->q3 = -sp * sr;  /* yaw=0 */
        }
        out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
        out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (p->q1*p->q3 - p->q0*p->q2)))) * 180.0f / M_PI;
        out->roll  = atan2f(2.0f * (p->q0*p->q1 + p->q2*p->q3), 1.0f - 2.0f * (p->q1*p->q1 + p->q2*p->q2)) * 180.0f / M_PI;
        out->yaw   = atan2f(2.0f * (p->q0*p->q3 + p->q1*p->q2), 1.0f - 2.0f * (p->q2*p->q2 + p->q3*p->q3)) * 180.0f / M_PI;
        return;
    }
    float ax = in->ax, ay = in->ay, az = in->az;
    float gx = in->gx * M_PI / 180.0f;
    float gy = in->gy * M_PI / 180.0f;
    float gz = in->gz * M_PI / 180.0f;

    float q0 = p->q0, q1 = p->q1, q2 = p->q2, q3 = p->q3;

    /* 归一化加速度 */
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 0.0f) {
        ax /= norm; ay /= norm; az /= norm;
    }

    /* 梯度下降步 */
    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    /* 梯度下降法修正 */
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm > 0.0f) {
        s0 /= norm; s1 /= norm; s2 /= norm; s3 /= norm;
    }

    /* 四元数微分方程 */
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - p->beta * s0;
    float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - p->beta * s1;
    float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - p->beta * s2;
    float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - p->beta * s3;

    /* 积分 */
    p->q0 += qDot0 * dt;
    p->q1 += qDot1 * dt;
    p->q2 += qDot2 * dt;
    p->q3 += qDot3 * dt;

    /* 归一化 */
    norm = sqrtf(p->q0 * p->q0 + p->q1 * p->q1 + p->q2 * p->q2 + p->q3 * p->q3);
    if (norm > 0.0f) {
        p->q0 /= norm; p->q1 /= norm; p->q2 /= norm; p->q3 /= norm;
    }

    /* 输出 */
    out->q0 = p->q0; out->q1 = p->q1; out->q2 = p->q2; out->q3 = p->q3;
    out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -2.0f * (p->q1 * p->q3 - p->q0 * p->q2)))) * 180.0f / M_PI;
    out->roll  = atan2f(2.0f * (p->q0 * p->q1 + p->q2 * p->q3),
                        1.0f - 2.0f * (p->q1 * p->q1 + p->q2 * p->q2)) * 180.0f / M_PI;
    out->yaw   = atan2f(2.0f * (p->q0 * p->q3 + p->q1 * p->q2),
                        1.0f - 2.0f * (p->q2 * p->q2 + p->q3 * p->q3)) * 180.0f / M_PI;
}

static void madgwick_reset(filter_t *self)
{
    madgwick_priv_t *p = (madgwick_priv_t *)self->priv;
    p->q0 = 1.0f; p->q1 = p->q2 = p->q3 = 0.0f;
}

static void madgwick_set_param(filter_t *self, filter_param_t param, float value)
{
    madgwick_priv_t *p = (madgwick_priv_t *)self->priv;
    if (param == FILTER_PARAM_KP) p->beta = value;  /* Madgwick 使用 beta */
}

static void madgwick_destroy(filter_t *self)
{
    free(self->priv);
    free(self);
}

filter_t* filter_create_madgwick(float beta)
{
    filter_t *f = (filter_t *)malloc(sizeof(filter_t));
    if (!f) return NULL;

    madgwick_priv_t *p = (madgwick_priv_t *)malloc(sizeof(madgwick_priv_t));
    if (!p) { free(f); return NULL; }

    p->beta = beta;
    p->q0 = 1.0f; p->q1 = p->q2 = p->q3 = 0.0f;

    f->update    = madgwick_update;
    f->reset     = madgwick_reset;
    f->set_param = madgwick_set_param;
    f->destroy   = madgwick_destroy;
    f->type      = FILTER_TYPE_MADGWICK;
    f->degrade   = FILTER_DEGRADE_NONE;
    f->priv      = p;

    return f;
}

/* ============================================================
 * 6. 工厂函数
 * ============================================================ */

filter_t* filter_create(filter_type_t type)
{
    switch (type) {
        case FILTER_TYPE_COMPLEMENTARY:
            return filter_create_complementary(0.98f);  /* 默认 α */
        case FILTER_TYPE_LPF:
            return filter_create_lpf(10.0f);  /* 默认 10Hz 截止频率 */
        case FILTER_TYPE_EKF:
            return filter_create_ekf(0.001f, 0.003f, 0.03f);  /* 默认参数 */
        case FILTER_TYPE_MAHONY:
            return filter_create_mahony(2.0f, 0.0f);  /* 默认 kp=2, ki=0 */
        case FILTER_TYPE_MADGWICK:
            return filter_create_madgwick(0.1f);  /* 默认 beta=0.1 */
        default:
            return NULL;
    }
}

const char* filter_type_name(filter_type_t type)
{
    static const char *names[] = {
        "Complementary",
        "LPF",
        "EKF",
        "Mahony",
        "Madgwick"
    };
    if (type >= 0 && type < FILTER_TYPE_COUNT) {
        return names[type];
    }
    return "Unknown";
}

/* ============================================================
 * 7. 退化模式API
 * ============================================================ */

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

int filter_check_acc_quality(float ax, float ay, float az) {
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    return (norm >= 0.5f && norm <= 2.0f) ? 1 : 0;
}

int filter_check_gyro_quality(float gx, float gy, float gz) {
    return (fabsf(gx) <= 400.0f && fabsf(gy) <= 400.0f && fabsf(gz) <= 400.0f) ? 1 : 0;
}
