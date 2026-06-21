/**
 * @file    filter_internal.h
 * @brief   滤波器库内部头文件（不对外暴露）
 *
 * 仅 Core/Filter/Src/ 内部使用。声明各滤波器的 5 个统一接口函数，
 * 供 filter_factory.c 表驱动派发。公共 API 见 filter.h。
 *
 * 每个滤波器需实现：
 *   - <type>_update          滤波更新
 *   - <type>_reset           状态重置
 *   - <type>_set_param       参数设置
 *   - <type>_get_static_size 私有数据大小（sizeof(<type>_priv_t)）
 *   - <type>_init            私有数据默认初始化
 *
 * 设计说明：
 *   - <type>_priv_t 结构体定义在各自 .c 文件内，不暴露
 *   - 工厂通过 <type>_get_static_size() 间接查询大小，通过 <type>_init() 初始化
 *   - Phase 3 将为这些函数加 __attribute__((weak))，允许分支覆盖
 */

#ifndef FILTER_INTERNAL_H
#define FILTER_INTERNAL_H

#include "filter.h"
#include "filter_math.h"
#include <stddef.h>  /* size_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 共享辅助函数（filter_common.c 实现，各滤波器调用以消除重复）
 * ============================================================
 *
 * 设计动机：原 filter.c 中存在大量重复代码
 *   - ACC→欧拉角：complementary/lpf/lkf 重复 3 次
 *   - 四元数→欧拉角：ekf/mahony/madgwick 重复 10+ 次
 *   - ACC→四元数（ACC_ONLY 退化）：ekf/mahony/madgwick 重复 3 次
 *   - 四元数归一化：ekf/mahony/madgwick 重复 6+ 次
 * 抽出到 filter_common.c 统一实现，保证行为一致 + 减少 ~200 行重复代码。
 *
 * MCU 端考虑：辅助函数标记 static inline（短小的）或普通 static（含 atan2f），
 * 编译器可内联到调用点，无函数调用开销。
 */

/**
 * @brief 从加速度计算欧拉角（pitch/roll）
 * @param ax,ay,az  加速度 (g)
 * @param[out] pitch_deg  俯仰角 (度)
 * @param[out] roll_deg   横滚角 (度)
 *
 * 公式（重力沿 +Z 时）：
 *   pitch = atan2(-ax, sqrt(ay²+az²))
 *   roll  = atan2( ay, sqrt(ax²+az²))
 *
 * 注意：yaw 无法从 ACC 单独求得（无磁力仪），需陀螺积分。
 *      当 |a| ≈ 0（自由落体）时输出不可靠，调用方应做幅值检查。
 */
void filter_acc_to_euler(float ax, float ay, float az,
                         float *pitch_deg, float *roll_deg);

/**
 * @brief 四元数转欧拉角（ZYX 顺序，即 yaw-pitch-roll）
 * @param q0,q1,q2,q3  四元数（实部在前）
 * @param[out] pitch_deg, roll_deg, yaw_deg  欧拉角 (度)
 *
 * 公式（标准 ZYX 提取，对应 Tait-Bryan 角）：
 *   pitch = asin(-2(q1q3 - q0q2))     ← 钳位到 [-1,1] 防 NaN
 *   roll  = atan2(2(q0q1 + q2q3), 1 - 2(q1² + q2²))
 *   yaw   = atan2(2(q0q3 + q1q2), 1 - 2(q2² + q3²))
 *
 * 万向节锁：pitch 接近 ±90° 时 roll/yaw 退化（不可解）。
 */
void filter_quat_to_euler(float q0, float q1, float q2, float q3,
                          float *pitch_deg, float *roll_deg, float *yaw_deg);

/**
 * @brief 原地四元数归一化
 * @param[in,out] q0,q1,q2,q3  四元数
 *
 * 数值保护：
 *   - |q| < 1e-10 时重置为 (1,0,0,0)（单位四元数）
 *   - 否则除以 |q|，保证 ‖q‖=1
 *
 * 必要性：四元数积分（q += q_dot*dt）会累积数值误差，不归一化会导致
 *        协方差矩阵发散、欧拉角计算错误。
 */
void filter_quat_normalize_inplace(float *q0, float *q1, float *q2, float *q3);

/**
 * @brief 从加速度计算四元数（假设 yaw=0，用于 ACC_ONLY 退化）
 * @param ax,ay,az  加速度 (g)
 * @param[out] q0,q1,q2,q3  四元数
 *
 * 公式：
 *   先求 pitch/roll（见 filter_acc_to_euler），转半角四元数：
 *     q = q_pitch ⊗ q_roll，yaw=0
 *     q_pitch = (cos(p/2), 0, sin(p/2), 0)
 *     q_roll  = (cos(r/2), sin(r/2), 0, 0)
 *     q = (cp*cr, cp*sr, sp*cr, -sp*sr)
 *
 * 适用：ACC_ONLY 退化模式（陀螺饱和/失效时）。
 * 注意：|a| < 0.01g 时不更新（自由落体，ACC 不可靠）。
 */
void filter_acc_to_quat(float ax, float ay, float az,
                        float *q0, float *q1, float *q2, float *q3);

/* ============================================================
 * 滤波器接口（每个滤波器实现 5 个函数，工厂表驱动派发）
 * ============================================================ */
void  complementary_update(filter_t *self, const filter_input_t *in, filter_output_t *out);
void  complementary_reset(filter_t *self);
void  complementary_set_param(filter_t *self, filter_param_t param, float value);
size_t complementary_get_static_size(void);
void  complementary_init(void *priv);

/* ============================================================
 * 2. 一阶低通滤波器 (LPF)
 * ============================================================ */
void  lpf_update(filter_t *self, const filter_input_t *in, filter_output_t *out);
void  lpf_reset(filter_t *self);
void  lpf_set_param(filter_t *self, filter_param_t param, float value);
size_t lpf_get_static_size(void);
void  lpf_init(void *priv);

/* ============================================================
 * 3. 扩展卡尔曼滤波器 (EKF) — 7 状态
 * ============================================================ */
void  ekf_update(filter_t *self, const filter_input_t *in, filter_output_t *out);
void  ekf_reset(filter_t *self);
void  ekf_set_param(filter_t *self, filter_param_t param, float value);
size_t ekf_get_static_size(void);
void  ekf_init(void *priv);

/* ============================================================
 * 4. 线性卡尔曼滤波器 (LKF) — 6 状态
 * ============================================================ */
void  lkf_update(filter_t *self, const filter_input_t *in, filter_output_t *out);
void  lkf_reset(filter_t *self);
void  lkf_set_param(filter_t *self, filter_param_t param, float value);
size_t lkf_get_static_size(void);
void  lkf_init(void *priv);

/* ============================================================
 * 5. Mahony 滤波器
 * ============================================================ */
void  mahony_update(filter_t *self, const filter_input_t *in, filter_output_t *out);
void  mahony_reset(filter_t *self);
void  mahony_set_param(filter_t *self, filter_param_t param, float value);
size_t mahony_get_static_size(void);
void  mahony_init(void *priv);

/* ============================================================
 * 6. Madgwick 滤波器
 * ============================================================ */
void  madgwick_update(filter_t *self, const filter_input_t *in, filter_output_t *out);
void  madgwick_reset(filter_t *self);
void  madgwick_set_param(filter_t *self, filter_param_t param, float value);
size_t madgwick_get_static_size(void);
void  madgwick_init(void *priv);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_INTERNAL_H */
