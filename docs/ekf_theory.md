# EKF 姿态解算算法 — 项目实现详解

> 基于 `filter.c` 与 `filter_config.h`
> 滤波器类型: `FILTER_TYPE_EKF` | 传感器: LSM6DSR (6轴IMU) | 采样率: 100Hz

---

## 1. 系统模型概述

### 1.1 坐标系约定

| 轴 | 传感器方向 | 正方向 |
|---|---|---|
| X | 芯片前进方向 | 向右 |
| Y | 芯片侧方向 | 向前 |
| Z | 垂直方向 | 向上 |

姿态角定义 (欧拉角, 单位: 度):

| 角度 | 范围 | 计算公式 |
|---|---|---|
| Roll φ | ±180° | `atan2(2(q₀q₁+q₂q₃), 1-2(q₁²+q₂²))` |
| Pitch θ | ±90° | `asin(-2(q₁q₃-q₀q₂))` |
| Yaw ψ | ±180° | `atan2(2(q₀q₃+q₁q₂), 1-2(q₂²+q₃²))` |

### 1.2 输入/输出数据结构

```c
// filter_input_t — EKF 每帧输入
typedef struct {
    float ax, ay, az;  // 加速度计 (m/s²)
    float gx, gy, gz;  // 陀螺仪 (dps)
    float dt;           // 时间步长 (s)
} filter_input_t;

// filter_output_t — EKF 每帧输出
typedef struct {
    float pitch, roll, yaw;  // 姿态角 (度)
    float q0, q1, q2, q3;    // 四元数
} filter_output_t;
```

---

## 2. 状态向量

```
x = [q₀, q₁, q₂, q₃, bₓ, bᵧ, b₂]ᵀ   (7×1)
    ╔══════════════════════════════════╗
    ║  0: q₀ — 四元数标量部分          ║
    ║  1: q₁ — 四元数 X 分量            ║
    ║  2: q₂ — 四元数 Y 分量            ║
    ║  3: q₃ — 四元数 Z 分量            ║
    ║  4: bₓ — 陀螺仪 X 轴偏置 (dps)    ║
    ║  5: bᵧ — 陀螺仪 Y 轴偏置 (dps)    ║
    ║  6: b₂ — 陀螺仪 Z 轴偏置 (dps)    ║
    ╚══════════════════════════════════╝
```

---

## 3. EKF 算法流程

### 3.0 输入验证 & 退化模式

```c
if (dt <= 0 || isnan(ax) || isinf(ax) || isnan(gx) || isinf(gx)) {
    // 无效输入 → 保持上次输出不变
    return;
}

if (degrade == FILTER_DEGRADE_HOLD_LAST) {
    // 冻结模式 → 输出上次姿态
    return;
}
if (degrade == FILTER_DEGRADE_ACC_ONLY) {
    // 仅加速度计 → 从ACC计算姿态, 跳过GYRO积分
    // 直接从 ax,ay,az 计算 pitch/roll → 转四元数
    return;
}
```

### 3.1 步骤 ① — 偏置补偿

```c
// 输入 gyro 单位: dps (度/秒)
// 步骤: 减去偏置 → 转为 rad/s
gx_comp = (in->gx - bₓ) * π / 180
gy_comp = (in->gy - bᵧ) * π / 180
gz_comp = (in->gz - b₂) * π / 180
```

### 3.2 步骤 ② — 状态预测 (四元数运动学)

**四元数微分方程:**

```
q̇ = 0.5 · Ω(ω) · q

其中 Ω(ω) = [ 0,  -ωx, -ωy, -ωz;
              ωx,  0,   ωz,  -ωy;
              ωy, -ωz,  0,    ωx;
              ωz,  ωy, -ωx,   0  ]

展开:
q̇₀ = 0.5 · (-q₁·ωx - q₂·ωy - q₃·ωz)
q̇₁ = 0.5 · ( q₀·ωx + q₂·ωz - q₃·ωy)
q̇₂ = 0.5 · ( q₀·ωy - q₁·ωz + q₃·ωx)
q̇₃ = 0.5 · ( q₀·ωz + q₁·ωy - q₂·ωx)
```

**一阶欧拉积分:**

```c
q₊ = q + q̇ · dt
```

**四元数归一化 (防止数值漂移):**

```c
norm = sqrt(q₀² + q₁² + q₂² + q₃²)
if (norm > 1e-10) {
    q₀ /= norm;  q₁ /= norm;
    q₂ /= norm;  q₃ /= norm;
}
```

偏置状态 `bₓ, bᵧ, b₂` 在预测步骤中保持不变 (随机游走模型)。

### 3.3 步骤 ③ — 协方差预测

**原理:** `P₊ = F · P · Fᵀ + Q`

#### 3.3.1 F 矩阵 (7×7)

基于四元数运动学线性化:

```
F = [ F_qq  F_qb ]
    [ 0₃ₓ₄  I₃   ]
```

**F_qq (4×4) — 四元数转移矩阵:**
```
F_qq = I₄ + 0.5·dt·Ω(ω)

F_qq = [ 1,      -½dt·gx,  -½dt·gy,  -½dt·gz
         ½dt·gx,  1,       ½dt·gz,   -½dt·gy
         ½dt·gy, -½dt·gz,  1,        ½dt·gx
         ½dt·gz,  ½dt·gy, -½dt·gx,   1     ]
```

**F_qb (4×3) — 偏置对四元数的影响:**
本项目实现中简化为 0 (认为偏置变化缓慢, 在一个dt内影响可忽略)。

**F_bq = 0₃ₓ₄, F_bb = I₃**

#### 3.3.2 Q 矩阵 (7×7) — 过程噪声

```
Q = diag(Q_angle·dt, Q_angle·dt, Q_angle·dt, Q_angle·dt,
          Q_bias·dt,  Q_bias·dt,  Q_bias·dt)
```

| 参数 | 代码变量 | 本项目设置值 | 物理含义 |
|---|---|---|---|---|
| Q_angle | `p->Q_angle` | **0.001** | 角度随机游走噪声 (rad²/s), 越大越信任陀螺仪预测 |
| Q_bias | `p->Q_bias` | **0.003** | 偏置随机游走噪声 (dps²/s), 越大偏置收敛越快 |

#### 3.3.3 协方差更新计算

```c
// 1. 计算 F*P (四元数部分 4×7)
FP[i][j] = Σₖ F[i][k] · P[k][j]

// 2. 计算 (F*P)*Fᵀ (四元数部分 4×4)
P_new[i][j] = Σₖ FP[i][k] · Fᵀ[k][j]

// 3. 偏置交叉项 (4×3 和 3×4)
P_new[0:4][4:7] = FP[0:4][4:7]               // F_qb * P_b 项
P_new[4:7][0:4] = P[4:7][0:4] · F_qqᵀ         // P_bq * F_qqᵀ

// 4. 偏置自项不变
P_new[4:7][4:7] = P[4:7][4:7]

// 5. 加上过程噪声 Q
P_new[i][i] += Q_angle * dt   (i = 0..3)
P_new[i][i] += Q_bias  * dt   (i = 4..6)
```

### 3.4 步骤 ④ — 加速度计归一化与异常检查

```c
acc_norm = sqrt(ax² + ay² + az²)

if (acc_norm < 0.01 || acc_norm > 20.0) {
    // 加速度计读数异常 (自由落体/碰撞)
    // 跳过测量更新, 仅做状态预测
    return;
}

ax /= acc_norm;  ay /= acc_norm;  az /= acc_norm;
```

### 3.5 步骤 ⑤ — 测量残差 (创新)

**测量模型 h(x):** 预测的重力方向 (由四元数旋转 [0,0,1]):

```c
// 预测重力方向 (旋转矩阵第三列)
hₓ = 2(q₁q₃ - q₀q₂)
hᵧ = 2(q₀q₁ + q₂q₃)
h₂ = 1 - 2(q₁² + q₂²)
```

**创新向量 y = z − h(x) (3×1):**

```c
yₓ = aₓ - hₓ
yᵧ = aᵧ - hᵧ
y₂ = a₂ - h₂
```

**物理意义:** 创新 y 反映了加速度计实测重力方向与当前四元数预测的重力方向之间的偏差。这个偏差用于修正四元数。

### 3.6 步骤 ⑥ — 动态 R 适配

```c
if (r_adapt_enable) {
    acc_mag_error = fabs(acc_norm - 1.0);
    r_factor = 1.0 + acc_mag_error * 10.0;  // [0.1, 10.0]
} else {
    r_factor = 1.0;  // 已禁用 (本项目设置)
}
R_eff = R_measure * r_factor;
```

> **本项目已禁用** `r_adapt_enable = 0`，使用固定 `R = 0.03`（见 `filter_config.h` `EKF_R_MEASURE_DEFAULT`）

### 3.7 步骤 ⑦ — H 矩阵 (雅可比, 3×7)

H 是测量模型 h(x) 对状态 x 的偏导数:

```
H = ∂h/∂x

H = [ -2q₂,  2q₃, -2q₀,  2q₁,  0, 0, 0 ;
       2q₁,  2q₀,  2q₃,  2q₂,  0, 0, 0 ;
       0,   -4q₁, -4q₂,  0,    0, 0, 0 ]
```

注意: `∂h/∂b = 0₃ₓ₃` (加速度计测量不直接受陀螺仪偏置影响)。

### 3.8 步骤 ⑧ — S 矩阵与卡尔曼增益

```c
// 1. PHᵀ = P · Hᵀ  (7×3)
PHᵀ[i][j] = Σₖ P[i][k] · H[j][k]

// 2. S = H · PHᵀ + R  (3×3)
S[i][j] = Σₖ H[i][k] · PHᵀ[k][j] + (i==j ? R_eff : 0)

// 3. S⁻¹ 使用 3×3 解析逆矩阵公式
det = a(ek - fh) - b(dk - fg) + c(dh - eg)
S_inv = (1/det) · 伴随矩阵

// 4. 卡尔曼增益 K = P · Hᵀ · S⁻¹  (7×3)
K[i][j] = Σₖ PHᵀ[i][k] · S_inv[k][j]
```

### 3.9 步骤 ⑨ — Chi-squared 创新门控 (异常值拒绝)

```c
// 马氏距离平方
χ² = yᵀ · S⁻¹ · y

if (χ² > chi2_threshold) {
    // 创新过大 → 视为离群值
    // 跳过测量更新, 仅做预测
    return;
}
```

本项目阈值: `chi2_threshold = 11.34` (自由度=3, χ² 99%置信度)。

| χ² 阈值 | 置信度 | 行为 |
|---|---|---|
| 11.34 | 99% | 严格, 动态时容易拒测量 |
| **20.0** | ≈99.9% | 宽松, 只有极端异常才拒绝 |

### 3.10 步骤 ⑩ — 状态更新

**状态修正:** `x₊ = x₋ + K · y`

```c
for i = 0..6:
    state[i] += Σⱼ K[i][j] · y[j]   (j = 0,1,2)
```

**偏置幅值限制 (防发散):**

```c
bias_limit_dps = 20.0  // 本项目设置 (`EKF_BIAS_LIMIT_DEFAULT`)，匹配 LSM6DSR ±10dps 规格（保留余量以应对温漂）
for i = 4..6:
    if state[i] > bias_limit_dps:    state[i] = bias_limit_dps
    if state[i] < -bias_limit_dps:   state[i] = -bias_limit_dps
```

**四元数归一化 (修正后):**
```c
norm = sqrt(q₀² + q₁² + q₂² + q₃²)
if (norm > 1e-10) {
    q₀ /= norm;  q₁ /= norm;
    q₂ /= norm;  q₃ /= norm;
}
```

### 3.11 步骤 ⑪ — 协方差更新 (Joseph 形式)

使用 **Joseph 形式的协方差更新**，保证数值稳定性:

```
P₊ = (I − KH) · P · (I − KH)ᵀ + K · R · Kᵀ
```

相比标准形式 `P = (I-KH)·P`，Joseph 形式即使卡尔曼增益计算有数值误差，也能保证协方差矩阵对称正定。

```c
// 1. KH = K · H  (7×7)
KH[i][j] = Σₖ K[i][k] · H[k][j]

// 2. I_KH = I − KH
I_KH[i][j] = (i==j ? 1 : 0) − KH[i][j]

// 3. temp = I_KH · P
temp[i][j] = Σₖ I_KH[i][k] · P[k][j]

// 4. P_new = temp · I_KHᵀ
P_new[i][j] = Σₖ temp[i][k] · I_KH[j][k]

// 5. P_new += K · R · Kᵀ
P_new[i][j] += Σₖ Σₗ K[i][k] · R_eff · (k==l) · K[j][l]

// 6. 强制对称 + 正定
P[i][j] = P[j][i] = 0.5·(P_new[i][j] + P_new[j][i])
if P[i][i] < 1e-10: P[i][i] = 1e-10
```

### 3.12 步骤 ⑫ — 定期协方差正则化 (每100次更新)

```c
update_count++

if (update_count >= 100) {
    update_count = 0
    
    // 强制对称化
    for i, j: P[i][j] = P[j][i] = 0.5·(P[i][j] + P[j][i])
    
    // 对角线下界保护
    if P[i][i] < 1e-10: P[i][i] = 1e-10
    
    // 最大协方差限制 (防发散)
    if P[i][i] > 1e6: P[i][i] = 1e6
}
```

---

## 4. 参数总结

### 4.1 当前项目配置 (filter_config.h)

| 参数 | 设置值 | 可调范围 | 作用 |
|---|---|---|---|
| Q_angle | **0.001** | 0.0001 ~ 0.1 | ↑增大: 更信任陀螺仪积分, 动态响应更快 |
| Q_bias | **0.003** | 0.0001 ~ 0.1 | ↑增大: 偏置估计收敛更快, 但噪声更大 |
| R_measure | **0.03** | 0.0001 ~ 1.0 | ↑增大: 更不信任加速度计修正, 平滑但滞后 |
| bias_limit | **20.0 dps** | 5.0 ~ 50.0 | 偏置幅值上限，`EKF_BIAS_LIMIT_DEFAULT=20.0` |
| chi2_threshold | **11.34** | 5.0 ~ 20.0 | ↑增大: 接受更多动态测量 |
| r_adapt_enable | **0 (禁用)** | 0/1 | 禁用动态R适配, 使用固定R |
| r_adapt_factor | **1.0** | 0.1 ~ 10.0 | 动态R缩放因子，`EKF_R_ADAPT_FACTOR`默认1.0（filter.c硬编码） |

> **参数变更记录**: `EKF_R_MEASURE_DEFAULT` 在 v1.x 中为 `0.001f`，v2.0 起修正为 `0.03f`。前者基于 LSM6DSR 数据手册理论值推导，实测偏小导致加速度计修正过强、动态姿态抖动；0.03 更符合实际传感器噪声水平。

### 4.2 参数调优建议

| 现象 | 需要调整的参数 |
|---|---|
| 动态响应慢 (转角不足) | ↑Q_angle 或 ↓R_measure |
| 静态抖动大 | ↓Q_angle 或 ↑R_measure |
| 偏置收敛慢 | ↑Q_bias |
| 偏置发散 | ↓Q_bias 或 ↓bias_limit |
| 快速运动时姿态跳变 | ↑chi2_threshold |
| 静止时姿态漂移 | ↓R_measure (更信任ACC) |

---

### 4.3 filter_config.c 参数描述表

`filter_config.c` 中的 `ekf_params[]` 数组为每个 EKF 参数提供了描述元数据（默认值、范围、来源、单位），可通过 `filter_config_get_params()` 在运行时查询：

```c
int count;
const filter_param_desc_t *params = filter_config_get_params(FILTER_TYPE_EKF, &count);
// count == 7（当前 EKF 参数数量）
```

**ekf_params[] 完整列表（7 项）**:

| 参数枚举 | 默认值 | 范围 | 来源 | 单位 |
|---|---|---|---|---|
| `FILTER_PARAM_Q_ANGLE` | 0.001 | 0.0001~0.1 | LSM6DSR Datasheet | rad²/s |
| `FILTER_PARAM_Q_BIAS` | 0.003 | 0.0001~0.1 | LSM6DSR Datasheet | dps²/s |
| `FILTER_PARAM_R_MEASURE` | 0.03 | 0.001~1.0 | LSM6DSR Datasheet | g² |
| `FILTER_PARAM_BIAS_LIMIT_DPS` | 20.0 | 5.0~50.0 | LSM6DSR Datasheet | dps |
| `FILTER_PARAM_CHI2_THRESHOLD` | 11.34 | 5.0~20.0 | χ²分布, df=3, 99% | 无量纲 |
| `FILTER_PARAM_R_ADAPT_ENABLE` | 0 | 0/1 | 工程调优 | 布尔 |
| `FILTER_PARAM_R_ADAPT_FACTOR` | 1.0 | 0.1~10.0 | 工程调优 | 无量纲 |

> `source_type` 字段（`PARAM_SOURCE_DATASHEET` / `PARAM_SOURCE_PAPER` / `PARAM_SOURCE_TUNED`）标识参数来源类型，便于用户判断参数调整的参考依据。

---

## 5. 计算复杂度分析

| 操作 | 浮点运算量 | 说明 |
|---|---|---|
| 状态预测 | ~40 flops | 4个四元数微分 + 归一化 |
| 协方差预测 | ~512 flops | 7×7矩阵乘法 ×2 + Q |
| H 矩阵 | ~10 flops | 解析雅可比 |
| S 矩阵 + 逆 | ~350 flops | 3×3乘法 + 解析逆 |
| χ² 门控 | ~20 flops | 向量-矩阵-向量 |
| 卡尔曼增益 K | ~150 flops | 7×3 × 3×3 |
| 状态更新 | ~50 flops | K×y + 截断 + 归一化 |
| 协方差更新 (Joseph) | ~1,500 flops | 最重步骤 |
| 正则化 | ~50 flops | 每100次 |
| **总计** | **~2,700 flops/帧** | @100Hz = 270K flops/s |
| **Cortex-M0+ @80MHz** | 可用算力 | ~20M flops/s, EKF占用<2% |

---

## 6. 与 Mahony/Madgwick 对比

| 特性 | EKF | Mahony | Madgwick |
|---|---|---|---|
| 理论基础 | 贝叶斯最优估计 | 非线性互补滤波 | 梯度下降法 |
| 状态估计 | 7维 (含偏置) | 4维 (仅四元数) | 4维 (仅四元数) |
| 偏置估计 | ✅ 内置 | ✅ PI积分器 | ❌ 无 |
| 异常值拒绝 | ✅ χ²门控 | ❌ | ❌ |
| 计算量 | ~2,700 flops | ~200 flops | ~300 flops |
| 参数数量 | 6个 | 2个 | 1个 |
| 适用场景 | 高精度/动态 | 平衡/中等精度 | 低算力/快速部署 |

---

## 7. 退化模式体系

EKF 支持4种降级运行模式, 当传感器数据质量下降时自动切换:

| 模式 | 行为 | 适用场景 |
|---|---|---|
| `GYRO_ONLY` | 仅四元数积分, 跳过测量更新 | ACC饱和/碰撞 |
| `ACC_ONLY` | 从ACC直接计算姿态 | GYRO饱和/故障 |
| `STATIC_ONLY` | 静止时才做测量更新 | 持续剧烈运动 |
| `HOLD_LAST` | 冻结输出 | 传感器完全失效 |

---

## 8. 数据流图 (完整处理链)

```
GYRO (dps)                  ACC (m/s²)
    │                           │
    ▼                           ▼
┌─────────────┐           ┌───────────┐
│ 偏置补偿     │           │ 归一化     │
│ ω = g - b   │           │ a /= |a|  │
└──────┬──────┘           └─────┬─────┘
       │                        │
       ▼                        │
┌──────────────┐                │
│ 状态预测      │                │
│ q₊ = q + q̇·dt │                │
│ (四元数积分)  │                │
└──────┬──────┘                │
       │                        │
       ▼                        │
┌──────────────┐                │
│ 协方差预测    │                │
│ P₊ = F·P·Fᵀ+Q│                │
└──────┬──────┘                │
       │                        │
       ▼                        ▼
┌──────────────────┐    ┌──────────────┐
│ 预测重力 h(q)     │◄───│ 测量 z = [a] │
│ hₓ = 2(q₁q₃-q₀q₂)│    │ 加速度计     │
│ hᵧ = 2(q₀q₁+q₂q₃)│    └──────┬───────┘
│ h₂ = 1-2(q₁²+q₂²)│           │
└──────┬───────────┘           │
       │                       │
       ▼                       ▼
┌───────────────────────────────────┐
│ 创新 y = z - h(x)                │
│ χ² = yᵀ · S⁻¹ · y                │
│ if χ² > threshold → 跳过更新     │
└────────────────┬──────────────────┘
                 │
                 ▼
┌───────────────────────────────────┐
│ 卡尔曼增益 K = P·Hᵀ·S⁻¹          │
│ 状态更新 x₊ = x₋ + K·y           │
│ 协方差更新 (Joseph形式)           │
└────────────────┬──────────────────┘
                 │
                 ▼
┌───────────────────────────────────┐
│ 输出姿态角                        │
│ pitch = asin(-2(q₁q₃-q₀q₂))      │
│ roll  = atan2(2(q₀q₁+q₂q₃), ...) │
│ yaw   = atan2(2(q₀q₃+q₁q₂), ...) │
└───────────────────────────────────┘
```

---

## 9. EKF API 使用示例

### 9.1 创建 EKF 实例

```c
#include "filter.h"
#include "filter_config.h"

/* 动态创建 */
filter_t *f = filter_create(FILTER_TYPE_EKF);
if (!f) { /* 错误处理 */ }

/* 或静态分配（嵌入式推荐） */
size_t buf_size = filter_get_static_size(FILTER_TYPE_EKF);
static uint8_t ekf_buffer[256];  /* 确保 >= buf_size */
filter_t *f2 = filter_create_static(FILTER_TYPE_EKF, ekf_buffer, sizeof(ekf_buffer));
```

### 9.2 配置 EKF 参数

通过 `set_param()` 覆盖默认值（只调用需要修改的参数，其余自动使用 `filter_config.h` 的默认值）：

```c
f->set_param(f, FILTER_PARAM_Q_ANGLE,        0.01f);    /* 过程噪声-角度 */
f->set_param(f, FILTER_PARAM_Q_BIAS,         0.01f);    /* 过程噪声-偏置 */
f->set_param(f, FILTER_PARAM_R_MEASURE,      0.05f);    /* 覆盖库默认值 0.03 */
f->set_param(f, FILTER_PARAM_BIAS_LIMIT_DPS, 10.0f);    /* 匹配 LSM6DSR ±10dps */
f->set_param(f, FILTER_PARAM_CHI2_THRESHOLD, 20.0f);    /* ≈99.9% 置信度, 宽松门限 */
f->set_param(f, FILTER_PARAM_R_ADAPT_ENABLE, 0.0f);     /* 禁用动态 R 适配 */
f->set_param(f, FILTER_PARAM_R_ADAPT_FACTOR, 1.0f);     /* R 缩放因子 */
```

### 9.3 运行 EKF

```c
filter_input_t in = {
    .ax = acc[0], .ay = acc[1], .az = acc[2],
    .gx = gyro[0], .gy = gyro[1], .gz = gyro[2],
    .dt = 0.01f    /* 100Hz */
};
filter_output_t out;
f->update(f, &in, &out);
printf("pitch=%.2f roll=%.2f yaw=%.2f\n", out.pitch, out.roll, out.yaw);
```

### 9.4 释放

```c
f->destroy(f);  /* 静态分配只需 destroy，buffer 由用户管理 */
```

---

## 10. 构建验证与回归测试

### 10.1 编译验证

```bash
cd LSM6DSR
make clean && make
# 期望: 0 error, 无新增 warning
```

### 10.2 EKF 回归测试

项目提供 EKF 专用测试程序，覆盖正常收敛、旋转跟踪、离群值拒绝等场景：

```bash
# 静态测试: pitch/roll ≈ 0，验证静止收敛
./test_ekf --mode=static --duration=10

# 旋转测试: 90dps 输入 → 验证航向角跟踪
./test_ekf --mode=rotation --rate=90dps --duration=2

# 离群值测试: 注入 5g → 验证 Chi-squared 门控跳过测量更新
./test_ekf --mode=outlier --inject=5g
```

---

## 11. 开发者注意事项

### 11.1 API 稳定性

所有公开 API（`filter_create` / `filter_create_static` / `update` / `set_param` / `destroy`）的函数签名保持稳定。`filter_param_t` 枚举仅在末尾 `FILTER_PARAM_COUNT` 之前增加新值，不影响已有代码的枚举序号。

### 11.2 参数默认值安全

| 新增参数 | 默认值 | 说明 |
|---|---|---|
| `FILTER_PARAM_BIAS_LIMIT_DPS` | 20.0 dps | 不调用 `set_param` 即使用此值 |
| `FILTER_PARAM_CHI2_THRESHOLD` | 11.34 | 等效 χ² 99% 置信度 |
| `FILTER_PARAM_R_ADAPT_ENABLE` | 0 (禁用) | 默认不使用动态 R |
| `FILTER_PARAM_R_ADAPT_FACTOR` | 1.0 | R 缩放因子，禁用时无效 |

所有新增参数在 `filter_create_ekf()` 中被初始化为安全默认值，应用层不调用 `set_param()` 不会引发异常行为。

### 11.3 静态分配注意事项

EKF 内部结构体 `ekf_priv_t` 包含 5 个增强字段（约 20 字节增量）。使用 `filter_create_static()` 时：

- 始终通过 `filter_get_static_size(FILTER_TYPE_EKF)` 查询实际所需缓冲区大小
- 不要假设固定大小（如 `sizeof(ekf_priv_t)` 或硬编码 128 字节）
- 跨编译器版本升级时，建议重新编译所有使用静态 EKF 实例的模块
