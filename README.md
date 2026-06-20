# LSM6DSR IMU 姿态估计库

基于 LSM6DSR 六轴 IMU 的互补滤波姿态估计实现，面向机器狗 (robot dog) 等强振动、强线性加速度场景。

## 架构

```
├── Core/
│   ├── Src/          # 通用核心源文件
│   │   ├── lsm6dsr.c      # 驱动层（平台无关）
│   │   └── bsp_lsm6dsr.c  # 业务层（滤波、偏置跟踪）
│   ├── Inc/          # 通用核心头文件
│   └── Filter/       # 滤波器库
│       ├── Inc/      # filter.h, filter_config.h
│       └── Src/      # filter.c, filter_config.c
├── lsm6dsr_STdC/      # ST 官方寄存器定义
├── ports/             # 平台桩文件（各平台分支维护）
├── scripts/           # 自动化脚本
└── docs/              # 文档
```

## 滤波器库

滤波器库位于 `Core/Src/filter*.c` 和 `Core/Inc/filter*.h`，提供统一的姿态估计算法接口。

### 设计原则

- 零成本抽象：函数指针调用，无虚函数表开销
- 可扩展：添加新滤波器只需实现 `filter_t` 接口的四个函数指针
- 向后兼容：保留原有互补滤波器作为默认类型

### 滤波器类型

| 类型 | 枚举值 | 说明 |
|------|--------|------|
| Complementary | `FILTER_TYPE_COMPLEMENTARY` | 经典陀螺仪+加速度计融合，α权重控制信任比例 |
| LPF | `FILTER_TYPE_LPF` | 一阶低通滤波器，截止频率可调，适合噪声滤除 |
| EKF | `FILTER_TYPE_EKF` | 7状态扩展卡尔曼滤波器，含陀螺偏置在线估计 |
| Mahony | `FILTER_TYPE_MAHONY` | PI控制器互补滤波，快速收敛，适合动态场景 |
| Madgwick | `FILTER_TYPE_MADGWICK` | 梯度下降法四元数姿态估计，计算量小 |
| LKF | `FILTER_TYPE_LKF` | 6状态线性卡尔曼滤波器，含陀螺偏置在线估计 |

### 退化模式

传感器数据质量差时，滤波器自动或手动切换退化模式：

| 模式 | 说明 |
|------|------|
| `FILTER_DEGRADE_NONE` | 正常运行 |
| `FILTER_DEGRADE_STATIC_ONLY` | 仅静态模式，禁用动态补偿 |
| `FILTER_DEGRADE_GYRO_ONLY` | ACC不可靠时，仅用陀螺仪积分 |
| `FILTER_DEGRADE_ACC_ONLY` | GYRO饱和时，仅用加速度计 |
| `FILTER_DEGRADE_HOLD_LAST` | 冻结输出，保持上次结果 |

### 配置系统

`filter_config.h/c` 提供完整的参数管理：

- 参数来源标注：每个参数标明来源（论文/经验值/传感器手册/调优）
- 范围验证：`filter_config_validate()` 校验参数有效范围
- 预设配置：`FILTER_PRESET_DEFAULT`、`HIGH_PRECISION`、`FAST_RESPONSE`、`ROBUST`、`LOW_POWER`
- 退化策略：根据传感器质量自动选择退化模式

### API 概览

**工厂与生命周期**

| 函数 | 说明 |
|------|------|
| `filter_create(type)` | 创建滤波器实例 |
| `f->update(f, &in, &out)` | 执行一步滤波更新 |
| `f->reset(f)` | 重置滤波器状态 |
| `f->set_param(f, param, value)` | 运行时设置参数 |
| `f->destroy(f)` | 释放滤波器资源 |
| `filter_type_name(type)` | 获取类型名称字符串 |

**退化与质量评估**

| 函数 | 说明 |
|------|------|
| `filter_set_degrade(f, mode)` | 设置退化模式 |
| `filter_check_acc_quality(ax,ay,az)` | 评估加速度计数据质量 |
| `filter_check_gyro_quality(gx,gy,gz)` | 评估陀螺仪数据质量 |

**配置系统**

| 函数 | 说明 |
|------|------|
| `filter_config_get_params(type, &count)` | 获取参数描述表 |
| `filter_config_get_default(type, param)` | 获取参数默认值 |
| `filter_config_validate(type, param, val)` | 验证参数有效性 |
| `filter_config_apply_preset(f, preset)` | 应用预设配置 |

### 使用示例

```c
#include "filter.h"
#include "filter_config.h"

/* 创建滤波器 */
filter_t *f = filter_create(FILTER_TYPE_MADGWICK);
if (!f) { /* 错误处理 */ }

/* 可选：应用预设配置 */
filter_config_apply_preset(f, FILTER_PRESET_HIGH_PRECISION);

/* 循环更新 */
while (1) {
    filter_input_t in = {
        .ax = acc[0], .ay = acc[1], .az = acc[2],
        .gx = gyro[0], .gy = gyro[1], .gz = gyro[2],
        .dt = 0.01f
    };
    filter_output_t out;
    f->update(f, &in, &out);
    printf("pitch=%.2f roll=%.2f yaw=%.2f\n", out.pitch, out.roll, out.yaw);
}

/* 释放 */
f->destroy(f);
```

**运行时切换滤波器**

```c
/* 销毁旧滤波器，创建新滤波器 */
f->destroy(f);
f = filter_create(FILTER_TYPE_EKF);
filter_config_apply_preset(f, FILTER_PRESET_ROBUST);
```

**退化模式手动控制**

```c
/* 传感器数据异常时冻结输出 */
if (!filter_check_acc_quality(ax, ay, az)) {
    filter_set_degrade(f, FILTER_DEGRADE_GYRO_ONLY);
} else {
    filter_set_degrade(f, FILTER_DEGRADE_NONE);
}
```

### BSP 集成

业务层 `bsp_lsm6dsr.c` 封装了滤波器的创建和切换：

- `bsp_lsm6dsr_set_filter(type)` — 切换滤波器类型
- `bsp_lsm6dsr_set_filter_param(param, value)` — 设置滤波器参数

### 测试

滤波器库包含 230 个单元测试（146 + 84），覆盖所有滤波器类型、退化模式、参数验证和边界条件，另有 6 个滤波器收敛诊断测试，全部通过。

## 平台支持

| 平台 | 分支 | 状态 |
|------|------|------|
| STM32F407 | `stm32f407` | 完整实现 |
| MSPM0G3507 | `mspm0g3507` | 桩文件 |
| CH32 | `ch32` | 桩文件 |
| AT32 | `at32` | 桩文件 |

## 快速开始

1. 克隆仓库
2. 切换到目标平台分支：`git checkout stm32f407`
3. 使用 Keil MDK 打开 `MDK-ARM/LSM6DSR_F407_TEST.uvprojx`
4. 编译并烧录

## 通用核心

- `lsm6dsr.c/h` — LSM6DSR 驱动层，通过 `lsm6dsr_io_t` 回调实现 I/O 抽象
- `bsp_lsm6dsr.c/h` — 业务层，6种可切换滤波器、自适应 α、静止检测、偏置跟踪
- `lsm6dsr_STdC/` — ST 官方寄存器定义库

## 文档

- [分支管理规范](docs/BRANCHING.md)
- [BSP 调参指南](docs/bsp_tuning_guide.md)
- [硬件接线说明](docs/hardware_wiring.md)

## 最新改进 (v1.1.0)

### 🔧 关键修复

#### 1. 线程安全 - 多实例支持

重构 BSP 层以支持多个 IMU 实例：

```c
// 新 API（推荐）
bsp_lsm6dsr_ctx_t imu1_ctx, imu2_ctx;

bsp_lsm6dsr_init_ctx(&imu1_ctx);
bsp_lsm6dsr_init_ctx(&imu2_ctx);

while (1) {
    bsp_lsm6dsr_data_t data1, data2;
    bsp_lsm6dsr_update_ctx(&imu1_ctx, &data1);
    bsp_lsm6dsr_update_ctx(&imu2_ctx, &data2);
}

bsp_lsm6dsr_destroy_ctx(&imu1_ctx);
bsp_lsm6dsr_destroy_ctx(&imu2_ctx);
```

**优势**：
- ✅ 支持多 IMU 同时工作
- ✅ RTOS 环境下线程安全
- ✅ 独立的状态管理
- ✅ 易于测试和调试

#### 2. 安全的滤波器销毁

新增 `filter_destroy_safe()` 函数，防止崩溃：

```c
// 推荐：使用安全销毁函数
filter_t *f = filter_create(FILTER_TYPE_EKF);
// ... 使用滤波器 ...
filter_destroy_safe(f);  // 自动处理 NULL 和静态分配

// 旧方式（可能导致崩溃）
f->destroy(f);  // 如果 f 为 NULL 会崩溃
```

#### 3. 统一的四元数归一化

所有滤波器使用统一的归一化实现，确保数值稳定性：

- ✅ 统一阈值：`1e-10f`
- ✅ NaN/Inf 检测和恢复
- ✅ 接近零时重置为单位四元数
- ✅ 防止数值漂移

#### 4. 64 位时间戳支持

修复长时间运行后的时间戳溢出问题：

```c
// 使用 64 位时间戳
typedef struct {
    uint64_t last_tick_us;  // 约 584,942 年才会溢出
    // ...
} bsp_lsm6dsr_ctx_t;
```

#### 5. 完善的错误处理

新增错误码和回调机制：

```c
// 设置错误回调
void my_error_handler(const filter_error_info_t *info, void *user_data) {
    printf("Error: %s at %s:%d\n", info->message, info->file, info->line);
}

filter_set_error_callback(my_error_handler, NULL);

// 创建滤波器
filter_t *f = filter_create(FILTER_TYPE_EKF);
if (!f) {
    filter_error_info_t err = filter_get_last_error();
    printf("Failed: %s (code=%d)\n", err.message, err.code);
}
```

#### 6. 参数验证

自动验证参数有效性，拒绝无效设置：

```c
filter_t *f = filter_create(FILTER_TYPE_COMPLEMENTARY);

// 有效参数
f->set_param(f, FILTER_PARAM_ALPHA, 0.5f);  // ✅ 接受

// 无效参数（自动拒绝，不会崩溃）
f->set_param(f, FILTER_PARAM_ALPHA, 1.5f);  // ❌ 拒绝
f->set_param(f, FILTER_PARAM_ALPHA, NAN);   // ❌ 拒绝
```

### 📊 改进统计

| 项目 | 改进前 | 改进后 |
|------|--------|--------|
| 多实例支持 | ❌ | ✅ |
| 线程安全 | ❌ | ✅ |
| 时间戳溢出 | ❌ | ✅ 64 位 |
| 错误处理 | 基础 | 完善 |
| 参数验证 | ❌ | ✅ |
| 单元测试 | 137 | 183 (+46) |
| 四元数归一化 | 不一致 | 统一 |

### 🚀 迁移指南

#### 从旧版本迁移

**步骤 1：更新头文件引用**

```c
// 旧
#include "bsp_lsm6dsr.h"

// 新（相同，无需修改）
#include "bsp_lsm6dsr.h"
```

**步骤 2：使用新 API（推荐）**

```c
// 旧方式（仍然支持，向后兼容）
bsp_lsm6dsr_init();
while (1) {
    bsp_lsm6dsr_data_t data;
    bsp_lsm6dsr_update(&data);
}

// 新方式（推荐）
bsp_lsm6dsr_ctx_t ctx;
bsp_lsm6dsr_init_ctx(&ctx);
while (1) {
    bsp_lsm6dsr_data_t data;
    bsp_lsm6dsr_update_ctx(&ctx, &data);
}
bsp_lsm6dsr_destroy_ctx(&ctx);
```

**步骤 3：使用安全销毁函数**

```c
// 旧方式（可能崩溃）
f->destroy(f);

// 新方式（推荐）
filter_destroy_safe(f);
```

**步骤 4：添加错误处理**

```c
// 设置错误回调
filter_set_error_callback(my_error_handler, NULL);

// 检查错误
filter_error_info_t err = filter_get_last_error();
if (err.code != FILTER_OK) {
    printf("Error: %s\n", err.message);
}
```

### 📝 最佳实践

#### 1. 嵌入式系统

```c
// 使用静态分配（无 malloc/free）
uint8_t filter_buf[512];
filter_t *f = filter_create_static(FILTER_TYPE_MADGWICK, filter_buf, sizeof(filter_buf));

// 使用上下文 API
bsp_lsm6dsr_ctx_t imu_ctx;
bsp_lsm6dsr_init_ctx(&imu_ctx);
```

#### 2. RTOS 环境

```c
// 每个任务独立的上下文
void imu_task_1(void *param) {
    bsp_lsm6dsr_ctx_t ctx;
    bsp_lsm6dsr_init_ctx(&ctx);
    // ...
}

void imu_task_2(void *param) {
    bsp_lsm6dsr_ctx_t ctx;
    bsp_lsm6dsr_init_ctx(&ctx);
    // ...
}
```

#### 3. 生产环境

```c
// 启用错误日志
filter_set_error_callback(log_error_to_file, NULL);

// 启用参数验证（自动）
// 无需额外配置，set_param 自动验证

// 定期校准
if (bsp_lsm6dsr_is_stationary_ctx(&ctx)) {
    bsp_lsm6dsr_calibrate_ctx(&ctx);
}
```

### 🧪 测试

运行修复验证测试：

```bash
cd test
gcc -o test_fixes test_fixes.c ../Core/Filter/Src/filter.c \
    -I../Core/Filter/Inc -lm -Wall -Wextra -DDEBUG
./test_fixes
```

预期结果：✅ All 46 tests passed!

### 📚 相关文档

- [代码审查报告](docs/CODE_REVIEW_REPORT.md) - 详细的审查结果
- [修复指南](docs/FIX_GUIDE.md) - 具体的修复代码示例
- [改进计划](docs/IMPROVEMENT_PLAN.md) - 完整的改进路线图
- [快速开始清单](docs/QUICK_START_CHECKLIST.md) - 实施检查清单

---

## EKF 增强 (v1.2.0)

### 5 项算法增强

针对强振动、强线性加速度场景（机器狗），增强了 EKF 的鲁棒性：

| 增强 | 说明 | 默认值 |
|------|------|--------|
| **(A) Chi-squared 创新门控** | 检测加速度计离群值，异常时跳过测量更新 | 阈值=11.34 (p=0.01, df=3) |
| **(B) 动态 R 适配** | 加速度幅值偏离 1g 时自动增大 R，降低对加速度的信任 | 默认关闭 |
| **(C) 偏置幅值限制** | 陀螺仪偏置裁剪到 ±bias_limit_dps，防止发散 | ±20 dps |
| **(D) Joseph 形式 + R_eff** | 协方差更新使用动态 R_eff，结合自适应测量噪声 | — |
| **(E) 定期协方差正则化** | 每 100 次更新强制对称、对角线下界/上界 | 下界=1e-10, 上界=1e6 |

### 新增参数

```c
f->set_param(f, FILTER_PARAM_BIAS_LIMIT_DPS, 20.0f);   // 陀螺偏置限幅 (5~50 dps)
f->set_param(f, FILTER_PARAM_CHI2_THRESHOLD, 11.34f);    // Chi2 门限 (>0)
f->set_param(f, FILTER_PARAM_R_ADAPT_ENABLE, 1.0f);     // 启用动态 R 适配 (0/1)
f->set_param(f, FILTER_PARAM_R_ADAPT_FACTOR, 1.0f);     // R 适配系数 (0.1~10.0)
```

### 变更文件

| 文件 | 变更 |
|------|------|
| `Core/Filter/Inc/filter.h` | +4 新增 `FILTER_PARAM_*` 枚举 |
| `Core/Filter/Inc/filter_config.h` | +9 新增 EKF 配置宏 |
| `Core/Filter/Src/filter.c` | +`#include filter_config.h`; `ekf_priv_t` +5 字段; `ekf_update()` +5 算法增强; `ekf_set_param()` +4 case; `validate_filter_param()` +4 验证; `ekf_reset()` +2 字段重置 |

### 测试结果

| 测试套件 | 结果 |
|----------|------|
| test_filters.exe | 146/146 PASS (100%) |
| test_fixes.exe | 84/84 PASS (100%) |
| test_convergence.exe | 6/6 滤波器收敛 |

---

**版本历史**：
- v1.0.0 - 初始版本
- v1.1.0 - 关键修复和改进（线程安全、错误处理、参数验证等）
- v1.2.0 - EKF 增强（Chi-squared 门控、动态 R 适配、偏置截断、协方差正则化）
