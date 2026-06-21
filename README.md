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
- MCU友好：支持静态分配，无动态内存依赖

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
| `filter_create(type)` | 创建滤波器实例（动态分配，仅PC测试） |
| `filter_create_static(type, buf, size)` | 创建滤波器实例（静态分配，MCU推荐） |
| `f->update(f, &in, &out)` | 执行一步滤波更新 |
| `f->reset(f)` | 重置滤波器状态 |
| `f->set_param(f, param, value)` | 运行时设置参数 |
| `f->destroy(f)` | 释放滤波器资源（静态分配时无需调用） |
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

**PC测试（动态分配）**
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

**嵌入式系统（静态分配，推荐）**
```c
#include "filter.h"
#include "filter_config.h"

/* 静态缓冲区 */
static uint8_t filter_buf[512] __attribute__((aligned(4)));

/* 创建滤波器 */
size_t size = filter_get_static_size(FILTER_TYPE_MADGWICK);
assert(size <= sizeof(filter_buf));
filter_t *f = filter_create_static(FILTER_TYPE_MADGWICK, filter_buf, sizeof(filter_buf));
if (!f) { /* 错误处理 */ }

/* 循环更新（无需调用destroy） */
while (1) {
    filter_input_t in = {
        .ax = acc[0], .ay = acc[1], .az = acc[2],
        .gx = gyro[0], .gy = gyro[1], .gz = gyro[2],
        .dt = 0.01f
    };
    filter_output_t out;
    f->update(f, &in, &out);
    // 使用输出...
}
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

### 编译选项

| 宏定义 | 说明 |
|--------|------|
| `FILTER_DISABLE_DYNAMIC_ALLOC` | 禁用动态内存分配，仅允许静态分配（推荐嵌入式使用） |
| `FILTER_STATIC_ONLY` | 内部宏，由 `FILTER_DISABLE_DYNAMIC_ALLOC` 自动定义 |

**嵌入式系统推荐编译选项：**
```bash
gcc -DFILTER_DISABLE_DYNAMIC_ALLOC ...
```

### 测试

滤波器库包含 179 个单元测试，覆盖所有滤波器类型、退化模式、参数验证和边界条件，全部通过。

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
- [硬件接线说明](Doc/hardware_wiring.md)
