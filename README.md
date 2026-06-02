# LSM6DSR IMU 姿态估计库

基于 LSM6DSR 六轴 IMU 的互补滤波姿态估计实现，面向机器狗 (robot dog) 等强振动、强线性加速度场景。

## 架构

```
├── Core/Src/          # 通用核心源文件
│   ├── lsm6dsr.c      # 驱动层（平台无关）
│   └── bsp_lsm6dsr.c  # 业务层（滤波、偏置跟踪）
├── Core/Inc/          # 通用核心头文件
├── lsm6dsr_STdC/      # ST 官方寄存器定义
├── ports/             # 平台桩文件（各平台分支维护）
├── scripts/           # 自动化脚本
└── docs/              # 文档
```

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
- `bsp_lsm6dsr.c/h` — 业务层，互补滤波器、自适应 α、静止检测、偏置跟踪
- `lsm6dsr_STdC/` — ST 官方寄存器定义库

## 文档

- [分支管理规范](docs/BRANCHING.md)
- [BSP 调参指南](docs/bsp_tuning_guide.md)
- [硬件接线说明](Doc/hardware_wiring.md)
