# Learnings

## Task 3: 创建 stm32f407 分支

### 发现
- stm32f407 分支已存在（指向 0bf1821），包含所有平台特定文件
- 平台文件在 master 上通过 5 个提交被移除（7f62388→24c5693）
- 使用 `git checkout -b` 时如果分支已存在会报错

### 平台特定文件清单
**Core/Src/**
- test_lsm6dsr.c — 测试层（STM32 HAL I2C 桥接 + P1~P19）
- main.c — 主程序入口
- stm32f4xx_it.c — 中断处理
- system_stm32f4xx.c — 系统初始化
- gpio.c, i2c.c, usart.c — 外设初始化
- stm32f4xx_hal_msp.c — HAL MSP 配置

**Core/Inc/**
- test_lsm6dsr.h — 测试层头文件
- stm32f4xx_hal_conf.h — HAL 配置
- main.h, gpio.h, i2c.h, usart.h, stm32f4xx_it.h

**MDK-ARM/**
- LSM6DSR_F407_TEST.uvprojx — Keil MDK 项目文件
- startup_stm32f407xx.s — 启动文件

**Drivers/** — STM32 HAL 驱动（整个目录）

### 验证命令
```bash
git branch -a | grep stm32f407
git ls-tree -r stm32f407 --name-only | grep test_lsm6dsr.c
git ls-tree -r stm32f407 --name-only | grep .uvprojx
```

## Task 4: 创建 AT32 平台桩文件 (2026-06-02)

### 桩文件结构
- `ports/at32/test_lsm6dsr.c` — AT32 I2C 桥接桩（定义 lsm6dsr_io 实例）
- `ports/at32/test_lsm6dsr.h` — 测试层头文件桩（函数声明）
- `ports/at32/README.md` — 平台实现说明

### 关键发现
- lsm6dsr_io_t 是平台边界，BSP 层通过 extern 引用
- AT32 使用 i2c_transfer7() 或寄存器操作，非 STM32 HAL
- DWT 周期计数器在 ARM Cortex-M 上通用（AT32 和 STM32 相同）

### 桩文件设计原则
- 不复制 STM32 代码，只提供函数签名和 TODO 注释
- I2C 读写函数返回 -1（未实现），避免链接错误
- 保留 lsm6dsr_io 全局实例，BSP 层可正常编译

### 平台差异点
| 项目 | STM32 | AT32 |
|------|-------|------|
| HAL 库 | STM32 HAL | AT32 HAL (at32f403a_407.h) |
| I2C API | HAL_I2C_Mem_Read/Write | i2c_transfer7 |
| UART | HAL_UART | AT32 USART |

### 通用核心文件（从 Task 2 继承）
- lsm6dsr.c/h — 驱动层
- bsp_lsm6dsr.c/h — 业务层
- lsm6dsr_reg.c/h — ST 官方寄存器定义
