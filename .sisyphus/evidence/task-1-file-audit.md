# 文件审计报告：通用 vs 平台特定分类

> 生成时间：2026-06-02
> 仓库：LSM6DSR (STM32F407VET6 + LSM6DSR I2C IMU 姿态估计工程)
> 分支：stm32f407

## 分类标准

| 分类 | 定义 |
|------|------|
| **通用文件（平台无关）** | 不依赖任何特定 MCU/HAL/IDE 的源文件，可通过 `lsm6dsr_io_t` I/O 抽象层在任意平台复用 |
| **平台特定文件** | 依赖 STM32 HAL、CMSIS、Keil MDK-ARM 或 CubeMX 生成代码的文件，不可直接移植到其他平台 |

---

## 一、通用文件（平台无关）

### 1.1 项目驱动层 — `lsm6dsr.c/h`（4 个文件）

| 文件路径 | 说明 |
|----------|------|
| `Core/Src/lsm6dsr.c` | LSM6DSR 驱动层实现（592 行）。通过 `lsm6dsr_io_t` 回调完成所有 I/O，平台无关。覆盖：寄存器读写、ACC/GYRO/TEMP 数据读取（raw + float）、FIFO 全部模式、自检、功耗模式控制 |
| `Core/Inc/lsm6dsr.h` | 驱动层头文件（321 行）。定义 `lsm6dsr_io_t` I/O 抽象结构体（read/write 回调 + ctx 指针）、寄存器地址映射、枚举类型、灵敏度常量、所有驱动函数原型 |

**可移植性说明**：`lsm6dsr_io_t` 结构体是平台解耦的关键。上层只需实现 read/write 回调即可在任意 MCU 上使用本驱动。

### 1.2 项目业务层 — `bsp_lsm6dsr.c/h`（2 个文件）

| 文件路径 | 说明 |
|----------|------|
| `Core/Src/bsp_lsm6dsr.c` | BSP 业务层实现（426 行）。互补滤波器（pitch/roll/yaw）、自适应 α（运动 0.99 / 静止 0.30）、三重静止检测（方差+幅值+陀螺）、Runtime 偏置跟踪、DWT 计时、VOFA+ 格式化 |
| `Core/Inc/bsp_lsm6dsr.h` | BSP 业务层头文件（153 行）。可配置宏（`#ifndef` 允许编译器 `-D` 覆盖）、`bsp_lsm6dsr_data_t` 数据结构、生产 API 声明 |

**可移植性说明**：`bsp_lsm6dsr.c` 通过 `extern lsm6dsr_io_t lsm6dsr_io` 引用驱动层 I/O 实例，自身不直接调用任何 HAL 函数。唯一平台相关点是 DWT 计时器（`CoreDebug->DEMCR` / `DWT->CYCCNT`），属于 ARM Cortex-M 通用特性，非 STM32 特定。

### 1.3 ST 官方 LSM6DSR Standard C 驱动 — `lsm6dsr_STdC/`（15 个文件）

| 文件路径 | 说明 |
|----------|------|
| `lsm6dsr_STdC/driver/lsm6dsr_reg.c` | ST 官方 LSM6DSR 寄存器驱动（9897 行），BSD-3 许可证 |
| `lsm6dsr_STdC/driver/lsm6dsr_reg.h` | ST 官方寄存器映射头文件 |
| `lsm6dsr_STdC/example/activity.c` | 活动检测示例 |
| `lsm6dsr_STdC/example/compressed_fifo.c` | 压缩 FIFO 示例 |
| `lsm6dsr_STdC/example/fifo_pedo.c` | FIFO + 计步器示例 |
| `lsm6dsr_STdC/example/free_fall.c` | 自由落体检测示例 |
| `lsm6dsr_STdC/example/multi_read_fifo_simple.c` | 多次读取 FIFO 示例 |
| `lsm6dsr_STdC/example/orientation_6d_4d.c` | 6D/4D 方向检测示例 |
| `lsm6dsr_STdC/example/read_data_interrupt.c` | 中断读取数据示例 |
| `lsm6dsr_STdC/example/read_data_simple.c` | 简单读取数据示例 |
| `lsm6dsr_STdC/example/read_data_simple_offset.c` | 带偏移读取数据示例 |
| `lsm6dsr_STdC/example/read_data_simple_timestamp.c` | 带时间戳读取数据示例 |
| `lsm6dsr_STdC/example/single_double_tap.c` | 单击/双击检测示例 |
| `lsm6dsr_STdC/example/tilt.c` | 倾斜检测示例 |
| `lsm6dsr_STdC/example/wake_up.c` | 唤醒检测示例 |

**小计：15 个 .c 文件 + 2 个 .h 文件（驱动目录）+ 1 个 README.txt**

---

## 二、平台特定文件

### 2.1 测试层 — `test_lsm6dsr.c/h`（2 个文件）

| 文件路径 | 说明 |
|----------|------|
| `Core/Src/test_lsm6dsr.c` | 测试层实现（1163 行）。**直接调用 `HAL_I2C_Mem_Read/Write`** 实现 I2C 桥接，定义 `lsm6dsr_io` 实例，包含 P1~P19 共 19 项传感器合格性+性能测试 |
| `Core/Inc/test_lsm6dsr.h` | 测试层头文件（46 行）。测试函数声明、PASS/FAIL 宏 |

**平台绑定原因**：`stm32_i2c_read/write` 函数直接调用 STM32 HAL 库的 `HAL_I2C_Mem_Read/Write`，绑定 `I2C_HandleTypeDef`。

### 2.2 CubeMX 生成代码 — `Core/Src/` + `Core/Inc/`（13 个文件）

| 文件路径 | 说明 |
|----------|------|
| `Core/Src/main.c` | 应用入口（79 行）。HAL 初始化、外设初始化、`bsp_lsm6dsr_init()`、VOFA+ 输出循环。直接调用 STM32 HAL 函数 |
| `Core/Inc/main.h` | 主头文件，包含 STM32 HAL 头文件 |
| `Core/Src/i2c.c` | I2C1 外设初始化（CubeMX 生成），400kHz Fast Mode |
| `Core/Inc/i2c.h` | I2C 头文件（CubeMX 生成） |
| `Core/Src/usart.c` | USART1 外设初始化（CubeMX 生成），115200 8N1 |
| `Core/Inc/usart.h` | USART 头文件（CubeMX 生成） |
| `Core/Src/gpio.c` | GPIO 初始化（CubeMX 生成） |
| `Core/Inc/gpio.h` | GPIO 头文件（CubeMX 生成） |
| `Core/Src/stm32f4xx_it.c` | 中断服务程序（CubeMX 生成） |
| `Core/Inc/stm32f4xx_it.h` | 中断服务头文件（CubeMX 生成） |
| `Core/Src/stm32f4xx_hal_msp.c` | HAL MSP 初始化（CubeMX 生成） |
| `Core/Src/system_stm32f4xx.c` | 系统时钟配置（CubeMX 生成） |
| `Core/Inc/stm32f4xx_hal_conf.h` | HAL 库配置（CubeMX 生成） |

**平台绑定原因**：所有文件均依赖 STM32 HAL 库（`stm32f4xx_hal.h`），由 CubeMX 工具生成，绑定 STM32F4xx 系列 MCU。

### 2.3 Keil MDK-ARM 项目 — `MDK-ARM/`（5 个文件）

| 文件路径 | 说明 |
|----------|------|
| `MDK-ARM/LSM6DSR_F407_TEST.uvprojx` | Keil uVision5 项目文件 |
| `MDK-ARM/LSM6DSR_F407_TEST.uvoptx` | Keil uVision5 项目选项 |
| `MDK-ARM/LSM6DSR_F407_TEST.uvguix.59745` | Keil uVision5 GUI 配置 |
| `MDK-ARM/startup_stm32f407xx.s` | STM32F407 启动汇编文件 |
| `MDK-ARM/startup_stm32f407xx.lst` | 启动文件列表文件 |

**平台绑定原因**：Keil MDK-ARM 专用项目配置，启动文件为 STM32F407 特定汇编。

### 2.4 STM32 HAL/LL 驱动 — `Drivers/STM32F4xx_HAL_Driver/`（约 195 个文件）

| 目录 | 文件数 | 说明 |
|------|--------|------|
| `Drivers/STM32F4xx_HAL_Driver/Src/` | ~95 个 .c 文件 | STM32F4xx HAL/LL 驱动源文件（HAL、LL、DMA、GPIO、I2C、SPI、UART、TIM、ADC、DAC、RTC、RNG、SD、ETH、USB、CAN 等） |
| `Drivers/STM32F4xx_HAL_Driver/Inc/` | ~100 个 .h 文件 | STM32F4xx HAL/LL 驱动头文件 |

**平台绑定原因**：ST 官方 STM32F4xx HAL/LL 库，严格绑定 STM32F4 系列 MCU。

### 2.5 CMSIS — `Drivers/CMSIS/`（约 170+ 个文件）

| 目录 | 文件数 | 说明 |
|------|--------|------|
| `Drivers/cmsis/Core/Include/` | ~30 个 .h 文件 | ARM CMSIS-Core 头文件（core_cm4.h 等） |
| `Drivers/cmsis/Core_A/Include/` | ~9 个 .h 文件 | ARM CMSIS-Core-A 头文件 |
| `Drivers/cmsis/DSP/Include/` | ~60 个 .h 文件 | ARM CMSIS-DSP 头文件 |
| `Drivers/cmsis/DSP/Source/` | 若干 | ARM CMSIS-DSP 源文件模板 |
| `Drivers/cmsis/DAP/Firmware/` | ~2 个文件 | CMSIS-DAP 调试固件 |
| `Drivers/cmsis/RTOS/` + `RTOS2/` | ~4 个文件 | CMSIS-RTOS 头文件模板 |
| `Drivers/cmsis/Device/ST/STM32F4xx/` | ~70 个 .s 文件 | STM32F4xx 启动文件模板（arm/gcc/iar 三套工具链） |

**平台绑定原因**：CMSIS 是 ARM Cortex-M 架构标准库，STM32F4xx 设备文件绑定 STM32F4 系列。启动文件模板按工具链（arm/gcc/iar）和芯片型号细分。

---

## 三、统计汇总

| 分类 | .c 文件 | .h 文件 | 其他文件 | 合计 |
|------|---------|---------|----------|------|
| **通用文件** | 17 | 2 | 1 (README.txt) | **20** |
| **平台特定 — 测试层** | 1 | 1 | — | **2** |
| **平台特定 — CubeMX 生成** | 7 | 6 | — | **13** |
| **平台特定 — MDK-ARM** | — | — | 5 | **5** |
| **平台特定 — HAL/LL 驱动** | ~95 | ~100 | — | **~195** |
| **平台特定 — CMSIS** | — | ~100+ | ~70 (.s) | **~170+** |
| **总计** | **~120** | **~209** | **~76** | **~405** |

---

## 四、关键发现

1. **通用核心仅 6 个文件**：`lsm6dsr.c/h`（驱动层）+ `bsp_lsm6dsr.c/h`（业务层）+ `lsm6dsr_reg.c/h`（ST 官方寄存器驱动）。这 6 个文件是跨平台复用的核心资产。

2. **`test_lsm6dsr.c/h` 是平台边界**：虽然在 `Core/` 目录下，但因直接调用 `HAL_I2C_Mem_Read/Write` 而绑定 STM32。如需移植，只需重写这两个文件中的 I2C 桥接函数。

3. **`bsp_lsm6dsr.c` 几乎完全平台无关**：通过 `extern lsm6dsr_io_t lsm6dsr_io` 间接访问硬件，唯一平台相关点是 DWT 计时器（ARM Cortex-M 通用特性）。

4. **大量 HAL/CMSIS 文件为 STM32CubeMX 生成**：约 365+ 个文件属于 ST 官方库，不应手动编辑，移植时需替换为目标平台的 HAL 库。

5. **三层架构设计良好**：驱动层 → 业务层 → 测试层的分层使得通用代码（前两层）与平台特定代码（第三层）边界清晰。

---

## 五、移植指南（简要）

如需将本项目移植到其他 MCU 平台：

| 步骤 | 操作 | 涉及文件 |
|------|------|----------|
| 1 | 复制通用核心 | `lsm6dsr.c/h`、`bsp_lsm6dsr.c/h` |
| 2 | 实现 I2C 桥接 | 重写 `test_lsm6dsr.c` 中的 `stm32_i2c_read/write`（或新建等效文件） |
| 3 | 替换 HAL 库 | 移除 `Drivers/STM32F4xx_HAL_Driver/`，引入目标平台 HAL |
| 4 | 替换启动代码 | 移除 `MDK-ARM/`，使用目标平台 IDE 和启动文件 |
| 5 | 重新配置外设 | 替换 `Core/Src/i2c.c`、`usart.c`、`gpio.c` 等 CubeMX 生成代码 |
