# LSM6DSR MSPM0G3507 移植包

## 概述

本目录包含 LSM6DSR IMU 驱动在 MSPM0G3507 上的完整移植实现，支持三种通信方式：
- **硬件 I2C**（默认）
- **软件 I2C**（与 MPU6050 共用引脚时使用）
- **硬件 SPI**（高速通信）

## 目录结构

```
porting/
├── README.md                    # 本文件
├── AI_ONBOARDING.md            # AI 理解项目的关键提示词
├── src/
│   ├── i2c_bridge.c            # 硬件 I2C 桥接层
│   ├── i2c_bridge_soft.c       # 软件 I2C 桥接层
│   ├── spi_bridge.c            # 硬件 SPI 桥接层
│   ├── platform_mspm0.c        # 平台抽象实现
│   └── lsm6dsr_adapter.c       # 官方 ST 驱动适配器
├── inc/
│   └── lsm6dsr_adapter.h       # 适配器头文件
├── docs/
│   ├── hardware_setup.md       # 硬件配置说明
│   ├── api_reference.md        # API 参考手册
│   └── porting_guide.md        # 移植指南
└── examples/
    └── main_example.c          # 完整使用示例
```

## 快速开始

### 1. 选择通信方式

在 `project_config.h` 中定义宏：

```c
/* 选择一种通信方式 */
// #define USE_SPI           /* 硬件 SPI */
// #define USE_SOFT_I2C      /* 软件 I2C */
/* 默认使用硬件 I2C */
```

### 2. 添加源文件到工程

| 文件 | 说明 |
|------|------|
| `src/i2c_bridge.c` | 硬件 I2C（默认） |
| `src/i2c_bridge_soft.c` | 软件 I2C（可选） |
| `src/spi_bridge.c` | 硬件 SPI（可选） |
| `src/platform_mspm0.c` | 平台实现（必须） |
| `src/lsm6dsr_adapter.c` | 官方驱动适配器（可选） |

### 3. 添加包含路径

```
Core/Inc
Core/Filter/Inc
ports/mspm0g3507/porting/inc
lsm6dsr_STdC/driver
```

### 4. 初始化

```c
#include "platform.h"
#include "bsp_lsm6dsr.h"

int main(void)
{
    SYSCFG_DL_init();
    platform_timer_init();

    /* 根据选择的通信方式初始化 */
    #ifdef USE_SPI
        spi_bridge_init();
    #elif defined(USE_SOFT_I2C)
        soft_i2c_bridge_init();
    #endif

    bsp_lsm6dsr_init();

    while (1) {
        bsp_lsm6dsr_data_t data;
        bsp_lsm6dsr_update(&data);
        // ... 使用数据
    }
}
```

## 硬件连接

### I2C 模式

| MSPM0G3507 | LSM6DSR | 功能 |
|------------|---------|------|
| PB6 | SCL (Pin 13) | I2C 时钟 |
| PB7 | SDA (Pin 14) | I2C 数据 |

### SPI 模式

| MSPM0G3507 | LSM6DSR | 功能 |
|------------|---------|------|
| PB9 | SCK (Pin 13) | SPI 时钟 |
| PB8 | SDA/SDI (Pin 14) | SPI MOSI |
| PB7 | SDO/SDO (Pin 12) | SPI MISO |
| PB17 | CS (Pin 15) | SPI 片选 |

**SPI 协议注意事项**：
- 读操作：CS↓ → [REG|0x80] → 排空RX → [DUMMY]→读取 → CS↑
- 写操作：CS↓ → [REG&0x7F] → 排空RX → [DATA]→排空RX → CS↑
- MSPM0 DL API 是底层 API，不自动处理全双工，必须手动排空 RX FIFO

### 软件 I2C 模式

| MSPM0G3507 | LSM6DSR | 功能 |
|------------|---------|------|
| PA0 | SCL (Pin 13) | 软件 I2C 时钟 |
| PA1 | SDA (Pin 14) | 软件 I2C 数据 |

## 编译配置

### Keil MDK

1. 添加源文件到工程
2. 添加包含路径
3. 添加预定义宏：`LOG_LEVEL=LOG_LEVEL_INFO`

### SysConfig

1. **I2C0**：Controller 模式，400kHz（I2C 模式）
2. **SPI1**：Controller 模式，10MHz，Mode 0（SPI 模式）
3. **TimerG0**：1MHz 微秒计时器
4. **UART0**：115200 baud 调试串口

## 验证步骤

1. **编译验证**：Keil 编译无错误
2. **WHO_AM_I 验证**：读取寄存器 0x0F，应返回 0x6B
3. **数据读取验证**：读取 ACC/GYRO 数据，检查是否合理
4. **滤波器验证**：通过 VOFA+ 输出 pitch/roll/yaw 波形

## 相关文档

- [硬件配置说明](docs/hardware_setup.md)
- [API 参考手册](docs/api_reference.md)
- [移植指南](docs/porting_guide.md)
- [AI 理解项目提示词](AI_ONBOARDING.md)
