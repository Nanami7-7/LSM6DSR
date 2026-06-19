# MSPM0G3507 平台实现

## 概述

LSM6DSR IMU 驱动在 MSPM0G3507 上的平台适配层。

## 文件结构

```
ports/mspm0g3507/
├── platform_mspm0.c    # 平台抽象接口实现（延时、计时、调试输出）
├── i2c_bridge.c        # I2C 桥接层（lsm6dsr_io_t 回调实现）
├── spi_bridge.c        # SPI 桥接层（保留接口，暂未实现）
└── README.md           # 本文件
```

## 硬件配置

| 项目 | 配置 |
|------|------|
| MCU | MSPM0G3507 |
| 系统时钟 | 80 MHz |
| I2C 实例 | I2C0 |
| I2C 速率 | 400 kHz (Fast Mode) |
| I2C 地址 | 0x6A (SDO/SA0 接 GND) |
| 计时器 | TimerG0 (1MHz, 微秒级) |
| 调试串口 | UART0, 115200 baud |

## 引脚分配

| MSPM0G3507 | LSM6DSR | 功能 |
|------------|---------|------|
| PB6 | SCL (Pin 13) | I2C 时钟 |
| PB7 | SDA (Pin 14) | I2C 数据 |
| PA9 | UART0_TX | 调试串口 |
| PA10 | UART0_RX | 调试串口 |

## 编译配置

### Keil MDK

1. 添加源文件到工程：
   - `ports/mspm0g3507/platform_mspm0.c`
   - `ports/mspm0g3507/i2c_bridge.c`
   - `ports/mspm0g3507/spi_bridge.c`

2. 添加包含路径：
   - `Core/Inc`
   - `Core/Filter/Inc`
   - `ports/mspm0g3507`

3. 预定义宏：
   ```
   __MSPM0G3507__
   LOG_LEVEL=LOG_LEVEL_INFO
   ```

### SysConfig 配置

使用 TI SysConfig 工具配置外设：

1. **I2C0**：
   - 模式：Controller
   - 速率：400 kHz
   - 引脚：SCL=PB6, SDA=PB7

2. **UART0**：
   - 波特率：115200
   - 数据位：8
   - 停止位：1
   - 引脚：TX=PA9, RX=PA10

3. **TimerG0**：
   - 模式：周期计时器
   - 时钟：80 MHz
   - 分频：80 → 1 MHz
   - 周期：最大值 (32-bit)

## 使用示例

```c
#include "ti_msp_dl_config.h"
#include "platform.h"
#include "bsp_lsm6dsr.h"

int main(void)
{
    /* 1. 初始化系统外设 */
    SYSCFG_DL_init();
    
    /* 2. 启动计时器 */
    platform_timer_init();
    
    /* 3. 初始化 IMU */
    bsp_lsm6dsr_init();
    
    /* 4. 主循环 */
    bsp_lsm6dsr_data_t data;
    char buf[128];
    
    while (1) {
        bsp_lsm6dsr_update(&data);
        int len = bsp_lsm6dsr_vofa_format(buf, sizeof(buf), &data);
        g_platform->debug_printf("%s", buf);
        g_platform->delay_ms(10);
    }
}
```

## 调试

### 日志级别

编译时通过 `-DLOG_LEVEL=N` 控制：

| 级别 | 值 | 说明 |
|------|-----|------|
| LOG_LEVEL_NONE | 0 | 关闭所有日志 |
| LOG_LEVEL_ERROR | 1 | 仅错误 |
| LOG_LEVEL_WARN | 2 | 警告 + 错误 |
| LOG_LEVEL_INFO | 3 | 信息（默认） |
| LOG_LEVEL_DEBUG | 4 | 所有日志 |

### VOFA+ 波形

1. 连接串口（115200, 8N1）
2. VOFA+ 协议：FireWater
3. 通道顺序：ax, ay, az, gx, gy, gz, pitch, roll, yaw, temp

## 已知限制

1. **SPI 未实现**：当前仅支持 I2C，SPI 接口保留供将来使用
2. **计时器溢出**：TimerG0 为 32 位，约 71.6 分钟溢出一次，dt 计算正确处理溢出
3. **printf 阻塞**：调试输出使用阻塞发送，大量输出可能影响实时性

## 后续工作

- [ ] 实现 SPI 桥接层
- [ ] 添加 DMA 支持（减少 CPU 占用）
- [ ] 添加低功耗模式支持
- [ ] 添加中断驱动的数据就绪检测
