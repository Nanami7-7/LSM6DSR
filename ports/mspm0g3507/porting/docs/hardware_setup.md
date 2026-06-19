# 硬件配置说明

## MSPM0G3507 LaunchPad 配置

### 系统时钟

| 项目 | 配置 |
|------|------|
| CPU 时钟 | 80 MHz |
| Bus 时钟 | 80 MHz |
| TimerG0 | 1 MHz（微秒计时器） |

### 引脚分配

#### I2C 模式（默认）

| MSPM0G3507 | LSM6DSR | 功能 | 说明 |
|------------|---------|------|------|
| PB6 | SCL (Pin 13) | I2C0_SCL | 需要 4.7kΩ 上拉 |
| PB7 | SDA (Pin 14) | I2C0_SDA | 需要 4.7kΩ 上拉 |
| PA9 | — | UART0_TX | 调试串口 |
| PA10 | — | UART0_RX | 调试串口 |

#### SPI 模式

| MSPM0G3507 | LSM6DSR | 功能 | 说明 |
|------------|---------|------|------|
| PB9 | SCK (Pin 13) | SPI1_SCK | 10 MHz |
| PB8 | SDA/SDI (Pin 14) | SPI1_PICO | MOSI |
| PB7 | SDO (Pin 12) | SPI1_POCI | MISO |
| PB17 | CS (Pin 15) | GPIO_CS | 手动控制 |
| PA9 | — | UART0_TX | 调试串口 |
| PA10 | — | UART0_RX | 调试串口 |

#### 软件 I2C 模式

| MSPM0G3507 | LSM6DSR | 功能 | 说明 |
|------------|---------|------|------|
| PA0 | SCL (Pin 13) | GPIO_SCL | 需要 4.7kΩ 上拉 |
| PA1 | SDA (Pin 14) | GPIO_SDA | 需要 4.7kΩ 上拉 |
| PA9 | — | UART0_TX | 调试串口 |
| PA10 | — | UART0_RX | 调试串口 |

### SysConfig 配置

#### I2C0（I2C 模式）

```json
{
  "name": "I2C0",
  "mode": "Controller",
  "clockRate": 400000,
  "pins": {
    "scl": "PB6",
    "sda": "PB7"
  }
}
```

#### SPI1（SPI 模式）

```json
{
  "name": "SPI1",
  "mode": "Controller",
  "clockRate": 10000000,
  "frameFormat": "Motorola 4-Wire",
  "polarity": 0,
  "phase": 0,
  "dataSize": 8,
  "bitOrder": "MSB First",
  "pins": {
    "sck": "PB9",
    "pico": "PB8",
    "poci": "PB7"
  }
}
```

#### TimerG0（微秒计时器）

```json
{
  "name": "TimerG0",
  "mode": "Periodic",
  "clockSource": "BUSCLK",
  "prescaler": 80,
  "period": 4294967295
}
```

#### UART0（调试串口）

```json
{
  "name": "UART0",
  "baudRate": 115200,
  "dataBits": 8,
  "stopBits": 1,
  "pins": {
    "tx": "PA9",
    "rx": "PA10"
  }
}
```

### 电源配置

| 项目 | 配置 |
|------|------|
| VDD | 3.3V |
| VDDIO | 3.3V |
| 去耦电容 | 100nF + 10μF |

### 注意事项

1. **I2C 上拉电阻**：必须接 4.7kΩ 上拉到 VDDIO
2. **SPI CS 引脚**：使用 GPIO 手动控制，不用硬件 CS
3. **SPI RX FIFO 排空**：MSPM0 DL API 是底层 API，不自动处理全双工。每次发送后必须排空 RX FIFO（参考 spi_bridge.c 实现）
4. **软件 I2C 引脚**：PA0/PA1 与 MPU6050 共用，不能同时使用
5. **TimerG0**：用于微秒级计时，溢出周期约 71.6 分钟
