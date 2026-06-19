# 移植指南

## 概述

本指南说明如何将 LSM6DSR 驱动移植到其他 MSPM0 平台或不同的 MCU。

## 移植到其他 MSPM0 平台

### 步骤 1：修改硬件配置宏

编辑 `ports/mspm0g3507/i2c_bridge.c`：

```c
/* I2C 实例 */
#define LSM6DSR_I2C_INST    I2C0  // 改为你的 I2C 实例

/* 超时参数 */
#define I2C_TIMEOUT_CYCLES  4000000UL  // 根据时钟频率调整
```

编辑 `ports/mspm0g3507/spi_bridge.c`：

```c
/* SPI 实例 */
#define LSM6DSR_SPI_INST    SPI1  // 改为你的 SPI 实例

/* CS 引脚 */
#define LSM6DSR_CS_PORT     GPIOB
#define LSM6DSR_CS_PIN      DL_GPIO_PIN_17
#define LSM6DSR_CS_IOMUX    IOMUX_PINCM44
```

### 步骤 2：修改平台实现

编辑 `ports/mspm0g3507/platform_mspm0.c`：

```c
/* 系统时钟频率 */
.system_clock_hz = 80000000,  // 改为你的时钟频率

/* 延时函数 */
static void mspm0_delay_ms(uint32_t ms)
{
    // 根据你的时钟频率调整
    DL_Common_delayCycles(80000UL * ms);
}

/* 计时器 */
static uint32_t mspm0_get_tick_us(void)
{
    return DL_Timer_getTimerCount(TIMG0);  // 改为你的定时器
}
```

### 步骤 3：修改 SysConfig

使用 TI SysConfig 工具配置外设：

1. **I2C0**：Controller 模式，400kHz
2. **SPI1**：Controller 模式，10MHz，Mode 0
3. **TimerG0**：1MHz 微秒计时器
4. **UART0**：115200 baud 调试串口

### 步骤 4：编译验证

```bash
# Keil MDK
# 1. 添加源文件到工程
# 2. 添加包含路径
# 3. 编译

# GCC
gcc -o test_filters.exe test/test_filters.c Core/Filter/Src/filter.c Core/Filter/Src/filter_config.c -ICore/Filter/Inc -lm -Wall -Wextra
```

## 移植到其他 MCU

### 步骤 1：创建新的桥接层

创建 `ports/your_mcu/i2c_bridge.c`：

```c
#include "platform.h"
#include "lsm6dsr.h"

/* 你的 MCU I2C 实现 */
static int8_t your_mcu_i2c_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len)
{
    // 实现 I2C 读取
    // 1. 发送寄存器地址
    // 2. 读取数据
    // 3. 返回 0=成功, -1=失败
}

static int8_t your_mcu_i2c_write(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    // 实现 I2C 写入
    // 1. 发送寄存器地址
    // 2. 发送数据
    // 3. 返回 0=成功, -1=失败
}

lsm6dsr_io_t lsm6dsr_io = {
    .read  = your_mcu_i2c_read,
    .write = your_mcu_i2c_write,
    .ctx   = NULL,
};
```

### 步骤 2：创建平台实现

创建 `ports/your_mcu/platform.c`：

```c
#include "platform.h"

static void your_mcu_delay_ms(uint32_t ms)
{
    // 实现毫秒延时
}

static uint32_t your_mcu_get_tick_us(void)
{
    // 实现微秒时间戳
    return 0;
}

static int your_mcu_debug_printf(const char *fmt, ...)
{
    // 实现调试输出
    return 0;
}

const platform_t your_mcu_platform = {
    .delay_ms       = your_mcu_delay_ms,
    .delay_us       = NULL,
    .get_tick_us    = your_mcu_get_tick_us,
    .system_clock_hz = 80000000,
    .debug_printf   = your_mcu_debug_printf,
};

const platform_t *g_platform = &your_mcu_platform;
```

### 步骤 3：创建初始化函数

```c
void platform_timer_init(void)
{
    // 初始化你的定时器
}
```

## 添加新滤波器

### 步骤 1：定义私有数据结构

```c
typedef struct {
    float alpha;
    float pitch, roll, yaw;
} my_filter_priv_t;
```

### 步骤 2：实现滤波器接口

```c
static void my_filter_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    my_filter_priv_t *p = (my_filter_priv_t *)self->priv;
    // 实现滤波算法
}

static void my_filter_reset(filter_t *self)
{
    my_filter_priv_t *p = (my_filter_priv_t *)self->priv;
    // 重置状态
}

static void my_filter_set_param(filter_t *self, filter_param_t param, float value)
{
    my_filter_priv_t *p = (my_filter_priv_t *)self->priv;
    // 设置参数
}

static void my_filter_destroy(filter_t *self)
{
    if (self->is_static) return;
    free(self->priv);
    free(self);
}
```

### 步骤 3：实现工厂函数

```c
filter_t* filter_create_my_filter(float alpha)
{
    filter_t *f = (filter_t *)malloc(sizeof(filter_t));
    if (!f) return NULL;

    my_filter_priv_t *p = (my_filter_priv_t *)malloc(sizeof(my_filter_priv_t));
    if (!p) { free(f); return NULL; }

    p->alpha = alpha;
    p->pitch = p->roll = p->yaw = 0.0f;

    f->update    = my_filter_update;
    f->reset     = my_filter_reset;
    f->set_param = my_filter_set_param;
    f->destroy   = my_filter_destroy;
    f->type      = FILTER_TYPE_MY_FILTER;
    f->degrade   = FILTER_DEGRADE_NONE;
    f->priv      = p;
    f->is_static = 0;

    return f;
}
```

### 步骤 4：注册到工厂

在 `filter.c` 的 `filter_create()` 函数中添加：

```c
case FILTER_TYPE_MY_FILTER:
    return filter_create_my_filter(0.5f);
```

## 常见问题

### I2C 通信失败

1. 检查引脚配置
2. 检查上拉电阻（4.7kΩ）
3. 检查 I2C 地址（0x6A vs 0xD4）
4. 检查时钟频率（400kHz）

### SPI 通信失败

1. 检查 SPI 模式（Mode 0: CPOL=0, CPHA=0）
2. 检查 CS 引脚配置
3. 检查时钟频率（10MHz）
4. 检查读写标志（bit7）
5. **重要**：MSPM0 DL API 是底层 API，必须手动排空 RX FIFO。参考 `spi_bridge.c` 实现

### 滤波器发散

1. 检查 ACC/GYRO 数据是否合理
2. 检查 dt 计算是否正确
3. 调整滤波器参数
4. 检查偏置校准是否完成

### 编译错误

1. 检查包含路径
2. 检查宏定义
3. 检查头文件顺序
4. 检查类型定义
