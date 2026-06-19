# AI 理解 LSM6DSR 项目的关键提示词

## 项目概述

这是一个 **LSM6DSR 六轴 IMU 驱动库**，目标平台是 **MSPM0G3507**（TI MSPM0 系列 MCU）。

### 核心架构（三层）

```
┌─────────────────────────────────────────┐
│  BSP 层（业务层）                        │
│  bsp_lsm6dsr.c/h                        │
│  - 姿态估计（互补滤波器）                │
│  - 自适应 α（运动/静止检测）             │
│  - 偏置跟踪                             │
│  - VOFA+ 输出                           │
├─────────────────────────────────────────┤
│  驱动层（平台无关）                      │
│  lsm6dsr.c/h + lsm6dsr_STdC/            │
│  - 寄存器读写                           │
│  - ACC/GYRO/TEMP 数据读取               │
│  - FIFO 操作                            │
│  - 通过 lsm6dsr_io_t 抽象 I/O           │
├─────────────────────────────────────────┤
│  桥接层（平台相关）                      │
│  i2c_bridge.c / spi_bridge.c            │
│  - 实现 lsm6dsr_io_t 的 read/write      │
│  - 调用 MSPM0 DriverLib                 │
└─────────────────────────────────────────┘
```

### 关键接口

```c
/* I/O 抽象（驱动层与桥接层的契约） */
typedef struct {
    int8_t (*read)(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len);
    int8_t (*write)(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len);
    void *ctx;
} lsm6dsr_io_t;

/* 平台抽象（BSP 层与平台的契约） */
typedef struct {
    void (*delay_ms)(uint32_t ms);
    void (*delay_us)(uint32_t us);
    uint32_t (*get_tick_us)(void);
    uint32_t system_clock_hz;
    int (*debug_printf)(const char *fmt, ...);
} platform_t;

/* 滤波器接口（BSP 层与滤波器的契约） */
typedef struct filter {
    filter_update_fn    update;
    filter_reset_fn     reset;
    filter_set_param_fn set_param;
    filter_destroy_fn   destroy;
    filter_type_t       type;
    void               *priv;
} filter_t;
```

### 文件位置

| 文件 | 路径 | 说明 |
|------|------|------|
| 平台抽象头文件 | `Core/Inc/platform.h` | 平台接口定义 |
| 日志系统 | `Core/Inc/log.h` | LOG_INFO/LOG_ERR 宏 |
| BSP 层 | `Core/Src/bsp_lsm6dsr.c` | 姿态估计算法 |
| 驱动层 | `Core/Src/lsm6dsr.c` | 传感器驱动 |
| 滤波器库 | `Core/Filter/` | 6 种滤波器实现 |
| 官方 ST 驱动 | `lsm6dsr_STdC/driver/` | 325 个寄存器函数 |
| I2C 桥接 | `ports/mspm0g3507/i2c_bridge.c` | 硬件 I2C |
| SPI 桥接 | `ports/mspm0g3507/spi_bridge.c` | 硬件 SPI |
| 软件 I2C | `ports/mspm0g3507/i2c_bridge_soft.c` | GPIO 位操作 |
| 适配器 | `Core/Adapter/lsm6dsr_adapter.c` | lsm6dsr_io_t ↔ stmdev_ctx_t |
| 测试文件 | `test/test_filters.c` | 137 个单元测试 |

### 通信方式选择

```c
/* 在 project_config.h 中定义 */
// #define USE_SPI           /* 硬件 SPI */
// #define USE_SOFT_I2C      /* 软件 I2C */
/* 默认使用硬件 I2C */
```

### 关键宏定义

```c
/* I2C 地址（7-bit，左移 1 位） */
#define LSM6DSR_I2C_ADDR    (0x6A << 1)  /* 0xD4 */

/* SPI 读写标志 */
#define LSM6DSR_SPI_READ    0x80  /* bit7=1 表示读 */
#define LSM6DSR_SPI_WRITE   0x7F  /* bit7=0 表示写 */

/* SPI 全双工注意事项：
 * MSPM0 DL API 是底层 API，不自动处理全双工。
 * 每次发送后 RX FIFO 也收到一字节，必须手动排空。
 * 参考 spi_bridge.c 实现。 */

/* 滤波器类型 */
FILTER_TYPE_COMPLEMENTARY  /* 互补滤波器 */
FILTER_TYPE_LPF            /* 低通滤波器 */
FILTER_TYPE_EKF            /* 扩展卡尔曼滤波器 */
FILTER_TYPE_LKF            /* 线性卡尔曼滤波器 */
FILTER_TYPE_MAHONY         /* Mahony 滤波器 */
FILTER_TYPE_MADGWICK       /* Madgwick 滤波器 */
```

### 测试验证

```bash
# 编译测试（GCC）
gcc -o test_filters.exe test/test_filters.c Core/Filter/Src/filter.c Core/Filter/Src/filter_config.c -ICore/Filter/Inc -lm -Wall -Wextra

# 运行测试
./test_filters.exe

# 预期结果：137/137 PASS
```

### 常见问题

1. **WHO_AM_I 读取失败**：检查 I2C 地址（0x6A vs 0xD4）、引脚配置、上拉电阻
2. **数据为零**：检查传感器初始化是否完成、ODR 配置
3. **滤波器发散**：检查 ACC/GYRO 数据是否合理、dt 计算是否正确
4. **编译错误**：检查包含路径、宏定义、头文件顺序

### 扩展点

1. **添加新滤波器**：实现 `filter_t` 接口的 4 个函数指针
2. **添加新平台**：实现 `platform_t` 接口的 5 个函数指针
3. **添加新通信方式**：实现 `lsm6dsr_io_t` 接口的 read/write 回调
4. **添加新传感器**：复用 `lsm6dsr_io_t` 接口，修改寄存器地址
