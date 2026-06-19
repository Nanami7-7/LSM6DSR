# API 参考手册

## 平台抽象层

### platform_t 结构体

```c
typedef struct {
    void (*delay_ms)(uint32_t ms);        // 毫秒延时
    void (*delay_us)(uint32_t us);        // 微秒延时（可选）
    uint32_t (*get_tick_us)(void);        // 微秒时间戳
    uint32_t system_clock_hz;             // 系统时钟频率
    int (*debug_printf)(const char *fmt, ...);  // 调试输出
} platform_t;
```

### 全局变量

```c
extern const platform_t *g_platform;  // 全局平台实例指针
```

### 平台函数

| 函数 | 说明 |
|------|------|
| `platform_timer_init()` | 初始化微秒计时器 |

---

## BSP 层 API

### 数据结构

```c
typedef struct {
    float ax, ay, az;       // 加速度 (m/s²)
    float gx, gy, gz;       // 角速度 (dps)
    float pitch, roll, yaw; // 姿态角 (deg)
    float temperature;      // 温度 (°C)
} bsp_lsm6dsr_data_t;
```

### 初始化函数

| 函数 | 说明 | 返回值 |
|------|------|--------|
| `bsp_lsm6dsr_init()` | 初始化传感器和滤波器 | void |
| `bsp_lsm6dsr_calibrate()` | 陀螺零偏校准 | void |

### 数据读取函数

| 函数 | 说明 | 返回值 |
|------|------|--------|
| `bsp_lsm6dsr_update(data)` | 姿态更新（核心滤波） | void |
| `bsp_lsm6dsr_get_data()` | 获取最新缓存数据 | `const bsp_lsm6dsr_data_t*` |
| `bsp_lsm6dsr_get_bias(bx, by, bz)` | 获取陀螺零偏 | void |
| `bsp_lsm6dsr_get_last_variance()` | 获取 ACC 方差 | float |
| `bsp_lsm6dsr_is_stationary()` | 查询静止状态 | int (1=静止) |

### 滤波器控制函数

| 函数 | 说明 | 返回值 |
|------|------|--------|
| `bsp_lsm6dsr_set_filter(type)` | 切换滤波器类型 | int (0=成功) |
| `bsp_lsm6dsr_get_filter_type()` | 获取当前滤波器类型 | filter_type_t |
| `bsp_lsm6dsr_set_filter_param(param, value)` | 设置滤波器参数 | int (0=成功) |
| `bsp_lsm6dsr_get_filter_name()` | 获取滤波器名称 | const char* |

### 输出函数

| 函数 | 说明 | 返回值 |
|------|------|--------|
| `bsp_lsm6dsr_vofa_format(buf, size, data)` | VOFA+ 格式化 | int (字符数) |

---

## 滤波器 API

### 滤波器类型

| 类型 | 说明 | 状态维度 | 偏置估计 |
|------|------|----------|----------|
| `FILTER_TYPE_COMPLEMENTARY` | 互补滤波器 | 2 | ❌ |
| `FILTER_TYPE_LPF` | 低通滤波器 | 2 | ❌ |
| `FILTER_TYPE_EKF` | 扩展卡尔曼滤波器 | 7 | ✅ |
| `FILTER_TYPE_LKF` | 线性卡尔曼滤波器 | 6 | ✅ |
| `FILTER_TYPE_MAHONY` | Mahony 滤波器 | 4 | ✅ |
| `FILTER_TYPE_MADGWICK` | Madgwick 滤波器 | 4 | ❌ |

### 滤波器参数

| 参数 | 说明 | 范围 |
|------|------|------|
| `FILTER_PARAM_ALPHA` | 互补滤波器 α | [0, 1] |
| `FILTER_PARAM_CUTOFF_FREQ` | LPF 截止频率 (Hz) | >0 |
| `FILTER_PARAM_Q_ANGLE` | EKF/LKF 过程噪声-角度 | >0 |
| `FILTER_PARAM_Q_BIAS` | EKF/LKF 过程噪声-偏置 | >0 |
| `FILTER_PARAM_R_MEASURE` | EKF/LKF 测量噪声 | >0 |
| `FILTER_PARAM_KP` | Mahony/Madgwick 比例增益 | ≥0 |
| `FILTER_PARAM_KI` | Mahony 积分增益 | ≥0 |

### 退化模式

| 模式 | 说明 |
|------|------|
| `FILTER_DEGRADE_NONE` | 正常运行 |
| `FILTER_DEGRADE_STATIC_ONLY` | 仅静态模式 |
| `FILTER_DEGRADE_GYRO_ONLY` | 仅陀螺仪 |
| `FILTER_DEGRADE_ACC_ONLY` | 仅加速度计 |
| `FILTER_DEGRADE_HOLD_LAST` | 保持上次输出 |

---

## I/O 抽象层

### lsm6dsr_io_t 结构体

```c
typedef struct {
    int8_t (*read)(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len);
    int8_t (*write)(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len);
    void *ctx;
} lsm6dsr_io_t;
```

### 桥接层实例

| 实例 | 说明 | 文件 |
|------|------|------|
| `lsm6dsr_io` | 硬件 I2C | i2c_bridge.c |
| `lsm6dsr_io_spi` | 硬件 SPI | spi_bridge.c |
| `lsm6dsr_io_soft` | 软件 I2C | i2c_bridge_soft.c |

### SPI 协议注意事项

MSPM0 DL SPI API 是底层 API，不自动处理全双工特性。每次发送字节后，RX FIFO 也会收到一个字节（全双工），必须手动排空：

- **读操作**：CS↓ → [REG|0x80] → 排空RX → [DUMMY]→读取 → CS↑
- **写操作**：CS↓ → [REG&0x7F] → 排空RX → [DATA]→排空RX → CS↑

参考 `spi_bridge.c` 中的 `mspm0_spi_read()` 和 `mspm0_spi_write()` 实现。

---

## 官方 ST 驱动适配器

### 适配器函数

| 函数 | 说明 | 返回值 |
|------|------|--------|
| `lsm6dsr_adapter_init(io, ctx)` | 初始化适配器 | int (0=成功) |
| `lsm6dsr_adapter_who_am_i(ctx)` | 获取 WHO_AM_I | int (0x6B=正确) |
| `lsm6dsr_adapter_read_accel_raw(ctx, ax, ay, az)` | 读取原始加速度 | int (0=成功) |
| `lsm6dsr_adapter_read_gyro_raw(ctx, gx, gy, gz)` | 读取原始陀螺 | int (0=成功) |
| `lsm6dsr_adapter_read_temp_raw(ctx, temp)` | 读取原始温度 | int (0=成功) |

### 使用示例

```c
stmdev_ctx_t ctx;
lsm6dsr_adapter_init(&lsm6dsr_io, &ctx);

uint8_t whoami;
lsm6dsr_device_id_get(&ctx, &whoami);

uint8_t buf[6];
lsm6dsr_acceleration_raw_get(&ctx, buf);
```

---

## 日志系统

### 日志宏

| 宏 | 说明 | 级别 |
|----|------|------|
| `LOG_ERR(fmt, ...)` | 错误日志 | 1 |
| `LOG_WARN(fmt, ...)` | 警告日志 | 2 |
| `LOG_INFO(fmt, ...)` | 信息日志 | 3 |
| `LOG_DBG(fmt, ...)` | 调试日志 | 4 |
| `LOG_RAW(fmt, ...)` | 原始输出 | - |
| `LOG_INDENT(fmt, ...)` | 缩进输出 | - |

### 日志级别控制

```c
#define LOG_LEVEL LOG_LEVEL_INFO  // 编译时控制
```
