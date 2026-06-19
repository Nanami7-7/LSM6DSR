# LSM6DSR 项目修复快速开始指南

本指南提供具体的代码修复示例，帮助你快速修复代码审查中发现的问题。

---

## 🔧 修复 1: 线程安全问题

### 问题
所有状态都是全局静态变量，不支持多实例。

### 解决方案
将状态封装到上下文结构体中。

### 示例代码

**Core/Inc/bsp_lsm6dsr.h** - 添加上下文结构体：

```c
/**
 * @brief BSP 上下文结构体
 *
 * 支持多实例和线程安全操作
 */
typedef struct {
    // IMU 状态
    float   bgx, bgy, bgz;
    int     cal_ok;

    // 滤波器状态
    double  pitch, roll, yaw;
    uint64_t last_tick;  // 改用 64 位避免溢出
    int     initialized;

    // 自适应滤波器状态
    float   acc_buffer[3][BSP_ACC_VAR_WINDOW];  // 3 轴 x 窗口大小
    int     buffer_idx;
    int     buffer_count;
    float   alpha;
    float   last_variance;

    // 滤波器实例
    filter_t *active_filter;
    filter_type_t filter_type;

    // 数据缓存
    bsp_lsm6dsr_data_t last_data;
    int     is_stationary;

    // 错误处理
    filter_error_t last_error;
    uint32_t error_count;
} bsp_lsm6dsr_ctx_t;

// 新 API（推荐）
int bsp_lsm6dsr_init_ctx(bsp_lsm6dsr_ctx_t *ctx);
int bsp_lsm6dsr_update_ctx(bsp_lsm6dsr_ctx_t *ctx);
int bsp_lsm6dsr_calibrate_ctx(bsp_lsm6dsr_ctx_t *ctx);

// 旧 API（向后兼容，使用默认全局上下文）
static bsp_lsm6dsr_ctx_t default_ctx;

void bsp_lsm6dsr_init(void) {
    bsp_lsm6dsr_init_ctx(&default_ctx);
}

void bsp_lsm6dsr_update(void) {
    bsp_lsm6dsr_update_ctx(&default_ctx);
}
```

**Core/Src/bsp_lsm6dsr.c** - 实现新 API：

```c
int bsp_lsm6dsr_init_ctx(bsp_lsm6dsr_ctx_t *ctx)
{
    if (!ctx) {
        return -1;
    }

    // 清零所有状态
    memset(ctx, 0, sizeof(bsp_lsm6dsr_ctx_t));

    // 初始化默认值
    ctx->filter_type = FILTER_TYPE_COMPLEMENTARY;
    ctx->alpha = BSP_ALPHA_STATIONARY;
    ctx->last_tick = 0;
    ctx->initialized = 0;

    // ... 其他初始化代码 ...

    ctx->initialized = 1;
    return 0;
}

int bsp_lsm6dsr_update_ctx(bsp_lsm6dsr_ctx_t *ctx)
{
    if (!ctx || !ctx->initialized) {
        return -1;
    }

    if (!ctx->active_filter) {
        return -2;
    }

    // 读取传感器数据
    float acc[3], gyro[3];
    if (lsm6dsr_read_accel_float(&lsm6dsr_io, &acc[0], &acc[1], &acc[2],
                                  LSM6DSR_ACCEL_FS_4G) != 0) {
        ctx->error_count++;
        ctx->last_error = FILTER_ERR_SENSOR_READ;
        return -3;
    }

    if (lsm6dsr_read_gyro_float(&lsm6dsr_io, &gyro[0], &gyro[1], &gyro[2],
                                 LSM6DSR_GYRO_FS_250DPS) != 0) {
        ctx->error_count++;
        ctx->last_error = FILTER_ERR_SENSOR_READ;
        return -4;
    }

    // 计算 dt
    uint64_t now = get_tick_us();
    uint64_t elapsed = now - ctx->last_tick;
    float dt = (float)elapsed / 1e6f;
    ctx->last_tick = now;

    // 填充滤波器输入
    filter_input_t in = {
        .ax = acc[0], .ay = acc[1], .az = acc[2],
        .gx = gyro[0], .gy = gyro[1], .gz = gyro[2],
        .dt = dt
    };

    filter_output_t out;

    // 更新滤波器
    ctx->active_filter->update(ctx->active_filter, &in, &out);

    // 更新状态
    ctx->pitch = out.pitch;
    ctx->roll = out.roll;
    ctx->yaw = out.yaw;

    // 更新数据缓存
    ctx->last_data.pitch = out.pitch;
    ctx->last_data.roll = out.roll;
    ctx->last_data.yaw = out.yaw;
    // ... 其他数据 ...

    return 0;
}
```

### 使用示例

```c
// 单实例使用（向后兼容）
bsp_lsm6dsr_init();
while (1) {
    bsp_lsm6dsr_update();
}

// 多实例使用（推荐）
bsp_lsm6dsr_ctx_t imu1_ctx, imu2_ctx;

bsp_lsm6dsr_init_ctx(&imu1_ctx);
bsp_lsm6dsr_init_ctx(&imu2_ctx);

while (1) {
    bsp_lsm6dsr_update_ctx(&imu1_ctx);
    bsp_lsm6dsr_update_ctx(&imu2_ctx);

    // 融合两个 IMU 的数据
    float fused_pitch = (imu1_ctx.pitch + imu2_ctx.pitch) / 2.0f;
}
```

---

## 🔧 修复 2: destroy 函数安全检查

### 问题
静态分配的滤波器 destroy 为 NULL，调用会崩溃。

### 解决方案
提供安全的销毁函数。

**Core/Filter/Src/filter.c** - 添加安全销毁函数：

```c
/**
 * @brief 安全销毁滤波器（自动检查 NULL 和静态分配）
 *
 * 推荐使用此函数替代直接调用 f->destroy(f)
 */
void filter_destroy_safe(filter_t *f)
{
    if (!f) {
        return;  // NULL 指针安全
    }

    if (f->is_static) {
        // 静态分配：重置状态但不释放内存
        f->reset(f);
        return;
    }

    // 动态分配：正常销毁
    if (f->destroy) {
        f->destroy(f);
    }
}

/**
 * @brief 静态分配的空操作销毁函数
 */
static void static_destroy_noop(filter_t *self)
{
    // 静态分配不需要销毁，但调用是安全的
    (void)self;
}

// 在 filter_create_static 中设置
filter_t* filter_create_static(filter_type_t type, void *buf, size_t buf_size)
{
    // ... 其他代码 ...

    f->destroy = static_destroy_noop;  // 设置为空操作，而非 NULL

    // ... 其他代码 ...
}
```

### 更新文档

在 README.md 中添加警告：

```markdown
## ⚠️ 重要提示：滤波器销毁

### 动态分配
```c
filter_t *f = filter_create(FILTER_TYPE_EKF);
// ... 使用滤波器 ...
filter_destroy_safe(f);  // 推荐：安全销毁
// 或
f->destroy(f);  // 也可以，但需确保 f 不为 NULL
```

### 静态分配
```c
uint8_t buf[1024];
filter_t *f = filter_create_static(FILTER_TYPE_EKF, buf, sizeof(buf));
// ... 使用滤波器 ...
filter_destroy_safe(f);  // 推荐：自动检测静态分配
// 不会释放内存，只重置状态
// 或
f->destroy(f);  // 也可以：内部是空操作
```
```

---

## 🔧 修复 3: 四元数归一化统一

### 问题
不同位置的归一化阈值不一致。

### 解决方案
统一归一化函数。

**Core/Filter/Src/filter.c** - 统一归一化：

```c
/**
 * @brief 安全的四元数归一化
 *
 * 处理所有边界情况：
 * - 正常情况：归一化
 * - 接近零：重置为单位四元数
 * - NaN/Inf：重置为单位四元数
 *
 * @param q0, q1, q2, q3  四元数（输入/输出）
 */
void filter_normalize_quaternion_safe(float *q0, float *q1, float *q2, float *q3)
{
    // 检查 NaN
    if (isnan(*q0) || isnan(*q1) || isnan(*q2) || isnan(*q3)) {
        *q0 = 1.0f; *q1 = 0.0f; *q2 = 0.0f; *q3 = 0.0f;
        return;
    }

    // 检查 Inf
    if (isinf(*q0) || isinf(*q1) || isinf(*q2) || isinf(*q3)) {
        *q0 = 1.0f; *q1 = 0.0f; *q2 = 0.0f; *q3 = 0.0f;
        return;
    }

    // 计算范数
    float norm = sqrtf((*q0)*(*q0) + (*q1)*(*q1) + (*q2)*(*q2) + (*q3)*(*q3));

    // 阈值检查（统一使用 1e-10f）
    if (norm < 1e-10f) {
        // 范数太小，可能是数值下溢
        // 重置为单位四元数
        *q0 = 1.0f; *q1 = 0.0f; *q2 = 0.0f; *q3 = 0.0f;
        return;
    }

    // 正常归一化
    *q0 /= norm;
    *q1 /= norm;
    *q2 /= norm;
    *q3 /= norm;
}

/**
 * @brief 带统计的归一化（用于调试）
 */
void filter_normalize_quaternion_debug(float *q0, float *q1, float *q2, float *q3,
                                        filter_stats_t *stats)
{
    float norm_before = sqrtf((*q0)*(*q0) + (*q1)*(*q1) + (*q2)*(*q2) + (*q3)*(*q3));

    filter_normalize_quaternion_safe(q0, q1, q2, q3);

    float norm_after = sqrtf((*q0)*(*q0) + (*q1)*(*q1) + (*q2)*(*q2) + (*q3)*(*q3));

    // 记录归一化修正量
    if (stats) {
        float correction = fabsf(norm_before - 1.0f);
        if (correction > stats->max_norm_correction) {
            stats->max_norm_correction = correction;
        }
        stats->norm_corrections++;
    }
}
```

### 更新所有滤波器使用统一函数

```c
// 在 ekf_update 中
static void ekf_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    // ... 状态预测 ...

    // 替换原来的归一化代码
    filter_normalize_quaternion_safe(&p->state[0], &p->state[1],
                                      &p->state[2], &p->state[3]);

    // ... 其他代码 ...
}

// 在 mahony_update 中
static void mahony_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    // ... PI 控制器更新 ...

    // 统一归一化
    filter_normalize_quaternion_safe(&p->q0, &p->q1, &p->q2, &p->q3);

    // ... 其他代码 ...
}
```

---

## 🔧 修复 4: 时间戳溢出处理

### 问题
uint32_t 微秒计数器约 71.6 分钟溢出。

### 解决方案
使用 64 位或正确处理回绕。

**方案 1: 使用 64 位（推荐）**

```c
// Core/Inc/bsp_lsm6dsr.h
typedef struct {
    uint64_t last_tick_us;  // 改用 64 位
    // ... 其他字段 ...
} bsp_lsm6dsr_ctx_t;

// Core/Src/bsp_lsm6dsr.c
int bsp_lsm6dsr_update_ctx(bsp_lsm6dsr_ctx_t *ctx)
{
    uint64_t now = get_tick_us_64();  // 需要平台提供 64 位时间戳

    if (ctx->last_tick_us > 0) {
        uint64_t elapsed_us = now - ctx->last_tick_us;
        float dt = (float)elapsed_us / 1e6f;

        // 有效性检查
        if (dt <= 0.0f || dt > 1.0f) {
            // 时间戳异常
            dt = 0.01f;  // 默认 10ms
        }

        // ... 使用 dt 更新滤波器 ...
    }

    ctx->last_tick_us = now;
    return 0;
}
```

**方案 2: 处理 32 位回绕（兼容方案）**

```c
// Core/Inc/bsp_lsm6dsr.h
static inline uint32_t tick_diff_safe(uint32_t now, uint32_t last)
{
    // 正确处理回绕
    return (now >= last) ? (now - last) : (0xFFFFFFFF - last + now + 1);
}

// Core/Src/bsp_lsm6dsr.c
int bsp_lsm6dsr_update_ctx(bsp_lsm6dsr_ctx_t *ctx)
{
    uint32_t now = get_tick_us_32();

    if (ctx->last_tick > 0) {
        uint32_t elapsed_us = tick_diff_safe(now, ctx->last_tick);
        float dt = (float)elapsed_us / 1e6f;

        // 有效性检查
        if (dt <= 0.0f || dt > 1.0f) {
            dt = 0.01f;
        }

        // ... 使用 dt 更新滤波器 ...
    }

    ctx->last_tick = now;
    return 0;
}
```

---

## 🔧 修复 5: 错误处理改进

### 添加错误码

**Core/Inc/filter.h**：

```c
/**
 * @brief 滤波器错误码
 */
typedef enum {
    FILTER_OK = 0,                    /**< 成功 */
    FILTER_ERR_INVALID_TYPE,          /**< 无效的滤波器类型 */
    FILTER_ERR_ALLOC_FAILED,          /**< 内存分配失败 */
    FILTER_ERR_INVALID_PARAM,         /**< 无效的参数 */
    FILTER_ERR_NOT_INITIALIZED,       /**< 未初始化 */
    FILTER_ERR_INVALID_INPUT,         /**< 无效的输入数据 */
    FILTER_ERR_SENSOR_READ,           /**< 传感器读取失败 */
    FILTER_ERR_NUMERICAL,             /**< 数值错误（NaN/Inf） */
    FILTER_ERR_COVARIANCE,            /**< 协方差矩阵异常 */
    FILTER_ERR_QUATERNION,            /**< 四元数异常 */
} filter_error_t;

/**
 * @brief 错误信息结构体
 */
typedef struct {
    filter_error_t code;
    const char *message;
    const char *file;
    int line;
} filter_error_info_t;

/**
 * @brief 错误回调函数类型
 */
typedef void (*filter_error_callback_t)(const filter_error_info_t *info, void *user_data);

/**
 * @brief 设置全局错误回调
 */
void filter_set_error_callback(filter_error_callback_t callback, void *user_data);

/**
 * @brief 获取最后的错误信息
 */
filter_error_info_t filter_get_last_error(void);
```

**Core/Filter/Src/filter.c**：

```c
// 全局错误回调
static filter_error_callback_t error_callback = NULL;
static void *error_user_data = NULL;
static filter_error_info_t last_error = {0};

void filter_set_error_callback(filter_error_callback_t callback, void *user_data)
{
    error_callback = callback;
    error_user_data = user_data;
}

filter_error_info_t filter_get_last_error(void)
{
    return last_error;
}

static void report_error(filter_error_t code, const char *message,
                          const char *file, int line)
{
    filter_error_info_t info = {
        .code = code,
        .message = message,
        .file = file,
        .line = line
    };

    last_error = info;

    if (error_callback) {
        error_callback(&info, error_user_data);
    }

    // 也可以输出到日志
    #ifdef DEBUG
    fprintf(stderr, "[FILTER ERROR] %s:%d: %s (code=%d)\n",
            file, line, message, code);
    #endif
}

// 使用宏简化错误报告
#define FILTER_REPORT_ERROR(code, msg) \
    report_error(code, msg, __FILE__, __LINE__)

// 示例：改进的 filter_create
filter_t* filter_create(filter_type_t type)
{
    if (type < 0 || type >= FILTER_TYPE_COUNT) {
        FILTER_REPORT_ERROR(FILTER_ERR_INVALID_TYPE, "Invalid filter type");
        return NULL;
    }

    filter_t *f = NULL;

    switch (type) {
        case FILTER_TYPE_COMPLEMENTARY:
            f = filter_create_complementary(0.98f);
            break;
        case FILTER_TYPE_EKF:
            f = filter_create_ekf(0.001f, 0.003f, 0.01f);
            break;
        // ... 其他类型 ...
        default:
            FILTER_REPORT_ERROR(FILTER_ERR_INVALID_TYPE, "Unknown filter type");
            return NULL;
    }

    if (!f) {
        FILTER_REPORT_ERROR(FILTER_ERR_ALLOC_FAILED, "Failed to allocate filter");
        return NULL;
    }

    f->safety_config = (filter_safety_config_t)FILTER_SAFETY_DEFAULT;
    return f;
}
```

### 使用示例

```c
// 设置错误回调
void my_error_handler(const filter_error_info_t *info, void *user_data)
{
    printf("Error: %s (code %d) at %s:%d\n",
           info->message, info->code, info->file, info->line);

    // 可以记录到文件、发送到云端等
}

// 初始化时设置
filter_set_error_callback(my_error_handler, NULL);

// 使用滤波器
filter_t *f = filter_create(FILTER_TYPE_EKF);
if (!f) {
    filter_error_info_t err = filter_get_last_error();
    printf("Failed: %s\n", err.message);
}
```

---

## 📋 修复检查清单

- [ ] 重构 bsp_lsm6dsr 支持多实例
- [ ] 添加 filter_destroy_safe 函数
- [ ] 统一四元数归一化函数
- [ ] 修复时间戳溢出（改用 64 位）
- [ ] 添加错误处理机制
- [ ] 添加参数验证
- [ ] 更新文档
- [ ] 补充单元测试

---

## 🧪 测试建议

```c
// 测试多实例
void test_multi_instance(void) {
    bsp_lsm6dsr_ctx_t ctx1, ctx2;

    assert(bsp_lsm6dsr_init_ctx(&ctx1) == 0);
    assert(bsp_lsm6dsr_init_ctx(&ctx2) == 0);

    for (int i = 0; i < 100; i++) {
        assert(bsp_lsm6dsr_update_ctx(&ctx1) == 0);
        assert(bsp_lsm6dsr_update_ctx(&ctx2) == 0);
    }

    // 两个实例应该独立工作
    printf("IMU1: pitch=%.2f, IMU2: pitch=%.2f\n",
           ctx1.pitch, ctx2.pitch);
}

// 测试 destroy 安全性
void test_destroy_safety(void) {
    filter_t *f = filter_create(FILTER_TYPE_EKF);

    // 动态分配：正常销毁
    filter_destroy_safe(f);

    // 静态分配：安全销毁
    uint8_t buf[1024];
    f = filter_create_static(FILTER_TYPE_EKF, buf, sizeof(buf));
    filter_destroy_safe(f);  // 不会崩溃

    // NULL 指针：安全
    filter_destroy_safe(NULL);  // 不会崩溃
}
```

---

**修复完成** ✅
