# LSM6DSR 库迁移指南

从 v1.0.0 迁移到 v1.1.0

---

## 📋 概述

v1.1.0 版本包含多项关键修复和改进，提升了代码的可靠性、可维护性和可扩展性。

**主要改进**：
- ✅ 线程安全 - 支持多实例
- ✅ 内存安全 - 安全的销毁函数
- ✅ 数值稳定 - 统一的四元数归一化
- ✅ 时间戳 - 64 位防溢出
- ✅ 错误处理 - 完善的错误码和回调
- ✅ 参数验证 - 自动验证有效性

---

## 🔄 API 变化

### BSP 层 API

#### 旧 API（v1.0.0）

```c
// 全局状态，不支持多实例
void bsp_lsm6dsr_init(void);
void bsp_lsm6dsr_update(bsp_lsm6dsr_data_t *data);
void bsp_lsm6dsr_calibrate(void);
void bsp_lsm6dsr_get_bias(float *bx, float *by, float *bz);
int bsp_lsm6dsr_is_stationary(void);
```

#### 新 API（v1.1.0）

```c
// 上下文 API，支持多实例（推荐）
int bsp_lsm6dsr_init_ctx(bsp_lsm6dsr_ctx_t *ctx);
int bsp_lsm6dsr_update_ctx(bsp_lsm6dsr_ctx_t *ctx, bsp_lsm6dsr_data_t *data);
int bsp_lsm6dsr_calibrate_ctx(bsp_lsm6dsr_ctx_t *ctx);
void bsp_lsm6dsr_destroy_ctx(bsp_lsm6dsr_ctx_t *ctx);
void bsp_lsm6dsr_get_bias_ctx(const bsp_lsm6dsr_ctx_t *ctx, float *bx, float *by, float *bz);
int bsp_lsm6dsr_is_stationary_ctx(const bsp_lsm6dsr_ctx_t *ctx);
int bsp_lsm6dsr_set_filter_ctx(bsp_lsm6dsr_ctx_t *ctx, filter_type_t type);
int bsp_lsm6dsr_set_filter_param_ctx(bsp_lsm6dsr_ctx_t *ctx, filter_param_t param, float value);

// 旧 API 仍然支持（向后兼容）
void bsp_lsm6dsr_init(void);
void bsp_lsm6dsr_update(bsp_lsm6dsr_data_t *data);
// ... 等等
```

**变化说明**：
- 新 API 返回错误码（0=成功，<0=失败）
- 新 API 需要传入上下文指针
- 新 API 使用 64 位时间戳
- 旧 API 内部使用默认全局上下文

### Filter 库 API

#### 新增函数

```c
// 安全销毁（推荐）
void filter_destroy_safe(filter_t *f);

// 错误处理
void filter_set_error_callback(filter_error_callback_t callback, void *user_data);
filter_error_info_t filter_get_last_error(void);

// 错误码
typedef enum {
    FILTER_OK = 0,
    FILTER_ERR_INVALID_TYPE,
    FILTER_ERR_ALLOC_FAILED,
    FILTER_ERR_INVALID_PARAM,
    // ...
} filter_error_t;
```

---

## 🚀 迁移步骤

### 步骤 1：更新头文件

无需更改，头文件保持向后兼容。

```c
#include "bsp_lsm6dsr.h"
#include "filter.h"
```

### 步骤 2：迁移到新 API（可选但推荐）

#### 方案 A：完全迁移到新 API

```c
// 旧代码
bsp_lsm6dsr_init();
while (1) {
    bsp_lsm6dsr_data_t data;
    bsp_lsm6dsr_update(&data);
    printf("pitch=%.2f\n", data.pitch);
}

// 新代码
bsp_lsm6dsr_ctx_t ctx;
if (bsp_lsm6dsr_init_ctx(&ctx) != 0) {
    printf("Init failed!\n");
    return -1;
}

while (1) {
    bsp_lsm6dsr_data_t data;
    if (bsp_lsm6dsr_update_ctx(&ctx, &data) != 0) {
        printf("Update failed!\n");
        break;
    }
    printf("pitch=%.2f\n", data.pitch);
}

bsp_lsm6dsr_destroy_ctx(&ctx);
```

#### 方案 B：保持使用旧 API

```c
// 无需修改，旧 API 仍然工作
bsp_lsm6dsr_init();
while (1) {
    bsp_lsm6dsr_data_t data;
    bsp_lsm6dsr_update(&data);
    printf("pitch=%.2f\n", data.pitch);
}
```

### 步骤 3：使用安全的销毁函数

```c
// 旧代码（可能崩溃）
filter_t *f = filter_create(FILTER_TYPE_EKF);
// ... 使用滤波器 ...
f->destroy(f);  // ❌ 如果 f 为 NULL 会崩溃

// 新代码（推荐）
filter_t *f = filter_create(FILTER_TYPE_EKF);
// ... 使用滤波器 ...
filter_destroy_safe(f);  // ✅ 安全
```

### 步骤 4：添加错误处理（可选）

```c
// 定义错误回调
void my_error_handler(const filter_error_info_t *info, void *user_data)
{
    printf("Error: %s at %s:%d\n",
           info->message, info->file, info->line);
}

// 初始化时设置
filter_set_error_callback(my_error_handler, NULL);

// 创建滤波器并检查错误
filter_t *f = filter_create(FILTER_TYPE_EKF);
if (!f) {
    filter_error_info_t err = filter_get_last_error();
    printf("Failed to create filter: %s (code=%d)\n",
           err.message, err.code);
    return -1;
}
```

### 步骤 5：利用参数验证（自动）

参数验证现在是自动的，无需额外代码：

```c
filter_t *f = filter_create(FILTER_TYPE_COMPLEMENTARY);

// 有效参数 - 接受
f->set_param(f, FILTER_PARAM_ALPHA, 0.5f);

// 无效参数 - 自动拒绝，不会崩溃
f->set_param(f, FILTER_PARAM_ALPHA, 1.5f);  // 超出范围
f->set_param(f, FILTER_PARAM_ALPHA, NAN);   // NaN
```

---

## 📝 代码示例

### 示例 1：多 IMU 系统

```c
#include "bsp_lsm6dsr.h"

int main(void)
{
    // 初始化两个 IMU
    bsp_lsm6dsr_ctx_t imu1, imu2;

    if (bsp_lsm6dsr_init_ctx(&imu1) != 0) {
        printf("IMU1 init failed\n");
        return -1;
    }

    if (bsp_lsm6dsr_init_ctx(&imu2) != 0) {
        printf("IMU2 init failed\n");
        return -1;
    }

    // 主循环
    while (1) {
        bsp_lsm6dsr_data_t data1, data2;

        bsp_lsm6dsr_update_ctx(&imu1, &data1);
        bsp_lsm6dsr_update_ctx(&imu2, &data2);

        // 融合数据
        float fused_pitch = (data1.pitch + data2.pitch) / 2.0f;
        printf("Fused pitch: %.2f\n", fused_pitch);
    }

    // 清理
    bsp_lsm6dsr_destroy_ctx(&imu1);
    bsp_lsm6dsr_destroy_ctx(&imu2);

    return 0;
}
```

### 示例 2：RTOS 环境

```c
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_lsm6dsr.h"

// 每个任务独立的 IMU 上下文
void imu_task_1(void *param)
{
    bsp_lsm6dsr_ctx_t ctx;
    bsp_lsm6dsr_init_ctx(&ctx);

    while (1) {
        bsp_lsm6dsr_data_t data;
        bsp_lsm6dsr_update_ctx(&ctx, &data);

        // 处理数据
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void imu_task_2(void *param)
{
    bsp_lsm6dsr_ctx_t ctx;
    bsp_lsm6dsr_init_ctx(&ctx);

    while (1) {
        bsp_lsm6dsr_data_t data;
        bsp_lsm6dsr_update_ctx(&ctx, &data);

        // 处理数据
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

### 示例 3：错误处理最佳实践

```c
#include "filter.h"
#include <stdio.h>

// 错误日志
void log_error(const filter_error_info_t *info, void *user_data)
{
    FILE *log_file = (FILE *)user_data;
    fprintf(log_file, "FILTER ERROR: %s at %s:%d (code=%d)\n",
            info->message, info->file, info->line, info->code);
}

int main(void)
{
    // 打开日志文件
    FILE *log = fopen("filter_errors.log", "a");
    if (!log) {
        printf("Failed to open log file\n");
        return -1;
    }

    // 设置错误回调
    filter_set_error_callback(log_error, log);

    // 创建滤波器
    filter_t *f = filter_create(FILTER_TYPE_EKF);
    if (!f) {
        filter_error_info_t err = filter_get_last_error();
        printf("Failed: %s\n", err.message);
        fclose(log);
        return -1;
    }

    // 使用滤波器
    filter_input_t in = {
        .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
        .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
        .dt = 0.01f
    };
    filter_output_t out;
    f->update(f, &in, &out);

    // 安全销毁
    filter_destroy_safe(f);
    fclose(log);

    return 0;
}
```

---

## ⚠️ 注意事项

### 1. 向后兼容性

- ✅ 旧 API 完全支持
- ✅ 无需修改现有代码
- ✅ 逐步迁移到新 API

### 2. 内存使用

新 API 使用的上下文结构体大小：
- `bsp_lsm6dsr_ctx_t`: ~200 字节
- 加上滤波器实例：~50-200 字节（取决于类型）

### 3. 性能影响

- ✅ 无额外开销（静态分配）
- ✅ 参数验证开销很小
- ✅ 错误回调开销可忽略

### 4. 编译选项

```bash
# 启用调试日志
gcc -DDEBUG -o test test.c filter.c

# 禁用调试日志（生产环境）
gcc -o test test.c filter.c
```

---

## 🧪 验证迁移

### 运行测试

```bash
cd test
gcc -o test_fixes test_fixes.c ../Core/Filter/Src/filter.c \
    -I../Core/Filter/Inc -lm -Wall -Wextra -DDEBUG
./test_fixes
```

预期结果：
```
✅ All 46 tests passed!
```

### 检查清单

- [ ] 所有测试通过
- [ ] 编译无警告
- [ ] 无内存泄漏
- [ ] 错误处理正常工作
- [ ] 多实例正常工作

---

## 📚 相关文档

- [README.md](../README.md) - 项目概述
- [FIX_GUIDE.md](FIX_GUIDE.md) - 修复指南
- [CODE_REVIEW_REPORT.md](CODE_REVIEW_REPORT.md) - 代码审查报告
- [IMPROVEMENT_PLAN.md](IMPROVEMENT_PLAN.md) - 改进计划

---

## ❓ 常见问题

### Q1: 旧代码还能用吗？

**A**: 是的，旧 API 完全支持，无需修改。

### Q2: 为什么推荐使用新 API？

**A**: 新 API 提供：
- 多实例支持
- 更好的错误处理
- 线程安全
- 更易于测试

### Q3: 性能会下降吗？

**A**: 不会。新 API 的开销很小，与旧 API 性能相当。

### Q4: 如何回退到旧版本？

**A**: Git 回退：
```bash
git checkout v1.0.0
```

---

**迁移完成** ✅
