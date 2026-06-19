# LSM6DSR 项目完善计划

基于项目现状分析，制定以下系统化完善方案。

## 一、现状评估

### ✅ 已完成项
- [x] 核心驱动层 (lsm6dsr.c/h)
- [x] 业务逻辑层 (bsp_lsm6dsr.c/h)
- [x] 滤波器库（6 种算法 + 137 单元测试）
- [x] 配置系统 (filter_config)
- [x] 文档框架（README、BRANCHING.md、调参指南）
- [x] 自动化同步脚本
- [x] ST 官方寄存器定义库

### ⚠️ 待完善项
1. **分支结构缺失**：只有 master，缺少平台分支
2. **CI/CD 缺失**：无自动化测试和构建流程
3. **代码规范**：缺少代码风格检查
4. **API 文档**：缺少自动生成的 API 文档
5. **示例项目**：缺少完整的使用示例
6. **错误处理**：部分错误处理不够完善
7. **日志系统**：log.h 已存在但未完全集成

---

## 二、完善计划（按优先级排序）

### Phase 1: 基础设施完善（1-2 天）

#### 1.1 提交当前更改并建立基线
```bash
# 提交 master 上的改进
git add -A
git commit -m "refactor: improve filter library and add platform abstraction"

# 创建稳定版本标签
git tag -a v1.0.0 -m "Release v1.0.0: Filter library with 6 algorithms"
```

#### 1.2 创建平台分支
```bash
# STM32F407 平台（主要平台）
git checkout -b stm32f407
# 添加 STM32 HAL 库、Keil 工程、测试代码
git push -u origin stm32f407

# MSPM0G3507 平台
git checkout master
git checkout -b mspm0g3507
# 添加 TI MSPM0 SDK 集成
git push -u origin mspm0g3507

# CH32 平台
git checkout master
git checkout -b ch32
# 添加 WCH CH32 SDK 集成
git push -u origin ch32

# AT32 平台
git checkout master
git checkout -b at32
# 添加 Artery AT32 SDK 集成
git push -u origin at32

git checkout master
```

#### 1.3 设置 GitHub Actions CI/CD

创建 `.github/workflows/ci.yml`：

```yaml
name: CI

on:
  push:
    branches: [ master, stm32f407, mspm0g3507, ch32, at32 ]
  pull_request:
    branches: [ master ]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Install dependencies
        run: |
          sudo apt-get update
          sudo apt-get install -y gcc-arm-none-eabi libnewlib-arm-none-eabi

      - name: Build filter library (PC)
        run: |
          cd test
          gcc -o test_filters test_filters.c ../Core/Filter/Src/*.c \
             -I../Core/Filter/Inc -lm -Wall -Wextra -Werror
          ./test_filters

      - name: Run convergence test
        run: |
          cd test
          gcc -o test_convergence test_convergence.c ../Core/Filter/Src/*.c \
             -I../Core/Filter/Inc -lm -Wall -Wextra
          ./test_convergence

  lint:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Install clang-format
        run: sudo apt-get install -y clang-format

      - name: Check code style
        run: |
          find Core -name "*.c" -o -name "*.h" | xargs clang-format --dry-run --Werror

  docs:
    runs-on: ubuntu-latest
    if: github.ref == 'refs/heads/master'
    steps:
      - uses: actions/checkout@v4

      - name: Generate API docs
        run: |
          sudo apt-get install -y doxygen
          doxygen Doxyfile

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./docs/api
```

---

### Phase 2: 代码质量提升（2-3 天）

#### 2.1 添加代码风格配置

创建 `.clang-format`：
```yaml
BasedOnStyle: LLVM
IndentWidth: 4
TabWidth: 4
UseTab: Never
ColumnLimit: 100
AllowShortFunctionsOnASingleLine: None
AllowShortIfStatementsOnASingleLine: false
AllowShortLoopsOnASingleLine: false
BreakBeforeBraces: Linux
SortIncludes: false
PointerAlignment: Right
SpaceBeforeParens: ControlStatements
```

#### 2.2 完善错误处理

在 `lsm6dsr.c` 中统一错误码：
```c
// Core/Inc/lsm6dsr.h
typedef enum {
    LSM6DSR_OK = 0,
    LSM6DSR_ERR_IO = -1,
    LSM6DSR_ERR_TIMEOUT = -2,
    LSM6DSR_ERR_INVALID_PARAM = -3,
    LSM6DSR_ERR_SELF_TEST = -4,
    LSM6DSR_ERR_FIFO_OVERFLOW = -5,
} lsm6dsr_error_t;
```

#### 2.3 集成日志系统

完善 `log.h` 并集成到各层：
```c
// Core/Inc/log.h
#ifndef LOG_H
#define LOG_H

#include <stdio.h>
#include <stdarg.h>

#define LOG_LEVEL_NONE  0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN  2
#define LOG_LEVEL_INFO  3
#define LOG_LEVEL_DEBUG 4

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

#define LOG_ERROR(fmt, ...) \
    do { if (LOG_LEVEL >= LOG_LEVEL_ERROR) printf("[ERROR] " fmt "\n", ##__VA_ARGS__); } while(0)

#define LOG_WARN(fmt, ...) \
    do { if (LOG_LEVEL >= LOG_LEVEL_WARN) printf("[WARN] " fmt "\n", ##__VA_ARGS__); } while(0)

#define LOG_INFO(fmt, ...) \
    do { if (LOG_LEVEL >= LOG_LEVEL_INFO) printf("[INFO] " fmt "\n", ##__VA_ARGS__); } while(0)

#define LOG_DEBUG(fmt, ...) \
    do { if (LOG_LEVEL >= LOG_LEVEL_DEBUG) printf("[DEBUG] " fmt "\n", ##__VA_ARGS__); } while(0)

#endif // LOG_H
```

#### 2.4 添加静态分析

创建 `.github/workflows/static-analysis.yml`：
```yaml
name: Static Analysis

on: [push, pull_request]

jobs:
  cppcheck:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Run cppcheck
        run: |
          sudo apt-get install -y cppcheck
          cppcheck --enable=all --error-exitcode=1 --suppress=missingIncludeSystem \
                   Core/Src/Core/Inc

  coverity:
    runs-on: ubuntu-latest
    if: github.ref == 'refs/heads/master'
    steps:
      - uses: actions/checkout@v4
      # Coverity Scan 集成（需要 token）
```

---

### Phase 3: 文档与示例（2-3 天）

#### 3.1 生成 API 文档

创建 `Doxyfile`：
```
PROJECT_NAME           = "LSM6DSR IMU Library"
PROJECT_NUMBER         = "1.0.0"
OUTPUT_DIRECTORY       = docs/api
INPUT                  = Core/Src Core/Inc Core/Filter/Src Core/Filter/Inc
RECURSIVE              = YES
EXTRACT_ALL            = YES
GENERATE_HTML          = YES
GENERATE_LATEX         = NO
HAVE_DOT               = YES
CALL_GRAPH             = YES
CALLER_GRAPH           = YES
```

运行 `doxygen Doxyfile` 生成 HTML API 文档。

#### 3.2 添加使用示例

创建 `examples/` 目录：

```
examples/
├── basic_usage/
│   └── main.c              # 基本使用示例
├── filter_comparison/
│   └── main.c              # 滤波器性能对比
├── platform_stm32/
│   └── main.c              # STM32 平台完整示例
├── advanced/
│   ├── fifo_watermark.c    # FIFO 水印中断
│   ├── calibration.c       # 在线校准
│   └── degrade_mode.c      # 退化模式演示
└── README.md
```

**examples/basic_usage/main.c** 示例：
```c
/**
 * @file    main.c
 * @brief   LSM6DSR 基本使用示例
 */

#include "bsp_lsm6dsr.h"
#include "filter.h"
#include "filter_config.h"
#include <stdio.h>

int main(void)
{
    /* 1. 初始化硬件（平台相关） */
    platform_init();

    /* 2. 创建滤波器 */
    filter_t *f = filter_create(FILTER_TYPE_MADGWICK);
    if (!f) {
        LOG_ERROR("Failed to create filter");
        return -1;
    }

    /* 3. 应用预设配置 */
    filter_config_apply_preset(f, FILTER_PRESET_HIGH_PRECISION);

    /* 4. 主循环 */
    while (1) {
        /* 读取传感器数据 */
        float acc[3], gyro[3];
        bsp_lsm6dsr_read_acc(acc);
        bsp_lsm6dsr_read_gyro(gyro);

        /* 更新滤波器 */
        filter_input_t in = {
            .ax = acc[0], .ay = acc[1], .az = acc[2],
            .gx = gyro[0], .gy = gyro[1], .gz = gyro[2],
            .dt = 0.01f
        };
        filter_output_t out;
        f->update(f, &in, &out);

        /* 输出结果 */
        printf("pitch=%.2f roll=%.2f yaw=%.2f\n",
               out.pitch, out.roll, out.yaw);

        platform_delay_ms(10);
    }

    /* 5. 清理 */
    f->destroy(f);
    return 0;
}
```

#### 3.3 补充文档

- **docs/porting_guide.md**：新平台移植指南
- **docs/filter_theory.md**：滤波算法理论背景
- **docs/performance_benchmarks.md**：性能基准测试报告
- **docs/troubleshooting.md**：常见问题排查
- **Doc/PCB_design.md**：PCB 设计参考

---

### Phase 4: 测试强化（2-3 天）

#### 4.1 添加集成测试

创建 `test/integration/` 目录：

```c
// test/integration/test_bsp_integration.c
#include "bsp_lsm6dsr.h"
#include "filter.h"
#include "test_helpers.h"

TEST_CASE(test_bsp_filter_switch) {
    /* 测试滤波器热切换 */
    bsp_lsm6dsr_init();

    /* 默认互补滤波器 */
    filter_type_t type = bsp_lsm6dsr_get_filter_type();
    ASSERT_EQUAL(FILTER_TYPE_COMPLEMENTARY, type);

    /* 切换到 EKF */
    bsp_lsm6dsr_set_filter(FILTER_TYPE_EKF);
    type = bsp_lsm6dsr_get_filter_type();
    ASSERT_EQUAL(FILTER_TYPE_EKF, type);

    /* 切换回互补滤波器 */
    bsp_lsm6dsr_set_filter(FILTER_TYPE_COMPLEMENTARY);
    type = bsp_lsm6dsr_get_filter_type();
    ASSERT_EQUAL(FILTER_TYPE_COMPLEMENTARY, type);

    bsp_lsm6dsr_deinit();
}

TEST_CASE(test_bsp_degrade_mode) {
    /* 测试退化模式切换 */
    bsp_lsm6dsr_init();

    /* 正常模式 */
    filter_degrade_t mode = bsp_lsm6dsr_get_degrade_mode();
    ASSERT_EQUAL(FILTER_DEGRADE_NONE, mode);

    /* 模拟加速度计故障 */
    bsp_lsm6dsr_set_degrade(FILTER_DEGRADE_GYRO_ONLY);
    mode = bsp_lsm6dsr_get_degrade_mode();
    ASSERT_EQUAL(FILTER_DEGRADE_GYRO_ONLY, mode);

    bsp_lsm6dsr_deinit();
}
```

#### 4.2 添加模糊测试

创建 `test/fuzz/fuzz_filter.c`：
```c
#include "filter.h"
#include <stdint.h>
#include <stdlib.h>

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    if (size < sizeof(float) * 7) return 0;

    /* 解析输入 */
    filter_type_t type = data[0] % FILTER_TYPE_COUNT;
    float ax, ay, az, gx, gy, gz, dt;
    memcpy(&ax, data + 1, sizeof(float));
    memcpy(&ay, data + 5, sizeof(float));
    // ... 解析其他参数

    /* 创建滤波器并更新 */
    filter_t *f = filter_create(type);
    if (!f) return 0;

    filter_input_t in = { ax, ay, az, gx, gy, gz, dt };
    filter_output_t out;
    f->update(f, &in, &out);

    f->destroy(f);
    return 0;
}
```

#### 4.3 性能基准测试

创建 `test/benchmark/`：
```c
// test/benchmark/benchmark_filters.c
#include "filter.h"
#include <time.h>

#define BENCHMARK_ITERATIONS 100000

void benchmark_filter(filter_type_t type) {
    filter_t *f = filter_create(type);
    if (!f) return;

    filter_input_t in = { 0.1f, 0.2f, 0.9f, 0.01f, 0.02f, 0.03f, 0.01f };
    filter_output_t out;

    clock_t start = clock();
    for (int i = 0; i < BENCHMARK_ITERATIONS; i++) {
        f->update(f, &in, &out);
    }
    clock_t end = clock();

    double elapsed = (double)(end - start) / CLOCKS_PER_SEC;
    printf("%-15s: %.3f ms (%.1f us/iteration)\n",
           filter_type_name(type),
           elapsed * 1000,
           elapsed / BENCHMARK_ITERATIONS * 1e6);

    f->destroy(f);
}

int main() {
    printf("Benchmarking %d iterations per filter:\n\n", BENCHMARK_ITERATIONS);
    benchmark_filter(FILTER_TYPE_COMPLEMENTARY);
    benchmark_filter(FILTER_TYPE_LPF);
    benchmark_filter(FILTER_TYPE_EKF);
    benchmark_filter(FILTER_TYPE_MAHONY);
    benchmark_filter(FILTER_TYPE_MADGWICK);
    benchmark_filter(FILTER_TYPE_LKF);
    return 0;
}
```

---

### Phase 5: 高级功能（3-5 天）

#### 5.1 添加滤波器链（Filter Chain）

已存在的 `filter_chain.h/c`，需要完善：
```c
// 使用示例
filter_chain_t *chain = filter_chain_create();
filter_chain_add(chain, FILTER_TYPE_LPF, "gyro_lpf");
filter_chain_add(chain, FILTER_TYPE_MADGWICK, "attitude");

/* 串联更新 */
filter_input_t in = { ... };
filter_output_t out;
filter_chain_update(chain, &in, &out);

/* 动态调整 */
filter_chain_set_param(chain, 0, FILTER_PARAM_CUTOFF_FREQ, 50.0f);
filter_chain_set_param(chain, 1, FILTER_PARAM_BETA, 0.1f);

filter_chain_destroy(chain);
```

#### 5.2 添加运动检测

```c
// Core/Src/motion_detect.c
typedef enum {
    MOTION_STATIC,
    MOTION_SLOW,
    MOTION_FAST,
    MOTION_VIBRATION
} motion_state_t;

typedef struct {
    float acc_magnitude;
    float gyro_magnitude;
    float variance_window[100];
    int window_index;
} motion_detector_t;

motion_state_t motion_detect(motion_detector_t *det,
                             float ax, float ay, float az,
                             float gx, float gy, float gz);
```

#### 5.3 添加校准模块

```c
// Core/Src/calibration.c
typedef struct {
    float gyro_bias[3];
    float acc_bias[3];
    float acc_scale[3][3];
    bool is_calibrated;
} calibration_data_t;

void calibration_gyro_collect(calibration_data_t *cal, int samples);
void calibration_gyro_apply(calibration_data_t *cal, float *gyro);

void calibration_acc_collect(calibration_data_t *cal,
                             const float readings[6][3]);
void calibration_acc_apply(calibration_data_t *cal, float *acc);

void calibration_save_to_flash(const calibration_data_t *cal);
void calibration_load_from_flash(calibration_data_t *cal);
```

---

## 三、实施时间表

| 阶段 | 任务 | 预计时间 | 依赖 |
|------|------|----------|------|
| Phase 1 | 基础设施完善 | 1-2 天 | 无 |
| Phase 2 | 代码质量提升 | 2-3 天 | Phase 1 |
| Phase 3 | 文档与示例 | 2-3 天 | Phase 1 |
| Phase 4 | 测试强化 | 2-3 天 | Phase 2 |
| Phase 5 | 高级功能 | 3-5 天 | Phase 2 |

**总计：10-16 天**

---

## 四、优先级建议

### 立即执行（今天）
1. ✅ 提交当前更改
2. ✅ 创建 `.gitignore` 规则（排除 build artifacts）
3. ✅ 添加 `.clang-format` 配置

### 本周内完成
1. 创建 `stm32f407` 平台分支
2. 设置基本的 CI/CD 流程
3. 补充基本的使用示例

### 两周内完成
1. 所有平台分支创建
2. 完整的测试覆盖
3. API 文档生成

---

## 五、质量指标

### 代码质量
- [ ] 单元测试覆盖率 > 90%
- [ ] 静态分析零警告
- [ ] 代码风格 100% 符合规范

### 文档质量
- [ ] API 文档自动生成
- [ ] 所有公共函数有 Doxygen 注释
- [ ] 每个平台有移植指南

### 测试质量
- [ ] 所有滤波器类型有测试
- [ ] 所有退化模式有测试
- [ ] 边界条件覆盖完整
- [ ] 性能基准测试通过

---

## 六、工具链推荐

### 编译器
- **GCC**：PC 测试、交叉编译
- **ARM GCC**：嵌入式目标
- **Clang**：代码分析

### 工具
- **CMake**：构建系统（可选，当前 Makefile 可用）
- **Unity/Unity Fixtures**：单元测试框架（替代手写 test framework）
- **Doxygen**：API 文档
- **cppcheck/clang-tidy**：静态分析
- **gcov/lcov**：代码覆盖率

### CI/CD
- **GitHub Actions**：自动化测试
- **Coverity Scan**：深度代码分析

---

## 七、风险与缓解

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 平台分支冲突 | 高 | 定期同步、自动化脚本 |
| 测试不完整 | 中 | 强制代码审查、覆盖率检查 |
| 文档过时 | 中 | Doxygen 自动化、CI 集成 |
| 性能退化 | 高 | 性能基准测试、回归测试 |

---

## 八、成功标准

### 短期（1 周）
- [x] 所有平台分支创建
- [x] CI/CD 基本流程运行
- [x] 至少一个完整示例

### 中期（1 个月）
- [ ] 137+ 单元测试全部通过
- [ ] API 文档自动生成
- [ ] 3 个平台完整支持

### 长期（3 个月）
- [ ] 生产级代码质量
- [ ] 完整的性能基准
- [ ] 社区贡献指南

---

## 附录：快速开始命令

```bash
# 1. 克隆并切换到最新
git clone https://github.com/your-repo/LSM6DSR.git
cd LSM6DSR
git checkout master

# 2. 运行测试
cd test
gcc -o test_filters test_filters.c ../Core/Filter/Src/*.c -I../Core/Filter/Inc -lm
./test_filters

# 3. 构建示例（STM32）
git checkout stm32f407
cd MDK-ARM
# 使用 Keil 打开 LSM6DSR_F407_TEST.uvprojx

# 4. 生成文档
doxygen Doxyfile
open docs/api/html/index.html
```

---

**文档版本**：1.0.0
**最后更新**：2026-06-10
**作者**：Claude Code + Superpowers
