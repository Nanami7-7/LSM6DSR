# LSM6DSR 分支契约 (BRANCH_CONTRACT.md)

## 1. CAN/CANNOT 表

| 文件/目录 | 分支可修改？ | 方式 |
|-----------|------------|------|
| `Core/Filter/Inc/filter.h` | **NO** | — |
| `Core/Filter/Inc/filter_config.h` | **NO** | — |
| `Core/Filter/Inc/filter_platform.h` | **NO** | — |
| `Core/Filter/Inc/filter_math.h` | **NO** | — |
| `Core/Filter/Inc/abi_expected.h` | **NO** | — |
| `Core/Filter/Src/filter_factory.c` | **NO** | — |
| `Core/Filter/Src/filter_common.c` | **NO** | — |
| `Core/Filter/Src/filter_internal.h` | **NO** | — |
| `Core/Filter/Src/filter_config.c` | **NO** | — |
| `Core/Filter/Src/filter_complementary.c` | **NO** (参考实现) | 覆盖方式见 §3 |
| `Core/Filter/Src/filter_lpf.c` | **NO** (参考实现) | 覆盖方式见 §3 |
| `Core/Filter/Src/filter_ekf.c` | **NO** (参考实现) | 覆盖方式见 §3 |
| `Core/Filter/Src/filter_lkf.c` | **NO** (参考实现) | 覆盖方式见 §3 |
| `Core/Filter/Src/filter_mahony.c` | **NO** (参考实现) | 覆盖方式见 §3 |
| `Core/Filter/Src/filter_madgwick.c` | **NO** (参考实现) | 覆盖方式见 §3 |
| `Core/Filter/Src/filter_dynamic.c` | **NO** | — |
| `Core/Filter/Src/opt_<mcu>/**` | **YES** (分支专属) | 新文件 |
| `Core/Filter/Platform/filter_platform_default.c` | **NO** (master 提供默认) | 分支排除编译，提供 `platform_<mcu>.c` |
| `Core/Filter/Doc/**` | **NO** | — |
| `test/abi_check.c` | **NO** | — |
| `test/golden_master_baseline.txt` | **NO** | — |
| `Core/Src/bsp_lsm6dsr.c` | YES (已解耦到 fp_* 钩子) | 保持钩子调用 |
| `Core/Src/main.c`, `Drivers/`, 工程文件 | YES | 平台特定 |

## 2. 每 MCU 编译宏集

| MCU | `-D` 宏 | 说明 |
|-----|---------|------|
| **master (PC 测试)** | `-DFILTER_STATIC_ONLY` | libm 默认后端 |
| **stm32f407** | `-DFILTER_STATIC_ONLY -DFILTER_USE_CMSIS_DSP` | CMSIS-DSP 后端 + DWT 周期计数器 |
| **mspm0g3507** | `-DFILTER_STATIC_ONLY -DFILTER_USE_FAST_MATH -DFILTER_DISABLE_EKF -DFILTER_DISABLE_LKF` | 多项式逼近，禁用 EKF/LKF（M0+ 无 FPU） |
| **ch32** | `-DFILTER_STATIC_ONLY -DFILTER_USE_FAST_MATH` | 根据具体型号决定是否禁用 EKF/LKF |
| **at32** | `-DFILTER_STATIC_ONLY -DFILTER_USE_CMSIS_DSP` | AT32M4 有 FPU，可用 CMSIS-DSP |

## 3. 覆盖机制

### 3.1 弱符号覆盖（推荐，ARM GCC/ArmClang v6+）

master 的 6 个滤波器文件各导出 5 个 `FILTER_WEAK` 函数。
分支在 `opt_<mcu>/` 下提供非 weak 定义，链接器自动替换。

**示例：** stm32f407 优化 EKF

新建 `Core/Filter/Src/opt_stm32f407/filter_ekf_cmsis.c`：

```c
#include "filter.h"
#include "filter_internal.h"
#include "arm_math.h"

/* 仅覆盖 ekf_update（热路径），其他函数仍用 master weak 版本 */
void ekf_update(filter_t *self, const filter_input_t *in, filter_output_t *out)
{
    /* 使用 arm_mat_mult_f32/arm_sqrt_f32 的 CMSIS-DSP 优化实现 */
}
/* ekf_reset/set_param/get_static_size/init 不重新定义 → 走 weak fallback */
```

构建时加入 `opt_stm32f407/filter_ekf_cmsis.c`，排除 master 的 `filter_ekf.c` 可能导致重复定义（因为 weak 符号在 MinGW 上退化）。正确做法：

- **ARM GCC/ArmClang**：同时编译 master 的 `filter_ekf.c`(weak) + 分支的 `filter_ekf_cmsis.c`(strong)，链接器选 strong → ✓
- **MinGW**：`FILTER_WEAK` 退化为空 → 重复定义错误。分支改用 `include_swap` 机制。

### 3.2 include_swap 回退（MinGW / 不支持 weak 的工具链）

分支在构建脚本中将源文件列表中的 `filter_<type>.c` 替换为 `opt_<mcu>/filter_<type>_<mcu>.c`。

```makefile
# stm32f407 Makefile 示例
FILTER_IMPL = $(wildcard Core/Filter/Src/filter_*.c)
# 替换 EKF 实现
FILTER_IMPL := $(filter-out Core/Filter/Src/filter_ekf.c, $(FILTER_IMPL))
FILTER_IMPL += Core/Filter/Src/opt_stm32f407/filter_ekf_cmsis.c
```

## 4. 平台后端文件结构

```
Core/Filter/Src/opt_<mcu>/
├── platform_<mcu>.c     — fp_* 钩子实现（必须）
└── filter_<type>_<mcu>.c — 单滤波器覆盖（可选，需要时才建）
```

### platform_<mcu>.c 模板

```c
#include "filter_platform.h"
/* 平台特定头文件 */
#include "stm32f4xx.h"
#include "arm_math.h"

float fp_sqrt(float x) { float y; arm_sqrt_f32(x, &y); return y; }
float fp_atan2(float y, float x) { return atan2f(y, x); }
/* ... 其他 fp_* 实现 ... */

uint32_t fp_get_cycles(void) { return DWT->CYCCNT; }
void fp_init_timing(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
void fp_delay_ms(uint32_t ms) { HAL_Delay(ms); }
```

## 5. 接口完整性检查

同步脚本 `scripts/sync-to-platforms.py --check-interface` 验证：
1. `filter.h` / `filter_config.h` / `filter_platform.h` / `abi_expected.h` 字节一致
2. `filter_factory.c` / `filter_common.c` 字节一致
3. 6 个参考实现 `filter_<type>.c` 字节一致
4. 分支上 `abi_check` 编译运行通过

所有检查失败 → 同步被阻断。

## 6. 跨版本兼容

- `FILTER_API_VERSION` 在 filter.h 定义（当前版本 1）
- 分支可在运行时检查版本号兼容性
- 版本升级需在 master 提 PR + 更新 abi_expected.h + 重新生成 golden baseline + 同步所有分支
