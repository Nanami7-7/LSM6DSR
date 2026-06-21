# LSM6DSR 通用滤波库 — 冻结接口规范 (INTERFACE.md)

## 1. 冻结承诺

本文件所列类型、结构体、枚举、函数签名在所有平台分支上**永久不变**。

**编程约束：**
- 枚举值仅可追加（append-only），数值不可改
- 结构体布局不可变（字段不可删/改/重排，只能末尾 append）
- 函数签名不可变（参数类型、顺序、返回类型不可改）
- `filter_platform.h` 中的 `fp_*` 钩子签名永久固定

**违反后果：** 分支代码二进制不兼容，`--check-interface` 脚本阻断同步。

---

## 2. 冻结类型清单

### 2.1 枚举

```c
filter_type_t          // 值 0..5 + FILTER_TYPE_COUNT = 6
filter_degrade_t       // 值 0..4 + FILTER_DEGRADE_COUNT = 5
filter_param_t         // 值 0..6 + FILTER_PARAM_COUNT = 7
filter_preset_t        // 值 0..4 + FILTER_PRESET_COUNT = 5
param_source_t         // 值 0..3
degrade_mode_t         // 值 0..4
sensor_quality_t       // 值 0..3
```

### 2.2 结构体

```c
filter_input_t          // { float ax,ay,az,gx,gy,gz,dt; }      28 bytes
filter_output_t         // { float pitch,roll,yaw; float q0..q3; } 28 bytes
filter_safety_config_t  // { float angle_min,max; float q_norm_thresh; float cov_reg_factor; uint16_t norm_interval; }
filter_param_desc_t     // { filter_param_t param; float default_value,min_value,max_value; param_source_t source_type; const char *source_name,*source_detail,*unit; }
degrade_config_t        // { degrade_mode_t mode; float acc_threshold_low,acc_threshold_high,gyro_threshold,variance_threshold; const char *description; }
filter_t                // 见 §3
```

### 2.3 函数指针类型

```c
filter_update_fn      // void (*)(filter_t*, const filter_input_t*, filter_output_t*)
filter_reset_fn       // void (*)(filter_t*)
filter_set_param_fn   // void (*)(filter_t*, filter_param_t, float)
```

### 2.4 公共 API 函数

```c
filter_t*  filter_create_static(filter_type_t type, void *buf, size_t buf_size);
size_t     filter_get_static_size(filter_type_t type);
void       filter_set_safety_config(filter_t *f, const filter_safety_config_t *config);
int        filter_validate_output(const filter_output_t *out);
void       filter_normalize_quaternion(float *q0, float *q1, float *q2, float *q3);
void       filter_regularize_covariance(float P[][7], int size, float factor);
const char* filter_type_name(filter_type_t type);
void       filter_set_degrade(filter_t *f, filter_degrade_t degrade);
const char* filter_degrade_name(filter_degrade_t degrade);
int        filter_check_acc_quality(float ax, float ay, float az);
int        filter_check_gyro_quality(float gx, float gy, float gz);
```

**条件 API（仅 -DFILTER_ALLOW_DYNAMIC 时存在）：**

```c
filter_t*  filter_create(filter_type_t type);
void       filter_destroy(filter_t *f);
```

### 2.5 filter_config.h API

```c
const filter_param_desc_t* filter_config_get_params(filter_type_t type, int *count);
float    filter_config_get_default(filter_type_t type, filter_param_t param);
int      filter_config_validate(filter_type_t type, filter_param_t param, float value);
float    filter_config_clamp(filter_type_t type, filter_param_t param, float value);
const degrade_config_t*    filter_config_get_degrade(degrade_mode_t mode);
sensor_quality_t           filter_config_assess_quality(float ax,float ay,float az, float gx,float gy,float gz);
degrade_mode_t             filter_config_select_degrade(sensor_quality_t acc, sensor_quality_t gyro);
void    filter_config_apply_preset(filter_t *f, filter_preset_t preset);
const char* filter_config_preset_name(filter_preset_t preset);
void    filter_config_print(filter_type_t type);
```

### 2.6 filter_platform.h fp_* 钩子

```c
/* 数学 */
float   fp_sqrt(float x);
float   fp_atan2(float y, float x);
float   fp_asin(float x);
float   fp_sin(float x);
float   fp_cos(float x);
float   fp_fabs(float x);
int     fp_isnan(float x);
int     fp_isinf(float x);
/* 时序 */
void     fp_init_timing(void);
uint32_t fp_get_cycles(void);
float    fp_get_dt(void);
void     fp_delay_ms(uint32_t ms);
/* 内存 */
void*    fp_malloc(size_t n);
void     fp_free(void *p);
/* 矩阵 */
void     fp_mat_mult(float *dst, const float *a, const float *b, int ar, int ac, int bc);
int      fp_mat_inverse_3x3(float *dst, const float *src);
```

### 2.7 配置宏（永久保留）

```c
FILTER_USE_CMSIS_DSP
FILTER_USE_FAST_MATH
FILTER_USE_FIXED_POINT
FILTER_STATIC_ONLY
FILTER_ALLOW_DYNAMIC
FILTER_DISABLE_COMPLEMENTARY
FILTER_DISABLE_LPF
FILTER_DISABLE_EKF
FILTER_DISABLE_LKF
FILTER_DISABLE_MAHONY
FILTER_DISABLE_MADGWICK
FILTER_OVERRIDE_MECHANISM
FILTER_WEAK
```

---

## 3. `filter_t` 结构体布局（永久冻结）

```c
struct filter {
    filter_update_fn      update;        // offset 0,   8 bytes (pointer)
    filter_reset_fn       reset;         // offset 8,   8 bytes
    filter_set_param_fn   set_param;     // offset 16,  8 bytes
    filter_type_t         type;          // offset 24,  4 bytes (int)
    filter_degrade_t      degrade;       // offset 28,  4 bytes
    void                 *priv;          // offset 32,  8 bytes
    filter_safety_config_t safety_config; // offset 40, 24 bytes
};
// sizeof(filter_t) = 64 bytes (64-bit) / 36 bytes (32-bit)
```

---

## 4. `_Static_assert` ABI 锁

`filter.h` 末尾包含：
```c
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(filter_t) == FILTER_ABI_SIZEOF_FILTER_T,
    "filter_t size changed — ABI break");
#endif
```

---

## 5. 添加新滤波器食谱

1. 在 master 的 `filter.h` 中：
   - `filter_type_t` 枚举在 `FILTER_TYPE_COUNT` 前追加新值
   - `filter_param_t` 枚举在 `FILTER_PARAM_COUNT` 前追加新参数（如需要）
2. 新建 `Core/Filter/Src/filter_<name>.c`，实现 5 个 `FILTER_WEAK` 函数
3. 在 `filter_internal.h` 声明 5 个函数
4. 在 `filter_factory.c` 的 `filter_table` 另加一项（用 `#ifndef FILTER_DISABLE_<name>` 守卫）
5. 在 `filter_config.c` 添加参数描述表
6. 在 `abi_expected.h` 更新（如 FILTER_TYPE_COUNT 改变）
7. 提交到 master，同步到所有分支

**禁止**：直接在分支上添加滤波器（必须走 master→同步流程）。

---

## 6. 版本号

```c
#define FILTER_API_VERSION 1  /* bump on any ABI-breaking change */
```

当前版本：**1**，Phase 1-4 稳定。
