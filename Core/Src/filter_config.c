/**
 * @file    filter_config.c
 * @brief   滤波器参数配置实现
 */

#include "filter_config.h"
#include <stdio.h>
#include <math.h>

/* ============================================================
 * 参数描述表 - 互补滤波器
 * ============================================================ */
static const filter_param_desc_t comp_params[] = {
    {
        .param = FILTER_PARAM_ALPHA,
        .default_value = COMP_ALPHA_DEFAULT,
        .min_value = COMP_ALPHA_MIN,
        .max_value = COMP_ALPHA_MAX,
        .source_type = PARAM_SOURCE_PAPER,
        .source_name = "Mahony et al., 2008",
        .source_detail = "Nonlinear complementary filters on the special orthogonal group",
        .unit = "无量纲"
    }
};

/* ============================================================
 * 参数描述表 - LPF
 * ============================================================ */
static const filter_param_desc_t lpf_params[] = {
    {
        .param = FILTER_PARAM_CUTOFF_FREQ,
        .default_value = LPF_CUTOFF_DEFAULT,
        .min_value = LPF_CUTOFF_MIN,
        .max_value = LPF_CUTOFF_MAX,
        .source_type = PARAM_SOURCE_EMPIRICAL,
        .source_name = "工程经验值",
        .source_detail = "一阶低通滤波器截止频率，取决于应用需求",
        .unit = "Hz"
    }
};

/* ============================================================
 * 参数描述表 - EKF
 * ============================================================ */
static const filter_param_desc_t ekf_params[] = {
    {
        .param = FILTER_PARAM_Q_ANGLE,
        .default_value = EKF_Q_ANGLE_DEFAULT,
        .min_value = EKF_Q_ANGLE_MIN,
        .max_value = EKF_Q_ANGLE_MAX,
        .source_type = PARAM_SOURCE_DATASHEET,
        .source_name = "LSM6DSR Datasheet",
        .source_detail = "陀螺仪噪声密度0.01dps/√Hz，采样率100Hz时约0.0017rad/s",
        .unit = "rad²/s"
    },
    {
        .param = FILTER_PARAM_Q_BIAS,
        .default_value = EKF_Q_BIAS_DEFAULT,
        .min_value = EKF_Q_BIAS_MIN,
        .max_value = EKF_Q_BIAS_MAX,
        .source_type = PARAM_SOURCE_DATASHEET,
        .source_name = "LSM6DSR Datasheet",
        .source_detail = "陀螺仪偏置稳定性±10dps",
        .unit = "dps²/s"
    },
    {
        .param = FILTER_PARAM_R_MEASURE,
        .default_value = EKF_R_MEASURE_DEFAULT,
        .min_value = EKF_R_MEASURE_MIN,
        .max_value = EKF_R_MEASURE_MAX,
        .source_type = PARAM_SOURCE_DATASHEET,
        .source_name = "LSM6DSR Datasheet",
        .source_detail = "加速度计噪声密度0.08mg/√Hz，采样率100Hz时约0.0008g",
        .unit = "g²"
    }
};

/* ============================================================
 * 参数描述表 - Mahony
 * ============================================================ */
static const filter_param_desc_t mahony_params[] = {
    {
        .param = FILTER_PARAM_KP,
        .default_value = MAHONY_KP_DEFAULT,
        .min_value = MAHONY_KP_MIN,
        .max_value = MAHONY_KP_MAX,
        .source_type = PARAM_SOURCE_PAPER,
        .source_name = "Mahony et al., 2008",
        .source_detail = "论文默认值kp=1.0，经验值kp=2.0（快速响应）",
        .unit = "无量纲"
    },
    {
        .param = FILTER_PARAM_KI,
        .default_value = MAHONY_KI_DEFAULT,
        .min_value = MAHONY_KI_MIN,
        .max_value = MAHONY_KI_MAX,
        .source_type = PARAM_SOURCE_PAPER,
        .source_name = "Mahony et al., 2008",
        .source_detail = "论文默认值ki=0.3，用于估计陀螺仪偏置",
        .unit = "无量纲"
    }
};

/* ============================================================
 * 参数描述表 - Madgwick
 * ============================================================ */
static const filter_param_desc_t madgwick_params[] = {
    {
        .param = FILTER_PARAM_KP,  /* Madgwick的β映射到KP */
        .default_value = MADGWICK_BETA_DEFAULT,
        .min_value = MADGWICK_BETA_MIN,
        .max_value = MADGWICK_BETA_MAX,
        .source_type = PARAM_SOURCE_PAPER,
        .source_name = "Madgwick, 2010",
        .source_detail = "An efficient orientation filter for inertial and inertial/magnetic sensor arrays. "
                         "论文第3.6节：β=0.033（IMU），β=0.041（MARG）",
        .unit = "无量纲"
    }
};

/* ============================================================
 * 退化策略配置表
 * ============================================================ */
static const degrade_config_t degrade_configs[] = {
    [DEGRADE_NONE] = {
        .mode = DEGRADE_NONE,
        .acc_threshold_low = DEGRADE_ACC_LOW,
        .acc_threshold_high = DEGRADE_ACC_HIGH,
        .gyro_threshold = DEGRADE_GYRO_THRESHOLD,
        .variance_threshold = DEGRADE_VARIANCE_THRESH,
        .description = "正常运行，所有传感器数据有效"
    },
    [DEGRADE_STATIC_ONLY] = {
        .mode = DEGRADE_STATIC_ONLY,
        .acc_threshold_low = DEGRADE_ACC_LOW,
        .acc_threshold_high = DEGRADE_ACC_HIGH,
        .gyro_threshold = DEGRADE_GYRO_THRESHOLD,
        .variance_threshold = DEGRADE_VARIANCE_THRESH,
        .description = "仅静态模式，禁用动态补偿（ACC异常时）"
    },
    [DEGRADE_GYRO_ONLY] = {
        .mode = DEGRADE_GYRO_ONLY,
        .acc_threshold_low = DEGRADE_ACC_LOW,
        .acc_threshold_high = DEGRADE_ACC_HIGH,
        .gyro_threshold = DEGRADE_GYRO_THRESHOLD,
        .variance_threshold = DEGRADE_VARIANCE_THRESH,
        .description = "仅陀螺仪模式，禁用ACC修正（ACC无效时）"
    },
    [DEGRADE_ACC_ONLY] = {
        .mode = DEGRADE_ACC_ONLY,
        .acc_threshold_low = DEGRADE_ACC_LOW,
        .acc_threshold_high = DEGRADE_ACC_HIGH,
        .gyro_threshold = DEGRADE_GYRO_THRESHOLD,
        .variance_threshold = DEGRADE_VARIANCE_THRESH,
        .description = "仅加速度计模式，禁用GYRO积分（GYRO饱和时）"
    },
    [DEGRADE_HOLD_LAST] = {
        .mode = DEGRADE_HOLD_LAST,
        .acc_threshold_low = DEGRADE_ACC_LOW,
        .acc_threshold_high = DEGRADE_ACC_HIGH,
        .gyro_threshold = DEGRADE_GYRO_THRESHOLD,
        .variance_threshold = DEGRADE_VARIANCE_THRESH,
        .description = "保持上次输出，冻结状态（所有传感器异常时）"
    }
};

/* ============================================================
 * 预设配置参数
 * ============================================================ */
typedef struct {
    const char *name;
    const char *description;
    struct {
        filter_param_t param;
        float value;
    } params[4];  /* 最多4个参数 */
    int param_count;
} preset_config_t;

static const preset_config_t preset_configs[] = {
    [FILTER_PRESET_DEFAULT] = {
        .name = "Default",
        .description = "默认配置，平衡精度和响应速度",
        .params = {0},
        .param_count = 0  /* 使用各滤波器默认值 */
    },
    [FILTER_PRESET_HIGH_PRECISION] = {
        .name = "High Precision",
        .description = "高精度模式，低噪声，适合静态测量",
        .params = {
            {.param = FILTER_PARAM_ALPHA, .value = 0.99f},
            {.param = FILTER_PARAM_KP, .value = 0.5f},
            {.param = FILTER_PARAM_KI, .value = 0.1f},
        },
        .param_count = 3
    },
    [FILTER_PRESET_FAST_RESPONSE] = {
        .name = "Fast Response",
        .description = "快速响应模式，低延迟，适合动态应用",
        .params = {
            {.param = FILTER_PARAM_ALPHA, .value = 0.90f},
            {.param = FILTER_PARAM_KP, .value = 5.0f},
            {.param = FILTER_PARAM_KI, .value = 0.0f},
        },
        .param_count = 3
    },
    [FILTER_PRESET_ROBUST] = {
        .name = "Robust",
        .description = "鲁棒模式，抗干扰强，适合恶劣环境",
        .params = {
            {.param = FILTER_PARAM_ALPHA, .value = 0.95f},
            {.param = FILTER_PARAM_KP, .value = 2.0f},
            {.param = FILTER_PARAM_KI, .value = 0.3f},
        },
        .param_count = 3
    },
    [FILTER_PRESET_LOW_POWER] = {
        .name = "Low Power",
        .description = "低功耗模式，简化计算，适合电池供电",
        .params = {
            {.param = FILTER_PARAM_ALPHA, .value = 0.95f},
            {.param = FILTER_PARAM_CUTOFF_FREQ, .value = 5.0f},
        },
        .param_count = 2
    }
};

/* ============================================================
 * API实现
 * ============================================================ */

const filter_param_desc_t* filter_config_get_params(filter_type_t type, int *count)
{
    switch (type) {
        case FILTER_TYPE_COMPLEMENTARY:
            *count = sizeof(comp_params) / sizeof(comp_params[0]);
            return comp_params;
        case FILTER_TYPE_LPF:
            *count = sizeof(lpf_params) / sizeof(lpf_params[0]);
            return lpf_params;
        case FILTER_TYPE_EKF:
            *count = sizeof(ekf_params) / sizeof(ekf_params[0]);
            return ekf_params;
        case FILTER_TYPE_MAHONY:
            *count = sizeof(mahony_params) / sizeof(mahony_params[0]);
            return mahony_params;
        case FILTER_TYPE_MADGWICK:
            *count = sizeof(madgwick_params) / sizeof(madgwick_params[0]);
            return madgwick_params;
        default:
            *count = 0;
            return NULL;
    }
}

float filter_config_get_default(filter_type_t type, filter_param_t param)
{
    int count;
    const filter_param_desc_t *params = filter_config_get_params(type, &count);

    for (int i = 0; i < count; i++) {
        if (params[i].param == param) {
            return params[i].default_value;
        }
    }

    return 0.0f;  /* 未找到 */
}

int filter_config_validate(filter_type_t type, filter_param_t param, float value)
{
    int count;
    const filter_param_desc_t *params = filter_config_get_params(type, &count);

    for (int i = 0; i < count; i++) {
        if (params[i].param == param) {
            return (value >= params[i].min_value && value <= params[i].max_value);
        }
    }

    return 0;  /* 未找到参数 */
}

float filter_config_clamp(filter_type_t type, filter_param_t param, float value)
{
    int count;
    const filter_param_desc_t *params = filter_config_get_params(type, &count);

    for (int i = 0; i < count; i++) {
        if (params[i].param == param) {
            if (value < params[i].min_value) return params[i].min_value;
            if (value > params[i].max_value) return params[i].max_value;
            return value;
        }
    }

    return value;  /* 未找到参数，返回原值 */
}

const degrade_config_t* filter_config_get_degrade(degrade_mode_t mode)
{
    if (mode >= 0 && mode < DEGRADE_COUNT) {
        return &degrade_configs[mode];
    }
    return &degrade_configs[DEGRADE_NONE];
}

sensor_quality_t filter_config_assess_quality(float ax, float ay, float az,
                                               float gx, float gy, float gz)
{
    /* 检查NaN/Inf */
    if (isnan(ax) || isnan(ay) || isnan(az) ||
        isnan(gx) || isnan(gy) || isnan(gz) ||
        isinf(ax) || isinf(ay) || isinf(az) ||
        isinf(gx) || isinf(gy) || isinf(gz)) {
        return SENSOR_QUALITY_INVALID;
    }

    /* 检查ACC幅值 */
    float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
    if (acc_mag < DEGRADE_ACC_LOW || acc_mag > DEGRADE_ACC_HIGH) {
        return SENSOR_QUALITY_SATURATED;
    }

    /* 检查GYRO幅值 */
    float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
    if (gyro_mag > DEGRADE_GYRO_THRESHOLD) {
        return SENSOR_QUALITY_SATURATED;
    }

    return SENSOR_QUALITY_GOOD;
}

degrade_mode_t filter_config_select_degrade(sensor_quality_t acc_quality,
                                            sensor_quality_t gyro_quality)
{
    /* 优先级：INVALID > SATURATED > NOISY > GOOD */

    /* 两者都无效 → 冻结 */
    if (acc_quality == SENSOR_QUALITY_INVALID &&
        gyro_quality == SENSOR_QUALITY_INVALID) {
        return DEGRADE_HOLD_LAST;
    }

    /* ACC无效 → 仅GYRO */
    if (acc_quality == SENSOR_QUALITY_INVALID ||
        acc_quality == SENSOR_QUALITY_SATURATED) {
        return DEGRADE_GYRO_ONLY;
    }

    /* GYRO无效 → 仅ACC */
    if (gyro_quality == SENSOR_QUALITY_INVALID ||
        gyro_quality == SENSOR_QUALITY_SATURATED) {
        return DEGRADE_ACC_ONLY;
    }

    /* 有噪声 → 静态模式 */
    if (acc_quality == SENSOR_QUALITY_NOISY ||
        gyro_quality == SENSOR_QUALITY_NOISY) {
        return DEGRADE_STATIC_ONLY;
    }

    return DEGRADE_NONE;
}

void filter_config_apply_preset(filter_t *f, filter_preset_t preset)
{
    if (!f || preset < 0 || preset >= FILTER_PRESET_COUNT) {
        return;
    }

    const preset_config_t *config = &preset_configs[preset];

    for (int i = 0; i < config->param_count; i++) {
        if (f->set_param) {
            f->set_param(f, config->params[i].param, config->params[i].value);
        }
    }
}

const char* filter_config_preset_name(filter_preset_t preset)
{
    if (preset >= 0 && preset < FILTER_PRESET_COUNT) {
        return preset_configs[preset].name;
    }
    return "Unknown";
}

void filter_config_print(filter_type_t type)
{
    int count;
    const filter_param_desc_t *params = filter_config_get_params(type, &count);

    printf("=== %s 滤波器参数配置 ===\n", filter_type_name(type));

    for (int i = 0; i < count; i++) {
        printf("\n参数: %d\n", params[i].param);
        printf("  默认值: %.4f %s\n", params[i].default_value, params[i].unit);
        printf("  有效范围: [%.4f, %.4f]\n", params[i].min_value, params[i].max_value);
        printf("  来源类型: %s\n",
               params[i].source_type == PARAM_SOURCE_PAPER ? "论文" :
               params[i].source_type == PARAM_SOURCE_EMPIRICAL ? "经验值" :
               params[i].source_type == PARAM_SOURCE_DATASHEET ? "数据手册" : "调优");
        printf("  来源名称: %s\n", params[i].source_name);
        printf("  来源说明: %s\n", params[i].source_detail);
    }
}
