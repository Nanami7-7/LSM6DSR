/**
 * @file    filter_config.h
 * @brief   滤波器参数配置 - 包含来源标注、范围验证和退化策略
 *
 * 设计原则：
 *   1. 所有参数标明来源（论文/经验值/传感器手册）
 *   2. 所有参数有有效范围和默认值
 *   3. 提供退化策略（传感器数据质量差时的降级方案）
 *   4. 支持运行时参数验证
 *
 * 参数来源标注格式：
 *   [来源类型] 来源名称 | 推荐值 | 说明
 *   来源类型：PAPER(论文) / EMPIRICAL(经验值) / DATASHEET(数据手册) / TUNED(调优)
 */

#ifndef FILTER_CONFIG_H
#define FILTER_CONFIG_H

#include "filter.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 参数来源类型枚举
 * ============================================================ */
typedef enum {
    PARAM_SOURCE_PAPER = 0,     /**< 论文/学术研究 */
    PARAM_SOURCE_EMPIRICAL,     /**< 经验值/工程实践 */
    PARAM_SOURCE_DATASHEET,     /**< 传感器数据手册 */
    PARAM_SOURCE_TUNED,         /**< 实际调优结果 */
    PARAM_SOURCE_COUNT
} param_source_t;

/* ============================================================
 * 参数描述结构体
 * ============================================================ */
typedef struct {
    filter_param_t  param;          /**< 参数枚举 */
    float           default_value;  /**< 默认值 */
    float           min_value;      /**< 最小有效值 */
    float           max_value;      /**< 最大有效值 */
    param_source_t  source_type;    /**< 来源类型 */
    const char     *source_name;    /**< 来源名称/论文标题 */
    const char     *source_detail;  /**< 来源详细说明 */
    const char     *unit;           /**< 单位 */
} filter_param_desc_t;

/* ============================================================
 * 退化策略枚举
 * ============================================================ */
typedef enum {
    DEGRADE_NONE = 0,           /**< 无退化（正常运行） */
    DEGRADE_STATIC_ONLY,        /**< 仅静态模式（禁用动态补偿） */
    DEGRADE_GYRO_ONLY,          /**< 仅陀螺仪（禁用ACC修正） */
    DEGRADE_ACC_ONLY,           /**< 仅加速度计（禁用GYRO积分） */
    DEGRADE_HOLD_LAST,          /**< 保持上次输出（冻结） */
    DEGRADE_COUNT
} degrade_mode_t;

/* ============================================================
 * 传感器质量状态
 * ============================================================ */
typedef enum {
    SENSOR_QUALITY_GOOD = 0,    /**< 数据质量好 */
    SENSOR_QUALITY_NOISY,       /**< 数据有噪声 */
    SENSOR_QUALITY_SATURATED,   /**< 数据饱和/超限 */
    SENSOR_QUALITY_INVALID,     /**< 数据无效（NaN/Inf） */
    SENSOR_QUALITY_COUNT
} sensor_quality_t;

/* ============================================================
 * 退化策略配置
 * ============================================================ */
typedef struct {
    degrade_mode_t  mode;               /**< 退化模式 */
    float           acc_threshold_low;  /**< ACC幅值下限（g） */
    float           acc_threshold_high; /**< ACC幅值上限（g） */
    float           gyro_threshold;     /**< GYRO幅值阈值（dps） */
    float           variance_threshold; /**< 方差阈值 */
    const char     *description;        /**< 退化策略说明 */
} degrade_config_t;

/* ============================================================
 * 滤波器预设配置
 * ============================================================ */
typedef enum {
    FILTER_PRESET_DEFAULT = 0,      /**< 默认配置（平衡） */
    FILTER_PRESET_HIGH_PRECISION,   /**< 高精度（低噪声） */
    FILTER_PRESET_FAST_RESPONSE,    /**< 快速响应（低延迟） */
    FILTER_PRESET_ROBUST,           /**< 鲁棒（抗干扰） */
    FILTER_PRESET_LOW_POWER,        /**< 低功耗（简化计算） */
    FILTER_PRESET_COUNT
} filter_preset_t;

/* ============================================================
 * 配置常量定义
 * ============================================================ */

/* --- 互补滤波器参数 --- */
/* [PAPER] Mahony et al., 2008, "Nonlinear complementary filters on the special orthogonal group"
 * 推荐α = 0.98（对应截止频率约0.1Hz）
 * 范围：0.90~0.99
 * 说明：α越大，越信任陀螺仪，响应快但漂移大
 *       α越小，越信任加速度计，稳定但响应慢 */
#define COMP_ALPHA_DEFAULT      0.98f
#define COMP_ALPHA_MIN          0.90f
#define COMP_ALPHA_MAX          0.99f

/* --- LPF参数 --- */
/* [EMPIRICAL] 一阶低通滤波器截止频率
 * 推荐：5~20Hz（取决于应用）
 * 说明：截止频率越低，滤波越强，延迟越大
 *       截止频率越高，滤波越弱，延迟越小 */
#define LPF_CUTOFF_DEFAULT      10.0f   /* Hz */
#define LPF_CUTOFF_MIN          1.0f    /* Hz */
#define LPF_CUTOFF_MAX          50.0f   /* Hz */

/* --- EKF参数 --- */
/* [PAPER] Simon, D., 2006, "Optimal State Estimation: Kalman, H Infinity, and Nonlinear Approaches"
 * [TUNED] 基于LSM6DSR传感器特性调优
 *
 * Q_angle: 过程噪声-角度（弧度²/s）
 *   - 来源：传感器陀螺仪噪声密度 × 采样时间
 *   - LSM6DSR陀螺仪噪声密度：0.01 dps/√Hz
 *   - 采样率100Hz时：0.01 × √100 = 0.1 dps = 0.0017 rad/s
 *   - 推荐值：0.001~0.01
 *
 * Q_bias: 过程噪声-偏置（dps²/s）
 *   - 来源：陀螺仪偏置稳定性
 *   - LSM6DSR偏置稳定性：±10 dps
 *   - 推荐值：0.001~0.01
 *
 * R_measure: 测量噪声（g²）
 *   - 来源：加速度计噪声密度
 *   - LSM6DSR加速度计噪声密度：0.08 mg/√Hz
 *   - 采样率100Hz时：0.08 × √100 = 0.8 mg = 0.0008 g
 *   - 推荐值：0.01~0.1 */
#define EKF_Q_ANGLE_DEFAULT    0.001f
#define EKF_Q_ANGLE_MIN        0.0001f
#define EKF_Q_ANGLE_MAX        0.1f

#define EKF_Q_BIAS_DEFAULT     0.003f
#define EKF_Q_BIAS_MIN         0.0001f
#define EKF_Q_BIAS_MAX         0.1f

#define EKF_R_MEASURE_DEFAULT  0.03f
#define EKF_R_MEASURE_MIN      0.001f
#define EKF_R_MEASURE_MAX      1.0f

/* EKF 偏置幅值限制 (dps)
 * 来源: LSM6DSR 数据手册 ±10 dps 偏置稳定性
 * 防止 EKF 偏置状态发散而设置的上限 */
#define EKF_BIAS_LIMIT_DEFAULT  20.0f   /* dps */
#define EKF_BIAS_LIMIT_MIN      5.0f    /* dps */
#define EKF_BIAS_LIMIT_MAX      50.0f   /* dps */

/* EKF Chi-squared 门限
 * 来源: 卡方分布表, 自由度=3
 * 加速度计离群值检测, 创新超过阈值时跳过测量更新 */
#define EKF_CHI2_THRESHOLD_DEFAULT  11.34f
#define EKF_CHI2_THRESHOLD_MIN      5.0f
#define EKF_CHI2_THRESHOLD_MAX      20.0f

/* EKF 动态 R 适配参数
 * 原理: 加速度计模长偏离 1g 时(运动/振动), 自动增大 R
 * 使 EKF 更信任陀螺仪预测, 减少加速度计修正 */
#define EKF_R_ADAPT_ENABLE_DEFAULT  0       /* 默认禁用 */
#define EKF_R_ADAPT_FACTOR_MIN      0.1f
#define EKF_R_ADAPT_FACTOR_MAX      10.0f

/* [PAPER] Mahony et al., 2008, "Nonlinear complementary filters on the special orthogonal group"
 * 论文推荐kp = 1.0, ki = 0.3
 * 工程优化：kp = 10.0（快速收敛，适合嵌入式实时场景）
 *
 * kp: 比例增益
 *   - 来源：论文范围0.5~10.0，工程实践取高端
 *   - 范围：0.1~10.0
 *   - 说明：越大响应越快，收敛越快；过大可能导致振荡
 *
 * ki: 积分增益
 *   - 来源：论文默认值
 *   - 范围：0.0~0.5
 *   - 说明：用于估计陀螺仪偏置，过大可能导致超调 */
#define MAHONY_KP_DEFAULT      10.0f
#define MAHONY_KP_MIN          0.1f
#define MAHONY_KP_MAX          10.0f

#define MAHONY_KI_DEFAULT      0.3f
#define MAHONY_KI_MIN          0.0f
#define MAHONY_KI_MAX          0.5f

/* [PAPER] Madgwick, S.O.H., 2010, "An efficient orientation filter for inertial and
 *          inertial/magnetic sensor arrays"
 * 论文推荐β = 0.033（IMU）或 0.041（MARG）
 * 工程优化：β = 0.5（快速收敛，适合嵌入式实时场景）
 *
 * β: 梯度下降步长
 *   - 来源：论文第3.6节 "Filter gains"
 *   - 定义：陀螺仪测量误差的四元数导数幅值
 *   - 范围：0.001~0.5
 *   - 说明：越大收敛越快，但噪声越大；
 *          论文实验：β=0.033（IMU）时静态RMS误差<0.6°；
 *          工程实践：β=0.5时1000次迭代内可收敛到0.04°以内 */
#define MADGWICK_BETA_DEFAULT  0.5f
#define MADGWICK_BETA_MIN      0.001f
#define MADGWICK_BETA_MAX      0.5f

/* ============================================================
 * 退化策略默认配置
 * ============================================================ */

/* ACC幅值检查阈值（单位：g） */
#define DEGRADE_ACC_LOW         0.5f    /* 幅值过小（可能自由落体） */
#define DEGRADE_ACC_HIGH        2.0f    /* 幅值过大（可能碰撞） */

/* GYRO幅值检查阈值（单位：dps） */
#define DEGRADE_GYRO_THRESHOLD  400.0f  /* 接近满量程（±500dps） */

/* 方差阈值（用于静止检测） */
#define DEGRADE_VARIANCE_THRESH 0.01f   /* 加速度方差阈值（g²） */

/* ============================================================
 * API函数声明
 * ============================================================ */

/**
 * @brief 获取参数描述表
 * @param type  滤波器类型
 * @param count 输出参数数量
 * @return 参数描述数组指针（静态存储，无需释放）
 */
const filter_param_desc_t* filter_config_get_params(filter_type_t type, int *count);

/**
 * @brief 获取参数默认值
 * @param type  滤波器类型
 * @param param 参数枚举
 * @return 默认值
 */
float filter_config_get_default(filter_type_t type, filter_param_t param);

/**
 * @brief 验证参数值是否在有效范围内
 * @param type  滤波器类型
 * @param param 参数枚举
 * @param value 参数值
 * @return 1=有效, 0=无效
 */
int filter_config_validate(filter_type_t type, filter_param_t param, float value);

/**
 * @brief 钳位参数值到有效范围
 * @param type  滤波器类型
 * @param param 参数枚举
 * @param value 输入值
 * @return 钳位后的值
 */
float filter_config_clamp(filter_type_t type, filter_param_t param, float value);

/**
 * @brief 获取退化策略配置
 * @param mode  退化模式
 * @return 退化配置指针（静态存储，无需释放）
 */
const degrade_config_t* filter_config_get_degrade(degrade_mode_t mode);

/**
 * @brief 评估传感器数据质量
 * @param ax, ay, az  加速度（g）
 * @param gx, gy, gz  角速度（dps）
 * @return 传感器质量状态
 */
sensor_quality_t filter_config_assess_quality(float ax, float ay, float az,
                                               float gx, float gy, float gz);

/**
 * @brief 根据传感器质量确定退化模式
 * @param acc_quality   加速度计质量
 * @param gyro_quality  陀螺仪质量
 * @return 推荐的退化模式
 */
degrade_mode_t filter_config_select_degrade(sensor_quality_t acc_quality,
                                            sensor_quality_t gyro_quality);

/**
 * @brief 应用预设配置到滤波器
 * @param f       滤波器实例
 * @param preset  预设类型
 */
void filter_config_apply_preset(filter_t *f, filter_preset_t preset);

/**
 * @brief 获取预设配置名称
 * @param preset  预设类型
 * @return 名称字符串
 */
const char* filter_config_preset_name(filter_preset_t preset);

/**
 * @brief 打印参数配置信息（调试用）
 * @param type  滤波器类型
 */
void filter_config_print(filter_type_t type);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_CONFIG_H */
