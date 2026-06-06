/**
 * @file    filter.h
 * @brief   滤波器统一接口定义
 *
 * 设计理念：
 *   - 0成本抽象：函数指针调用，无虚函数表开销
 *   - 可读性：清晰的命名和注释
 *   - 可扩展性：添加新滤波器只需实现接口
 *   - 向后兼容：保留原有互补滤波器
 *
 * 支持的滤波器（6种）：
 *   - 互补滤波器 (Complementary) — 经典陀螺仪+加速度计融合，α权重
 *   - 一阶低通滤波器 (LPF) — 截止频率可调，适合噪声滤除
 *   - 扩展卡尔曼滤波器 (EKF) — 7状态EKF，含陀螺偏置估计
 *   - 线性卡尔曼滤波器 (LKF) — 6状态LKF，线性近似，低算力场景
 *   - Mahony滤波器 — PI控制器互补滤波，快速收敛
 *   - Madgwick滤波器 — 梯度下降法，四元数姿态估计
 *
 * 退化模式（5种）：
 *   - NONE: 正常运行
 *   - STATIC_ONLY: 仅静态模式（禁用动态补偿）
 *   - GYRO_ONLY: ACC不可靠时仅用陀螺仪
 *   - ACC_ONLY: GYRO饱和时仅用加速度计
 *   - HOLD_LAST: 冻结输出（保持上次结果）
 *
 * 配置系统：
 *   - filter_config.h: 参数来源标注、范围验证、退化策略、预设配置
 *   - 所有参数标明来源（论文/经验值/传感器手册/调优）
 *   - 支持运行时参数验证和自动退化模式选择
 *
 * 使用示例：
 * @code
 *   #include "filter.h"
 *
 *   filter_t *f = filter_create(FILTER_TYPE_MADGWICK);
 *   filter_input_t in = { .ax=ax, .ay=ay, .az=az, .gx=gx, .gy=gy, .gz=gz, .dt=0.01f };
 *   filter_output_t out;
 *   f->update(f, &in, &out);
 *   printf("pitch=%.2f roll=%.2f yaw=%.2f\n", out.pitch, out.roll, out.yaw);
 *   f->destroy(f);
 * @endcode
 *
 * @see filter_config.h  参数配置与退化策略
 * @see bsp_lsm6dsr.h   BSP层集成接口
 */

#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 滤波器类型枚举 ---- */
typedef enum {
    FILTER_TYPE_COMPLEMENTARY = 0,  /**< 现有互补滤波器 */
    FILTER_TYPE_LPF,                /**< 一阶低通滤波器 */
    FILTER_TYPE_EKF,                /**< 扩展卡尔曼滤波器 */
    FILTER_TYPE_LKF,                /**< 线性卡尔曼滤波器 */
    FILTER_TYPE_MAHONY,             /**< Mahony 互补滤波器 */
    FILTER_TYPE_MADGWICK,           /**< Madgwick 梯度下降滤波器 */
    FILTER_TYPE_COUNT               /**< 滤波器数量（勿用） */
} filter_type_t;

/* ---- 滤波器输入数据 ---- */
typedef struct {
    float ax, ay, az;   /**< 加速度 (g) */
    float gx, gy, gz;   /**< 角速度 (dps) */
    float dt;           /**< 时间差 (秒) */
} filter_input_t;

/* ---- 退化模式（用于传感器数据质量差时） ---- */
typedef enum {
    FILTER_DEGRADE_NONE = 0,        /**< 正常运行 */
    FILTER_DEGRADE_STATIC_ONLY,     /**< 仅静态模式（禁用动态补偿） */
    FILTER_DEGRADE_GYRO_ONLY,       /**< 仅陀螺仪（禁用ACC修正） */
    FILTER_DEGRADE_ACC_ONLY,        /**< 仅加速度计（禁用GYRO积分） */
    FILTER_DEGRADE_HOLD_LAST,       /**< 保持上次输出（冻结） */
    FILTER_DEGRADE_COUNT
} filter_degrade_t;

/* ---- 滤波器输出数据 ---- */
typedef struct {
    float pitch;        /**< 俯仰角 (度) */
    float roll;         /**< 横滚角 (度) */
    float yaw;          /**< 偏航角 (度) */
    float q0, q1, q2, q3;  /**< 四元数（可选，某些滤波器内部使用） */
} filter_output_t;

/* ---- 滤波器参数枚举（用于 set_param） ---- */
typedef enum {
    FILTER_PARAM_ALPHA = 0,         /**< 互补滤波器 α */
    FILTER_PARAM_CUTOFF_FREQ,       /**< 低通滤波器截止频率 (Hz) */
    FILTER_PARAM_Q_ANGLE,           /**< EKF 过程噪声-角度 */
    FILTER_PARAM_Q_BIAS,            /**< EKF 过程噪声-偏置 */
    FILTER_PARAM_R_MEASURE,         /**< EKF 测量噪声 */
    FILTER_PARAM_KP,                /**< Mahony/Madgwick 比例增益 */
    FILTER_PARAM_KI,                /**< Mahony/Madgwick 积分增益 */
    FILTER_PARAM_COUNT              /**< 参数数量（勿用） */
} filter_param_t;

/* ---- 滤波器接口（前向声明） ---- */
typedef struct filter filter_t;

/**
 * @brief 滤波器更新函数类型
 * @param self  滤波器实例指针
 * @param in    输入数据（加速度、角速度、dt）
 * @param out   输出数据（姿态角、四元数）
 */
typedef void (*filter_update_fn)(filter_t *self, const filter_input_t *in, filter_output_t *out);

/**
 * @brief 滤波器重置函数类型
 * @param self  滤波器实例指针
 */
typedef void (*filter_reset_fn)(filter_t *self);

/**
 * @brief 滤波器参数设置函数类型
 * @param self  滤波器实例指针
 * @param param 参数枚举
 * @param value 参数值
 */
typedef void (*filter_set_param_fn)(filter_t *self, filter_param_t param, float value);

/**
 * @brief 滤波器销毁函数类型
 * @param self  滤波器实例指针
 */
typedef void (*filter_destroy_fn)(filter_t *self);

/* ---- 滤波器接口结构体 ---- */
struct filter {
    filter_update_fn    update;     /**< 更新函数 */
    filter_reset_fn     reset;      /**< 重置函数 */
    filter_set_param_fn set_param;  /**< 参数设置函数 */
    filter_destroy_fn   destroy;    /**< 销毁函数（释放私有数据） */
    filter_type_t       type;       /**< 滤波器类型 */
    filter_degrade_t    degrade;    /**< 退化模式（传感器数据异常时降级） */
    void               *priv;       /**< 私有数据指针 */
    uint8_t             is_static;  /**< 是否静态分配（1=静态，0=动态） */
};

/* ---- 数值安全保护配置 ---- */
typedef struct {
    float angle_min;        /**< 角度最小值 (度) */
    float angle_max;        /**< 角度最大值 (度) */
    float q_norm_thresh;    /**< 四元数归一化阈值 */
    float cov_reg_factor;   /**< 协方差矩阵正则化因子 */
    uint16_t norm_interval; /**< 四元数归一化间隔（帧数） */
} filter_safety_config_t;

/* 默认安全配置 */
#define FILTER_SAFETY_DEFAULT { \
    .angle_min = -180.0f, \
    .angle_max =  180.0f, \
    .q_norm_thresh = 0.001f, \
    .cov_reg_factor = 1e-6f, \
    .norm_interval = 10 \
}

/* ---- 工厂函数 ---- */

/**
 * @brief 创建滤波器实例
 * @param type  滤波器类型
 * @return 滤波器指针，失败返回 NULL
 *
 * 使用示例：
 * @code
 *   filter_t *f = filter_create(FILTER_TYPE_MADGWICK);
 *   if (!f) { error handling... }
 *
 *   filter_input_t in = { .ax=ax, .ay=ay, .az=az, .gx=gx, .gy=gy, .gz=gz, .dt=dt };
 *   filter_output_t out;
 *   f->update(f, &in, &out);
 *
 *   printf("pitch=%.2f roll=%.2f yaw=%.2f\n", out.pitch, out.roll, out.yaw);
 *
 *   f->destroy(f);  // 释放资源
 * @endcode
 */
filter_t* filter_create(filter_type_t type);

/**
 * @brief 静态创建滤波器实例（无动态内存分配）
 * @param type  滤波器类型
 * @param buf   预分配的缓冲区（大小需 >= filter_get_static_size(type)）
 * @param buf_size  缓冲区大小
 * @return 滤波器指针，失败返回 NULL
 *
 * MCU友好：不使用malloc/free，适合无MMU的嵌入式系统
 */
filter_t* filter_create_static(filter_type_t type, void *buf, size_t buf_size);

/**
 * @brief 获取静态分配所需的缓冲区大小
 * @param type  滤波器类型
 * @return 所需字节数
 */
size_t filter_get_static_size(filter_type_t type);

/**
 * @brief 设置滤波器安全配置
 * @param f       滤波器实例
 * @param config  安全配置
 */
void filter_set_safety_config(filter_t *f, const filter_safety_config_t *config);

/**
 * @brief 验证输出有效性
 * @param out  输出数据
 * @return 1=有效, 0=无效（包含NaN/Inf）
 */
int filter_validate_output(const filter_output_t *out);

/**
 * @brief 四元数归一化
 * @param q0, q1, q2, q3  四元数（输入/输出）
 */
void filter_normalize_quaternion(float *q0, float *q1, float *q2, float *q3);

/**
 * @brief 协方差矩阵正则化（确保正定性）
 * @param P       协方差矩阵
 * @param size    矩阵维度
 * @param factor  正则化因子
 */
void filter_regularize_covariance(float P[][7], int size, float factor);
/**
 * @brief 获取滤波器类型名称（用于调试）
 * @param type  滤波器类型
 * @return 类型名称字符串
 */
const char* filter_type_name(filter_type_t type);

/**
 * @brief 设置滤波器退化模式
 * @param f       滤波器实例
 * @param degrade 退化模式
 *
 * 当传感器数据质量差时，可以降级滤波器运行模式：
 * - FILTER_DEGRADE_NONE: 正常运行
 * - FILTER_DEGRADE_GYRO_ONLY: ACC不可靠时仅用陀螺仪
 * - FILTER_DEGRADE_ACC_ONLY: GYRO饱和时仅用加速度计
 * - FILTER_DEGRADE_HOLD_LAST: 冻结输出
 */
void filter_set_degrade(filter_t *f, filter_degrade_t degrade);

/**
 * @brief 获取退化模式名称
 * @param degrade 退化模式
 * @return 名称字符串
 */
const char* filter_degrade_name(filter_degrade_t degrade);

/**
 * @brief 评估加速度计数据质量
 * @param ax, ay, az  加速度（g）
 * @return 1=正常（幅值在阈值内）, 0=异常
 */
int filter_check_acc_quality(float ax, float ay, float az);

/**
 * @brief 评估陀螺仪数据质量
 * @param gx, gy, gz  角速度（dps）
 * @return 1=正常（未饱和）, 0=异常
 */
int filter_check_gyro_quality(float gx, float gy, float gz);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */
