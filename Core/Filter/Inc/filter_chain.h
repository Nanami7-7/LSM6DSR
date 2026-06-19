/**
 * @file    filter_chain.h
 * @brief   滤波器链 — 多级滤波器串联接口
 *
 * 设计目的：
 *   - 支持多级滤波器串联（如 LPF → EKF）
 *   - 每级可独立配置参数和退化模式
 *   - 支持动态和静态内存分配
 *
 * 使用示例：
 * @code
 *   // 创建两级滤波器链：LPF 降噪 → EKF 融合
 *   filter_type_t types[] = {FILTER_TYPE_LPF, FILTER_TYPE_EKF};
 *   filter_chain_t chain;
 *   filter_chain_init(&chain, types, 2);
 *
 *   // 设置各级参数
 *   filter_chain_set_param(&chain, 0, FILTER_PARAM_CUTOFF_FREQ, 20.0f);  // LPF 截止频率
 *   filter_chain_set_param(&chain, 1, FILTER_PARAM_Q_ANGLE, 0.001f);    // EKF 过程噪声
 *
 *   // 循环更新
 *   while (1) {
 *       filter_input_t in = { .ax=ax, .ay=ay, .az=az, .gx=gx, .gy=gy, .gz=gz, .dt=dt };
 *       filter_output_t out;
 *       filter_chain_update(&chain, &in, &out);
 *       printf("pitch=%.2f roll=%.2f yaw=%.2f\n", out.pitch, out.roll, out.yaw);
 *   }
 *
 *   // 销毁
 *   filter_chain_destroy(&chain);
 * @endcode
 *
 * 级间数据流：
 *   输入 → stage[0].update → intermediate[0]
 *        → stage[1].update(intermediate[0]) → intermediate[1]
 *        → ...
 *        → stage[N-1].update → 输出
 *
 * 级间数据转换：
 *   - 第一级接收原始 sensor 数据（ax,ay,az,gx,gy,gz）
 *   - 后续级将上一级的 pitch/roll/yaw 转换为虚拟 sensor 输入
 *   - 转换公式：
 *     - ax = -sin(pitch), ay = sin(roll)*cos(pitch), az = cos(roll)*cos(pitch)
 *     - gx = 0, gy = 0, gz = 0（后续级不使用角速度）
 */

#ifndef FILTER_CHAIN_H
#define FILTER_CHAIN_H

#include "filter.h"
#include "filter_config.h"  /* for filter_preset_t */

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 滤波器链最大级数 */
#define FILTER_CHAIN_MAX_STAGES 4

/**
 * @brief 滤波器链结构体
 */
typedef struct {
    filter_t       *stages[FILTER_CHAIN_MAX_STAGES];  /**< 各级滤波器实例 */
    filter_output_t intermediate[FILTER_CHAIN_MAX_STAGES]; /**< 中间结果 */
    uint8_t         num_stages;                        /**< 实际级数 */
    uint8_t         is_static;                         /**< 是否静态分配 */
} filter_chain_t;

/* ============================================================
 * 初始化与销毁
 * ============================================================ */

/**
 * @brief 初始化滤波器链（动态内存分配）
 *
 * @param chain  滤波器链指针
 * @param types  各级滤波器类型数组
 * @param count  级数（最大 FILTER_CHAIN_MAX_STAGES）
 * @return 0=成功, -1=失败
 */
int filter_chain_init(filter_chain_t *chain, filter_type_t types[], uint8_t count);

/**
 * @brief 初始化滤波器链（静态内存分配）
 *
 * @param chain      滤波器链指针
 * @param types      各级滤波器类型数组
 * @param count      级数
 * @param bufs       各级预分配缓冲区指针数组
 * @param buf_sizes  各级缓冲区大小数组
 * @return 0=成功, -1=失败
 */
int filter_chain_init_static(filter_chain_t *chain, filter_type_t types[], uint8_t count,
                             void *bufs[], size_t buf_sizes[]);

/**
 * @brief 销毁滤波器链
 *
 * @param chain 滤波器链指针
 * @note  静态分配的链不需要调用此函数
 */
void filter_chain_destroy(filter_chain_t *chain);

/* ============================================================
 * 更新与重置
 * ============================================================ */

/**
 * @brief 执行一级滤波器链更新
 *
 * @param chain  滤波器链指针
 * @param in     输入数据（原始 sensor 数据）
 * @param out    输出数据（最后一级的输出）
 */
void filter_chain_update(filter_chain_t *chain, const filter_input_t *in, filter_output_t *out);

/**
 * @brief 重置所有级滤波器
 *
 * @param chain 滤波器链指针
 */
void filter_chain_reset(filter_chain_t *chain);

/* ============================================================
 * 参数设置
 * ============================================================ */

/**
 * @brief 设置指定级的参数
 *
 * @param chain  滤波器链指针
 * @param stage  级索引（0-based）
 * @param param  参数枚举
 * @param value  参数值
 * @return 0=成功, -1=失败
 */
int filter_chain_set_param(filter_chain_t *chain, uint8_t stage, filter_param_t param, float value);

/**
 * @brief 设置指定级的退化模式
 *
 * @param chain   滤波器链指针
 * @param stage   级索引（0-based）
 * @param degrade 退化模式
 */
void filter_chain_set_degrade(filter_chain_t *chain, uint8_t stage, filter_degrade_t degrade);

/**
 * @brief 应用预设配置到指定级
 *
 * @param chain  滤波器链指针
 * @param stage  级索引（0-based）
 * @param preset 预设类型
 */
void filter_chain_apply_preset(filter_chain_t *chain, uint8_t stage, filter_preset_t preset);

/* ============================================================
 * 查询
 * ============================================================ */

/**
 * @brief 获取级数
 *
 * @param chain 滤波器链指针
 * @return 级数
 */
uint8_t filter_chain_get_stages(const filter_chain_t *chain);

/**
 * @brief 获取指定级的滤波器类型
 *
 * @param chain 滤波器链指针
 * @param stage 级索引（0-based）
 * @return 滤波器类型，无效索引返回 FILTER_TYPE_COUNT
 */
filter_type_t filter_chain_get_type(const filter_chain_t *chain, uint8_t stage);

/**
 * @brief 获取指定级的中间输出
 *
 * @param chain 滤波器链指针
 * @param stage 级索引（0-based）
 * @return 中间输出指针，无效索引返回 NULL
 */
const filter_output_t* filter_chain_get_intermediate(const filter_chain_t *chain, uint8_t stage);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_CHAIN_H */
