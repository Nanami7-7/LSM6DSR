/**
 * @file    lsm6dsr_adapter.h
 * @brief   LSM6DSR 官方驱动适配器 — lsm6dsr_io_t ↔ stmdev_ctx_t
 *
 * 将 lsm6dsr_io_t 接口适配到官方 ST 驱动的 stmdev_ctx_t 接口，
 * 使官方 325 个函数可通过任意硬件桥接层调用。
 *
 * 设计目的：
 *   - 平台无关：适配器不依赖任何硬件
 *   - 零开销：仅函数指针赋值，无额外抽象层
 *   - 可组合：与 I2C/SPI/SoftI2C 桥接层自由组合
 *
 * 使用示例：
 * @code
 *   // 1. 初始化硬件桥接
 *   extern lsm6dsr_io_t lsm6dsr_io_spi;
 *
 *   // 2. 创建适配器
 *   stmdev_ctx_t ctx;
 *   lsm6dsr_adapter_init(&lsm6dsr_io_spi, &ctx);
 *
 *   // 3. 调用官方函数
 *   uint8_t whoami;
 *   lsm6dsr_device_id_get(&ctx, &whoami);
 *
 *   // 4. 读取传感器数据
 *   uint8_t buf[6];
 *   lsm6dsr_acceleration_raw_get(&ctx, buf);
 * @endcode
 */

#ifndef LSM6DSR_ADAPTER_H
#define LSM6DSR_ADAPTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lsm6dsr.h"       /**< lsm6dsr_io_t 定义 */
#include "lsm6dsr_reg.h"   /**< stmdev_ctx_t + 官方函数原型 */

/**
 * @brief  初始化适配器
 *
 * 将 lsm6dsr_io_t 包装为 stmdev_ctx_t，供官方 ST 驱动使用。
 * 内部创建两个静态回调函数，将 read/write 调用转发到 lsm6dsr_io_t。
 *
 * @param  io    硬件桥接层 I/O 实例（I2C/SPI/SoftI2C）
 * @param  ctx   输出：官方驱动上下文
 * @return 0=成功, -1=失败（参数为空）
 */
int lsm6dsr_adapter_init(lsm6dsr_io_t *io, stmdev_ctx_t *ctx);

/**
 * @brief  获取 WHO_AM_I（适配器版本）
 * @param  ctx  官方驱动上下文
 * @return WHO_AM_I 值（0x6B=正确），-1=通信失败
 */
int lsm6dsr_adapter_who_am_i(stmdev_ctx_t *ctx);

/**
 * @brief  读取原始加速度数据
 * @param  ctx  官方驱动上下文
 * @param  ax   输出：X 轴原始值
 * @param  ay   输出：Y 轴原始值
 * @param  az   输出：Z 轴原始值
 * @return 0=成功, -1=失败
 */
int lsm6dsr_adapter_read_accel_raw(stmdev_ctx_t *ctx,
                                     int16_t *ax, int16_t *ay, int16_t *az);

/**
 * @brief  读取原始陀螺数据
 * @param  ctx  官方驱动上下文
 * @param  gx   输出：X 轴原始值
 * @param  gy   输出：Y 轴原始值
 * @param  gz   输出：Z 轴原始值
 * @return 0=成功, -1=失败
 */
int lsm6dsr_adapter_read_gyro_raw(stmdev_ctx_t *ctx,
                                    int16_t *gx, int16_t *gy, int16_t *gz);

/**
 * @brief  读取原始温度数据
 * @param  ctx   官方驱动上下文
 * @param  temp  输出：温度原始值
 * @return 0=成功, -1=失败
 */
int lsm6dsr_adapter_read_temp_raw(stmdev_ctx_t *ctx, int16_t *temp);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSR_ADAPTER_H */
