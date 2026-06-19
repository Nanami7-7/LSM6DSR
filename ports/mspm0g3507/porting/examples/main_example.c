/**
 * @file    main_example.c
 * @brief   LSM6DSR MSPM0G3507 完整使用示例
 *
 * 演示内容：
 *   1. 系统初始化（时钟、外设、IMU）
 *   2. 通信方式选择（I2C/SPI/SoftI2C）
 *   3. 滤波器配置与切换
 *   4. 主循环：传感器数据采集 → 滤波 → 业务处理 → VOFA+ 输出
 *   5. 错误处理与日志输出
 *
 * 硬件连接：
 *   MSPM0G3507    LSM6DSR
 *   PB6 (SCL) →  SCL (Pin 13)  [I2C 模式]
 *   PB7 (SDA) →  SDA (Pin 14)  [I2C 模式]
 *   PB9 (SCK) →  SCK (Pin 13)  [SPI 模式]
 *   PB8 (MOSI)→  SDA (Pin 14)  [SPI 模式]
 *   PB7 (MISO)→  SDO (Pin 12)  [SPI 模式]
 *   PB17 (CS) →  CS  (Pin 15)  [SPI 模式]
 *   PA9 (TX)  →  UART TX       [调试串口]
 *   PA10 (RX) →  UART RX
 *
 * 编译：Keil MDK + MSPM0 SDK
 */

#include "ti_msp_dl_config.h"
#include "platform.h"
#include "bsp_lsm6dsr.h"
#include "filter.h"
#include "log.h"
#include <stdio.h>
#include <math.h>

/* ============================================================
 * 通信方式选择（在 project_config.h 中定义）
 * ============================================================ */
#ifdef USE_SPI
    extern lsm6dsr_io_t lsm6dsr_io_spi;
    extern void spi_bridge_init(void);
    #define LSM6DSR_IO  lsm6dsr_io_spi
#elif defined(USE_SOFT_I2C)
    extern lsm6dsr_io_t lsm6dsr_io_soft;
    extern void soft_i2c_bridge_init(void);
    #define LSM6DSR_IO  lsm6dsr_io_soft
#else
    extern lsm6dsr_io_t lsm6dsr_io;
    #define LSM6DSR_IO  lsm6dsr_io
#endif

/* ============================================================
 * 业务数据结构
 * ============================================================ */

/**
 * @brief 机器狗姿态控制数据
 */
typedef struct {
    float pitch_deg;        /**< 俯仰角 (度) */
    float roll_deg;         /**< 横滚角 (度) */
    float yaw_deg;          /**< 偏航角 (度) */
    float pitch_rate;       /**< 俯仰角速度 (dps) */
    float roll_rate;        /**< 横滚角速度 (dps) */
    float yaw_rate;         /**< 偏航角速度 (dps) */
    float accel_magnitude;  /**< 加速度幅值 (m/s²) */
    int   is_stationary;    /**< 静止标志 */
    int   is_level;         /**< 水平标志 */
} attitude_data_t;

/* ============================================================
 * 全局变量
 * ============================================================ */

static attitude_data_t g_attitude;
static uint32_t g_loop_count = 0;

static const char *filter_names[] = {
    "Complementary", "LPF", "EKF", "LKF", "Mahony", "Madgwick"
};

/* ============================================================
 * 业务处理函数
 * ============================================================ */

static void update_attitude_data(const bsp_lsm6dsr_data_t *imu_data)
{
    g_attitude.pitch_deg   = imu_data->pitch;
    g_attitude.roll_deg    = imu_data->roll;
    g_attitude.yaw_deg     = imu_data->yaw;
    g_attitude.pitch_rate  = imu_data->gx;
    g_attitude.roll_rate   = imu_data->gy;
    g_attitude.yaw_rate    = imu_data->gz;

    float ax = imu_data->ax;
    float ay = imu_data->ay;
    float az = imu_data->az;
    g_attitude.accel_magnitude = sqrtf(ax*ax + ay*ay + az*az);

    g_attitude.is_level = (fabsf(g_attitude.pitch_deg) < 5.0f &&
                           fabsf(g_attitude.roll_deg) < 5.0f);

    g_attitude.is_stationary = bsp_lsm6dsr_is_stationary();
}

static void check_balance(void)
{
    const float PITCH_LIMIT = 30.0f;
    const float ROLL_LIMIT  = 25.0f;

    if (fabsf(g_attitude.pitch_deg) > PITCH_LIMIT) {
        LOG_WARN("Pitch over limit: %.1f°", (double)g_attitude.pitch_deg);
    }

    if (fabsf(g_attitude.roll_deg) > ROLL_LIMIT) {
        LOG_WARN("Roll over limit: %.1f°", (double)g_attitude.roll_deg);
    }
}

static void output_vofa(const bsp_lsm6dsr_data_t *data)
{
    char buf[128];
    int len = bsp_lsm6dsr_vofa_format(buf, sizeof(buf), data);
    if (len > 0) {
        g_platform->debug_printf("%s", buf);
    }
}

static void output_debug_info(const bsp_lsm6dsr_data_t *data)
{
    if (g_loop_count % 100 == 0) {
        float bx, by, bz;
        bsp_lsm6dsr_get_bias(&bx, &by, &bz);

        LOG_INFO("Frame %lu: P=%.1f R=%.1f Y=%.1f | "
                 "Stationary=%d Level=%d | "
                 "Filter=%s Bias=[%.3f, %.3f, %.3f]",
                 (unsigned long)g_loop_count,
                 (double)g_attitude.pitch_deg,
                 (double)g_attitude.roll_deg,
                 (double)g_attitude.yaw_deg,
                 g_attitude.is_stationary,
                 g_attitude.is_level,
                 bsp_lsm6dsr_get_filter_name(),
                 (double)bx, (double)by, (double)bz);
    }
}

/* ============================================================
 * 滤波器切换
 * ============================================================ */

static void switch_to_next_filter(void)
{
    filter_type_t current = bsp_lsm6dsr_get_filter_type();
    filter_type_t next = (filter_type_t)((current + 1) % FILTER_TYPE_COUNT);

    int ret = bsp_lsm6dsr_set_filter(next);
    if (ret == 0) {
        LOG_INFO("Filter switched: %s -> %s",
                 filter_names[current], filter_names[next]);

        switch (next) {
            case FILTER_TYPE_COMPLEMENTARY:
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_ALPHA, 0.98f);
                break;
            case FILTER_TYPE_LPF:
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_CUTOFF_FREQ, 10.0f);
                break;
            case FILTER_TYPE_EKF:
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_Q_ANGLE, 0.001f);
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_Q_BIAS, 0.003f);
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_R_MEASURE, 0.03f);
                break;
            case FILTER_TYPE_LKF:
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_Q_ANGLE, 0.001f);
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_R_MEASURE, 0.03f);
                break;
            case FILTER_TYPE_MAHONY:
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_KP, 10.0f);
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_KI, 0.3f);
                break;
            case FILTER_TYPE_MADGWICK:
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_KP, 0.1f);
                break;
            default:
                break;
        }
    } else {
        LOG_ERR("Failed to switch filter to %d", next);
    }
}

/* ============================================================
 * 错误回调
 * ============================================================ */

static void filter_error_callback(const filter_error_info_t *info, void *user_data)
{
    (void)user_data;
    LOG_ERR("Filter error: %s (code=%d) at %s:%d",
            info->message, info->code, info->file, info->line);
}

/* ============================================================
 * 主函数
 * ============================================================ */

int main(void)
{
    /* ===== 1. 系统初始化 ===== */
    SYSCFG_DL_init();
    platform_timer_init();

    /* ===== 2. 通信方式初始化 ===== */
    #ifdef USE_SPI
        spi_bridge_init();
        LOG_INFO("Using SPI bridge");
    #elif defined(USE_SOFT_I2C)
        soft_i2c_bridge_init();
        LOG_INFO("Using Soft I2C bridge");
    #else
        LOG_INFO("Using Hardware I2C bridge");
    #endif

    /* 设置滤波器错误回调 */
    filter_set_error_callback(filter_error_callback, NULL);

    LOG_INFO("=== LSM6DSR MSPM0G3507 Example ===");
    LOG_INFO("System clock: %lu Hz", (unsigned long)g_platform->system_clock_hz);

    /* ===== 3. 初始化 IMU ===== */
    LOG_INFO("Initializing IMU...");
    bsp_lsm6dsr_init();

    const bsp_lsm6dsr_data_t *init_data = bsp_lsm6dsr_get_data();
    if (init_data) {
        LOG_INFO("IMU initialized: pitch=%.2f roll=%.2f yaw=%.2f",
                 (double)init_data->pitch,
                 (double)init_data->roll,
                 (double)init_data->yaw);
    }

    /* ===== 4. 配置滤波器 ===== */
    bsp_lsm6dsr_set_filter(FILTER_TYPE_EKF);
    bsp_lsm6dsr_set_filter_param(FILTER_PARAM_Q_ANGLE, 0.001f);
    bsp_lsm6dsr_set_filter_param(FILTER_PARAM_Q_BIAS, 0.003f);
    bsp_lsm6dsr_set_filter_param(FILTER_PARAM_R_MEASURE, 0.03f);
    LOG_INFO("Filter: EKF (Q_angle=0.001, Q_bias=0.003, R=0.03)");

    /* ===== 5. 主循环 ===== */
    LOG_INFO("Entering main loop...");
    bsp_lsm6dsr_data_t imu_data;

    while (1) {
        /* 5.1 读取传感器 + 滤波 */
        bsp_lsm6dsr_update(&imu_data);

        /* 5.2 更新业务数据 */
        update_attitude_data(&imu_data);

        /* 5.3 业务处理 */
        check_balance();

        /* 5.4 输出 VOFA+ 波形 */
        output_vofa(&imu_data);

        /* 5.5 输出调试信息 */
        output_debug_info(&imu_data);

        /* 5.6 模拟按键切换滤波器（每 1000 帧切换一次） */
        if (g_loop_count > 0 && g_loop_count % 1000 == 0) {
            switch_to_next_filter();
        }

        /* 5.7 更新计数器 */
        g_loop_count++;

        /* 5.8 控制循环频率 (100Hz) */
        g_platform->delay_ms(10);
    }

    return 0;
}
