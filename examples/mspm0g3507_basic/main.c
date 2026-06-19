/**
 * @file    main.c
 * @brief   LSM6DSR 在 MSPM0G3507 上的完整使用示例
 *
 * 演示内容：
 *   1. 系统初始化（时钟、外设、IMU）
 *   2. 陀螺仪校准
 *   3. 滤波器选择与参数调优
 *   4. 主循环：传感器数据采集 → 滤波 → 业务处理 → VOFA+ 输出
 *   5. 运行时滤波器切换（按键触发）
 *   6. 错误处理与日志输出
 *
 * 硬件连接：
 *   MSPM0G3507    LSM6DSR
 *   PB6 (SCL) →  SCL (Pin 13)
 *   PB7 (SDA) →  SDA (Pin 14)
 *   PA9 (TX)  →  UART TX (调试串口)
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
 * 业务数据结构
 * ============================================================ */

/**
 * @brief 机器狗姿态控制数据
 *
 * 从 IMU 姿态数据派生，用于步态控制
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
    int   is_level;         /**< 水平标志 (|pitch| < 5° 且 |roll| < 5°) */
} attitude_data_t;

/* ============================================================
 * 全局变量
 * ============================================================ */

static attitude_data_t g_attitude;          /**< 姿态数据 */
static uint32_t g_loop_count = 0;           /**< 主循环计数 */
static filter_type_t g_current_filter = FILTER_TYPE_EKF;  /**< 当前滤波器 */

/* 滤波器类型名称表（用于显示） */
static const char *filter_names[] = {
    "Complementary", "LPF", "EKF", "LKF", "Mahony", "Madgwick"
};

/* ============================================================
 * 业务处理函数
 * ============================================================ */

/**
 * @brief 更新姿态控制数据
 *
 * 从 IMU 数据派生业务所需的姿态信息
 */
static void update_attitude_data(const bsp_lsm6dsr_data_t *imu_data)
{
    g_attitude.pitch_deg   = imu_data->pitch;
    g_attitude.roll_deg    = imu_data->roll;
    g_attitude.yaw_deg     = imu_data->yaw;
    g_attitude.pitch_rate  = imu_data->gx;  /**< 俯仰角速度 ≈ gx */
    g_attitude.roll_rate   = imu_data->gy;  /**< 横滚角速度 ≈ gy */
    g_attitude.yaw_rate    = imu_data->gz;  /**< 偏航角速度 ≈ gz */

    /* 计算加速度幅值 (m/s²) */
    float ax = imu_data->ax;
    float ay = imu_data->ay;
    float az = imu_data->az;
    g_attitude.accel_magnitude = sqrtf(ax*ax + ay*ay + az*az);

    /* 判断是否水平 (|pitch| < 5° 且 |roll| < 5°) */
    g_attitude.is_level = (fabsf(g_attitude.pitch_deg) < 5.0f &&
                           fabsf(g_attitude.roll_deg) < 5.0f);

    /* 获取静止状态 */
    g_attitude.is_stationary = bsp_lsm6dsr_is_stationary();
}

/**
 * @brief 简单的平衡检测
 *
 * 当姿态角超过阈值时输出警告
 */
static void check_balance(void)
{
    const float PITCH_LIMIT = 30.0f;  /**< 俯仰角限幅 (度) */
    const float ROLL_LIMIT  = 25.0f;  /**< 横滚角限幅 (度) */

    if (fabsf(g_attitude.pitch_deg) > PITCH_LIMIT) {
        LOG_WARN("Pitch over limit: %.1f°", (double)g_attitude.pitch_deg);
    }

    if (fabsf(g_attitude.roll_deg) > ROLL_LIMIT) {
        LOG_WARN("Roll over limit: %.1f°", (double)g_attitude.roll_deg);
    }
}

/**
 * @brief 输出 VOFA+ 波形数据
 *
 * 10 通道：ax, ay, az, gx, gy, gz, pitch, roll, yaw, temp
 */
static void output_vofa(const bsp_lsm6dsr_data_t *data)
{
    char buf[128];
    int len = bsp_lsm6dsr_vofa_format(buf, sizeof(buf), data);
    if (len > 0) {
        g_platform->debug_printf("%s", buf);
    }
}

/**
 * @brief 输出调试信息（每 100 帧打印一次）
 */
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
 * 滤波器切换（模拟按键触发）
 * ============================================================ */

/**
 * @brief 切换到下一个滤波器
 *
 * 在实际应用中，可通过按键或命令触发
 */
static void switch_to_next_filter(void)
{
    /* 获取当前类型 */
    filter_type_t current = bsp_lsm6dsr_get_filter_type();

    /* 计算下一个类型 */
    filter_type_t next = (filter_type_t)((current + 1) % FILTER_TYPE_COUNT);

    /* 切换滤波器 */
    int ret = bsp_lsm6dsr_set_filter(next);
    if (ret == 0) {
        LOG_INFO("Filter switched: %s -> %s",
                 filter_names[current], filter_names[next]);

        /* 根据滤波器类型设置参数 */
        switch (next) {
            case FILTER_TYPE_COMPLEMENTARY:
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_ALPHA, 0.98f);
                LOG_INFO("  Complementary: alpha=0.98");
                break;

            case FILTER_TYPE_LPF:
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_CUTOFF_FREQ, 10.0f);
                LOG_INFO("  LPF: cutoff=10Hz");
                break;

            case FILTER_TYPE_EKF:
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_Q_ANGLE, 0.001f);
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_Q_BIAS, 0.003f);
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_R_MEASURE, 0.03f);
                LOG_INFO("  EKF: Q_angle=0.001, Q_bias=0.003, R=0.03");
                break;

            case FILTER_TYPE_LKF:
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_Q_ANGLE, 0.001f);
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_R_MEASURE, 0.03f);
                LOG_INFO("  LKF: Q_angle=0.001, R=0.03");
                break;

            case FILTER_TYPE_MAHONY:
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_KP, 10.0f);
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_KI, 0.3f);
                LOG_INFO("  Mahony: Kp=10, Ki=0.3");
                break;

            case FILTER_TYPE_MADGWICK:
                bsp_lsm6dsr_set_filter_param(FILTER_PARAM_KP, 0.1f);
                LOG_INFO("  Madgwick: beta=0.1");
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

/**
 * @brief 滤波器错误回调函数
 */
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

    /* 初始化 MSPM0 外设（由 SysConfig 生成） */
    SYSCFG_DL_init();

    /* 启动微秒计时器 */
    platform_timer_init();

    /* 设置滤波器错误回调 */
    filter_set_error_callback(filter_error_callback, NULL);

    LOG_INFO("=== LSM6DSR MSPM0G3507 Example ===");
    LOG_INFO("System clock: %lu Hz", (unsigned long)g_platform->system_clock_hz);

    /* ===== 2. 初始化 IMU ===== */

    LOG_INFO("Initializing IMU...");
    bsp_lsm6dsr_init();

    /* 验证初始化 */
    const bsp_lsm6dsr_data_t *init_data = bsp_lsm6dsr_get_data();
    if (init_data) {
        LOG_INFO("IMU initialized: pitch=%.2f roll=%.2f yaw=%.2f",
                 (double)init_data->pitch,
                 (double)init_data->roll,
                 (double)init_data->yaw);
    }

    /* ===== 3. 配置滤波器 ===== */

    /* 默认使用 EKF（推荐机器狗场景） */
    bsp_lsm6dsr_set_filter(FILTER_TYPE_EKF);
    bsp_lsm6dsr_set_filter_param(FILTER_PARAM_Q_ANGLE, 0.001f);
    bsp_lsm6dsr_set_filter_param(FILTER_PARAM_Q_BIAS, 0.003f);
    bsp_lsm6dsr_set_filter_param(FILTER_PARAM_R_MEASURE, 0.03f);
    LOG_INFO("Filter: EKF (Q_angle=0.001, Q_bias=0.003, R=0.03)");

    /* ===== 4. 主循环 ===== */

    LOG_INFO("Entering main loop...");
    bsp_lsm6dsr_data_t imu_data;

    while (1) {
        /* 4.1 读取传感器 + 滤波 */
        bsp_lsm6dsr_update(&imu_data);

        /* 4.2 更新业务数据 */
        update_attitude_data(&imu_data);

        /* 4.3 业务处理 */
        check_balance();

        /* 4.4 输出 VOFA+ 波形 */
        output_vofa(&imu_data);

        /* 4.5 输出调试信息 */
        output_debug_info(&imu_data);

        /* 4.6 模拟按键切换滤波器（每 1000 帧切换一次） */
        if (g_loop_count > 0 && g_loop_count % 1000 == 0) {
            switch_to_next_filter();
        }

        /* 4.7 更新计数器 */
        g_loop_count++;

        /* 4.8 控制循环频率 (100Hz) */
        g_platform->delay_ms(10);
    }

    /* 不会执行到这里 */
    return 0;
}
