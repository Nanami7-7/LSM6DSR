/**
 * @file    bench_filters.c
 * @brief   跨平台滤波器基准测试
 *
 * 编译（PC）：
 *   gcc -o bench_filters bench_filters.c \
 *       ../Core/Filter/Src/filter_factory.c ../Core/Filter/Src/filter_common.c \
 *       ../Core/Filter/Src/filter_complementary.c ../Core/Filter/Src/filter_lpf.c \
 *       ../Core/Filter/Src/filter_ekf.c ../Core/Filter/Src/filter_lkf.c \
 *       ../Core/Filter/Src/filter_mahony.c ../Core/Filter/Src/filter_madgwick.c \
 *       ../Core/Filter/Src/filter_config.c \
 *       ../Core/Filter/Platform/filter_platform_default.c \
 *       -I../Core/Filter/Inc -lm -O2 -Wall -Wextra
 *
 * 输出 TSV（Tab-Separated Values）：
 *   # filter          avg_cycles  max_cycles  avg_us    pitch     roll      yaw
 *   complementary     142         158         0.89      12.345    -1.234    0.000
 *   ...
 *
 * MCU 移植说明：
 *   - 在 MCU 上，用 fp_get_cycles() 替代 clock_gettime 获取周期计数
 *   - SystemCoreClock 替换为 MCU 实际主频
 *   - 需要 #include 相应的 HAL 头文件
 */

#include <stdio.h>
#include <string.h>
#include "filter.h"
#include "bench_dataset.h"

/* 静态缓冲区（足够容纳 EKF 228 字节） */
#define FILTER_BUF_SIZE 512
static uint8_t filter_buf[FILTER_BUF_SIZE] __attribute__((aligned(4)));

/* 滤波器类型列表（所有 6 种） */
static const filter_type_t bench_types[] = {
    FILTER_TYPE_COMPLEMENTARY,
    FILTER_TYPE_LPF,
    FILTER_TYPE_EKF,
    FILTER_TYPE_LKF,
    FILTER_TYPE_MAHONY,
    FILTER_TYPE_MADGWICK,
};
#define BENCH_COUNT (sizeof(bench_types) / sizeof(bench_types[0]))

int main(void)
{
    printf("# filter\tavg_cycles\tmax_cycles\tavg_us\tpitch\troll\tyaw\n");

    for (int t = 0; t < (int)BENCH_COUNT; t++) {
        filter_type_t type = bench_types[t];

        /* 创建滤波器 */
        memset(filter_buf, 0, FILTER_BUF_SIZE);
        filter_t *f = filter_create_static(type, filter_buf, FILTER_BUF_SIZE);
        if (!f) {
            printf("# %s\tSKIPPED (type disabled or buffer too small)\n",
                   filter_type_name(type));
            continue;
        }

        /* 跑 1000 帧，记录最后一帧输出用于数值一致性比对 */
        float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
        for (int i = 0; i < 1000; i++) {
            filter_output_t out;
            f->update(f, &bench_data[i], &out);
            pitch = out.pitch;
            roll  = out.roll;
            yaw   = out.yaw;
        }

        /* 输出 TSV（PC 无真实周期，填 0）
         * MCU 版本应替换为 fp_get_cycles 差值 / SystemCoreClock */
        printf("%s\t0\t0\t0.00\t%.4f\t%.4f\t%.4f\n",
               filter_type_name(type), pitch, roll, yaw);
    }

    return 0;
}
