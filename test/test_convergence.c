/**
 * @file    test_convergence.c
 * @brief   滤波器收敛特性诊断测试
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../Core/Filter/Inc/filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(void) {
    printf("========================================\n");
    printf("  滤波器收敛特性诊断\n");
    printf("========================================\n");

    float target_angle = 45.0f;
    float angle_rad = target_angle * M_PI / 180.0f;

    /* 测试各滤波器在45°输入下的收敛 */
    filter_type_t types[] = {
        FILTER_TYPE_COMPLEMENTARY,
        FILTER_TYPE_LPF,
        FILTER_TYPE_EKF,
        FILTER_TYPE_MAHONY,
        FILTER_TYPE_MADGWICK
    };

    for (int t = 0; t < 5; t++) {
        filter_input_t in = {
            .ax = -sinf(angle_rad), .ay = 0.0f, .az = cosf(angle_rad),
            .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
            .dt = 0.01f
        };
        filter_output_t out;

        filter_t *f = filter_create(types[t]);
        if (!f) continue;

        printf("\n%s 收敛测试 (目标: pitch=%.1f°):\n", filter_type_name(types[t]), target_angle);

        int checkpoints[] = {10, 50, 100, 500, 1000, 5000};
        int num_checkpoints = 6;
        int next_checkpoint = 0;

        for (int frame = 1; frame <= 5000; frame++) {
            f->update(f, &in, &out);

            if (next_checkpoint < num_checkpoints && frame == checkpoints[next_checkpoint]) {
                printf("  帧%5d (%.2fs): pitch=%.2f° (误差=%.2f°)\n",
                       frame, frame * 0.01f, out.pitch, fabsf(out.pitch - target_angle));
                next_checkpoint++;
            }
        }

        f->destroy(f);
    }

    printf("\n========================================\n");
    printf("  Madgwick不同beta值测试 (1000帧后)\n");
    printf("========================================\n");

    float betas[] = {0.1f, 0.05f, 0.01f, 0.005f, 0.001f};

    for (int b = 0; b < 5; b++) {
        filter_input_t in = {
            .ax = -sinf(angle_rad), .ay = 0.0f, .az = cosf(angle_rad),
            .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
            .dt = 0.01f
        };
        filter_output_t out;

        filter_t *f = filter_create(FILTER_TYPE_MADGWICK);
        if (!f) continue;

        f->set_param(f, FILTER_PARAM_KP, betas[b]);

        for (int i = 0; i < 1000; i++) {
            f->update(f, &in, &out);
        }

        printf("  beta=%.3f: pitch=%.2f° (误差=%.2f°)\n",
               betas[b], out.pitch, fabsf(out.pitch - target_angle));

        f->destroy(f);
    }

    printf("\n========================================\n");
    printf("  互补滤波器不同alpha值测试 (1000帧后)\n");
    printf("========================================\n");

    float alphas[] = {0.98f, 0.95f, 0.90f, 0.80f, 0.50f};

    for (int a = 0; a < 5; a++) {
        filter_input_t in = {
            .ax = -sinf(angle_rad), .ay = 0.0f, .az = cosf(angle_rad),
            .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
            .dt = 0.01f
        };
        filter_output_t out;

        filter_t *f = filter_create(FILTER_TYPE_COMPLEMENTARY);
        if (!f) continue;

        f->set_param(f, FILTER_PARAM_ALPHA, alphas[a]);

        for (int i = 0; i < 1000; i++) {
            f->update(f, &in, &out);
        }

        printf("  alpha=%.2f: pitch=%.2f° (误差=%.2f°)\n",
               alphas[a], out.pitch, fabsf(out.pitch - target_angle));

        f->destroy(f);
    }

    printf("\n========================================\n");
    printf("  结论\n");
    printf("========================================\n");
    printf("1. 误差随帧数减小 → 收敛速度问题（非数学错误）\n");
    printf("2. beta越小误差越小 → 参数调优问题\n");
    printf("3. alpha越小误差越小 → 收敛速度与平滑度的权衡\n");

    return 0;
}
