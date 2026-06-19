/**
 * @file    filter_chain.c
 * @brief   滤波器链 — 多级滤波器串联实现
 */

#include "../Inc/filter_chain.h"
#include "../Inc/filter_config.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ============================================================
 * 辅助函数：将姿态角转换为虚拟 sensor 输入
 * ============================================================ */

/**
 * @brief 将 pitch/roll/yaw 转换为虚拟加速度和角速度
 *
 * 用于级间数据传递：上一级输出的姿态角转换为下一级的输入。
 *
 * @param pitch  俯仰角（度）
 * @param roll   横滚角（度）
 * @param yaw    偏航角（度，未使用）
 * @param out    输出的虚拟 sensor 数据
 */
static void angles_to_sensor_input(float pitch, float roll, float yaw,
                                   filter_input_t *out)
{
    float pitch_rad = pitch * M_PI / 180.0f;
    float roll_rad  = roll  * M_PI / 180.0f;

    /* 虚拟加速度：重力方向 */
    out->ax = -sinf(pitch_rad);
    out->ay = sinf(roll_rad) * cosf(pitch_rad);
    out->az = cosf(roll_rad) * cosf(pitch_rad);

    /* 虚拟角速度：设为 0（姿态角无法还原角速度） */
    out->gx = 0.0f;
    out->gy = 0.0f;
    out->gz = 0.0f;

    /* dt 保持不变（由调用者设置） */
    (void)yaw;  /* yaw 暂未使用 */
}

/* ============================================================
 * 初始化与销毁
 * ============================================================ */

int filter_chain_init(filter_chain_t *chain, filter_type_t types[], uint8_t count)
{
    if (!chain || !types || count == 0 || count > FILTER_CHAIN_MAX_STAGES) {
        return -1;
    }

    memset(chain, 0, sizeof(filter_chain_t));
    chain->num_stages = count;
    chain->is_static = 0;

    for (uint8_t i = 0; i < count; i++) {
        chain->stages[i] = filter_create(types[i]);
        if (!chain->stages[i]) {
            /* 创建失败，销毁已创建的 */
            for (uint8_t j = 0; j < i; j++) {
                chain->stages[j]->destroy(chain->stages[j]);
            }
            memset(chain, 0, sizeof(filter_chain_t));
            return -1;
        }
    }

    return 0;
}

int filter_chain_init_static(filter_chain_t *chain, filter_type_t types[], uint8_t count,
                             void *bufs[], size_t buf_sizes[])
{
    if (!chain || !types || !bufs || !buf_sizes ||
        count == 0 || count > FILTER_CHAIN_MAX_STAGES) {
        return -1;
    }

    memset(chain, 0, sizeof(filter_chain_t));
    chain->num_stages = count;
    chain->is_static = 1;

    for (uint8_t i = 0; i < count; i++) {
        chain->stages[i] = filter_create_static(types[i], bufs[i], buf_sizes[i]);
        if (!chain->stages[i]) {
            /* 静态分配不需要销毁已创建的 */
            memset(chain, 0, sizeof(filter_chain_t));
            return -1;
        }
    }

    return 0;
}

void filter_chain_destroy(filter_chain_t *chain)
{
    if (!chain || chain->is_static) {
        return;
    }

    for (uint8_t i = 0; i < chain->num_stages; i++) {
        if (chain->stages[i] && chain->stages[i]->destroy) {
            chain->stages[i]->destroy(chain->stages[i]);
            chain->stages[i] = NULL;
        }
    }

    chain->num_stages = 0;
}

/* ============================================================
 * 更新与重置
 * ============================================================ */

void filter_chain_update(filter_chain_t *chain, const filter_input_t *in, filter_output_t *out)
{
    if (!chain || !in || !out || chain->num_stages == 0) {
        return;
    }

    /* 第一级：直接使用原始输入 */
    filter_input_t current_in = *in;
    filter_output_t current_out;

    for (uint8_t i = 0; i < chain->num_stages; i++) {
        if (!chain->stages[i]) {
            return;
        }

        /* 执行当前级更新 */
        chain->stages[i]->update(chain->stages[i], &current_in, &current_out);

        /* 保存中间结果 */
        chain->intermediate[i] = current_out;

        /* 如果不是最后一级，转换输出为下一级输入 */
        if (i < chain->num_stages - 1) {
            current_in.dt = in->dt;  /* dt 保持不变 */
            angles_to_sensor_input(current_out.pitch, current_out.roll,
                                   current_out.yaw, &current_in);
        }
    }

    /* 返回最后一级的输出 */
    *out = current_out;
}

void filter_chain_reset(filter_chain_t *chain)
{
    if (!chain) {
        return;
    }

    for (uint8_t i = 0; i < chain->num_stages; i++) {
        if (chain->stages[i]) {
            chain->stages[i]->reset(chain->stages[i]);
        }
    }

    memset(chain->intermediate, 0, sizeof(chain->intermediate));
}

/* ============================================================
 * 参数设置
 * ============================================================ */

int filter_chain_set_param(filter_chain_t *chain, uint8_t stage,
                           filter_param_t param, float value)
{
    if (!chain || stage >= chain->num_stages || !chain->stages[stage]) {
        return -1;
    }

    chain->stages[stage]->set_param(chain->stages[stage], param, value);
    return 0;
}

void filter_chain_set_degrade(filter_chain_t *chain, uint8_t stage,
                              filter_degrade_t degrade)
{
    if (!chain || stage >= chain->num_stages || !chain->stages[stage]) {
        return;
    }

    chain->stages[stage]->degrade = degrade;
}

void filter_chain_apply_preset(filter_chain_t *chain, uint8_t stage,
                               filter_preset_t preset)
{
    if (!chain || stage >= chain->num_stages || !chain->stages[stage]) {
        return;
    }

    filter_config_apply_preset(chain->stages[stage], preset);
}

/* ============================================================
 * 查询
 * ============================================================ */

uint8_t filter_chain_get_stages(const filter_chain_t *chain)
{
    return chain ? chain->num_stages : 0;
}

filter_type_t filter_chain_get_type(const filter_chain_t *chain, uint8_t stage)
{
    if (!chain || stage >= chain->num_stages || !chain->stages[stage]) {
        return FILTER_TYPE_COUNT;
    }

    return chain->stages[stage]->type;
}

const filter_output_t* filter_chain_get_intermediate(const filter_chain_t *chain, uint8_t stage)
{
    if (!chain || stage >= chain->num_stages) {
        return NULL;
    }

    return &chain->intermediate[stage];
}
