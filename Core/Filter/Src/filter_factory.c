/**
 * @file    filter_factory.c
 * @brief   滤波器工厂 — 表驱动派发（静态分配，MCU 专用）
 *
 * 设计：
 *   - 表驱动：filter_table[] 索引各滤波器的 5 个接口函数
 *   - 静态分配：filter_create_static(type, buf, size)，无 malloc
 *   - 工厂逻辑冻结：分支不得修改此文件
 *
 * Phase 3 将在此加入 #ifndef FILTER_DISABLE_<TYPE> 守卫，
 * 并通过 FILTER_WEAK 宏为表项指向 weak 符号，允许分支覆盖。
 *
 * 从原 filter.c:1100-1206 拆出，改为表驱动。零行为变化。
 */

#include "filter.h"
#include "filter_internal.h"
#include <stdint.h>
#include <string.h>

/* ============================================================
 * 滤波器描述符表（编译期常量）
 * ============================================================
 *
 * 每个条目指向对应滤波器的 5 个接口函数。
 * 滤波器私有结构体大小通过 <type>_get_static_size() 间接查询，
 * 工厂无需知道 <type>_priv_t 的定义。
 */
typedef struct {
    filter_update_fn     update;      /**< 更新函数 */
    filter_reset_fn      reset;       /**< 重置函数 */
    filter_set_param_fn  set_param;   /**< 参数设置函数 */
    size_t             (*get_size)(void);  /**< 私有数据大小查询 */
    void               (*init)(void *priv); /**< 私有数据初始化 */
} filter_descriptor_t;

static const filter_descriptor_t filter_table[FILTER_TYPE_COUNT] = {
#ifndef FILTER_DISABLE_COMPLEMENTARY
    [FILTER_TYPE_COMPLEMENTARY] = {
        complementary_update, complementary_reset, complementary_set_param,
        complementary_get_static_size, complementary_init
    },
#else
    [FILTER_TYPE_COMPLEMENTARY] = { NULL, NULL, NULL, NULL, NULL },
#endif
#ifndef FILTER_DISABLE_LPF
    [FILTER_TYPE_LPF] = {
        lpf_update, lpf_reset, lpf_set_param,
        lpf_get_static_size, lpf_init
    },
#else
    [FILTER_TYPE_LPF] = { NULL, NULL, NULL, NULL, NULL },
#endif
#ifndef FILTER_DISABLE_EKF
    [FILTER_TYPE_EKF] = {
        ekf_update, ekf_reset, ekf_set_param,
        ekf_get_static_size, ekf_init
    },
#else
    [FILTER_TYPE_EKF] = { NULL, NULL, NULL, NULL, NULL },
#endif
#ifndef FILTER_DISABLE_LKF
    [FILTER_TYPE_LKF] = {
        lkf_update, lkf_reset, lkf_set_param,
        lkf_get_static_size, lkf_init
    },
#else
    [FILTER_TYPE_LKF] = { NULL, NULL, NULL, NULL, NULL },
#endif
#ifndef FILTER_DISABLE_MAHONY
    [FILTER_TYPE_MAHONY] = {
        mahony_update, mahony_reset, mahony_set_param,
        mahony_get_static_size, mahony_init
    },
#else
    [FILTER_TYPE_MAHONY] = { NULL, NULL, NULL, NULL, NULL },
#endif
#ifndef FILTER_DISABLE_MADGWICK
    [FILTER_TYPE_MADGWICK] = {
        madgwick_update, madgwick_reset, madgwick_set_param,
        madgwick_get_static_size, madgwick_init
    },
#else
    [FILTER_TYPE_MADGWICK] = { NULL, NULL, NULL, NULL, NULL },
#endif
};

/* ============================================================
 * 工厂 API
 * ============================================================ */

size_t filter_get_static_size(filter_type_t type) {
    if (type < 0 || type >= FILTER_TYPE_COUNT) return 0;
    if (filter_table[type].get_size == NULL) return 0;  /* 禁用类型（Phase 3） */
    return sizeof(filter_t) + filter_table[type].get_size();
}

filter_t* filter_create_static(filter_type_t type, void *buf, size_t buf_size) {
    if (!buf || type < 0 || type >= FILTER_TYPE_COUNT) return NULL;

    const filter_descriptor_t *desc = &filter_table[type];
    if (desc->update == NULL || desc->init == NULL || desc->get_size == NULL) {
        return NULL;  /* 禁用类型（Phase 3） */
    }

    size_t priv_size = desc->get_size();
    size_t required = sizeof(filter_t) + priv_size;
    if (buf_size < required) return NULL;

    /* 对齐检查 */
    if ((uintptr_t)buf % sizeof(void*) != 0) return NULL;

    filter_t *f = (filter_t *)buf;
    void *priv = (uint8_t*)buf + sizeof(filter_t);

    /* 初始化私有数据（先清零再调用 init 设置默认值） */
    memset(priv, 0, priv_size);
    desc->init(priv);

    /* 设置接口函数指针 */
    f->update     = desc->update;
    f->reset      = desc->reset;
    f->set_param  = desc->set_param;
    f->type       = type;
    f->degrade    = FILTER_DEGRADE_NONE;
    f->priv       = priv;
    f->safety_config = (filter_safety_config_t)FILTER_SAFETY_DEFAULT;

    return f;
}
