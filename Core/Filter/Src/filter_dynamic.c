/**
 * @file    filter_dynamic.c
 * @brief   动态分配包装器 — 仅 PC 测试用（FILTER_ALLOW_DYNAMIC 时编译）
 *
 * 设计：
 *   - MCU 永不启用（FILTER_ALLOW_DYNAMIC 默认未定义）
 *   - PC 测试可通过 -DFILTER_ALLOW_DYNAMIC 编译本文件，
 *     方便用 filter_create(type) 替代 filter_create_static + 手动分 buffer
 *   - 内部调用 fp_malloc/fp_free，在 FILTER_STATIC_ONLY 下它们返回 NULL/空
 *
 * 编译：
 *   gcc -DFILTER_ALLOW_DYNAMIC -DFILTER_STATIC_ONLY ...
 *   等价于不编译本文件（fp_malloc 返回 NULL，filter_create 失败）
 *
 *   gcc -DFILTER_ALLOW_DYNAMIC -DFILTER_STATIC_ONLY=0 ...
 *   启用 malloc，支持 filter_create/filter_destroy
 */

#include "filter.h"
#include "filter_platform.h"
#include <stdlib.h>
#include <string.h>

#if defined(FILTER_ALLOW_DYNAMIC)
filter_t* filter_create(filter_type_t type)
{
    /* 动态分配：先查大小，再 malloc + 调用 filter_create_static */
    size_t required = filter_get_static_size(type);
    if (required == 0) return NULL;

    void *buf = fp_malloc(required);
    if (!buf) return NULL;

    filter_t *f = filter_create_static(type, buf, required);
    if (!f) {
        fp_free(buf);
        return NULL;
    }
    return f;
}

void filter_destroy(filter_t *f)
{
    if (!f) return;
    /* 注意：filter_t 实例 + priv 在同一连续内存块中，
     * filter_create_static 时从 buf 头部放置 filter_t，尾部放 priv。
     * 释放整块即可。 */
    fp_free(f);
}
#endif /* FILTER_ALLOW_DYNAMIC */
