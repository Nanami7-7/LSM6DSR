/**
 * @file    log.h
 * @brief   轻量日志系统 — 编译时级别控制，零开销禁用
 *
 * 设计目的：
 *   - 替换散落的 printf 调用，统一日志格式
 *   - 编译时通过宏控制日志级别，生产固件可关闭所有日志
 *   - 禁用级别的日志调用展开为空，零运行时开销
 *
 * 日志级别：
 *   LOG_LEVEL_NONE  (0) — 关闭所有日志
 *   LOG_LEVEL_ERROR (1) — 仅错误
 *   LOG_LEVEL_WARN  (2) — 警告 + 错误
 *   LOG_LEVEL_INFO  (3) — 信息 + 警告 + 错误（默认）
 *   LOG_LEVEL_DEBUG (4) — 所有日志
 *
 * 使用方式：
 *   1. 编译时指定级别：-DLOG_LEVEL=LOG_LEVEL_DEBUG
 *   2. 代码中使用：LOG_INFO("Sensor ready, WHO_AM_I=0x%02X", id)
 *   3. 生产固件：-DLOG_LEVEL=LOG_LEVEL_NONE（所有日志展开为空）
 *
 * 输出格式：
 *   [ERR] 消息内容\r\n
 *   [WRN] 消息内容\r\n
 *   [INF] 消息内容\r\n
 *   [DBG] 消息内容\r\n
 *
 * 依赖：
 *   - platform.h 中的 g_platform->debug_printf
 */

#ifndef LOG_H
#define LOG_H

#include "platform.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 日志级别定义
 * ============================================================ */
#define LOG_LEVEL_NONE    0  /**< 关闭所有日志 */
#define LOG_LEVEL_ERROR   1  /**< 仅错误 */
#define LOG_LEVEL_WARN    2  /**< 警告 + 错误 */
#define LOG_LEVEL_INFO    3  /**< 信息 + 警告 + 错误（默认） */
#define LOG_LEVEL_DEBUG   4  /**< 所有日志 */

/* ============================================================
 * 编译时日志级别（可通过 -DLOG_LEVEL=N 覆盖）
 * ============================================================ */
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

/* ============================================================
 * 内部辅助宏 — 安全调用 debug_printf
 * ============================================================ */

/**
 * @brief 安全调用 debug_printf
 *
 * 检查 g_platform 和 debug_printf 是否有效。
 * 如果 g_platform 未初始化，尝试使用 printf 作为后备。
 */
#define _LOG_PRINTF(fmt, ...) \
    do { \
        if (g_platform && g_platform->debug_printf) { \
            g_platform->debug_printf(fmt, ##__VA_ARGS__); \
        } \
    } while (0)

/* ============================================================
 * 日志宏 — 按级别输出
 * ============================================================ */

/**
 * @brief 错误日志（级别 1）
 * @note  生产固件建议保留此级别
 */
#if LOG_LEVEL >= LOG_LEVEL_ERROR
    #define LOG_ERR(fmt, ...)  _LOG_PRINTF("[ERR] " fmt "\r\n", ##__VA_ARGS__)
#else
    #define LOG_ERR(fmt, ...)  ((void)0)
#endif

/**
 * @brief 警告日志（级别 2）
 */
#if LOG_LEVEL >= LOG_LEVEL_WARN
    #define LOG_WARN(fmt, ...) _LOG_PRINTF("[WRN] " fmt "\r\n", ##__VA_ARGS__)
#else
    #define LOG_WARN(fmt, ...) ((void)0)
#endif

/**
 * @brief 信息日志（级别 3，默认启用）
 */
#if LOG_LEVEL >= LOG_LEVEL_INFO
    #define LOG_INFO(fmt, ...) _LOG_PRINTF("[INF] " fmt "\r\n", ##__VA_ARGS__)
#else
    #define LOG_INFO(fmt, ...) ((void)0)
#endif

/**
 * @brief 调试日志（级别 4）
 * @note  仅开发阶段启用
 */
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
    #define LOG_DBG(fmt, ...)  _LOG_PRINTF("[DBG] " fmt "\r\n", ##__VA_ARGS__)
#else
    #define LOG_DBG(fmt, ...)  ((void)0)
#endif

/* ============================================================
 * 特殊用途宏
 * ============================================================ */

/**
 * @brief 原始输出（无前缀，无换行）
 *
 * 用于 VOFA+ 等数据输出，不添加任何日志格式。
 * 此宏不受 LOG_LEVEL 控制，始终可用。
 *
 * @code
 *   LOG_RAW("%.3f,%.3f,%.3f\r\n", data.ax, data.ay, data.az);
 * @endcode
 */
#define LOG_RAW(fmt, ...) _LOG_PRINTF(fmt, ##__VA_ARGS__)

/**
 * @brief 带缩进的输出（用于初始化序列）
 *
 * 输出前缀为 2 个空格，与原有 printf 格式兼容。
 * 此宏不受 LOG_LEVEL 控制，始终可用。
 */
#define LOG_INDENT(fmt, ...) _LOG_PRINTF("  " fmt "\r\n", ##__VA_ARGS__)

/* ============================================================
 * 断言宏（仅 DEBUG 级别启用）
 * ============================================================ */

/**
 * @brief 调试断言
 *
 * 仅在 LOG_LEVEL >= LOG_LEVEL_DEBUG 时生效。
 * 断言失败时输出错误信息并进入死循环。
 *
 * @code
 *   LOG_ASSERT(ptr != NULL, "Null pointer in %s", __func__);
 * @endcode
 */
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
    #define LOG_ASSERT(cond, fmt, ...) \
        do { \
            if (!(cond)) { \
                LOG_ERR("ASSERT FAILED: %s:%d: " fmt, \
                        __FILE__, __LINE__, ##__VA_ARGS__); \
                while (1) { } \
            } \
        } while (0)
#else
    #define LOG_ASSERT(cond, fmt, ...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* LOG_H */
