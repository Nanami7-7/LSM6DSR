/**
 * @file    platform.h
 * @brief   平台抽象接口 — 延时、计时、调试输出
 *
 * 设计目的：
 *   - 将 BSP 层的平台依赖抽象为接口
 *   - 各平台分支实现此接口，BSP 层通过 g_platform 调用
 *   - 新平台移植只需实现 platform_t 结构体的 4 个函数指针
 *
 * 使用方式：
 *   1. 在平台分支中定义 const platform_t 实例
 *   2. 在平台初始化代码中设置 g_platform 指针
 *   3. BSP 层通过 g_platform->xxx() 调用平台功能
 *
 * 示例（MSPM0G3507 实现）：
 * @code
 *   // ports/mspm0g3507/platform_mspm0.c
 *   #include "platform.h"
 *   #include "ti_msp_dl_config.h"
 *
 *   static void mspm0_delay_ms(uint32_t ms) {
 *       DL_Common_delayCycles(80000 * ms);
 *   }
 *
 *   static uint32_t mspm0_get_tick_us(void) {
 *       return DL_Timer_getTimerCount(TIMG0);
 *   }
 *
 *   const platform_t mspm0_platform = {
 *       .delay_ms       = mspm0_delay_ms,
 *       .get_tick_us    = mspm0_get_tick_us,
 *       .system_clock_hz = 80000000,
 *       .debug_printf   = mspm0_printf,
 *   };
 *
 *   const platform_t *g_platform = &mspm0_platform;
 * @endcode
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 平台抽象接口结构体
 *
 * 各平台分支实现此结构体的所有函数指针。
 * 使用 const 修饰，实例应存储在 Flash 中。
 */
typedef struct {
    /**
     * @brief 毫秒延时
     * @param ms 延时毫秒数
     */
    void (*delay_ms)(uint32_t ms);

    /**
     * @brief 微秒延时（可选，可为 NULL）
     * @param us 延时微秒数
     * @note  某些平台不支持微秒级延时，可设为 NULL
     */
    void (*delay_us)(uint32_t us);

    /**
     * @brief 获取微秒级时间戳
     * @return 自某个起点以来的微秒数
     *
     * 要求：
     *   - 分辨率至少 10us
     *   - 溢出周期至少 1 秒（推荐 32 位计数器）
     *   - 调用开销 < 1us
     *
     * BSP 层使用此函数计算 dt：
     * @code
     *   uint32_t now = g_platform->get_tick_us();
     *   float dt = (now - last_tick) * 1e-6f;
     *   last_tick = now;
     * @endcode
     */
    uint32_t (*get_tick_us)(void);

    /**
     * @brief 系统时钟频率（Hz）
     *
     * 用于某些需要时钟信息的计算。
     * MSPM0G3507: 80000000 (80MHz)
     * STM32F407:  168000000 (168MHz)
     */
    uint32_t system_clock_hz;

    /**
     * @brief 调试输出函数（printf 兼容）
     * @param fmt 格式字符串
     * @param ... 可变参数
     * @return 输出字符数
     *
     * 实现要求：
     *   - 支持 printf 格式化语法
     *   - 线程安全（如需多线程环境）
     *   - 可选：支持 DMA 发送以减少 CPU 占用
     */
    int (*debug_printf)(const char *fmt, ...);
} platform_t;

/**
 * @brief 全局平台实例指针
 *
 * 各平台分支在初始化代码中设置此指针。
 * BSP 层通过此指针调用平台功能。
 *
 * @note 使用 const 修饰，防止意外修改指针指向。
 *       但指针本身是非 const 的，允许在运行时切换平台实例（如测试）。
 */
extern const platform_t *g_platform;

/**
 * @brief 默认延时实现（忙等待，精度低）
 *
 * 当平台未实现 delay_us 时，可使用此函数作为后备。
 * 精度取决于编译器优化级别和系统时钟。
 *
 * @param us 延时微秒数
 */
void platform_default_delay_us(uint32_t us);

/**
 * @brief 平台计时器初始化
 *
 * 在 SYSCFG_DL_init() 之后调用，启动微秒级计时器。
 * 各平台分支实现此函数。
 */
void platform_timer_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_H */
