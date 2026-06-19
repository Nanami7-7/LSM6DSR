/**
 * @file    platform_mspm0.c
 * @brief   MSPM0G3507 平台接口实现
 *
 * 实现 platform.h 定义的平台抽象接口：
 *   - delay_ms / delay_us — 延时函数
 *   - get_tick_us — 微秒级时间戳（TimerG0）
 *   - debug_printf — UART 调试输出
 *
 * 系统配置：
 *   - 系统时钟：80 MHz
 *   - 计时器：TimerG0，配置为 1MHz（1us 分辨率）
 *   - 调试串口：UART0，115200 baud
 */

#include "platform.h"
#include "ti_msp_dl_config.h"
#include <stdarg.h>
#include <stdio.h>

/* ============================================================
 * 延时函数
 * ============================================================ */

/**
 * @brief 毫秒延时（忙等待）
 * @param ms 延时毫秒数
 *
 * 使用 DL_Common_delayCycles 实现，精度取决于系统时钟。
 * 80MHz 时：1ms = 80,000 个时钟周期
 */
static void mspm0_delay_ms(uint32_t ms)
{
    /* 分段延时避免溢出：80000 * ms 在 ms > 53687 时 uint32_t 溢出
     * 每段最大 50000ms（4G cycles），安全余量充足 */
    while (ms > 50000) {
        DL_Common_delayCycles(80000UL * 50000UL);
        ms -= 50000;
    }
    DL_Common_delayCycles(80000UL * ms);
}

/**
 * @brief 微秒延时（忙等待）
 * @param us 延时微秒数
 *
 * 80MHz 时：1us = 80 个时钟周期
 * 注意：函数调用开销约 1-2us，实际延时略长
 */
static void mspm0_delay_us(uint32_t us)
{
    DL_Common_delayCycles(80 * us);
}

/* ============================================================
 * 计时函数
 * ============================================================ */

/**
 * @brief 获取微秒级时间戳
 * @return 自 TimerG0 启动以来的微秒数
 *
 * TimerG0 配置为 1MHz（1us 分辨率），32 位计数器。
 * 溢出周期：2^32 us ≈ 71.6 分钟
 *
 * 使用方式：
 * @code
 *   uint32_t t0 = g_platform->get_tick_us();
 *   // ... 执行操作 ...
 *   uint32_t t1 = g_platform->get_tick_us();
 *   uint32_t elapsed_us = t1 - t0;  // 正确处理溢出
 * @endcode
 */
static uint32_t mspm0_get_tick_us(void)
{
    return DL_Timer_getTimerCount(TIMG0);
}

/* ============================================================
 * 调试输出
 * ============================================================ */

/**
 * @brief 调试输出函数（UART0）
 * @param fmt 格式字符串（printf 兼容）
 * @param ... 可变参数
 * @return 输出字符数
 *
 * 使用阻塞方式发送每个字符，确保输出完整性。
 * 波特率 115200 时，发送 1 字节约 87us。
 */
static int mspm0_debug_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0) return 0;
    if (len > (int)sizeof(buf) - 1) len = (int)sizeof(buf) - 1;

    for (int i = 0; i < len; i++) {
        DL_UART_main_transmitDataBlocking(UART0, (uint8_t)buf[i]);
    }

    return len;
}

/* ============================================================
 * 平台实例
 * ============================================================ */

/**
 * @brief MSPM0G3507 平台实例
 *
 * 存储在 Flash 中（const 修饰），包含所有平台函数指针。
 */
const platform_t mspm0_platform = {
    .delay_ms       = mspm0_delay_ms,
    .delay_us       = mspm0_delay_us,
    .get_tick_us    = mspm0_get_tick_us,
    .system_clock_hz = 80000000,  /* 80 MHz */
    .debug_printf   = mspm0_debug_printf,
};

/**
 * @brief 全局平台指针
 *
 * 指向 MSPM0 平台实例。
 * 在 main() 中设置：g_platform = &mspm0_platform;
 *
 * @note 使用非 const 指针，允许运行时切换（如测试时使用 mock）
 */
const platform_t *g_platform = &mspm0_platform;

/* ============================================================
 * 平台初始化辅助函数
 * ============================================================ */

/**
 * @brief 初始化 TimerG0 为微秒计时器
 *
 * 在 SYSCFG_DL_init() 之后调用此函数启动计时器。
 * TimerG0 已在 ti_msp_dl_config.c 中配置为 1MHz 周期计时器。
 */
void platform_timer_init(void)
{
    /* TimerG0 已由 SYSCFG_DL_init() 配置 */
    /* 此函数仅启动计时器 */
    DL_Timer_startTimer(TIMG0);
}
