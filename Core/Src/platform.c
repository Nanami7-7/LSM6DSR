/**
 * @file    platform.c
 * @brief   平台抽象层默认实现
 *
 * 提供 platform.h 中声明的默认函数实现。
 * 各平台分支可覆盖这些弱符号实现。
 */

#include "platform.h"

/* 编译器兼容的 weak 属性 */
#if defined(__GNUC__) || defined(__clang__)
    #define WEAK __attribute__((weak))
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
    #define WEAK __weak
#else
    #define WEAK
#endif

/**
 * @brief 默认微秒延时（忙等待）
 *
 * 此实现非常不精确，仅作为后备。
 * 各平台应提供自己的精确实现。
 *
 * 假设 ~10 个时钟周期每循环迭代，80MHz 时约 8 次/us。
 * 实际精度取决于编译器优化级别和系统时钟。
 */
WEAK
void platform_default_delay_us(uint32_t us)
{
    /* 粗略估计：每次迭代 ~10 个时钟周期
     * 80MHz 时：80MHz / 10 = 8M 次/秒 = 8 次/us
     * 这个值需要根据实际平台调整 */
    volatile uint32_t count = us * 8;
    while (count--) {
        /* 忙等待 */
    }
}
