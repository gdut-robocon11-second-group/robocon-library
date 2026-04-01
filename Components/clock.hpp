#ifndef COMPONENTS_CLOCK_HPP
#define COMPONENTS_CLOCK_HPP

#include <chrono>
#include <cmsis_os2.h>
#include <cstdint>
#include "stm32f4xx.h"

namespace gdut {

struct basic_kernel_clock {
  basic_kernel_clock() = delete;

  static std::uint32_t get_tick_count() noexcept {
    return osKernelGetTickCount();
  }

  static std::uint32_t get_tick_freq() noexcept {
    return osKernelGetTickFreq();
  }

  static std::uint32_t get_sys_timer_count() noexcept {
    return osKernelGetSysTimerCount();
  }

  static std::uint32_t get_sys_timer_freq() noexcept {
    return osKernelGetSysTimerFreq();
  }
};

struct system_clock {
  using duration = std::chrono::milliseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<system_clock>;

  static constexpr bool is_steady = true;

  static time_point now() noexcept {
    const uint32_t ticks = basic_kernel_clock::get_tick_count();
    const uint32_t freq = basic_kernel_clock::get_tick_freq();
    const uint64_t ms = (static_cast<uint64_t>(ticks) * duration::period::den) /
                        (freq == 0U ? 1U : freq);
    return time_point(duration(static_cast<rep>(ms)));
  }
};

struct steady_clock {
  // 需要与 system_stm32f4xx.c 中的 SystemCoreClock 定义保持一致
  static constexpr uint32_t system_core_clock = 168000000;
  static constexpr uint32_t system_tick_frequence = 1000;

  using rep = int64_t;
    // 分辨率 = 1 / system_core_clock 秒
  using period = std::ratio<1, system_core_clock>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<steady_clock>;

  static constexpr bool is_steady = true;

  static time_point now() noexcept {
    uint32_t irqmask = __get_PRIMASK();
    __disable_irq();                     // 关中断，保证 tick 和 SysTick 读取原子性

    // 获取 RTOS 滴答计数（32 位，单调递增，约 50 天回绕）
    uint32_t tick = osKernelGetTickCount();

    // 读取 SysTick 当前值（递减），转换为已过周期数
    uint32_t load = SysTick->LOAD;
    uint32_t elapsed = load - SysTick->VAL; // 当前滴答内已过的周期数

    // 检查 SysTick 是否在读取期间发生溢出（即刚进入滴答中断）
    if ((SysTick->CTRL >> 16) & 1U) {       // 溢出标志位
      elapsed = load - SysTick->VAL;        // 重新读取，确保正确
      tick++;                               // 补偿这一个滴答
    }

    // 每滴答的周期数 = LOAD + 1
    const uint32_t interval = load + 1U;

    // 计算总周期数（64 位）
    uint64_t total = static_cast<uint64_t>(tick) * static_cast<uint64_t>(interval) + static_cast<uint64_t>(elapsed);

    if (irqmask == 0U) {
      __enable_irq();
    }

    return time_point(duration(static_cast<int64_t>(total)));
  }
};

using high_resolution_clock = steady_clock;

static_assert(std::chrono::is_clock_v<system_clock>);
static_assert(std::chrono::is_clock_v<steady_clock>);

} // namespace gdut

#endif // COMPONENTS_CLOCK_HPP
