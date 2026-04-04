#ifndef COMPONENTS_CLOCK_HPP
#define COMPONENTS_CLOCK_HPP

#include "stm32f4xx.h"
#include <chrono>
#include <cmath>
#include <cmsis_os2.h>
#include <cstdint>

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

  static constexpr bool is_steady = false;

  static time_point now() noexcept {
    const std::uint32_t tick = basic_kernel_clock::get_tick_count();
    const std::uint32_t freq = basic_kernel_clock::get_tick_freq();
    if (freq == 0U) {
      return time_point{};
    }
    const std::uint64_t ms64 = (static_cast<std::uint64_t>(tick) * 1000ULL) /
                               static_cast<std::uint64_t>(freq);
    return time_point(duration(static_cast<rep>(ms64)));
  }
};

struct steady_clock {
  using duration = std::chrono::nanoseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<steady_clock>;

  static constexpr bool is_steady = true;

  static time_point now() noexcept {
    std::uint32_t irqmask = __get_PRIMASK();
    __disable_irq(); // 关中断，保证 tick 和 SysTick 读取原子性

    // 获取 RTOS 滴答计数（32 位，单调递增，约 50 天回绕）
    std::uint32_t tick = basic_kernel_clock::get_tick_count();
    // 读取 SysTick 当前值（递减），转换为已过周期数
    std::uint32_t load = SysTick->LOAD;
    std::uint32_t elapsed = load - SysTick->VAL; // 当前滴答内已过的周期数

    // 检查 SysTick 是否在读取期间发生溢出（即刚进入滴答中断）
    if ((SysTick->CTRL >> 16) & 1U) { // 溢出标志位
      elapsed = load - SysTick->VAL;  // 重新读取，确保正确
      tick++;                         // 补偿这一个滴答
    }

    // 每滴答的周期数 = LOAD + 1
    const std::uint32_t interval = load + 1U;

    // 计算总周期数（64 位）
    std::uint64_t total = static_cast<std::uint64_t>(tick) *
                              static_cast<std::uint64_t>(interval) +
                          static_cast<std::uint64_t>(elapsed);

    // 获取系统定时器频率并防止除零
    const std::uint32_t freq = basic_kernel_clock::get_sys_timer_freq();
    if (freq == 0U) {
      if (irqmask == 0U) {
        __enable_irq();
      }
      return time_point{};
    }

    // 将周期数转换为秒，注意频率可能不整除 1 秒
    double ticks_per_second = period::den / static_cast<double>(freq);

    if (irqmask == 0U) {
      __enable_irq();
    }

    return time_point(
        duration(static_cast<rep>(std::floor(total * ticks_per_second))));
  }
};

using high_resolution_clock = steady_clock;

static_assert(std::chrono::is_clock_v<system_clock>);
static_assert(std::chrono::is_clock_v<steady_clock>);

} // namespace gdut

#endif // COMPONENTS_CLOCK_HPP
