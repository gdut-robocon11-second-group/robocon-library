#ifndef COMPONENTS_CLOCK_HPP
#define COMPONENTS_CLOCK_HPP

#include <chrono>
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
    const uint32_t ticks = basic_kernel_clock::get_tick_count();
    const uint32_t freq = basic_kernel_clock::get_tick_freq();
    const uint64_t ms = (static_cast<uint64_t>(ticks) * duration::period::den) /
                        (freq == 0U ? 1U : freq);
    return time_point(duration(static_cast<rep>(ms)));
  }
};

struct steady_clock {
  using duration = std::chrono::microseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<steady_clock>;

  static constexpr bool is_steady = true;

  static time_point now() noexcept {
    // 注意：此实现尽量保证单调，但仍依赖 CMSIS-RTOS2 底层端口对
    // osKernelGetSysTimerCount()/osKernelGetTickCount() 的语义一致性。
    // 在不同内核移植层上，不保证“绝对完全正确”。
    //
    // 最简单的 now()（仅示例，不推荐直接用于 delay_until）:
    // static time_point now() noexcept {
    //   const uint32_t freq = basic_kernel_clock::get_sys_timer_freq();
    //   const uint32_t cnt = basic_kernel_clock::get_sys_timer_count();
    //   const uint64_t us = (static_cast<uint64_t>(cnt) * duration::period::den) /
    //                       (freq == 0U ? 1U : freq);
    //   return time_point(duration(static_cast<rep>(us)));
    // }

    const uint32_t tick_freq = basic_kernel_clock::get_tick_freq();
    const uint32_t sys_freq = basic_kernel_clock::get_sys_timer_freq();
    if (tick_freq == 0U || sys_freq == 0U) {
      return time_point(duration(0));
    }

    // 采样一致性：避免在 tick 边界把 tick 与子计数拼接错位
    uint32_t tick0 = 0U;
    uint32_t tick1 = 0U;
    uint32_t sys = 0U;
    do {
      tick0 = basic_kernel_clock::get_tick_count();
      sys = basic_kernel_clock::get_sys_timer_count();
      tick1 = basic_kernel_clock::get_tick_count();
    } while (tick0 != tick1);

    const uint32_t counts_per_tick = (sys_freq + tick_freq / 2U) / tick_freq;
    const uint32_t safe_counts_per_tick = counts_per_tick == 0U ? 1U : counts_per_tick;

    // 低位子计数（假设 osKernelGetSysTimerCount 与系统定时器同频）
    const uint32_t sub_count = sys % safe_counts_per_tick;
    const uint64_t total_counts =
        static_cast<uint64_t>(tick1) * safe_counts_per_tick + sub_count;
    const uint64_t us = (total_counts * duration::period::den) / sys_freq;
    return time_point(duration(static_cast<rep>(us)));
  }
};

using high_resolution_clock = steady_clock;

static_assert(std::chrono::is_clock_v<system_clock>);
static_assert(std::chrono::is_clock_v<steady_clock>);

} // namespace gdut

#endif // COMPONENTS_CLOCK_HPP
