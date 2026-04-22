#ifndef ASYNC_SEMAPHORE_HPP
#define ASYNC_SEMAPHORE_HPP

#include "bsp_utility.hpp"
#include <chrono>
#include <cmsis_os2.h>
#include <cstddef>
#include <memory>
#include <utility>

namespace gdut {

struct empty_semaphore_t {
  explicit empty_semaphore_t() = default;
};
inline constexpr empty_semaphore_t empty_semaphore{};

/**
 * @brief 基于 CMSIS-RTOS2 的计数信号量
 *
 * 该类提供了 C++ 风格的计数信号量包装。
 * 特性：
 * - 标准信号量操作（acquire, release, try_acquire）
 * - 支持 std::chrono 超时
 * - 支持移动语义
 *
 * 线程安全：所有方法都是线程安全的。
 *
 * 重要：信号量创建可能失败，原因如下：
 * - 系统资源（FreeRTOS 堆）耗尽
 * - osKernelInitialize() 尚未被调用
 * 若信号量无效，acquire() 和 release() 会返回 osError，
 * 不会调用 std::terminate()。
 *
 * @tparam LeastMaxValue 信号量能覾的最大值
 */
template <std::size_t LeastMaxValue> class counting_semaphore {
public:
  static constexpr std::size_t max() noexcept { return LeastMaxValue; }

  explicit counting_semaphore(std::size_t desired) {
    m_semaphore_id = osSemaphoreNew(LeastMaxValue, desired, nullptr);
  }

  explicit counting_semaphore(empty_semaphore_t) {}

  /**
   * @brief 从已存在的 CMSIS-RTOS2 信号量句柄构造。
   *
   * 允许传递 nullptr，结果是一个无效的信号量对象。
   * 此时 valid() 和 operator bool() 返回 false，成员函数
   * 如 acquire() 和 release() 会返回 osError，不会调用
   * 下层 CMSIS-RTOS2 API。
   */
  template <typename T, typename = std::enable_if_t<
                            std::same_as<std::decay_t<T>, osSemaphoreId_t>>>
  explicit counting_semaphore(T semaphore_id) noexcept
      : m_semaphore_id(semaphore_id) {}

  ~counting_semaphore() noexcept {
    if (m_semaphore_id != nullptr) {
      osSemaphoreDelete(m_semaphore_id);
    }
  }

  counting_semaphore(const counting_semaphore &) = delete;
  counting_semaphore &operator=(const counting_semaphore &) = delete;

  counting_semaphore(counting_semaphore &&other) noexcept
      : m_semaphore_id(std::exchange(other.m_semaphore_id, nullptr)) {}
  counting_semaphore &operator=(counting_semaphore &&other) noexcept {
    if (this != std::addressof(other)) {
      if (m_semaphore_id != nullptr) {
        osSemaphoreDelete(m_semaphore_id);
      }
      m_semaphore_id = std::exchange(other.m_semaphore_id, nullptr);
    }
    return *this;
  }

  void release() {
    if (m_semaphore_id == nullptr) {
      std::terminate();
    }
    osSemaphoreRelease(m_semaphore_id);
  }

  void acquire(
      std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
    if (m_semaphore_id == nullptr) {
      std::terminate();
    }
    osSemaphoreAcquire(m_semaphore_id, time_to_ticks(timeout));
  }

  bool try_acquire() noexcept {
    if (m_semaphore_id == nullptr) {
      std::terminate();
    }
    return osSemaphoreAcquire(m_semaphore_id, 0) == osOK;
  }

  template <class Rep, class Period>
  bool try_acquire_for(const std::chrono::duration<Rep, Period> &rel_time) {
    if (m_semaphore_id == nullptr) {
      std::terminate();
    }
    return osSemaphoreAcquire(m_semaphore_id, time_to_ticks(rel_time)) == osOK;
  }

  bool valid() const noexcept { return m_semaphore_id != nullptr; }
  explicit operator bool() const noexcept { return valid(); }

private:
  osSemaphoreId_t m_semaphore_id{nullptr};
};

using binary_semaphore = counting_semaphore<1>;

} // namespace gdut

#endif // ASYNC_SEMAPHORE_HPP
