#ifndef ASYNC_THREAD_HPP
#define ASYNC_THREAD_HPP

#include "bsp_utility.hpp"
#include "memory_resource.hpp"
#include "mutex.hpp"
#include <cmsis_os2.h>
#include <cstddef>
#include <memory>
#include <memory_resource>
#include <mutex>
#include <type_traits>
#include <utility>
#include <string_view>

namespace gdut {

// 内部内存资源，用于线程函数对象的分配
struct thread_memory_resource {
  // 内存池总大小，单位字节。每个 thread<StackSize> 实例大约需要
  // sizeof(StaticTask_t) + StackSize + sizeof(函数对象) 字节，再加上 TLSF
  // 分配器自身开销（约 256 字节），可根据最大并发线程数和最大栈大小调整此值。
  static constexpr size_t pool_size = 4096;
  GDUT_CCMRAM inline static gdut::pmr::fixed_block_resource<pool_size>
      pool_resource{};
  inline static gdut::mutex pool_mutex{gdut::empty_mutex};

  static void init() {
    static std::once_flag once;
    std::call_once(once, []() { pool_mutex = gdut::mutex{}; });
  }
};

struct empty_thread_t {
  explicit empty_thread_t() = default;
};
inline constexpr empty_thread_t empty_thread{};

/**
 * @brief CMSIS-RTOS2 线程的 RAII 包装
 *
 * 该类提供了类似 std::thread 的 C++ 风格的线程包装。
 * 特性：
 * - 自动资源清理（RAII）
 * - 基于信号量的同步语义
 * - 支持移动语义
 *
 * 线程安全：
 * - join() 可以从任何线程调用，但仅调用一次
 * - terminate() 可以从任何线程调用，但不应在另一个线程
 *   正在 join() 中等待时调用
 *
 * 使用示例：
 *   gdut::thread<512> t([]{ do_work(); });
 *   t.join();  // 等待线程完成
 *
 * @tparam StackSize 线程栈的大小（字节）
 * @tparam Priority 线程优先级（默认：osPriorityNormal）
 */
template <size_t StackSize, osPriority_t Priority = osPriorityNormal>
class thread {
public:
  static constexpr size_t stack_size = StackSize;
  static constexpr osPriority_t priority = Priority;

  explicit thread(empty_thread_t) {}
  thread() : thread(empty_thread) {}

  thread(osThreadId_t handle, osSemaphoreId_t semaphore) {
    if (handle && semaphore) {
      m_handle =
          std::unique_ptr<std::remove_pointer_t<osThreadId_t>, thread_deleter>{
              handle};
      m_semaphore = std::unique_ptr<std::remove_pointer_t<osSemaphoreId_t>,
                                    semaphore_deleter>{semaphore};
    }
  }

  template <typename Func, typename... Args>
  thread(std::string_view thread_name, Func &&func, Args &&...args) {
    static_assert(
        std::is_invocable_v<Func, Args...>,
        "gdut::thread constructor requires a callable that can be invoked "
        "with the provided argument types");
    gdut::thread_memory_resource::init();
    // 使用 unique_ptr 管理信号量资源，确保异常安全
    m_semaphore =
        std::unique_ptr<std::remove_pointer_t<osSemaphoreId_t>,
                        semaphore_deleter>{osSemaphoreNew(1, 0, nullptr)};
    if (!m_semaphore) {
      return;
    }

    // 捕获信号量句柄的值（而非 this），使线程对象在运行中安全移动
    osSemaphoreId_t sem_handle = m_semaphore.get();
    auto bound = [sem_handle, func = std::forward<Func>(func),
                  ... args = std::forward<Args>(args)]() mutable {
      func(args...);
      osSemaphoreRelease(sem_handle);
    };
    using bound_type = decltype(bound);
    static std::pmr::polymorphic_allocator<bound_type> allocator{
        &thread_memory_resource::pool_resource};
    bound_type *data = nullptr;
    {
      // 线程安全地从内存池分配内存用于存储绑定的函数对象
      std::lock_guard lock(thread_memory_resource::pool_mutex);
      data = allocator.allocate(1);
    }
    // fixed_block_resource::do_allocate() 在内存不足时返回 nullptr
    if (!data) {
      m_semaphore.reset();
      return;
    }
    allocator.template construct<bound_type>(data, std::move(bound));
    osThreadAttr_t attributes = {
        .name = thread_name.data(), .stack_size = StackSize, .priority = Priority};
    m_handle =
        std::unique_ptr<std::remove_pointer_t<osThreadId_t>, thread_deleter>{
            osThreadNew(
                [](void *ptr) {
                  bound_type *data = static_cast<bound_type *>(ptr);
                  (*data)();
                  allocator.template destroy<bound_type>(data);
                  {
                    std::lock_guard lock(thread_memory_resource::pool_mutex);
                    allocator.deallocate(data, 1);
                  }
                  osThreadExit();
                },
                static_cast<void *>(data), &attributes)};
    if (!m_handle) {
      std::terminate();
      allocator.template destroy<bound_type>(data);
      {
        std::lock_guard lock(thread_memory_resource::pool_mutex);
        allocator.deallocate(data, 1);
      }
      m_semaphore.reset();
    }
  }

  ~thread() noexcept {
    terminate();
    m_semaphore.reset();
  }

  bool joinable() const noexcept {
    return m_handle != nullptr &&
           osThreadGetState(m_handle.get()) != osThreadTerminated;
  }

  void join() {
    if (m_handle == nullptr || m_semaphore == nullptr ||
        osThreadGetState(m_handle.get()) == osThreadTerminated) {
      return;
    }
    osSemaphoreAcquire(m_semaphore.get(), osWaitForever);
    // 线程已通过 osThreadExit() 正常退出，使用 release() 而非 reset()
    // 以避免 thread_deleter 再次调用 osThreadTerminate()
    m_handle.release();
    m_semaphore.reset();
  }

  void terminate() {
    if (m_handle) {
      m_handle.reset();
    }
    if (m_semaphore) {
      m_semaphore.reset();
    }
  }

  thread(const thread &) = delete;
  thread &operator=(const thread &) = delete;

  thread(thread &&other) noexcept
      : m_handle(std::move(other.m_handle)),
        m_semaphore(std::move(other.m_semaphore)) {}

  thread &operator=(thread &&other) noexcept {
    if (this != std::addressof(other)) {
      terminate();
      m_handle = std::move(other.m_handle);
      m_semaphore = std::move(other.m_semaphore);
    }
    return *this;
  }

  bool valid() const noexcept { return m_handle && m_semaphore; }

  explicit operator bool() const noexcept { return valid(); }

private:
  struct thread_deleter {
    void operator()(osThreadId_t thread) const {
      if (thread != nullptr) {
        osThreadTerminate(thread);
      }
    }
  };

  struct semaphore_deleter {
    void operator()(osSemaphoreId_t sem) const {
      if (sem != nullptr) {
        osSemaphoreDelete(sem);
      }
    }
  };

private:
  std::unique_ptr<std::remove_pointer_t<osThreadId_t>, thread_deleter> m_handle{
      nullptr};
  std::unique_ptr<std::remove_pointer_t<osSemaphoreId_t>, semaphore_deleter>
      m_semaphore{nullptr};
};

} // namespace gdut

#endif // ASYNC_THREAD_HPP
