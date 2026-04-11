
#ifndef BSP_ASYNC_TEST_HPP
#define BSP_ASYNC_TEST_HPP

#include <cstdint>

namespace bsp_test {

// 测试入口函数，执行所有异步组件测试
void run_bsp_async_tests();

// 单个组件测试（可选，便于单独调试）
void test_event_group();
void test_semaphore();
void test_mutex();
void test_queue();
void test_thread();
void test_timer();

} // namespace bsp_test

#endif // BSP_ASYNC_TEST_HPP