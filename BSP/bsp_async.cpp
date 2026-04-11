
#include "bsp_async_test.hpp"
#include "bsp_async.hpp"         
#include "FreeRTOS.h"
#include "task.h"
#include <cstdio>              
#include <cstring>
// 辅助宏：简单的断言打印
#define TEST_ASSERT(cond, msg)

    do { \
        if (!(cond)) { \
            printf("[TEST FAIL] %s\n", msg); \
        } else { \
            printf("[TEST PASS] %s\n", msg); \
        } \
    } while(0)

namespace bsp_test 
{
// 测试事件组
void test_event_group() {
    printf("\n=== Testing Event Group ===\n");
    bsp_async::EventGroup evt;
    const uint32_t BIT0 = (1 << 0);
    const uint32_t BIT1 = (1 << 1);

    // 设置事件位
    evt.setBits(BIT0);
    // 等待事件位（阻塞）
    uint32_t result = evt.waitBits(BIT0 | BIT1, pdTRUE, pdTRUE, pdMS_TO_TICKS(100));
    TEST_ASSERT((result & BIT0) != 0, "Event group wait got BIT0");
    TEST_ASSERT((result & BIT1) == 0, "Event group wait did not get BIT1");
}

// 测试二值信号量
void test_semaphore() {
    printf("\n=== Testing Binary Semaphore ===\n");
    bsp_async::Semaphore sem;  // 默认二值信号量

    // 初始状态应为不可用
    TEST_ASSERT(!sem.take(pdMS_TO_TICKS(10)), "Semaphore take timeout");

    sem.give();
    TEST_ASSERT(sem.take(pdMS_TO_TICKS(10)), "Semaphore take success after give");
}

// 测试互斥锁
void test_mutex() {
    printf("\n=== Testing Mutex ===\n");
    bsp_async::Mutex mutex;

    // 获取锁
    TEST_ASSERT(mutex.take(pdMS_TO_TICKS(100)), "Mutex take success");

    // 递归尝试（如果支持递归锁，应能成功；否则会失败）
    bool recursive = mutex.take(pdMS_TO_TICKS(10));
    if (recursive) {
        mutex.give(); // 释放第二次获取
        printf("[INFO] Mutex supports recursion\n");
    } else {
        printf("[INFO] Mutex does not support recursion\n");
    }

    mutex.give(); // 释放第一次获取
    TEST_ASSERT(mutex.take(pdMS_TO_TICKS(100)), "Mutex take after release");
}

// 测试消息队列
void test_queue() {
    printf("\n=== Testing Queue ===\n");
    bsp_async::Queue<int32_t> queue(2); // 队列深度 2

    int32_t data = 123;
    queue.send(data, pdMS_TO_TICKS(100));
    int32_t received;
    queue.receive(received, pdMS_TO_TICKS(100));
    TEST_ASSERT(received == 123, "Queue send/receive works");
}

// 测试线程（任务）
static void test_thread_task(void* param) {
    int* pValue = static_cast<int*>(param);
    *pValue = 42;
    vTaskDelete(NULL);
}

void test_thread() {
    printf("\n=== Testing Thread ===\n");
    int result = 0;
    bsp_async::Thread thread(test_thread_task, &result, "TestTask", 128, 1);
    thread.start();
    vTaskDelay(pdMS_TO_TICKS(100)); // 等待任务执行
    TEST_ASSERT(result == 42, "Thread executed and set result");
}

// 测试定时器
static void test_timer_callback(void* arg) {
    bool* flag = static_cast<bool*>(arg);
    *flag = true;
}

void test_timer() {
    printf("\n=== Testing Timer ===\n");
    bool triggered = false;
    bsp_async::Timer timer(test_timer_callback, &triggered, pdMS_TO_TICKS(100), false); // 单次定时器
    timer.start();
    vTaskDelay(pdMS_TO_TICKS(200));
    TEST_ASSERT(triggered, "Timer triggered");
}

// 运行所有测试
void run_bsp_async_tests() {
    printf("\n========== BSP Async Tests Start ==========\n");

    test_event_group();
    test_semaphore();
    test_mutex();
    test_queue();
    test_thread();
    test_timer();

    printf("\n========== BSP Async Tests Finished ==========\n");
}

} // namespace bsp_test