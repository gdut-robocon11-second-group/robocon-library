#include "bsp_gpio_pin.hpp"
#include "bsp_timer.hpp"
#include "bsp_utility.hpp"
#include "bsp_dma.hpp"
#include "clock.hpp"
#include "memory_resource.hpp"
#include "mutex.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "uncopyable.hpp"
#include <iostream>

// 声明 DMA 测试函数
extern int run_dma_tests();

/**
 * @brief 通用测试框架
 * 
 * 运行所有单元测试，包括：
 * - DMA 功能测试
 * - 其他模块测试（可扩展）
 */
class test_runner {
public:
    static int run_all_tests() {
        std::cout << "\n" << std::string(50, '=') << std::endl;
        std::cout << "     RoboCon R1 Unit Test Suite" << std::endl;
        std::cout << std::string(50, '=') << std::endl;
        
        int failed = 0;
        
        // 运行 DMA 测试
        std::cout << "\n[1/1] Running DMA Tests..." << std::endl;
        if (run_dma_tests() != 0) {
            std::cout << "✗ DMA tests FAILED" << std::endl;
            failed++;
        } else {
            std::cout << "✓ DMA tests PASSED" << std::endl;
        }
        
        // 打印总结
        std::cout << "\n" << std::string(50, '=') << std::endl;
        if (failed == 0) {
            std::cout << "      All tests PASSED ✓" << std::endl;
        } else {
            std::cout << "      " << failed << " test suite(s) FAILED ✗" << std::endl;
        }
        std::cout << std::string(50, '=') << std::endl;
        
        return failed;
    }
};

int main() {
    return test_runner::run_all_tests();
}
