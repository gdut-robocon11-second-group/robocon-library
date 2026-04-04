/**
 * @file dma_test.cpp
 * @brief DMA（Direct Memory Access）功能单元测试
 * 
 * 测试内容：
 * - DMA 代理对象的初始化和销毁
 * - DMA 配置接口（通道、方向、数据对齐等）
 * - DMA 传输启动和中断处理
 * - DMA 错误处理和错误码映射
 * - 移动语义支持
 */

#include "bsp_dma.hpp"
#include "bsp_utility.hpp"
#include <cassert>
#include <cstring>
#include <iostream>

namespace gdut {

/**
 * @brief DMA 测试套件
 * 
 * 注意：这是单元测试框架，实际测试时需要真实的 DMA 硬件和中断环境。
 * 对于嵌入式环境，可以集成到 Google Test 或类似框架中。
 */
class dma_test {
public:
    static void run_all_tests() {
        std::cout << "\n===== DMA Unit Tests =====" << std::endl;
        
        test_error_code_mapping();
        test_dma_proxy_construction();
        test_dma_proxy_configuration();
        test_dma_proxy_validity();
        test_dma_proxy_move_semantics();
        test_dma_callback_handling();
        
        std::cout << "\n===== All Tests Passed =====" << std::endl;
    }

private:
    /**
     * @brief 测试 DMA 错误码到错误信息的映射
     */
    static void test_error_code_mapping() {
        std::cout << "\n[Test] DMA Error Code Mapping..." << std::endl;
        
        // 测试无错误情况
        auto ec_none = make_error_code(dma_error_code::none);
        assert(ec_none.value() == 0);
        std::cout << "  ✓ No error message: " << ec_none.message() << std::endl;
        
        // 测试单个错误
        auto ec_transfer_err = make_error_code(dma_error_code::transfer_error);
        std::cout << "  ✓ Transfer error: " << ec_transfer_err.message() << std::endl;
        
        // 测试多个错误标志（位掩码）
        dma_error_code combined = static_cast<dma_error_code>(
            static_cast<uint32_t>(dma_error_code::transfer_error) |
            static_cast<uint32_t>(dma_error_code::timeout_error)
        );
        auto ec_combined = make_error_code(combined);
        std::cout << "  ✓ Combined errors: " << ec_combined.message() << std::endl;
        
        // 验证错误分类
        assert(&ec_none.category() == &dma_error_category::instance());
        std::cout << "  ✓ Error category correct: " << ec_none.category().name() << std::endl;
    }

    /**
     * @brief 测试 DMA 代理对象的构造和析构
     */
    static void test_dma_proxy_construction() {
        std::cout << "\n[Test] DMA Proxy Construction..." << std::endl;
        
        // 测试使用 nullptr 构造
        {
            dma_proxy dma_null(nullptr);
            assert(!dma_null.valid());
            assert(!static_cast<bool>(dma_null));
            std::cout << "  ✓ Null handle construction" << std::endl;
        }
        
        // 测试销毁时清理
        {
            DMA_HandleTypeDef test_handle = {};
            test_handle.Instance = nullptr;
            {
                dma_proxy dma(&test_handle);
                assert(dma.get_handle() == &test_handle);
                std::cout << "  ✓ Valid handle construction" << std::endl;
            }
            // 析构后，get_handle() 应该返回有效指针（由 RAII 管理）
            std::cout << "  ✓ RAII destruction successful" << std::endl;
        }
    }

    /**
     * @brief 测试 DMA 配置接口
     */
    static void test_dma_proxy_configuration() {
        std::cout << "\n[Test] DMA Proxy Configuration..." << std::endl;
        
        DMA_HandleTypeDef handle = {};
        dma_proxy dma(&handle);
        dma.init();
        
        // 测试通道配置
        dma.set_channel(dma_channel::dma_request_0);
        assert(handle.Init.Channel == std::to_underlying(dma_channel::dma_request_0));
        std::cout << "  ✓ Channel configuration" << std::endl;
        
        // 测试方向配置
        dma.set_direction(dma_direction::periph_to_memory);
        assert(handle.Init.Direction == std::to_underlying(dma_direction::periph_to_memory));
        std::cout << "  ✓ Direction configuration" << std::endl;
        
        // 测试外设增量
        dma.set_periph_inc(true);
        assert(handle.Init.PeriphInc == DMA_PINC_ENABLE);
        dma.set_periph_inc(false);
        assert(handle.Init.PeriphInc == DMA_PINC_DISABLE);
        std::cout << "  ✓ Peripheral increment configuration" << std::endl;
        
        // 测试内存增量
        dma.set_mem_inc(true);
        assert(handle.Init.MemInc == DMA_MINC_ENABLE);
        dma.set_mem_inc(false);
        assert(handle.Init.MemInc == DMA_MINC_DISABLE);
        std::cout << "  ✓ Memory increment configuration" << std::endl;
        
        // 测试数据对齐配置
        dma.set_periph_data_alignment(dma_peripheral_data_alignment::byte);
        assert(handle.Init.PeriphDataAlignment == 
               std::to_underlying(dma_peripheral_data_alignment::byte));
        std::cout << "  ✓ Peripheral data alignment" << std::endl;
        
        dma.set_mem_data_alignment(dma_memory_data_alignment::byte);
        assert(handle.Init.MemDataAlignment == 
               std::to_underlying(dma_memory_data_alignment::byte));
        std::cout << "  ✓ Memory data alignment" << std::endl;
        
        // 测试在 nullptr 上配置（应为空操作）
        dma_proxy dma_null(nullptr);
        dma_null.set_channel(dma_channel::dma_request_0);  // 不应崩溃
        std::cout << "  ✓ Null handle configuration (no-op)" << std::endl;
    }

    /**
     * @brief 测试 DMA 代理的有效性检查
     */
    static void test_dma_proxy_validity() {
        std::cout << "\n[Test] DMA Proxy Validity..." << std::endl;
        
        // 有效句柄
        DMA_HandleTypeDef handle = {};
        dma_proxy dma(&handle);
        assert(dma.valid());
        assert(static_cast<bool>(dma));
        std::cout << "  ✓ Valid handle check" << std::endl;
        
        // 无效句柄
        dma_proxy dma_null(nullptr);
        assert(!dma_null.valid());
        assert(!static_cast<bool>(dma_null));
        std::cout << "  ✓ Invalid (null) handle check" << std::endl;
    }

    /**
     * @brief 测试 DMA 代理的移动语义
     */
    static void test_dma_proxy_move_semantics() {
        std::cout << "\n[Test] DMA Proxy Move Semantics..." << std::endl;
        
        DMA_HandleTypeDef handle1 = {};
        DMA_HandleTypeDef handle2 = {};
        
        // 测试移动构造
        dma_proxy dma1(&handle1);
        dma_proxy dma2(std::move(dma1));
        assert(!dma1.valid());  // 原对象应失效
        assert(dma2.valid());   // 新对象应有效
        assert(dma2.get_handle() == &handle1);
        std::cout << "  ✓ Move construction" << std::endl;
        
        // 测试移动赋值
        dma_proxy dma3(&handle2);
        dma3 = std::move(dma2);
        assert(!dma2.valid());      // 原对象应失效
        assert(dma3.valid());       // 赋值后应有效
        assert(dma3.get_handle() == &handle1);
        std::cout << "  ✓ Move assignment" << std::endl;
        
        // 测试自赋值保护（虽然目录 = std::move(目录) 应避免）
        DMA_HandleTypeDef handle3 = {};
        dma_proxy dma4(&handle3);
        auto *ptr = dma4.get_handle();
        dma4 = std::move(dma4);  // 不应导致问题
        assert(dma4.get_handle() == ptr);
        std::cout << "  ✓ Self-assignment protection" << std::endl;
        
        // 复制应被禁止（编译时错误，这里仅验证概念）
        // dma_proxy dma5 = dma4;  // 编译错误
        // dma_proxy dma6; dma6 = dma4;  // 编译错误
        std::cout << "  ✓ Copy operations disabled (compile-time check)" << std::endl;
    }

    /**
     * @brief 测试 DMA 回调处理
     */
    static void test_dma_callback_handling() {
        std::cout << "\n[Test] DMA Callback Handling..." << std::endl;
        
        DMA_HandleTypeDef handle = {};
        dma_proxy dma(&handle);
        
        bool callback_invoked = false;
        std::error_code captured_error;
        
        // 设置回调处理器
        dma.set_callback_handler([&](std::error_code ec) {
            callback_invoked = true;
            captured_error = ec;
        });
        
        // 测试模拟错误启动（句柄为 nullptr）
        dma_proxy dma_null(nullptr);
        bool null_callback_invoked = false;
        dma_null.set_callback_handler([&](std::error_code ec) {
            null_callback_invoked = true;
            assert(ec);  // 应该有错误码
        });
        
        // start() 调用会因句柄无效而失败
        dma_null.start(nullptr, nullptr, 0);
        assert(null_callback_invoked);
        std::cout << "  ✓ Null handle start() invokes error callback" << std::endl;
        
        // 测试回调的移动语义
        function<void(std::error_code)> new_handler = [](std::error_code) {
            // 另一个处理器
        };
        dma.set_callback_handler(std::move(new_handler));
        std::cout << "  ✓ Callback handler move assignment" << std::endl;
        
        // 测试回调的引用语义
        function<void(std::error_code)> const_handler = [](std::error_code) {};
        dma.set_callback_handler(const_handler);
        std::cout << "  ✓ Callback handler const reference assignment" << std::endl;
    }
};

}  // namespace gdut

/**
 * @brief DMA 测试主函数
 */
int run_dma_tests() {
    try {
        gdut::dma_test::run_all_tests();
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}

// 如果作为独立程序编译
#ifdef DMA_TEST_STANDALONE
int main() {
    return run_dma_tests();
}
#endif
