/**
 * @file dma_integration_example.cpp
 * @brief DMA 集成测试示例 - 实际应用场景
 * 
 * 此文件展示如何在真实硬件环境中使用 DMA 代理进行数据传输测试。
 * 包含 UART 接收、SPI 数据传输等常见应用场景。
 * 
 * 注意：此文件仅作为示例，需要根据实际硬件配置进行调整。
 */

#include "bsp_dma.hpp"
#include "stm32f4xx_hal.h"
#include <cstring>
#include <cstdint>

namespace gdut::example {

/**
 * @brief 演示 DMA UART 接收的集成测试
 * 
 * 场景：使用 DMA 从 UART 接收数据到缓冲区
 */
class uart_dma_receiver {
private:
    static constexpr size_t BUFFER_SIZE = 256;
    uint8_t rx_buffer[BUFFER_SIZE] = {};
    
    // 假设由 CubeMX 生成
    extern DMA_HandleTypeDef hdma_usart1_rx;
    dma_proxy dma_rx{nullptr};
    
    bool transfer_complete = false;
    bool transfer_error = false;

public:
    uart_dma_receiver() : dma_rx(&hdma_usart1_rx) {
        // 设置 DMA 回调处理器
        dma_rx.set_callback_handler([this](std::error_code ec) {
            this->handle_dma_complete(ec);
        });
    }

    /**
     * @brief 初始化 DMA 并启动 UART 接收
     */
    void start_reception() {
        if (!dma_rx.valid()) {
            return;
        }

        // 配置 DMA 参数（通常由 CubeMX 配置，这里仅演示）
        dma_rx.set_direction(dma_direction::periph_to_memory);
        dma_rx.set_mem_inc(true);      // 内存地址自增
        dma_rx.set_periph_inc(false);  // 外设地址不增
        dma_rx.set_mem_data_alignment(dma_memory_data_alignment::byte);
        dma_rx.set_periph_data_alignment(dma_peripheral_data_alignment::byte);
        dma_rx.set_mode(dma_mode::normal);
        dma_rx.set_priority(dma_priority::low);

        // 初始化 DMA
        dma_rx.init();

        // 启动 DMA 传输：从 UART 数据寄存器读取到接收缓冲区
        // 源地址：UART1 数据寄存器地址
        // 目标地址：接收缓冲区
        // 数据长度：BUFFER_SIZE 字节
        dma_rx.start(
            reinterpret_cast<void *>(0x40011004),  // USART1->DR
            static_cast<void *>(rx_buffer),
            BUFFER_SIZE
        );
    }

    /**
     * @brief 获取接收到的数据
     */
    const uint8_t *get_data() const {
        return rx_buffer;
    }

    /**
     * @brief 检查传输是否完成
     */
    bool is_complete() const {
        return transfer_complete;
    }

    /**
     * @brief 检查是否发生错误
     */
    bool has_error() const {
        return transfer_error;
    }

private:
    /**
     * @brief DMA 传输完成/错误回调
     */
    void handle_dma_complete(std::error_code ec) {
        if (!ec) {
            // 传输成功完成
            transfer_complete = true;
            // 可以在这里处理接收到的数据
            process_received_data();
        } else {
            // 传输出错
            transfer_error = true;
            // 处理错误
            handle_error(ec);
        }
    }

    void process_received_data() {
        // 数据处理逻辑
        // 例如：解析协议、验证校验和等
    }

    void handle_error(std::error_code ec) {
        // 错误处理逻辑
        // ec.message() 提供人类可读的错误描述
    }
};

/**
 * @brief 演示 DMA 内存到内存传输的单元测试
 * 
 * 场景：使用 DMA 进行高速内存数据复制
 */
class memory_dma_transfer_test {
public:
    static void run() {
        const size_t TRANSFER_SIZE = 1024;
        
        // 源数据缓冲区
        uint8_t source[TRANSFER_SIZE];
        // 目标缓冲区
        uint8_t destination[TRANSFER_SIZE] = {};

        // 初始化源数据
        for (size_t i = 0; i < TRANSFER_SIZE; ++i) {
            source[i] = static_cast<uint8_t>(i & 0xFF);
        }

        // 创建 DMA 代理（实际使用时需要真实的 DMA 句柄）
        DMA_HandleTypeDef dma_handle = {};
        dma_proxy mem_dma(&dma_handle);

        bool transfer_done = false;
        
        // 设置完成回调
        mem_dma.set_callback_handler(
            [&transfer_done](std::error_code ec) {
                if (!ec) {
                    transfer_done = true;
                    // 验证数据
                }
            }
        );

        // 配置为内存到内存模式
        mem_dma.set_direction(dma_direction::memory_to_memory);
        mem_dma.set_mem_inc(true);
        mem_dma.set_periph_inc(true);  // 在内存到内存模式中，这个字段被用作"流 1 增量"
        mem_dma.set_mem_data_alignment(dma_memory_data_alignment::word);
        mem_dma.set_periph_data_alignment(dma_peripheral_data_alignment::word);

        // 启动传输
        mem_dma.init();
        mem_dma.start(source, destination, TRANSFER_SIZE / 4);

        // 等待传输完成（在实际代码中，这通常通过等待回调完成）
        // while (!transfer_done) { /* 等待 */ }

        // 验证数据
        verify_data(source, destination, TRANSFER_SIZE);
    }

private:
    static void verify_data(
        const uint8_t *source,
        const uint8_t *destination,
        size_t size
    ) {
        if (std::memcmp(source, destination, size) == 0) {
            // 数据匹配，传输成功
        } else {
            // 数据不匹配，传输失败
        }
    }
};

/**
 * @brief DMA 错误恢复测试
 * 
 * 演示如何处理 DMA 传输错误和进行恢复
 */
class dma_error_recovery_test {
public:
    static void run() {
        // 创建无效句柄来演示错误处理
        dma_proxy invalid_dma(nullptr);
        
        std::error_code last_error;
        bool error_handled = false;

        // 设置错误处理回调
        invalid_dma.set_callback_handler(
            [&last_error, &error_handled](std::error_code ec) {
                if (ec) {
                    last_error = ec;
                    error_handled = true;
                    
                    // 根据错误类型进行恢复处理
                    switch (static_cast<dma_error_code>(ec.value())) {
                        case dma_error_code::timeout_error:
                            // 超时：重新启动传输
                            break;
                        case dma_error_code::transfer_error:
                            // 传输错误：检查地址和大小
                            break;
                        case dma_error_code::parameter_error:
                            // 参数错误：验证配置
                            break;
                        default:
                            // 其他错误
                            break;
                    }
                }
            }
        );

        // 尝试启动传输，预期会失败
        invalid_dma.start(nullptr, nullptr, 0);

        // 验证错误已被捕获
        if (error_handled) {
            // 错误已正确处理
        }
    }
};

/**
 * @brief DMA 资源清理测试 (RAII)
 * 
 * 演示 RAII 模式确保资源正确清理
 */
class dma_raii_cleanup_test {
public:
    static void test_scope_cleanup() {
        {
            DMA_HandleTypeDef handle = {};
            dma_proxy scoped_dma(&handle);
            
            // 配置 DMA
            scoped_dma.set_channel(dma_channel::dma_request_0);
            scoped_dma.init();
            
            // 作用域结束时自动调用析构函数
            // deinit() 会被自动调用，清理所有资源和回调指针
        }
        // 在这里，DMA 句柄已经被完全清理
    }

    static void test_move_cleanup() {
        DMA_HandleTypeDef handle1 = {};
        DMA_HandleTypeDef handle2 = {};
        
        dma_proxy dma(&handle1);
        dma.init();
        
        // 移动到另一个对象
        dma_proxy moved_dma(std::move(dma));
        
        // 原对象现在无效，不会重复清理
        // moved_dma 拥有 handle1 的生命周期管理
    }
};

}  // namespace gdut::example

/**
 * @brief 运行所有集成测试示例
 */
int run_dma_integration_tests() {
    // 注意：这些是示例代码，不会在没有真实硬件的情况下完整执行
    
    // gdut::example::memory_dma_transfer_test::run();
    // gdut::example::dma_error_recovery_test::run();
    // gdut::example::dma_raii_cleanup_test::test_scope_cleanup();
    // gdut::example::dma_raii_cleanup_test::test_move_cleanup();
    
    return 0;
}
