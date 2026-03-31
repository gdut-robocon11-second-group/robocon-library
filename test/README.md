# RoboCon R1 单元测试指南

## 概述

该项目包含全面的单元测试套件，用于验证各个模块的功能正确性。当前测试覆盖以下模块：

- **DMA（直接存储器访问）**：包括配置、传输和错误处理

## 项目结构

```
Middlewares/robocon-library/test/
├── test.cpp           # 测试主入口和测试框架
├── dma_test.cpp       # DMA 功能单元测试
└── README.md          # 此文件
```

## DMA 测试内容

`dma_test.cpp` 包含以下测试用例：

### 1. 错误码映射测试 (`test_error_code_mapping`)
- 验证无错误情况 (`dma_error_code::none`)
- 验证单个错误类型的消息映射
- 验证多个错误标志（位掩码）的组合处理
- 验证错误分类正确性

### 2. 构造和析构测试 (`test_dma_proxy_construction`)
- 测试使用 `nullptr` 构造
- 测试使用有效句柄构造
- 验证 RAII 自动资源管理

### 3. DMA 配置测试 (`test_dma_proxy_configuration`)
测试所有配置接口：
- 通道配置 (`set_channel`)
- 传输方向配置 (`set_direction`)
- 外设地址增量 (`set_periph_inc`)
- 内存地址增量 (`set_mem_inc`)
- 外设数据对齐 (`set_periph_data_alignment`)
- 内存数据对齐 (`set_mem_data_alignment`)
- 在无效句柄上进行空操作（不崩溃）

### 4. 有效性检查测试 (`test_dma_proxy_validity`)
- 验证有效句柄的 `valid()` 和 `operator bool()`
- 验证 `nullptr` 句柄的无效状态

### 5. 移动语义测试 (`test_dma_proxy_move_semantics`)
- 测试移动构造函数
- 测试移动赋值操作符
- 验证自赋值保护
- 验证复制操作已禁用（编译时检查）

### 6. 回调处理测试 (`test_dma_callback_handling`)
- 设置回调处理器
- 测试错误回调调用
- 验证回调的移动语义和引用语义

## 编译和运行

### 在 CMake 中编译

在项目的 `CMakeLists.txt` 中，确保包含测试目标：

```cmake
add_executable(robocon_tests
    Middlewares/robocon-library/test/test.cpp
    Middlewares/robocon-library/test/dma_test.cpp
    # 其他源文件...
)

target_link_libraries(robocon_tests
    robocon-library
    # 其他库...
)

# 注册为 CTest 测试
add_test(NAME unit_tests COMMAND robocon_tests)
```

### 独立测试编译

如果要单独编译 DMA 测试，可以定义 `DMA_TEST_STANDALONE` 宏：

```bash
g++ -std=c++17 -DDMA_TEST_STANDALONE \
    -I./Middlewares/robocon-library/BSP \
    -I./Middlewares/robocon-library/test \
    ./Middlewares/robocon-library/test/dma_test.cpp \
    -o dma_test
./dma_test
```

## 测试输出示例

```
==================================================
     RoboCon R1 Unit Test Suite
==================================================

[1/1] Running DMA Tests...

===== DMA Unit Tests =====

[Test] DMA Error Code Mapping...
  ✓ No error message: No error
  ✓ Transfer error: Transfer error; 
  ✓ Combined errors: Transfer error; Timeout error; 
  ✓ Error category correct: dma_error_code

[Test] DMA Proxy Construction...
  ✓ Null handle construction
  ✓ Valid handle construction
  ✓ RAII destruction successful

[Test] DMA Proxy Configuration...
  ✓ Channel configuration
  ✓ Direction configuration
  ✓ Peripheral increment configuration
  ✓ Memory increment configuration
  ✓ Peripheral data alignment
  ✓ Memory data alignment
  ✓ Null handle configuration (no-op)

[Test] DMA Proxy Validity...
  ✓ Valid handle check
  ✓ Invalid (null) handle check

[Test] DMA Proxy Move Semantics...
  ✓ Move construction
  ✓ Move assignment
  ✓ Self-assignment protection
  ✓ Copy operations disabled (compile-time check)

[Test] DMA Callback Handling...
  ✓ Null handle start() invokes error callback
  ✓ Callback handler move assignment
  ✓ Callback handler const reference assignment

===== All Tests Passed =====

✓ DMA tests PASSED

==================================================
      All tests PASSED ✓
==================================================
```

## 扩展测试

要添加新的测试模块，请按照以下步骤：

1. 创建新的测试文件，例如 `gpio_test.cpp`
2. 实现测试类和运行方法：
   ```cpp
   namespace gdut {
   class gpio_test {
   public:
       static void run_all_tests() { /*...*/ }
   };
   }
   extern int run_gpio_tests() { gdut::gpio_test::run_all_tests(); return 0; }
   ```
3. 在 `test.cpp` 中声明和调用新的测试函数
4. 在 CMakeLists.txt 中添加新的源文件

## 常见问题

### Q: 测试需要实际的硬件吗？
A: 当前的 DMA 测试是单元测试，不需要实际硬件。它们测试配置和错误处理逻辑。对于集成测试（涉及实际 DMA 传输），需要完整的硬件环境和中断处理。

### Q: 如何在 FreeRTOS 任务中运行测试？
A: 在 `main()` 函数中创建一个任务来运行测试：
```cpp
void test_task(void *arg) {
    test_runner::run_all_tests();
    osThreadExit();
}

osThreadNew(test_task, NULL, NULL);
```

### Q: 支持哪些 C++ 标准？
A: 测试代码使用 C++17 特性，如 `std::error_code`、`std::to_underlying()` 等。

## 注意事项

1. **线程安全**：DMA 代理本身不提供内部同步。回调在中断上下文执行；并发访问需要外部保护。

2. **内存限制**：DMA 传输数据不能放在 CCMRAM（CCM RAM 不能被 DMA 访问）。

3. **硬件特定**：测试针对 STM32F407 微控制器。其他平台需要调整 HAL 类型和宏。

## 许可证

遵循项目主许可证。
