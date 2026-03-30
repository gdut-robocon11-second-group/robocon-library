# BSP DMA 模块（bsp_dma.hpp）

## 原理

该模块对 STM32 HAL 的 DMA（Direct Memory Access，直接内存访问）功能进行 C++ 面向对象封装，提供类型安全、RAII 管理的 DMA 抽象层，支持 UART、I2C 和 SPI 外设的 DMA 数据传输。

## 核心设计

### 错误类型

- **`gdut::dma_error_category`**：`std::error_category` 子类，将 HAL DMA 错误码（`HAL_DMA_ERROR_*`）映射为可读字符串
- **`gdut::dma_error_code`**：类型安全枚举，对应 `HAL_DMA_ERROR_*` 系列宏
- 两者配合 `make_error_code()` 和 `std::is_error_code_enum` 特化，支持 `dma_error_code` 隐式转换为 `std::error_code`

### DMA 句柄代理 `dma_proxy`

- RAII 管理 `DMA_HandleTypeDef`：构造时接收句柄指针，析构时自动调用 `deinit()`
- 单独使用时，`start()` 在启动 DMA 前自动绑定 `Parent` 与 `XferCpltCallback`/`XferErrorCallback`；外设类（`dma_uart`/`dma_spi`/`dma_i2c`）在调用 `init()` 后自行配置回调
- 统一错误回调接口：传输完成/中止/出错均通过 `callback_t`（`function<void(std::error_code)>`）通知上层
- 所有 `set_*` 配置方法含 nullptr 有效性检查
- `start()` 方法为 `void`，启动是否成功及错误信息均通过回调通知上层，不返回 `HAL_StatusTypeDef`

### CRTP 基类 `dma_transfer_base<Derived>`

- 使用 CRTP（奇异递归模板模式）为外设提供统一的 DMA 操作接口，避免虚函数开销
- 提供 `bind_tx()`、`bind_rx()`、`bind()`、`transmit()`、`receive()` 接口
- 派生类须实现 `do_bind_tx`、`do_bind_rx`、`do_transmit`、`do_receive` 四个私有方法

### 外设特化类

| 类名 | 对应外设 | 说明 |
|------|----------|------|
| `dma_uart` | UART | address 参数忽略 |
| `dma_i2c`  | I2C（主机模式） | address 为 7 位从机地址（0x08~0x77） |
| `dma_spi`  | SPI | address 参数忽略；支持单向发送、全双工收发 |

## 如何使用

### 基础 DMA 代理（手动管理）

```cpp
#include "bsp_dma.hpp"

// CubeMX 生成的全局 DMA 句柄
extern DMA_HandleTypeDef hdma_usart1_rx;

// 创建 DMA 代理
gdut::dma_proxy dma_rx(&hdma_usart1_rx);

// 设置回调
dma_rx.set_callback_handler([](std::error_code ec) {
    if (!ec) {
        // 传输完成
        process_received_data();
    } else {
        // 传输出错，ec.message() 返回错误描述
        handle_error(ec);
    }
});

// 初始化 DMA 句柄
dma_rx.init();
```

### UART + DMA

```cpp
#include "bsp_dma.hpp"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

// 创建 DMA 代理
gdut::dma_proxy dma_rx(&hdma_usart1_rx);
gdut::dma_proxy dma_tx(&hdma_usart1_tx);

// 初始化 DMA 代理
dma_rx.init();
dma_tx.init();

// 创建 UART DMA 操作对象并绑定代理
gdut::dma_uart uart_dma(&huart1);
uart_dma.bind(&dma_tx, &dma_rx);

// 发送数据（HAL 不修改 data 内容，const_cast 仅用于适配 C 接口）
const uint8_t tx_buf[] = "Hello DMA\r\n";
uart_dma.transmit(tx_buf, sizeof(tx_buf) - 1);

// 接收数据
uint8_t rx_buf[64];
uart_dma.receive(rx_buf, sizeof(rx_buf));
```

### I2C + DMA

```cpp
#include "bsp_dma.hpp"

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;

gdut::dma_proxy dma_tx(&hdma_i2c1_tx);
gdut::dma_proxy dma_rx(&hdma_i2c1_rx);
dma_tx.init();
dma_rx.init();

gdut::dma_i2c i2c_dma(&hi2c1);
i2c_dma.bind(&dma_tx, &dma_rx);

// 向从机地址 0x50 发送数据（address 必须为有效 7 位地址，0 无效）
const uint8_t cmd[] = {0x01, 0x02};
i2c_dma.transmit(cmd, sizeof(cmd), 0x50);

// 从从机读取数据
uint8_t buf[4];
i2c_dma.receive(buf, sizeof(buf), 0x50);
```

### SPI + DMA

```cpp
#include "bsp_dma.hpp"

extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;

// 创建并初始化 DMA 代理
gdut::dma_proxy spi_tx_dma(&hdma_spi2_tx);
gdut::dma_proxy spi_rx_dma(&hdma_spi2_rx);

// 设置回调
spi_tx_dma.set_callback_handler([](std::error_code ec) {
    if (!ec) {
        // 发送完成
    }
});

spi_rx_dma.set_callback_handler([](std::error_code ec) {
    if (!ec) {
        // 接收完成
    }
});

spi_tx_dma.init();
spi_rx_dma.init();

// 创建 SPI DMA 对象并绑定
gdut::dma_spi spi_dma(&hspi2);
spi_dma.bind(&spi_tx_dma, &spi_rx_dma);

// 单向发送（address 参数忽略）
const uint8_t tx_data[] = {0xAA, 0xBB, 0xCC};
spi_dma.transmit(tx_data, sizeof(tx_data));

// 全双工传输（同时发送和接收，使用同一缓冲区）
uint8_t buffer[8] = {0x01, 0x02, /* ... */};
spi_dma.receive(buffer, sizeof(buffer));  // 发送 buffer 内容，接收后覆盖 buffer
```

### 错误处理

```cpp
// 使用 dma_error_code 枚举值直接构造 std::error_code（隐式转换）
std::error_code ec = gdut::dma_error_code::transfer_error;
if (ec) {
    // 输出错误描述
    // ec.message() => "Transfer error"
}
```

### 配置 DMA 参数

```cpp
gdut::dma_proxy dma(&hdma_usart1_rx);

// 配置各项参数（set_* 方法在句柄为 nullptr 时为空操作）
dma.set_channel(gdut::dma_channel::channel_4);
dma.set_direction(gdut::dma_direction::peripheral_to_memory);
dma.set_periph_inc(false);
dma.set_mem_inc(true);
dma.set_periph_data_alignment(gdut::dma_peripheral_data_alignment::byte);
dma.set_mem_data_alignment(gdut::dma_memory_data_alignment::byte);
dma.set_mode(gdut::dma_mode::circular);
dma.set_priority(gdut::dma_priority::high);
dma.init();
```

## 与代码规范的对应

- RAII 自动管理 DMA 资源，禁止复制，支持移动语义
- CRTP 避免虚函数开销，符合嵌入式性能要求
- 统一错误回调机制（`std::error_code`）
- 蛇形命名约定，私有成员 `m_` 前缀

## 注意事项/坑点

- DMA 传输所用数据缓冲区**不能**放在 CCMRAM（`GDUT_CCMRAM`）中，CCM RAM 不可被 DMA 访问
- 必须先调用 `init()` 再启动传输；直接使用 `dma_proxy` 时，`start()` 会在启动时自动绑定 HAL 回调
- 回调函数在**中断上下文**执行，禁止在回调中调用阻塞操作（如 `osMutexAcquire` 等）
- I2C 的 address 参数为 7 位从机地址（有效范围 0x08~0x77），传 0 无效
- SPI 的 address 参数被忽略（通过 `(void)address` 消除编译器警告），传任意值均可
- SPI 全双工传输（`dma_spi::receive()`）使用**同一缓冲区**进行发送和接收，先发后收
- SPI 必须配置为全双工模式（`SPI_DIRECTION_2LINES`），否则 `receive()` 会调用 `std::terminate()`
- `dma_spi` 手动实现了 HAL 的 DMA 启动流程，不直接调用 `HAL_SPI_Transmit_DMA` 等 HAL 函数，以确保回调正确触发
- `start_receive()` 已标记为弃用（`[[deprecated]]`），请使用 `receive()` 替代
- `dma_proxy` 不管理 `DMA_HandleTypeDef` 的内存，句柄的生命周期须由调用方保证

相关源码：[BSP/bsp_dma.hpp](../../BSP/bsp_dma.hpp)
