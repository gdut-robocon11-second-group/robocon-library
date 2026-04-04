# BSP SPI 通讯模块（bsp_spi.hpp）

## 原理
该模块对 STM32 HAL 的 SPI 通讯功能进行 C++ 包装，提供**阻塞模式**和 **DMA 模式**两种 SPI 数据传输方式，支持 C++20 chrono 超时机制（阻塞模式）和异步回调机制（DMA 模式）。

## 核心设计

### 1. SPI 代理类 `spi_proxy`（阻塞模式）
- 基于 STM32 HAL 的 SPI 阻塞模式（Blocking Mode）
- 支持三种传输模式：发送、接收、全双工收发
- RAII 自动管理，禁止复制（`uncopyable`）
- 集成 C++20 `std::chrono` 超时机制
- 参数合法性检查（nullptr、零长度检查）
- 适用于短时、低频的同步通讯场景

### 2. SPI DMA 类 `dma_spi`（DMA 模式）
- 基于 CRTP 继承 `dma_transfer_base`，提供 DMA 异步传输
- 需要绑定 TX/RX `dma_proxy` 后才能使用
- 支持单向发送、单向接收和全双工传输
- address 参数对 SPI 无意义（内部忽略）
- 手动实现 HAL DMA 启动流程，确保回调正确触发
- 适用于大数据量、高频的异步通讯场景

## 如何使用

### 模式一：阻塞模式（`spi_proxy`）

#### 基础初始化
```cpp
#include "bsp_spi.hpp"

// STM32 HAL SPI 句柄
SPI_HandleTypeDef hspi1;

// 创建 SPI 代理对象
gdut::spi_proxy spi(&hspi1);
```

### 发送数据
```cpp
// 发送数据，使用默认超时（最大值 = 无限等待）
const uint8_t tx_data[] = {0x01, 0x02, 0x03, 0x04};
if (spi.transmit(tx_data, sizeof(tx_data))) {
    // 发送成功
}

// 发送数据，指定超时时间为 1000ms
if (spi.transmit(tx_data, sizeof(tx_data), std::chrono::milliseconds(1000))) {
    // 发送成功
}

// 发送数据，立即超时（检查是否就绪）
if (spi.transmit(tx_data, sizeof(tx_data), std::chrono::milliseconds(0))) {
    // SPI 已就绪
}
```

### 接收数据
```cpp
// 接收数据，使用默认超时
uint8_t rx_data[4];
if (spi.receive(rx_data, sizeof(rx_data))) {
    // 接收成功
    // rx_data[0..3] 包含接收到的数据
}

// 接收数据，指定超时时间为 500ms
if (spi.receive(rx_data, sizeof(rx_data), std::chrono::milliseconds(500))) {
    // 接收成功
}
```

### 全双工传输（同时发送和接收）
```cpp
// 全双工传输：同时发送和接收，常用于 SPI 传感器和 IC 通讯
const uint8_t tx_data[] = {0xA5, 0x5A};  // 发送数据
uint8_t rx_data[2];                       // 接收缓冲区

if (spi.transmit_receive(tx_data, rx_data, sizeof(tx_data))) {
    // 传输完成
    // rx_data 包含接收到的数据
}

// 指定超时
if (spi.transmit_receive(tx_data, rx_data, sizeof(tx_data), 
                         std::chrono::milliseconds(1000))) {
    // 传输成功
}
```

### 模式二：DMA 模式（`dma_spi`）

#### 初始化与绑定 DMA
```cpp
#include "bsp_spi.hpp"
#include "bsp_dma.hpp"

// CubeMX 生成的句柄
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;

// 创建 DMA 代理
gdut::dma_proxy spi_tx_dma(&hdma_spi2_tx);
gdut::dma_proxy spi_rx_dma(&hdma_spi2_rx);

// 设置回调（传输完成或出错时触发）
spi_tx_dma.set_callback_handler([](std::error_code ec) {
    if (!ec) {
        // 发送完成
    } else {
        // 发送出错
    }
});

spi_rx_dma.set_callback_handler([](std::error_code ec) {
    if (!ec) {
        // 接收完成，处理数据
    } else {
        // 接收出错
    }
});

// 初始化 DMA（必须在传输前调用）
spi_tx_dma.init();
spi_rx_dma.init();

// 创建 SPI DMA 对象并绑定 DMA 代理
gdut::dma_spi spi_dma(&hspi2);
spi_dma.bind(&spi_tx_dma, &spi_rx_dma);
```

#### DMA 发送数据
```cpp
// 发送数据（异步，不阻塞）
const uint8_t tx_buf[] = {0x01, 0x02, 0x03, 0x04};
if (spi_dma.transmit(tx_buf, sizeof(tx_buf))) {
    // DMA 传输已启动（非阻塞）
    // 实际传输完成会通过 spi_tx_dma 的回调通知
} else {
    // 启动失败（SPI 繁忙或参数错误）
}
```

#### DMA 全双工传输
```cpp
// 全双工 DMA 传输（同时发送和接收）
uint8_t buffer[16] = {0xA5, 0x5A, /* ... */};  // 发送数据

if (spi_dma.receive(buffer, sizeof(buffer))) {
    // DMA 传输已启动
    // 传输完成后，buffer 将包含接收到的数据
    // 通过 spi_rx_dma 的回调通知完成
}
```

#### 结合 FreeRTOS 信号量实现同步
```cpp
#include "semaphore.hpp"

gdut::binary_semaphore spi_done_sem;  // 传输完成信号量

// 设置 DMA 回调
spi_tx_dma.set_callback_handler([](std::error_code ec) {
    if (!ec) {
        spi_done_sem.release();  // 通知传输完成
    }
});

// 发起 DMA 传输
const uint8_t data[] = {0xAA, 0xBB};
if (spi_dma.transmit(data, sizeof(data))) {
    // 等待传输完成（最多 1000ms）
    if (spi_done_sem.acquire(std::chrono::milliseconds(1000))) {
        // 传输成功完成
    } else {
        // 超时
    }
}
```

## 实际应用示例

### 与 SPI 传感器通讯
```cpp
// IMU 或加速度计等传感器通常采用全双工通讯

struct sensor_data {
    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;
};

gdut::spi_proxy imu_spi(hspi2);

// 读取传感器数据
uint8_t cmd[7] = {0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // 命令 + 占位符
uint8_t response[7];

if (imu_spi.transmit_receive(cmd, response, sizeof(cmd), 
                             std::chrono::milliseconds(100))) {
    sensor_data data;
    // 解析接收到的数据
    data.accel_x = (response[1] << 8) | response[2];
    data.accel_y = (response[3] << 8) | response[4];
    data.accel_z = (response[5] << 8) | response[6];
}
```

### 写入 SPI Flash
```cpp
gdut::spi_proxy flash_spi(hspi3);

// 发送写启用命令
const uint8_t write_enable[] = {0x06};
flash_spi.transmit(write_enable, sizeof(write_enable), 
                   std::chrono::milliseconds(10));

// 发送编程命令 + 地址 + 数据
uint8_t program_cmd[] = {0x02, 0x00, 0x00, 0x00, 
                         0xAA, 0xBB, 0xCC, 0xDD};
flash_spi.transmit(program_cmd, sizeof(program_cmd), 
                   std::chrono::milliseconds(50));
```

### 读取 SPI 存储器
```cpp
gdut::spi_proxy ram_spi(hspi4);

// 发送读命令 + 地址
uint8_t read_cmd[] = {0x03, 0x10, 0x20, 0x00};  // 读地址 0x102000
uint8_t read_data[16];

if (ram_spi.transmit_receive(read_cmd, read_data, sizeof(read_cmd), 
                             std::chrono::milliseconds(100))) {
    // read_data[1..16] 包含读取的内容
}
```

## 与代码规范的对应
- 对象禁止复制（`uncopyable`），避免句柄冲突
- 参数合法性检查，返回 `bool` 表示成功/失败
- C++20 `std::chrono` 超时机制，类型安全
- 蛇形命名约定
- 引用语义，直接使用 HAL 句柄

## 超时处理

### 超时转换规则
- `std::chrono::milliseconds::max()` → `osWaitForever`（无限等待）
- 其他值 → 直接传递给 HAL（以毫秒为单位）

### 常见超时场景
```cpp
// 快速操作（< 10ms）
spi.transmit(data, size, std::chrono::milliseconds(10));

// 中等操作（100ms 左右）
spi.receive(data, size, std::chrono::milliseconds(100));

// 长时间操作（>= 1s）
spi.transmit_receive(tx, rx, size, std::chrono::milliseconds(1000));

// 无限等待
spi.transmit(data, size, std::chrono::milliseconds::max());
```

## 注意事项/坑点

### 阻塞模式（`spi_proxy`）
- 所有操作都是**阻塞模式**，会阻塞当前任务，不能在关键路径上使用
- 仅支持 CPU 轮询模式，不支持 DMA 或中断
- 发送和接收数据必须 > 0 字节，否则返回 `false`
- 指针不能为 nullptr，否则返回 `false`
- HAL 期望 `uint8_t*` 但参数声明为 `const uint8_t*`，内部使用 `const_cast` 处理（HAL 不会修改数据）
- 超时过短（< 1ms）可能导致操作失败
- 对象生命周期必须长于所有 SPI 操作

### DMA 模式（`dma_spi`）
- **必须**先调用 `dma_proxy::init()` 后才能使用，否则回调不会触发
- DMA 传输的数据缓冲区**不能**放在 CCMRAM（`GDUT_CCMRAM`），CCM RAM 不能被 DMA 访问
- 回调函数在**中断上下文**执行，禁止调用阻塞操作（如 `osDelay`、`osMutexAcquire` 等）
- `transmit()` 和 `receive()` 的 address 参数对 SPI 无意义，内部会忽略
- 全双工传输（`receive`）时，发送和接收使用**同一缓冲区**（buffer），先发后收
- 不可在传输进行中再次调用 `transmit()` 或 `receive()`，会返回 `false`（SPI 状态检查）
- 传输失败时返回 `false`，同时通过 DMA 回调报告错误（`std::error_code`）
- 必须确保 SPI 配置为全双工模式（`SPI_DIRECTION_2LINES`），否则 `receive()` 会触发 `std::terminate()`
- 手动实现了 HAL 的 DMA 启动流程（参考 `HAL_SPI_Transmit_DMA` 和 `HAL_SPI_TransmitReceive_DMA`），以确保回调正确触发
- 使用 `reinterpret_cast` 将指针转换为 `uint32_t`（符合 HAL DMA API 要求）
- 对象生命周期必须长于所有 DMA 传输

相关源码：[BSP/bsp_spi.hpp](../../BSP/bsp_spi.hpp)
