# BSP UART 串行通讯模块（bsp_uart.hpp）

## 原理
该模块对 STM32 HAL 的 UART 通讯功能进行 C++ 面向对象的封装，提供中断驱动的异步串口通讯接口，支持 DMA、超时控制和多种回调机制。

## 核心设计

### 1. DMA UART 类 `dma_uart`（基于 dma_transfer_base）
- 通过 CRTP 继承 `dma_transfer_base`，提供 DMA 异步传输
- 需要绑定 TX/RX `dma_proxy` 后才能使用
- 支持 DMA 发送和接收
- address 参数对 UART 无意义（内部忽略）
- 手动实现 HAL DMA 启动流程，确保回调正确触发
- 适用于大数据量、高频的异步通讯场景

### 2. UART 代理类 `uart`（全功能封装）
- **传输模式**：
  - 阻塞模式：`send()` / `receive()` 支持 C++20 chrono 超时
  - 中断模式：`send_it()` / `receive_it()` 非阻塞异步传输
  - DMA 模式：`send_dma()` / `receive_dma()` 高效数据传输
- **回调机制**：
  - 接收数据回调：`rx_callback_t`
  - 发送完成回调：`tx_callback_t`
  - 错误处理回调：`error_callback_t`
  - 空闲帧检测：`idle_callback_t`
  - DMA 回调：接收完成、发送完成、错误
- **特性**：
  - RAII 自动管理 UART 生命周期
  - 禁止复制，支持移动语义
  - 灵活的 DMA 关联（`attach_dma_rx` / `attach_dma_tx`）
  - 类型安全的操作接口
  - 支持运行时配置（波特率、数据位等）

### 3. 中断分发器 `uart_irq_handler`
- 维护静态的 `uart` 实例注册表（支持 USART1~6）
- 为 HAL 中断回调提供统一分发入口
- 根据硬件句柄找到对应的 `uart` 对象并调用其回调
- 需要在使用前注册实例（`register_uart`），销毁前取消注册（`unregister_uart`）

## 如何使用

### 基础初始化
```cpp
#include "bsp_uart.hpp"

// STM32 HAL UART 句柄
UART_HandleTypeDef huart1;

// 创建 UART 对象
gdut::uart serial_port(&huart1);

// 初始化
serial_port.init();
```

### 与 DMA 一起使用
```cpp
// DMA 句柄
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

// 创建 UART 对象并绑定 DMA
gdut::uart serial_dma(&huart1, &hdma_usart1_rx, &hdma_usart1_tx);
serial_dma.init();
```

### 阻塞模式收发（带超时）
```cpp
gdut::uart uart(&huart1);
uart.init();

// 阻塞发送，超时 1000ms
const uint8_t tx_data[] = "Hello UART\r\n";
if (uart.send(tx_data, sizeof(tx_data) - 1, std::chrono::milliseconds(1000)) == HAL_OK) {
    // 发送成功
}

// 阻塞接收，超时 500ms
uint8_t rx_data[64];
if (uart.receive(rx_data, sizeof(rx_data), std::chrono::milliseconds(500)) == HAL_OK) {
    // 接收成功
}
```

### 中断驱动接收
```cpp
// 设置接收回调
gdut::uart uart(&huart1);
uart.init();

uart.register_rx_callback([](const uint8_t *data, uint16_t size) {
    // 接收到 size 个字节的数据
    process_received_data(data, size);
});

// 启动接收中断（接收单字节或指定长度）
uint8_t rx_buffer[128];
uart.receive_it(rx_buffer, sizeof(rx_buffer));
```

### 中断驱动发送
```cpp
gdut::uart uart(&huart1);
uart.init();

// 设置发送完成回调
uart.register_tx_callback([]() {
    // 所有待发送数据已发送
    on_transmit_complete();
});

// 准备发送数据
const uint8_t data[] = "Hello UART\r\n";
uart.send_it(data, sizeof(data) - 1);
```

### 错误处理
```cpp
gdut::uart uart(&huart1);
uart.init();

uint8_t rx_buffer[128];

// 设置错误回调
uart.register_error_callback([&uart, &rx_buffer](uint32_t error) {
    // error 包含 HAL 错误码：奇偶校验错误、帧错误等
    if (error & HAL_UART_ERROR_PARITY) {
        // 奇偶校验错误
    }
    if (error & HAL_UART_ERROR_FRAME) {
        // 帧错误
    }
    // 重启接收或其他恢复操作
    uart.receive_it(rx_buffer, sizeof(rx_buffer));
});
```

### 空闲帧检测与 DMA 接收
```cpp
// 使用空闲中断判断一帧数据接收完成
// （常用于接收可变长度数据包）

extern DMA_HandleTypeDef hdma_usart1_rx;
gdut::uart uart(&huart1, &hdma_usart1_rx, nullptr);
uart.init();

// 数据缓冲区
const uint16_t buffer_size = 256;
uint8_t rx_buffer[buffer_size];

// 设置空闲帧回调
uart.register_idle_callback([&uart, &rx_buffer]() {
    // 一帧数据接收完成（检测到空闲）
    // 获取已接收的数据长度
    DMA_HandleTypeDef* hdma = uart.get_hdma_rx();
    if (hdma) {
        uint16_t rx_len = buffer_size - __HAL_DMA_GET_COUNTER(hdma);
        process_frame(rx_buffer, rx_len);
        // 重新启动 DMA 接收
        uart.receive_dma(rx_buffer, buffer_size);
    }
});

// 启动 DMA 接收
uart.receive_dma(rx_buffer, buffer_size);
```

### 使用 `dma_uart` 类（简化 DMA 操作）
```cpp
#include "bsp_uart.hpp"
#include "bsp_dma.hpp"

// CubeMX 生成的句柄
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

// 创建 DMA 代理
gdut::dma_proxy uart_tx_dma(&hdma_usart1_tx);
gdut::dma_proxy uart_rx_dma(&hdma_usart1_rx);

// 设置回调
uart_tx_dma.set_callback_handler([](std::error_code ec) {
    if (!ec) {
        // 发送完成
    } else {
        // 发送错误
    }
});

uart_rx_dma.set_callback_handler([](std::error_code ec) {
    if (!ec) {
        // 接收完成
    } else {
        // 接收错误
    }
});

// 初始化 DMA
uart_tx_dma.init();
uart_rx_dma.init();

// 创建 UART DMA 对象并绑定
gdut::dma_uart uart_dma(&huart1);
uart_dma.bind(&uart_tx_dma, &uart_rx_dma);

// 发送数据（异步，不阻塞）
const uint8_t tx_data[] = "Hello DMA\r\n";
if (uart_dma.transmit(tx_data, sizeof(tx_data) - 1)) {
    // DMA 传输已启动
}

// 接收数据
uint8_t rx_buffer[128];
if (uart_dma.receive(rx_buffer, sizeof(rx_buffer))) {
    // DMA 接收已启动
}
```

## 实际应用示例

### 简单的串口调试接口
```cpp
class debug_uart {
public:
    debug_uart(UART_HandleTypeDef *huart)
        : m_uart(huart), m_rx_index(0) {
        m_uart.init();
        m_uart.register_rx_callback([this](const uint8_t *data, uint16_t size) {
            on_rx_data(data, size);
        });
        // 启动中断接收
        m_uart.receive_it(m_rx_buffer, 1);  // 逐字节接收
    }
    
    void send(const char *msg) {
        const uint8_t *data = reinterpret_cast<const uint8_t *>(msg);
        uint16_t len = strlen(msg);
        m_uart.send_it(data, len);
    }
    
private:
    gdut::uart m_uart;
    uint8_t m_rx_buffer[256];
    uint16_t m_rx_index;
    
    void on_rx_data(const uint8_t *data, uint16_t size) {
        for (uint16_t i = 0; i < size; i++) {
            if (data[i] == '\n') {
                // 完整命令行接收
                process_command(m_rx_buffer, m_rx_index);
                m_rx_index = 0;
            } else {
                m_rx_buffer[m_rx_index++] = data[i];
                if (m_rx_index >= sizeof(m_rx_buffer)) {
                    m_rx_index = 0;  // 防止缓冲区溢出
                }
            }
        }
    }
    
    void process_command(const uint8_t *cmd, uint16_t len) {
        // 处理接收到的命令
    }
};
```

### DMA 缓冲接收
```cpp
// 使用 DMA 和循环缓冲区实现高效接收

class uart_dma_receiver {
public:
    static const uint16_t buffer_size = 512;
    
    uart_dma_receiver(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx)
        : m_uart(huart, hdma_rx, nullptr) {
        m_uart.init();
        m_uart.register_dma_rx_cplt_callback([this]() {
            on_dma_complete();
        });
        m_uart.register_idle_callback([this]() {
            on_idle();
        });
    }
    
    void start() {
        m_uart.receive_dma(m_buffer, buffer_size);
    }
    
    uint16_t get_received_count() {
        // 计算已接收的字节数
        uint16_t current_pos = buffer_size - __HAL_DMA_GET_COUNTER(&hdma_uart_rx);
        return current_pos;
    }
    
private:
    gdut::uart m_uart;
    uint8_t m_buffer[buffer_size];
    uint16_t m_last_pos = 0;
    
    void on_dma_complete() {
        // DMA 缓冲区满
        uint16_t count = buffer_size - m_last_pos;
        process_data(m_buffer + m_last_pos, count);
        m_last_pos = 0;
    }
    
    void on_idle() {
        // 接收到空闲，可能有部分数据
        uint16_t current = get_received_count();
        if (current > m_last_pos) {
            uint16_t count = current - m_last_pos;
            process_data(m_buffer + m_last_pos, count);
            m_last_pos = current;
        }
    }
    
    void process_data(const uint8_t *data, uint16_t len) {
        // 处理接收到的数据
    }
};
```

### 模块间通讯协议
```cpp
// 实现一个简单的消息帧协议

struct message_frame {
    static const uint8_t header = 0xAA;
    static const uint8_t trailer = 0x55;
    
    uint8_t cmd;
    uint16_t data_len;
    uint8_t data[64];
    uint8_t checksum;
};

class frame_uart {
public:
    using frame_callback_t = std::function<void(const message_frame &)>;
    
    frame_uart(UART_HandleTypeDef *huart)
        : m_uart(huart), m_rx_state(state::waiting_header) {}
    
    void init() {
        m_uart.init();
        m_uart.register_rx_callback([this](const uint8_t *data, uint16_t size) {
            for (uint16_t i = 0; i < size; i++) {
                on_byte_received(data[i]);
            }
        });
        // 启动中断接收
        static uint8_t rx_byte;
        m_uart.receive_it(&rx_byte, 1);
    }
    
    void set_frame_callback(frame_callback_t cb) {
        m_frame_callback = cb;
    }
    
    void send_frame(const message_frame &frame) {
        std::vector<uint8_t> packet;
        packet.push_back(message_frame::header);
        packet.push_back(frame.cmd);
        packet.push_back(frame.data_len >> 8);
        packet.push_back(frame.data_len & 0xFF);
        
        for (uint16_t i = 0; i < frame.data_len; i++) {
            packet.push_back(frame.data[i]);
        }
        
        uint8_t checksum = calculate_checksum(packet.data(), packet.size());
        packet.push_back(checksum);
        packet.push_back(message_frame::trailer);
        
        m_uart.send_it(packet.data(), packet.size());
    }
    
private:
    enum class state { waiting_header, waiting_cmd, waiting_len_h, waiting_len_l, 
                       receiving_data, waiting_checksum, waiting_trailer };
    
    gdut::uart m_uart;
    state m_rx_state;
    message_frame m_current_frame;
    uint16_t m_data_count = 0;
    frame_callback_t m_frame_callback;
    
    void on_byte_received(uint8_t byte) {
        // 帧解析状态机
        switch (m_rx_state) {
            case state::waiting_header:
                if (byte == message_frame::header) {
                    m_rx_state = state::waiting_cmd;
                }
                break;
            case state::waiting_cmd:
                m_current_frame.cmd = byte;
                m_rx_state = state::waiting_len_h;
                break;
            // ... 其他状态处理
        }
    }
    
    uint8_t calculate_checksum(const uint8_t *data, uint16_t len) {
        uint8_t sum = 0;
        for (uint16_t i = 0; i < len; i++) {
            sum += data[i];
        }
        return sum;
    }
};
```

## 与代码规范的对应
- RAII 自动管理 UART 资源
- 禁止复制，支持移动语义
- 灵活的回调机制，支持 Lambda 和函数指针
- 蛇形命名约定
- DMA 和中断的无缝集成

## 接收模式选择

| 模式 | 优点 | 缺点 | 适用场景 |
|------|------|------|---------|
| `receive_it()` | 简单，占用内存少 | 每字节中断开销大 | 低速通讯 |
| `receive_dma()` | 高效，低中断频率 | 内存占用大 | 高速通讯，大数据量 |
| `receive_to_idle()` | 自动检测帧结束 | 需要配置空闲中断 | 可变长度数据包 |

## 注意事项/坑点

### 通用注意事项
- 回调函数运行在**中断上下文**，避免耗时操作或阻塞调用（如 `osDelay`、`osMutexAcquire` 等）
- 波特率和时钟配置必须在 CubeMX 中正确设置
- 发送未完成前不能开始新的发送操作
- 同时接收和发送时确保缓冲区不冲突
- UART 错误会导致接收中断停止，需在错误回调中重启接收
- 多个 UART 实例应使用不同的 DMA 通道

### DMA 相关注意事项
- DMA 接收缓冲区必须声明为 `static` 或全局，**不能是栈变量**
- DMA 传输的数据缓冲区**不能**放在 CCMRAM（`GDUT_CCMRAM`），CCM RAM 不能被 DMA 访问
- `dma_uart` 必须先调用 `dma_proxy::init()` 后才能使用，否则回调不会触发
- `dma_uart` 手动实现了 HAL DMA 启动流程（参考 `HAL_UART_Transmit_DMA`），以确保回调正确触发
- address 参数对 UART 无意义，内部会忽略（通过 `(void)address` 消除编译器警告）
- DMA 的循环模式需要手动跟踪已处理数据位置

### 中断与回调注意事项
- 空闲中断需要单独配置（通常在 CubeMX 中启用 IDLE Line Detection）
- 使用 `uart_irq_handler` 时，必须在使用前调用 `register_uart()` 注册实例
- 在 `uart` 对象销毁前，必须调用 `unregister_uart()` 取消注册，避免悬空指针
- `uart_irq_handler` 不是线程安全的，建议在调度器启动前或临界区内进行注册/取消注册
- 回调注册使用 `register_*_callback()`，而非 `set_*_callback()`

### API 名称
- 阻塞模式：`send()` / `receive()`（支持 chrono 超时）
- 中断模式：`send_it()` / `receive_it()`
- DMA 模式：`send_dma()` / `receive_dma()`（需要先关联 DMA 句柄）

## 性能优化建议
- 对高速通讯（> 115200 bps）优先使用 DMA
- 对帧长度固定的数据使用 DMA，变长使用空闲中断
- 回调中避免调用 `printf` 等耗时函数
- 使用消息队列将 UART 数据转移到普通任务处理

相关源码：[BSP/bsp_uart.hpp](../../BSP/bsp_uart.hpp)
