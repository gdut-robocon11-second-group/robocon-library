# BSP 类型工具与硬件映射模块（bsp_utility.hpp）

## 原理
该模块提供 GPIO 端口枚举、定时器 ID 枚举、以及硬件资源映射函数，确保类型安全并避免魔法数。还包含时间转换工具函数，用于 CMSIS-RTOS2 超时处理。与硬件无关的编译期类型工具已拆分至 `Components/utils_type_traits.hpp`。

## 核心设计

### 编译期类型工具
- `is_power_of_two<N>`：检查数值是否为 2 的幂（位于 `Components/utils_type_traits.hpp`）
- `always_false<T>`：用于 `static_assert` 的延迟失败机制（位于 `Components/utils_type_traits.hpp`）

### 硬件资源枚举
- `gpio_port`：GPIO 端口枚举（A-I）
- `timer_id`：定时器 ID 枚举（TIM1-TIM5, TIM9-TIM11）

### 映射函数
- `get_gpio_port_ptr()`：从枚举或地址获取 GPIO 端口指针
- `get_timer_ptr()`：从枚举获取定时器指针
- `get_uart_index()`：从 UART 实例获取索引

### 时间转换工具
- `time_to_ticks()`：将 `std::chrono::duration` 转换为 RTOS ticks

### CCM RAM 宏
- `GDUT_CCMRAM`：将变量放置到核心耦合存储器（高速访问，但 DMA 不可访问）

## 如何使用

### GPIO 端口映射
```cpp
#include "bsp_utility.hpp"

void configure_gpio() {
    // 使用强类型枚举获取 GPIO 端口
    GPIO_TypeDef *porta = gdut::get_gpio_port_ptr(gdut::gpio_port::A);
    GPIO_TypeDef *portb = gdut::get_gpio_port_ptr(gdut::gpio_port::B);
    
    if (porta != nullptr) {
        // 配置 GPIOA
        HAL_GPIO_WritePin(porta, GPIO_PIN_5, GPIO_PIN_SET);
    }
    
    // 也可以从地址获取（用于与 HAL 集成）
    GPIO_TypeDef *portc = gdut::get_gpio_port_ptr(GPIOC_BASE);
}
```

### 定时器映射
```cpp
void configure_timer() {
    // 获取定时器指针
    TIM_TypeDef *tim2 = gdut::get_timer_ptr(gdut::timer_id::tim2);
    TIM_TypeDef *tim3 = gdut::get_timer_ptr(gdut::timer_id::tim3);
    
    if (tim2 != nullptr) {
        // 使用 TIM2
        __HAL_TIM_SET_COUNTER(tim2, 0);
    }
}
```

### UART 索引获取
```cpp
void get_uart_info() {
    UART_HandleTypeDef huart1;
    huart1.Instance = USART1;
    
    // 获取 UART 索引（用于数组索引等）
    uint8_t uart_idx = gdut::get_uart_index(huart1.Instance);
    
    if (uart_idx != 0xFF) {
        printf("UART index: %d\n", uart_idx);  // 输出：0
    }
}
```

### 时间转换
```cpp
#include <chrono>

void use_time_conversion() {
    // 将 C++ chrono 时长转换为 RTOS ticks
    
    // 500 毫秒
    uint32_t ticks1 = gdut::time_to_ticks(std::chrono::milliseconds(500));
    
    // 1 秒
    uint32_t ticks2 = gdut::time_to_ticks(std::chrono::seconds(1));
    
    // 无限等待
    uint32_t ticks3 = gdut::time_to_ticks(std::chrono::milliseconds::max());
    // ticks3 == osWaitForever
    
    // 微秒（会向下截断到毫秒）
    uint32_t ticks4 = gdut::time_to_ticks(std::chrono::microseconds(1500));
    // 相当于 1 毫秒
}
```

### 编译期类型检查
```cpp
#include "utils_type_traits.hpp"

// 检查数值是否为 2 的幂
static_assert(gdut::is_power_of_two_v<16>, "16 is a power of 2");
static_assert(gdut::is_power_of_two_v<128>, "128 is a power of 2");
// static_assert(gdut::is_power_of_two_v<100>, "Error: 100 is not a power of 2");

// 用于模板特化失败提示
template <typename T>
void process() {
    static_assert(gdut::always_false_v<T>, 
                  "This template must be specialized for your type");
}
```

### CCM RAM 使用
```cpp
// 将变量放置到 CCM RAM（高速访问）
GDUT_CCMRAM static uint8_t fast_buffer[1024];
GDUT_CCMRAM static int counter = 0;

void use_ccm_ram() {
    // CCM RAM 访问速度快，但不能用于 DMA
    counter++;
    fast_buffer[0] = 0xAA;
}

// CCM RAM 用于内存池
GDUT_CCMRAM static gdut::pmr::fixed_block_resource<4096> ccm_pool;
```

## 实际应用示例

### GPIO 端口迭代器
```cpp
void initialize_all_gpio_ports() {
    // 遍历所有 GPIO 端口
    constexpr gdut::gpio_port ports[] = {
        gdut::gpio_port::A, gdut::gpio_port::B, gdut::gpio_port::C,
        gdut::gpio_port::D, gdut::gpio_port::E, gdut::gpio_port::F,
        gdut::gpio_port::G, gdut::gpio_port::H, gdut::gpio_port::I
    };
    
    for (auto port : ports) {
        GPIO_TypeDef *gpio = gdut::get_gpio_port_ptr(port);
        if (gpio != nullptr) {
            // 启用 GPIO 时钟或其他初始化
            printf("Initializing GPIO port\n");
        }
    }
}
```

### 定时器资源管理器
```cpp
class timer_manager {
public:
    TIM_TypeDef* get_timer(gdut::timer_id id) {
        TIM_TypeDef *tim = gdut::get_timer_ptr(id);
        if (tim != nullptr) {
            m_active_timers.push_back(id);
        }
        return tim;
    }
    
    void release_all() {
        for (auto id : m_active_timers) {
            TIM_TypeDef *tim = gdut::get_timer_ptr(id);
            if (tim != nullptr) {
                // 停止定时器
                __HAL_TIM_DISABLE(tim);
            }
        }
        m_active_timers.clear();
    }

private:
    std::vector<gdut::timer_id> m_active_timers;
};
```

### UART 实例数组
```cpp
class uart_pool {
public:
    static constexpr size_t max_uarts = 6;
    
    void register_uart(UART_HandleTypeDef *huart) {
        uint8_t idx = gdut::get_uart_index(huart->Instance);
        if (idx != 0xFF && idx < max_uarts) {
            m_uarts[idx] = huart;
        }
    }
    
    UART_HandleTypeDef* get_uart(uint8_t index) {
        if (index < max_uarts) {
            return m_uarts[index];
        }
        return nullptr;
    }

private:
    UART_HandleTypeDef *m_uarts[max_uarts] = {nullptr};
};
```

### 超时工具类
```cpp
class timeout_helper {
public:
    template <typename Rep, typename Period>
    static bool wait_for_condition(
        std::function<bool()> condition,
        std::chrono::duration<Rep, Period> timeout) {
        
        uint32_t start = osKernelGetTickCount();
        uint32_t timeout_ticks = gdut::time_to_ticks(timeout);
        
        while (!condition()) {
            if (osKernelGetTickCount() - start >= timeout_ticks) {
                return false;  // 超时
            }
            osDelay(1);
        }
        
        return true;  // 条件满足
    }
};

// 使用
bool ready = timeout_helper::wait_for_condition(
    []() { return sensor_ready(); },
    std::chrono::seconds(5)
);
```

### 类型安全的配置表
```cpp
struct gpio_config {
    gdut::gpio_port port;
    uint16_t pin;
    GPIO_InitTypeDef init;
};

// 编译期配置表
constexpr gpio_config led_configs[] = {
    {gdut::gpio_port::A, GPIO_PIN_5, 
     {.Pin = GPIO_PIN_5, .Mode = GPIO_MODE_OUTPUT_PP}},
    {gdut::gpio_port::B, GPIO_PIN_0, 
     {.Pin = GPIO_PIN_0, .Mode = GPIO_MODE_OUTPUT_PP}},
    {gdut::gpio_port::C, GPIO_PIN_13, 
     {.Pin = GPIO_PIN_13, .Mode = GPIO_MODE_OUTPUT_PP}}
};

void initialize_leds() {
    for (const auto &cfg : led_configs) {
        GPIO_TypeDef *port = gdut::get_gpio_port_ptr(cfg.port);
        if (port != nullptr) {
            HAL_GPIO_Init(port, const_cast<GPIO_InitTypeDef*>(&cfg.init));
        }
    }
}
```

### CCM RAM 高速缓冲区
```cpp
// FFT 计算缓冲区（需要高速访问，不需要 DMA）
GDUT_CCMRAM static float fft_input[512];
GDUT_CCMRAM static float fft_output[512];

void perform_fft() {
    // 在 CCM RAM 中进行计算（更快）
    for (int i = 0; i < 512; i++) {
        fft_input[i] = read_adc_value();
    }
    
    compute_fft(fft_input, fft_output, 512);
    
    // 结果可以传输到普通 RAM 或外设
}

// 线程栈放在 CCM RAM
GDUT_CCMRAM static StackType_t worker_stack[512];
```

## 工具函数详解

### time_to_ticks() 行为
```cpp
// 示例：假设 tick 频率为 1000 Hz (1ms per tick)

// 正常转换
gdut::time_to_ticks(std::chrono::milliseconds(100));  // → 100 ticks

// 无限等待
gdut::time_to_ticks(std::chrono::milliseconds::max()); // → osWaitForever

// 零超时（立即返回）
gdut::time_to_ticks(std::chrono::milliseconds(0));     // → 0

// 负数（非法，返回 0）
gdut::time_to_ticks(std::chrono::milliseconds(-100));  // → 0

// 亚毫秒精度（向下截断）
gdut::time_to_ticks(std::chrono::microseconds(1500));  // → 1 tick (1ms)
gdut::time_to_ticks(std::chrono::microseconds(500));   // → 0 ticks

// 秒级转换
gdut::time_to_ticks(std::chrono::seconds(2));          // → 2000 ticks

// 超大值（夹紧到 UINT32_MAX-1）
gdut::time_to_ticks(std::chrono::hours(1000));         // → UINT32_MAX-1
```

### GPIO 端口枚举值
```cpp
// 枚举值对应关系
gdut::gpio_port::A  // → GPIOA
gdut::gpio_port::B  // → GPIOB
gdut::gpio_port::C  // → GPIOC
gdut::gpio_port::D  // → GPIOD
gdut::gpio_port::E  // → GPIOE
gdut::gpio_port::F  // → GPIOF
gdut::gpio_port::G  // → GPIOG
gdut::gpio_port::H  // → GPIOH
gdut::gpio_port::I  // → GPIOI
```

### 定时器 ID 枚举值
```cpp
// STM32F407 可用定时器
gdut::timer_id::tim1   // → TIM1
gdut::timer_id::tim2   // → TIM2
gdut::timer_id::tim3   // → TIM3
gdut::timer_id::tim4   // → TIM4
gdut::timer_id::tim5   // → TIM5
gdut::timer_id::tim9   // → TIM9
gdut::timer_id::tim10  // → TIM10
gdut::timer_id::tim11  // → TIM11
```

## 与代码规范的对应
- 强类型枚举（`enum class`）避免隐式转换
- `constexpr` 函数实现编译期映射
- `[[nodiscard]]` 属性防止忽略返回值
- 蛇形命名约定
- 类型安全，减少魔法数

## 注意事项/坑点
- ⚠️ **非法枚举返回 nullptr**：调用 HAL 前必须检查返回值
- ⚠️ **芯片相关性**：枚举值与 STM32F407 绑定，移植到其他芯片需要调整
- ⚠️ **CCM RAM 限制**：CCM RAM 不能用于 DMA 操作
  - ✅ 可用于：线程栈、局部变量、计算缓冲区、RTOS 对象
  - ❌ 不可用于：DMA 源/目标、外设缓冲区、需要 DMA 的数据
- ⚠️ **时间转换精度**：亚毫秒精度会被截断
- ⚠️ **超时溢出**：超大超时值会被夹紧到 `UINT32_MAX-1`
- ⚠️ **UART 索引越界**：非法 UART 实例返回 `0xFF`

## 编译期检查示例

### 对齐检查
```cpp
template <typename T, size_t Alignment>
struct aligned_storage {
    static_assert(gdut::is_power_of_two_v<Alignment>,
                  "Alignment must be a power of 2");
    
    alignas(Alignment) T data;
};

// 合法
aligned_storage<int, 8> storage1;    // OK
aligned_storage<int, 16> storage2;   // OK

// 非法（编译错误）
// aligned_storage<int, 7> storage3;  // Error: 7 不是 2 的幂
```

### 延迟失败
```cpp
template <typename T>
class serializer {
    static_assert(gdut::always_false_v<T>,
                  "Serializer must be specialized for your type");
};

// 必须为每个类型提供特化
template <>
class serializer<int> {
    // int 的序列化实现
};

template <>
class serializer<float> {
    // float 的序列化实现
};

// 使用
serializer<int> s1;      // OK
serializer<float> s2;    // OK
// serializer<double> s3;   // Error: 没有 double 的特化
```

## CCM RAM 最佳实践

### 适合放在 CCM RAM 的内容
```cpp
// ✅ 线程栈
GDUT_CCMRAM static StackType_t task_stack[1024];

// ✅ 频繁访问的全局变量
GDUT_CCMRAM static int32_t fast_counter = 0;

// ✅ 计算缓冲区
GDUT_CCMRAM static float calculation_buffer[256];

// ✅ FreeRTOS 静态对象
GDUT_CCMRAM static StaticTask_t task_tcb;

// ✅ 内存池
GDUT_CCMRAM static gdut::pmr::fixed_block_resource<2048> pool;
```

### 不适合放在 CCM RAM 的内容
```cpp
// ❌ DMA 缓冲区（会失败）
// GDUT_CCMRAM static uint8_t dma_buffer[512];  // 错误！

// ❌ UART/SPI 接收缓冲区
// GDUT_CCMRAM static uint8_t uart_rx_buffer[256];  // 错误！

// ❌ ADC 数据缓冲区
// GDUT_CCMRAM static uint16_t adc_values[128];  // 错误！

// 正确做法：使用普通 RAM
static uint8_t dma_buffer[512];
static uint8_t uart_rx_buffer[256];
static uint16_t adc_values[128];
```

相关源码：[BSP/bsp_utility.hpp](../../BSP/bsp_utility.hpp)
