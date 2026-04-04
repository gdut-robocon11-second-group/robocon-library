# BSP 定时器模块（bsp_timer.hpp）

## 原理
该模块对 STM32 HAL 的定时器（Timer）功能进行 C++ 面向对象的封装，提供基于回调的定时器中断处理、PWM、编码器等功能的统一接口。

## 核心设计

### 回调机制
- 使用 `gdut::function` 实现灵活的回调存储
- 支持 DMA 操作的三级回调：完成、半完成、错误
- 支持用户自定义回调函数或 Lambda

### 定时器模式
- 基础定时器模式（Timer Base）
- PWM 输出（PWM Output）
- 输入捕获（Input Capture）
- 编码器模式（Encoder Mode）
- DMA 支持

### 类 `timer`
- RAII 自动管理定时器生命周期
- 禁止复制，支持移动语义
- 自动处理 DMA 关联和回调挂钩

## 如何使用

### 基础定时器初始化
```cpp
#include "bsp_timer.hpp"

// STM32 HAL 定时器句柄
TIM_HandleTypeDef htim2;

// 创建定时器对象（不带 DMA）
gdut::timer basic_timer(&htim2);

// 初始化定时器
if (basic_timer.init() == HAL_OK) {
    // 启动定时器
    basic_timer.start();
}
```

### 带回调的定时器
```cpp
// 定义定时器中断回调
gdut::timer timer_with_callback(&htim3);

// 设置溢出中断回调（每次计数器溢出时调用）
timer_with_callback.set_callback([]() {
    // 定时器溢出时执行
    toggle_led();
});

// 启动定时器中断
timer_with_callback.start_it();
```

### PWM 输出
```cpp
// PWM 是 timer 的嵌套类，需要单独构造

gdut::timer basic_timer(&htim4);
basic_timer.init();

// 创建 PWM 子对象
gdut::timer::timer_pwm pwm(&basic_timer);

// 启动 PWM 输出（通道 1）
pwm.pwm_start(TIM_CHANNEL_1);

// 更改 PWM 占空比
uint32_t duty = 500;  // 根据定时器频率和周期调整
pwm.set_duty(TIM_CHANNEL_1, duty);

// 停止 PWM
pwm.pwm_stop(TIM_CHANNEL_1);
```

### 输入捕获模式
```cpp
// 输入捕获是 timer 的嵌套类，需要单独构造

gdut::timer capture_timer(&htim5);
capture_timer.init();

// 设置捕获中断回调
capture_timer.register_capture_callback(TIM_CHANNEL_1, []() {
    // 捕获完成时执行
});

// 创建输入捕获子对象
gdut::timer::timer_ic ic(&capture_timer);

// 启动输入捕获（带中断）
ic.ic_start_it(TIM_CHANNEL_1);

// 在回调中获取捕获值
uint32_t captured_value = ic.get_capture(TIM_CHANNEL_1);
```

### 编码器模式
```cpp
// 编码器是 timer 的嵌套类，需要单独构造

gdut::timer encoder_timer(&htim8);
encoder_timer.init();

// 创建编码器子对象
gdut::timer::timer_encoder encoder(&encoder_timer);

// 启动编码器（双通道）
encoder.encoder_start(TIM_CHANNEL_ALL);

// 读取计数值
uint32_t count = encoder_timer.get_counter();

// 检查计数方向
bool is_down = encoder.is_counting_down();

// 重置计数
encoder_timer.set_counter(0);

// 停止编码器
encoder.encoder_stop(TIM_CHANNEL_ALL);
```

### 输出比较模式
```cpp
// 输出比较是 timer 的嵌套类，需要单独构造

gdut::timer oc_timer(&htim9);
oc_timer.init();

// 创建输出比较子对象
gdut::timer::timer_oc oc(&oc_timer);

// 启动输出比较
oc.oc_start(TIM_CHANNEL_1);

// 设置比较值
oc.set_compare(TIM_CHANNEL_1, 1000);

// 停止输出比较
oc.oc_stop(TIM_CHANNEL_1);
```

### 带 DMA 的数据传输
```cpp
// 定时器触发 DMA 传输数据

DMA_HandleTypeDef hdma_tim;
gdut::timer dma_timer(&htim6, &hdma_tim);

dma_timer.init();

// 设置 DMA 回调
dma_timer.set_dma_callback([]() {
    // DMA 传输完成
});

// 启动 DMA 传输
uint32_t data[] = {0x1111, 0x2222, 0x3333};
dma_timer.start_dma(data, sizeof(data) / sizeof(data[0]));
```

## 实际应用示例

### 周期性任务定时器
```cpp
// 创建 10ms 定时中断

gdut::timer task_timer(&htim2);
int tick_count = 0;

// 设置回调
task_timer.set_callback([]() {
    tick_count++;
    if (tick_count >= 10) {
        // 每 100ms 执行一次
        on_periodic_task();
        tick_count = 0;
    }
});

task_timer.init();
task_timer.start_it();
```

### PWM LED 呼吸灯
```cpp
// 使用 PWM 实现 LED 呼吸效果

gdut::timer pwm_timer(&htim1);
pwm_timer.init();

// 创建 PWM 子对象并启动
gdut::timer::timer_pwm pwm(&pwm_timer);
pwm.pwm_start(TIM_CHANNEL_1);

// 在主循环或任务中调整占空比
uint32_t brightness = 0;
bool increasing = true;

void breathing_task() {
    if (increasing) {
        brightness++;
        if (brightness >= 999) increasing = false;
    } else {
        brightness--;
        if (brightness == 0) increasing = true;
    }
    
    pwm.set_duty(TIM_CHANNEL_1, brightness);
    osDelay(10);
}
```

### 频率测量
```cpp
// 使用输入捕获测量外部信号频率

struct frequency_meter {
    gdut::timer &timer;
    gdut::timer::timer_ic ic;
    uint32_t last_capture;
    uint32_t capture_count;
    
    frequency_meter(gdut::timer &t) : timer(t), ic(&t), last_capture(0), capture_count(0) {}
    
    void on_capture() {
        uint32_t current = ic.get_capture(TIM_CHANNEL_1);
        uint32_t delta = current - last_capture;
        last_capture = current;
        capture_count++;
    }
    
    void start() {
        ic.ic_start_it(TIM_CHANNEL_1);
    }
};

gdut::timer freq_timer(&htim9);
freq_timer.init();

frequency_meter meter(freq_timer);
freq_timer.register_capture_callback(TIM_CHANNEL_1, 
    [&meter]() { meter.on_capture(); });

meter.start();
```

### PWM 伺服控制
```cpp
// 使用 PWM 控制伺服电机（舵机）

class servo_controller {
public:
    servo_controller(gdut::timer &timer, uint32_t channel)
        : m_timer(timer), m_pwm(&timer), m_channel(channel) {}
    
    void init() {
        m_timer.init();
        m_pwm.pwm_start(m_channel);
    }
    
    // 设置舵机角度：0-180 度
    void set_angle(uint32_t angle) {
        // 舵机脉宽通常：1ms (0°) - 2ms (180°)
        // 对于 50Hz PWM，周期为 20ms
        uint32_t pulse = 1000 + (angle * 10 / 18);  // 根据实际调整
        m_pwm.set_duty(m_channel, pulse);
    }
    
private:
    gdut::timer &m_timer;
    gdut::timer::timer_pwm m_pwm;
    uint32_t m_channel;
};

gdut::timer servo_timer(&htim3);
servo_controller servo(servo_timer, TIM_CHANNEL_2);

servo.init();
servo.set_angle(90);  // 转到 90 度
```

## 与代码规范的对应
- RAII 自动管理定时器资源
- 禁止复制，支持移动语义
- 基于 `std::function` 的灵活回调机制
- 蛇形命名约定
- 统一的接口设计，支持多种定时器模式

## 回调管理

### 中断回调
- 溢出中断：`set_callback()`
- 比较中断：`set_compare_callback()`
- 捕获中断：`set_capture_callback()`

### DMA 回调
- DMA 完成：`set_dma_callback()`
- DMA 半完成：`set_dma_half_callback()`
- DMA 错误：`set_dma_error_callback()`

## 注意事项/坑点
- 定时器必须在 CubeMX 或手动配置中初始化
- 回调函数运行在中断上下文，应避免耗时操作
- PWM 频率和分辨率由定时器预分频和周期决定
- 输入捕获和编码器模式有特定的引脚和通道要求
- 同一定时器的不同通道可独立配置
- 移动后的源对象不再拥有定时器资源
- DMA 操作需要正确配置 DMA 句柄和中断
- 定时器时钟必须在 RCC 初始化中启用

## 实用工具函数

```cpp
// 获取定时器计数值
uint32_t count = timer.get_counter();

// 重置计数值
timer.set_counter(0);

// 获取定时器频率（基于当前配置）
uint32_t freq = timer.get_frequency();

// 获取定时器周期
uint32_t period = timer.get_period();
```

相关源码：[BSP/bsp_timer.hpp](../../BSP/bsp_timer.hpp)
