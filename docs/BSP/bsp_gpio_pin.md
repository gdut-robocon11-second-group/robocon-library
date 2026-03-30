# BSP GPIO 引脚模块（bsp_gpio_pin.hpp）

## 概述
当前 GPIO BSP 提供的是运行时代理类 `gdut::gpio_proxy`，对 STM32 HAL 的 GPIO 操作进行了轻量封装。

该类主要职责：
- 保存 `GPIO_TypeDef*` 与 `GPIO_InitTypeDef*`。
- 提供初始化/反初始化接口。
- 提供读写和动态改配置接口。

## 设计要点
- `gpio_proxy` 继承自 `uncopyable`，**禁止拷贝**。
- 支持**移动构造**和**移动赋值**，便于在容器或返回值中转移所有权语义。
- 构造函数与析构函数默认**不自动初始化/反初始化**（源码中调用被注释）。
  - 需要手动调用 `initialize()`。
  - 若需要释放配置，手动调用 `deinitialize()`。

## API 一览

- `initialize()`：调用 `HAL_GPIO_Init(port, init)`。
- `deinitialize()`：调用 `HAL_GPIO_DeInit(port, init->Pin)`。
- `set_mode(gpio_mode)`：更新 `init->Mode` 并重新初始化。
- `set_pull(gpio_pull)`：更新 `init->Pull` 并重新初始化。
- `set_speed(gpio_speed)`：更新 `init->Speed` 并重新初始化。
- `set_pin(gpio_pin)`：更新 `init->Pin` 并重新初始化。
- `set_alternate(uint32_t)`：当前实现会把参数写入 `init->Pull` 后重新初始化（按源码现状记录）。
- `write(bool)`：写 GPIO 电平。
- `read() const`：读取 GPIO 电平。
- `toggle()`：翻转 GPIO 电平。

其中 `gpio_mode`、`gpio_pull`、`gpio_speed`、`gpio_pin` 定义于 `bsp_utility.hpp`，是对 HAL 宏的强类型封装。

## 使用示例

```cpp
#include "bsp_gpio_pin.hpp"
#include "bsp_utility.hpp"

void led_task() {
    static GPIO_InitTypeDef led_init{
        .Pin = GPIO_PIN_5,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Alternate = 0,
    };

    gdut::gpio_proxy led(GPIOA, &led_init);

    led.initialize();
    led.write(true);
    led.toggle();
    bool state = led.read();
    (void)state;

    // 如需释放：
    // led.deinitialize();
}
```

## 注意事项
- 使用前必须开启对应 GPIO 时钟。
- 传入的 `GPIO_InitTypeDef*` 必须在 `gpio_proxy` 生命周期内保持有效。
- 本模块当前不是 RAII 自动管理模式，请显式调用 `initialize()` / `deinitialize()`。
- 若期望设置 AF 复用，请关注 `set_alternate()` 的当前实现是否符合预期。

相关源码：[BSP/bsp_gpio_pin.hpp](../../BSP/bsp_gpio_pin.hpp)
