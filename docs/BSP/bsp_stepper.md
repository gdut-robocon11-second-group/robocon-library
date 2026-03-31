# BSP 步进电机模块（bsp_stepper.hpp）

## 原理
该模块为**42步进电机**（NEMA17/23 常用型号）提供高精度硬件PWM驱动封装。  
使用定时器的**PWM通道直接生成STEP脉冲**（约50%占空比），Update中断仅用于精确计数步数，不参与脉冲生成，保证脉冲频率和占空比完全由硬件定时器决定，抖动极低。

DIR和ENABLE仍使用普通GPIO控制，实现方向和使能管理。

## 实现思想
- **硬件PWM + Update中断计数**：PWM通道负责高频STEP脉冲生成，定时器Update中断（每个完整脉冲触发一次）仅负责剩余步数计数，实现**非阻塞定步数运动**。
- **gpio_proxy 适配**：DIR 和 ENABLE 引脚使用 `gdut::gpio_proxy` 代理对象（基于 `bsp_gpio_pin.hpp`）。
- **RAII风格**：继承 `uncopyable`，禁止拷贝，避免硬件资源冲突。
- **时钟自适应**：不再假设定时器时钟为1MHz，能自动根据实际APB1/APB2时钟计算PSC和ARR，支持任意定时器时钟频率。

## 如何使用

### 1. CubeMX 配置要求（必须满足）
1. STEP引脚连接到**定时器的PWM输出通道**（GPIO Alternate Function模式）。
2. **必须开启Update Interrupt**（NVIC）。
3. 推荐开启 **Auto-reload preload (ARPE)** 和对应PWM通道的 **OC Preload (OCxPE)**，使动态改变速度更平滑。
4. DIR 和 ENABLE 引脚配置为普通 **GPIO Output PP**。
5. **不需要**将定时器时钟强制设为1MHz（代码会自动适配）。

### 2. 创建步进电机对象（重要）
```cpp
#include "bsp_stepper.hpp"
#include "bsp_gpio_pin.hpp"
#include "bsp_timer.hpp"

// CubeMX 生成的定时器句柄（已配置为PWM）
extern TIM_HandleTypeDef htim3;

// 创建 gpio_proxy 对象
gdut::gpio_proxy motor_dir(GPIOA, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP);
gdut::gpio_proxy motor_enable(GPIOA, GPIO_PIN_1, GPIO_MODE_OUTPUT_PP);

gdut::timer step_timer(&htim3);

// 创建步进电机对象（关键参数：最后一个 bool 表示是否为 APB2 定时器）
gdut::pwm_stepper_motor stepper(
    motor_dir, 
    motor_enable, 
    step_timer, 
    TIM_CHANNEL_1, 
    false   // false = APB1（如TIM2~TIM7），true = APB2（如TIM1、TIM8）
);

#基础控制示例
void stepper_demo() {
    // 初始化GPIO（如果你的 gpio_proxy 需要手动初始化）
    motor_dir.initialize();
    motor_enable.initialize();

    stepper.enable(true);                    // 使能电机（低电平有效）
    stepper.set_direction(true);             // 正转（顺时针）

    // 持续恒速运行
    stepper.set_speed(5000);                 // 5000 steps/s

    // 运动指定步数（非阻塞，自动停止）
    stepper.move_steps(1600, 3000);          // 1600步，3000 steps/s

    while (stepper.is_moving()) {
        osDelay(10);                         // 可在其他任务中轮询
    }

    stepper.disable();                       // 关闭使能
}

#高级用法
// 立即停止
stepper.stop();

// 改变方向
stepper.set_direction(false);

// 只改变速度（不影响当前剩余步数）
stepper.set_speed(8000);

// 查询状态
if (stepper.is_moving()) { ... }
if (stepper.is_enabled()) { ... }

uint32_t remain = stepper.get_remaining_steps();