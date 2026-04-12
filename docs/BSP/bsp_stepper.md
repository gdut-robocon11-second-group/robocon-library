# BSP 步进电机模块（bsp_stepper.hpp）

## 原理
该模块为 **42步进电机**（NEMA17/23 等常用型号）提供高精度硬件PWM驱动封装。

使用定时器的 **PWM通道直接生成STEP脉冲**（约50%占空比），  
Update中断仅用于精确计数剩余步数，不参与脉冲生成。因此脉冲频率和占空比完全由硬件定时器决定，抖动极低。

DIR 和 ENABLE 仍使用普通GPIO控制，实现方向和使能管理。

## 实现思想
- **硬件PWM + Update中断计数**：PWM通道负责高频STEP脉冲生成，定时器Update中断（每个完整脉冲触发一次）仅负责剩余步数计数，实现**非阻塞定步数运动**。
- **gpio_proxy 适配**：DIR 和 ENABLE 引脚使用 `gdut::gpio_proxy` 代理对象（基于 `bsp_gpio_pin.hpp`）。
- **RAII风格**：继承 `uncopyable`，禁止拷贝，避免硬件资源冲突。
- **固定1us分辨率**：代码定时器计数频率为 **1MHz**（即1us分辨率），计算简单直观，适合大多数应用。

## 如何使用

### 1. CubeMX 配置要求（必须满足）
1. STEP引脚连接到**定时器的PWM输出通道**（GPIO Alternate Function模式）。
2. **定时器计数频率配置为 1MHz**（即分辨率为 1us），推荐通过设置合适的 PSC 实现。
3. **必须开启 Update Interrupt**（NVIC）。
4. 推荐开启 **Auto-reload preload (ARPE)** 和对应PWM通道的 **OC Preload (OCxPE)**，使动态改变速度更平滑。
5. DIR 和 ENABLE 引脚配置为普通 **GPIO Output PP**。

### 2. 创建步进电机对象
```cpp
#include "bsp_stepper.hpp"
#include "bsp_gpio_pin.hpp"
#include "bsp_timer.hpp"

// CubeMX 生成的定时器句柄（已配置为PWM模式）
extern TIM_HandleTypeDef htim3;

// 创建 gpio_proxy 对象
gdut::gpio_proxy motor_dir(GPIOA, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP);
gdut::gpio_proxy motor_enable(GPIOA, GPIO_PIN_1, GPIO_MODE_OUTPUT_PP);

gdut::timer step_timer(&htim3);

// 创建步进电机对象
gdut::pwm_stepper_motor stepper(
    motor_dir, 
    motor_enable, 
    step_timer, 
    TIM_CHANNEL_1
);

#控制示例
void stepper_demo() {
    // 初始化GPIO
    motor_dir.initialize();
    motor_enable.initialize();

    stepper.enable(true);                    // 使能电机（低电平有效）
    stepper.set_direction(true);             // 正转（顺时针）

    // 持续恒速运行
    stepper.set_speed(5000);                 // 5000 steps/s

    // 运动指定步数（非阻塞，运动完成后自动停止）
    stepper.move_steps(1600, 3000);          // 1600步 @ 3000 steps/s

    while (stepper.is_moving()) {
        osDelay(10);                         // 可在RTOS任务中轮询
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