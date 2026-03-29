<DOCUMENT filename="bsp_stepper.md"># BSP 步进电机模块（bsp_stepper.hpp）

## 原理
该模块为**42步进电机**（NEMA17/23 常用型号）提供高精度硬件PWM驱动封装。  
使用定时器的**PWM通道直接生成STEP脉冲**（50%占空比），Update中断仅用于精确计数步数，不参与脉冲生成，保证脉冲频率和占空比完全由硬件定时器决定，抖动极低。

DIR和ENABLE仍使用普通GPIO控制，实现方向和使能管理。

## 实现思想
- **硬件PWM + Update中断计数**：PWM通道负责高频STEP脉冲生成，定时器Update中断（每个完整脉冲触发一次）仅负责剩余步数计数，实现**非阻塞定步数运动**。
- **模板参数**：`DirPinType`、`EnablePinType` 直接复用 `gdut::gpio_pin`，编译期确定引脚配置。
- **RAII风格**：继承 `uncopyable`，禁止拷贝，避免硬件资源冲突。

## 如何使用

### 1. CubeMX 配置要求（必须满足）
1. STEP引脚连接到**定时器的PWM输出通道**（GPIO Alternate Function模式）。
2. 定时器时钟建议配置为 **1MHz**（PSC = APB时钟 / 1M - 1），方便速度计算。
3. 必须**开启Update Interrupt**（NVIC）。
4. DIR 和 ENABLE 引脚配置为普通 **GPIO Output PP**。

### 2. 基础使用示例
```cpp
#include "bsp_stepper.hpp"
#include "bsp_gpio_pin.hpp"
#include "bsp_timer.hpp"

// 定义引脚（使用 gpio_pin 模板）
using dir_pin = gdut::gpio_pin<gdut::gpio_port::B,
                               GPIO_InitTypeDef{.Pin = GPIO_PIN_0,
                                                .Mode = GPIO_MODE_OUTPUT_PP,
                                                .Pull = GPIO_NOPULL,
                                                .Speed = GPIO_SPEED_FREQ_HIGH}>;

using enable_pin = gdut::gpio_pin<gdut::gpio_port::B,
                                  GPIO_InitTypeDef{.Pin = GPIO_PIN_1,
                                                   .Mode = GPIO_MODE_OUTPUT_PP,
                                                   .Pull = GPIO_NOPULL,
                                                   .Speed = GPIO_SPEED_FREQ_HIGH}>;

// CubeMX 生成的定时器句柄（已配置为PWM）
extern TIM_HandleTypeDef htim3;

// 创建步进电机对象（使用TIM3的通道1）
dir_pin    motor_dir;
enable_pin motor_enable;
gdut::timer step_timer(&htim3);

gdut::pwm_stepper_motor<dir_pin, enable_pin> stepper(
    motor_dir, motor_enable, step_timer, TIM_CHANNEL_1);

#控制示例
void stepper_demo() {
    stepper.enable(true);                    // 使能电机（低电平有效）
    stepper.set_direction(true);             // 正转

    // 持续恒速运行（5000 steps/s）
    stepper.set_speed(5000);

    osDelay(2000);                           // 运行2秒

    // 运动指定步数（非阻塞，自动停止）
    stepper.move_steps(800, 3000);           // 800步，3000 steps/s

    while (stepper.is_moving()) {
        osDelay(10);                         // 可在其他任务中查询状态
    }

    stepper.disable();                       // 关闭使能
}

#高级用法
// 立即停止
stepper.stop();

// 反转方向
stepper.set_direction(false);

// 只改变速度（不影响当前剩余步数）
stepper.set_speed(8000);

// 查询运动状态
if (stepper.is_moving()) {
    // 正在运动中
}