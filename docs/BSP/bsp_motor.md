# BSP 电机状态采集模块（bsp_motor.hpp）

## 原理
这个模块更像“电机状态采集器 + 输出接口”，而不是完整控制器。它做两件事：

- 读取编码器计数，算出当前速度、累计圈数和原始计数
- 输出 PWM 和方向信号，驱动电机转动

它**不负责 PID**，也**不负责融合决策**。这样做的好处是分层清晰：电机驱动层只管“采集和输出”，上层可以自己决定什么时候加 PID、什么时候做速度控制、什么时候做位置融合。

## 核心设计

### 类 `motor`
`motor` 直接绑定四样东西：
- PWM 定时器
- PWM 通道
- 方向 GPIO
- 编码器定时器

然后通过 `refresh_encoder_state(control_period_sec)` 周期性刷新状态。

### 设计优点
- **状态清晰**：当前速度、累计圈数、原始编码器计数都能直接拿到
- **适合后续融合**：上层做控制时，只需要读状态，不用再解析底层计数
- **接口简单**：调用方式比较直观，适合新同学理解

### 设计缺点
- 不是闭环控制器，所以不能单独“把速度稳住”
- 需要调用者自己保证刷新周期，否则速度计算会不准
- 如果上层忘记调用 `refresh_encoder_state()`，状态就不会更新

## 如何使用

### 创建对象
```cpp
#include "bsp_motor.hpp"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

gdut::timer pwm_timer(&htim1);
gdut::timer encoder_timer(&htim2);

gdut::motor motor(&pwm_timer,
                  TIM_CHANNEL_1,
                  GPIOA,
                  GPIO_PIN_8,
                  &encoder_timer,
                  1024.0f);
```

### 刷新编码器状态
```cpp
motor.refresh_encoder_state(0.01f); // 10ms 周期

float speed_rps = motor.get_current_speed();
float revolutions = motor.get_total_revolutions();
uint32_t count = motor.get_current_encoder_count();
```

### 输出使能
```cpp
motor.enable(true);
motor.enable(false);
```

## 实际应用示例

### 在控制周期中刷新状态
```cpp
void control_loop(float dt) {
    motor.refresh_encoder_state(dt);

    float speed = motor.get_current_speed();
    float turns = motor.get_total_revolutions();
    (void)speed;
    (void)turns;
}
```

### 上层融合模块读取状态
```cpp
motor.refresh_encoder_state(dt);
float wheel_speed = motor.get_current_speed();
uint32_t wheel_count = motor.get_current_encoder_count();
```

## 为什么这样设计

### 和“把 PID 写进电机类”相比
#### 优点
- 更容易把控制层、驱动层分开
- 更适合之后加入传感器融合、速度环、位置环
- 更容易调试，因为你能直接看到原始编码器数据

#### 缺点
- 不能直接拿来做闭环控制
- 上层需要自己维护周期和控制逻辑

## 注意事项
- `refresh_encoder_state()` 的周期参数必须真实可靠，最好来自固定控制周期
- 编码器定时器可能回绕，模块已经处理，但前提是定时器配置正确
- `ppr` 一定要填对，不然速度和圈数都会偏差
- 这个模块默认只表示“采集 + 输出”，不要误以为它会自动帮你稳速

## PWM 方向与占空比语义说明
- `set_pwm_duty(duty)` 中，`duty` 会先被限制到 `[-1, 1]`
- 方向只由方向 GPIO 决定：`duty >= 0` 与 `duty < 0` 仅切换方向引脚电平
- PWM 比较值只使用 `abs(duty)` 计算，因此正负号**不改变占空比幅值**
    - 例如：`duty = +0.25` 和 `duty = -0.25`，PWM 幅值都应为 25%，只改变转向
- 这样可以避免“负占空比被反转后变成高占空比”（如 -25% 变成约 75%）的问题
- 若你的电机驱动器是“低电平有效 PWM”或需要反相语义，请在定时器极性/通道配置层处理，而不是在 `motor` 内部翻转比较值

## 常见坑
- **坑 1：** 刷新周期写死但实际周期变了，速度会明显漂移
- **坑 2：** 编码器接线方向和 GPIO 方向逻辑没对上，结果转向看起来反了
- **坑 3：** 误把 `total_revolutions_` 当成“当前圈数”，其实它是累计值

## 总结
如果你想要的是“一个能读编码器、能输出 PWM、但不替你做控制”的电机层，这个模块就很合适；如果你想要的是“直接给定目标速度然后自动稳住”，那应该把 PID 放到更上层。

## 与代码规范的对应
- 一个类只做“电机状态采集 + 输出控制”这一件事
- 禁止拷贝，避免同一电机对象被多处管理
- 使用清晰的状态访问函数，减少成员直接暴露
- 采用蛇形命名，让接口风格和库里其他模块一致

## 注意事项/坑点
- ⚠️ **周期必须真实**：`refresh_encoder_state()` 的周期参数直接影响速度计算
- ⚠️ **编码器回绕**：定时器计数会回绕，不能把原始差值直接当真实增量
- ⚠️ **PPR 要填对**：脉冲数错了，速度和圈数都会跟着错
- ⚠️ **方向逻辑要确认**：GPIO 高低电平和电机正反转必须先对上
- ⚠️ **别把它当控制器**：它不做 PID，别在这里期待“自动稳速”

## 与“电机类内置 PID”的对比

| 特性 | 当前设计 | 内置 PID |
|------|----------|----------|
| 分层清晰度 | ✅ 高 | ❌ 低 |
| 上层融合灵活性 | ✅ 高 | ❌ 一般 |
| 开箱即用 | ❌ 需要上层控制 | ✅ 更快 |
| 调试编码器 | ✅ 更方便 | ❌ 容易混在一起 |

## 调试技巧

### 先看原始计数
```cpp
printf("cnt=%lu\n", static_cast<unsigned long>(motor.get_current_encoder_count()));
```

### 再看速度和累计圈数
```cpp
printf("speed=%.3f rev=%.3f\n",
    motor.get_current_speed(), motor.get_total_revolutions());
```

### 验证方向是否正确
- 手动给小占空比
- 观察 `current_encoder_count_` 是增加还是减少
- 如果方向反了，优先检查 GPIO 方向逻辑和机械接线

相关源码：[Middlewares/robocon-library/BSP/bsp_motor.hpp](../BSP/bsp_motor.hpp)
