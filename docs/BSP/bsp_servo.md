# BSP 舵机模块（bsp_servo.hpp）

## 原理
舵机本质上是“接收固定周期 PWM 脉冲后，转到对应角度”的执行器。这个模块把“角度”和“脉宽”之间的关系封装起来，让上层不用直接操作定时器比较寄存器。

和直接写 `CCR` 相比，这种封装有两个好处：
- 上层逻辑更容易理解，写 `set_angle(90)` 比写 `set_pulse(2500)` 更直观
- 角度范围、默认位置、运动回调都能统一管理，不容易在多个地方写出互相冲突的参数

## 核心设计

### 结构体 `ServoConfig`
`ServoConfig` 用来描述“这只舵机怎么映射”：
- `min_pulse` / `max_pulse`：角度 0° 和 180° 对应的 PWM 比较值
- `min_angle` / `max_angle`：可用角度范围，不让舵机被命令到超出机械极限的位置
- `default_angle`：构造后默认记录的初始角度

### 类 `servo`
`servo` 主要做三件事：
- 把角度换算成脉宽，再写进定时器
- 提供平滑移动接口，方便避免“瞬间跳角”
- 提供回调机制，把“设置完成”和“错误”通知给上层

### 设计优点
- **语义清晰**：上层写角度就能控制，不需要知道具体计数值
- **可调性强**：不同舵机只要改 `ServoConfig` 就能复用
- **适合教学**：新同学更容易理解“角度 → PWM → 舵机转动”这条链路

### 局限性
- `set_angle()` 触发的是“命令写入完成”，不是“舵机物理到位完成”
- `move_smooth()` 是阻塞式的，调用期间会一直等待
- 角度映射默认按 0~180° 线性处理，不能自动适配所有非线性舵机

## 如何使用

### 创建舵机对象
```cpp
#include "bsp_servo.hpp"

extern TIM_HandleTypeDef htim4;

auto delay_cb = [](uint16_t ms) { osDelay(ms); };
gdut::servo servo(&htim4, TIM_CHANNEL_1, delay_cb);
```

### 直接设置角度
```cpp
servo.set_angle(90);
servo.set_angle(45);
servo.set_angle(135);
```

### 平滑移动
```cpp
servo.move_smooth(120, 2, 20);
```

### 直接设置脉宽
```cpp
servo.set_pulse(2500);
```

### 读取和修改配置
```cpp
auto config = servo.get_config();
config.default_angle = 60;
servo.update_config(config);
```

### 注册回调
```cpp
servo.register_move_complete_callback([](uint8_t angle) {
    printf("servo angle = %u\n", angle);
});

servo.register_error_callback([](HAL_StatusTypeDef err) {
    printf("servo error = %d\n", static_cast<int>(err));
});
```

## 实际应用示例

### 启动后回到默认姿态
```cpp
servo.set_angle(servo.get_config().default_angle);
```

### 机械臂缓慢抬起
```cpp
if (servo.move_smooth(120, 1, 15) != HAL_OK) {
    // 这里可以记录错误或者停止动作
}
```

### 云台微调
```cpp
uint8_t current = servo.get_angle();
if (current < 180) {
    servo.set_angle(current + 1);
}
```

## 使用建议
- 构造函数之后，最好尽快调用一次 `set_angle()`，这样舵机状态更明确
- 如果在 RTOS 任务里使用 `move_smooth()`，要明白它会阻塞当前任务
- 回调适合做日志、状态通知，不适合放太重的业务逻辑

## 注意事项
- 舵机电源不要直接从 MCU 引脚供电，电流通常不够
- PWM 频率和定时器配置要与舵机要求一致，常见是 50Hz
- `min_pulse` 和 `max_pulse` 不要乱设，否则可能把舵机顶到机械极限
- `set_pulse()` 属于“直接控制”，适合调试，不适合初学者默认使用

## 与直接写 PWM 的对比
### 这种封装的好处
- 更容易维护
- 更容易复用到不同舵机
- 更不容易把脉宽写错

### 直接写 PWM 的好处
- 最灵活
- 更适合底层调试和特殊舵机

## 总结
如果你的目标是“让上层只关心角度”，就优先使用 `set_angle()`；如果你在调试硬件参数、要手动标定行程，才考虑直接用 `set_pulse()`。

## 与代码规范的对应
- 类只负责“角度控制 + 脉宽映射 + 回调”
- 参数和配置结构体分离，方便后续扩展
- 接口命名直接表达动作，符合蛇形命名风格
- 通过回调对象传递行为，不依赖裸函数指针

## 注意事项/坑点
- ⚠️ **`set_angle()` 不等于到位**：它只是把控制命令写进去
- ⚠️ **`move_smooth()` 会阻塞**：不要在不合适的任务里长时间调用
- ⚠️ **PWM 频率要对**：常见舵机一般是 50Hz
- ⚠️ **行程别顶死**：`min_pulse` / `max_pulse` 设错会伤机构
- ⚠️ **供电要足**：舵机电流大，别从 MCU 小电源硬拉

## 与直接写比较寄存器的对比

| 特性 | `servo` | 直接写 CCR |
|------|---------|------------|
| 可读性 | ✅ 高 | ❌ 一般 |
| 调试方便 | ✅ 高 | ✅ 高 |
| 复用性 | ✅ 高 | ❌ 低 |
| 极限灵活性 | ❌ 较低 | ✅ 最高 |

## 调试技巧

### 先验证角度映射
```cpp
servo.set_angle(0);
servo.set_angle(90);
servo.set_angle(180);
```

### 再看脉宽是否变化
```cpp
printf("pulse=%lu\n", static_cast<unsigned long>(servo.get_current_pulse()));
```

### 如果转向不对
- 检查舵机安装方向
- 检查 `min_pulse` / `max_pulse` 是否和实际舵机匹配
- 检查 PWM 周期是不是舵机要求的频率

相关源码：[Middlewares/robocon-library/BSP/bsp_servo.hpp](../BSP/bsp_servo.hpp)
