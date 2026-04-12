# BSP 红外传感器模块（bsp_ir.hpp）

## 原理
这个模块把 5 路巡线红外和 1 路避障红外统一封装起来，方便小车在同一个接口下读取传感器状态。

它的核心特点是“简单、直接、好理解”：
- 传感器本质上就是 GPIO 输入
- 初始化时统一配置为上拉输入
- 读取时直接从引脚电平判断状态

## 核心设计

### 类 `ir_sensor`
`ir_sensor` 默认管理 6 个输入：5 路巡线 + 1 路避障。

它提供三个最常用的接口：
- `read_line(channel)`：读单路巡线状态
- `read_all_lines()`：一次性读出 5 路巡线位图
- `read_obstacle()`：读避障状态

### 设计优点
- **上手快**：逻辑非常直观，适合新同学理解 GPIO 输入
- **调用轻量**：读取就是一次 HAL 读引脚，没有复杂中间层
- **适合巡线**：位图返回方式很适合写巡线判断逻辑

### 设计缺点
- 没有做滤波，电平抖动要靠硬件或上层处理
- 只适合离散状态，不适合直接拿来做精细距离测量
- “高电平代表白线还是黑线”要看你传感器板子的输出逻辑，不能一概而论

## 如何使用

### 创建并初始化
```cpp
#include "bsp_ir.hpp"

std::array<GPIO_TypeDef *, 5> line_ports = {GPIOA, GPIOA, GPIOB, GPIOB, GPIOC};
std::array<uint16_t, 5> line_pins = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_0};

gdut::ir_sensor ir(line_ports, line_pins, GPIOD, GPIO_PIN_2);
ir.init();
```

### 读取单路状态
```cpp
bool left = ir.read_line(0);
bool center = ir.read_line(2);
bool right = ir.read_line(4);
```

### 读取整排巡线状态
```cpp
uint8_t mask = ir.read_all_lines();
// bit0~bit4 分别对应 5 路传感器
```

### 读取避障状态
```cpp
if (ir.read_obstacle()) {
    stop_motion();
}
```

## 实际应用示例

### 巡线决策
```cpp
uint8_t mask = ir.read_all_lines();

if (mask == 0b00100) {
    go_straight();
} else if (mask & 0b11000) {
    turn_left();
} else if (mask & 0b00011) {
    turn_right();
} else {
    search_line();
}
```

### 避障保护
```cpp
if (ir.read_obstacle()) {
    stop_motion();
}
```

## 为什么这样设计

### 和“一个传感器一个类”相比
#### 优点
- 统一管理，代码更整齐
- 巡线场景更方便，能直接拿到 5 位状态

#### 缺点
- 灵活性不如完全拆开，每一路都按统一结构处理

## 注意事项
- 初始化前请先在 CubeMX 中把 GPIO 配成输入上拉
- `read_line(channel)` 只接受 0~4，传错索引会直接返回 `false`
- 有些模块是“低电平有效”，有些是“高电平有效”，一定要先看硬件说明
- 如果传感器安装高度太高或太低，读数会很不稳定

## 常见坑
- **坑 1：** 以为所有巡线模块都是同样逻辑，结果判断反了
- **坑 2：** 忘了共地，GPIO 读数全是乱的
- **坑 3：** 没考虑环境光，导致白天和夜晚表现不一样

## 总结
这个模块适合做“巡线和基础避障”的第一层输入。如果后面要做更复杂的车道识别或滤波，建议在上层再封装一层判定逻辑。

## 与代码规范的对应
- 用固定数组表示固定数量的传感器，避免魔法数字散落
- 类职责单一，只负责 GPIO 读取和初始化
- 接口命名直接对应行为，容易读懂
- 默认不拷贝、不复制硬件语义

## 注意事项/坑点
- ⚠️ **电平含义要先确认**：不同模块可能是高电平有效或低电平有效
- ⚠️ **安装高度会影响识别**：离地太高或太低都可能导致误判
- ⚠️ **环境光干扰**：强光、反光地面都会影响红外检测
- ⚠️ **共地很重要**：传感器板和 MCU 不共地会读出乱值
- ⚠️ **索引范围**：`read_line(channel)` 只接受 0~4

## 与“分别读每个 GPIO” 的对比

| 特性 | `ir_sensor` | 逐个直接读 GPIO |
|------|-------------|------------------|
| 代码整洁度 | ✅ 高 | ❌ 低 |
| 巡线位图支持 | ✅ 直接支持 | ❌ 需要自己拼 |
| 灵活性 | ✅ 中等 | ✅ 最高 |
| 新手友好度 | ✅ 高 | ❌ 一般 |

## 调试技巧

### 先看单路电平
```cpp
printf("L0=%d L1=%d L2=%d L3=%d L4=%d\n",
    ir.read_line(0), ir.read_line(1), ir.read_line(2),
    ir.read_line(3), ir.read_line(4));
```

### 再看位图
```cpp
uint8_t mask = ir.read_all_lines();
printf("mask=0x%02X\n", mask);
```

### 对照小车位置调参
- 让车停在黑线正上方看中间位是否触发
- 左右偏一点，看两侧是否能正确变化
- 通过阈值和安装高度一起调，不要只改代码

相关源码：[Middlewares/robocon-library/BSP/bsp_ir.hpp](../BSP/bsp_ir.hpp)
