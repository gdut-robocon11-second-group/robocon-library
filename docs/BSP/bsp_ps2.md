# BSP PS2 通讯模块（bsp_ps2.hpp / bsp_ps2.cpp）

## 原理

PS2 手柄/2.4G 接收模块通过 **SPI 协议**与 MCU 通讯，是一种**主从全双工**通讯方式。与 I2C 的点对点寻址不同，PS2 采用 **SPI 同步收发**——每次请求（TX）都必须同时接收（RX）。

设计核心思路：

1. **分层传输**：区分"底层 SPI 收发"和"上层 PS2 事务"
   - 底层：单纯的 9 字节全双工收发（`transfer_frame`）
   - 上层：完整事务（ATT 时序 + 延时 + 帧合法性检查）（`transfer_packet`）

2. **握手协议**：PS2 设备支持多种工作模式（模拟、数字、压力等），需通过握手序列将其切到可用状态

3. **非阻塞式轮询**：`poll()` 调用不关心握手成功与否，允许接线或兼容性问题渐进式排查

---

## 核心设计

### 关键类与接口

| 类/方法 | 职责 |
|--------|------|
| `ps2_controller` | 手柄状态管理器；保存当前状态、握手标志、轮询次数统计 |
| `init()` | 初始化；自动尝试握手；握手失败不影响后续 `poll()` |
| `handshake()` | 握手协议序列；将设备从未知状态切到"可读按键"状态 |
| `poll()` | 单次轮询；返回 true 表示成功读到数据（回包有效）；通常每 10ms 调用一次 |
| `read_state()` | 读取最新状态（按键位图、摇杆模拟值）；只返回存储的值，不阻塞 |
| `on_change(callback)` | 状态有变化时触发回调；可用于事件响应而非持续轮询 |

### 分层传输

```
应用层      on_change()  →  poll()  →  read_state()
            ↓
事务层      transfer_packet()  [ATT 时序 + 延时 + 帧检查]
            ↓
传输层      transfer_frame()   [纯 SPI 全双工 9 字节]
            ↓
HAL         gdut::spi_proxy::transmit_receive()
```

**为什么分层？**

- **transfer_frame**：只做 SPI 底层，便于复用或替换（如 USART 模拟 SPI）
- **transfer_packet**：加入 ATT/CS 时序和帧有效性判断，隐藏细节复杂度
- **poll()**：上层无需关心 ATT 或超时，只需定期调用

### 握手序列

握手用于将 PS2 设备从"上电未知状态"切到"按键可读"状态。当前实现兼容性优先：

```
1. probe        → 0x42         [尝试读一次，验证设备响应]
2. enter cfg    → 0x43 ... 0x01 [进入配置模式]
3. set mode     → 0x44 ...      [设置模拟模式 + 锁定（可选）]
4. exit cfg     → 0x43 0x00 ... [退出配置模式]
5. confirm      → 0x42         [再次读，确认在线]
```

若握手失败（网络抖动、未接线、接线错误、接收器不支持），**不会** 阻止 `poll()`——允许稍后手动重试或通过看日志调试。

### 回包解析

当前实现假设 9 字节回包布局（标准 PS2 格式）：

```
Byte:   0     1     2     3      4      5       6       7       8
      [0xFF] [0x7X] [MMAP] [KEY_L] [KEY_H] [LX] [LY] [RX] [RY]
                     ^               ^
                  padding      按键位图（16-bit 小端）
```

按键位图定义：
- **bit 0-3**：Select, L3, R3, Start
- **bit 4-7**：↑, →, ↓, ←
- **bit 8-11**：L2, R2, L1, R1
- **bit 12-15**：△, ○, ✕, □

摇杆值是直接的 **ADC 采样值**（0～255）：
- 左摇：`rx[5]`=X，`rx[6]`=Y
- 右摇：`rx[7]`=X，`rx[8]`=Y

---

## 如何使用

### 基本接入：初始化与轮询

```cpp
#include "bsp_ps2.hpp"
#include "bsp_spi.hpp"

// CubeMX 生成（主工程提供）
extern SPI_HandleTypeDef hspi2;

// 第 1 步：准备 SPI 代理与 ATT 引脚
static gdut::spi_proxy ps2_spi(&hspi2);
gdut::gpio_proxy att_pin(/* 你的 ATT GPIO pin */);

// 第 2 步：创建 PS2 控制器
auto ps2 = gdut::make_ps2_controller(
    att_pin,
    [](uint32_t ms) { osDelay(ms); },  // delay 回调，可传 nullptr 用默认
    &ps2_spi);

// 第 3 步：初始化（自动握手）
ps2.init();

// 第 4 步：主循环轮询
for (;;) {
  if (ps2.poll()) {
    auto st = ps2.read_state();
    printf("Buttons: 0x%04X, LX=%d, LY=%d, RX=%d, RY=%d\n",
           st.buttons, st.left_x, st.left_y, st.right_x, st.right_y);
  }
  osDelay(10);  // 通常每 10~20ms 轮询一次
}
```

### 事件响应：使用 on_change 回调

```cpp
// 只在状态变化时处理（减少计算量）
ps2.on_change([](const gdut::ps2_state& st) {
  // 按钮松开
  if (st.buttons & PS2_BUTTON_CROSS) {
    printf("◯ 按下\n");
  }
  
  // 摇杆移动
  if (st.left_x > 200 || st.left_x < 50) {
    printf("左摇杆 X=%d（超出中心）\n", st.left_x);
  }
});
```

### 手动重新握手

```cpp
// 若发现设备失连，可手动重握手
if (!ps2.poll()) {
  printf("读取失败，尝试重新握手...\n");
  ps2.handshake();
  osDelay(100);
}
```

---

## 实际应用示例

### 场景 1：车辆遥控

```cpp
// 在 Task 中定期轮询手柄
void remote_control_task(void *arg) {
  auto ps2 = gdut::make_ps2_controller(/* ... */);
  ps2.init();
  
  struct MotorCmd {
    int16_t left_pwm, right_pwm;
  } cmd{};
  
  for (;;) {
    if (ps2.poll()) {
      auto st = ps2.read_state();
      
      // 左摇杆控制速度与转向
      int16_t speed = (int16_t)st.left_y - 128;  // [-128, 127]
      int16_t turn = (int16_t)st.left_x - 128;
      
      cmd.left_pwm = speed + turn;
      cmd.right_pwm = speed - turn;
      
      // 调用驱动层接口
      motor_left.set_pwm(cmd.left_pwm);
      motor_right.set_pwm(cmd.right_pwm);
    }
    osDelay(10);
  }
}
```

### 场景 2：模式切换

```cpp
// 用不同按键切换工作模式
ps2.on_change([](const gdut::ps2_state& st) {
  if (st.buttons & PS2_BUTTON_TRIANGLE) {
    set_mode(MODE_AUTO_LINE);
  }
  if (st.buttons & PS2_BUTTON_CROSS) {
    set_mode(MODE_MANUAL);
  }
  if (st.buttons & PS2_BUTTON_SQUARE) {
    set_mode(MODE_CALIBRATE);
  }
});
```

---

## 为什么这样设计

### 设计选择对比

| 方案 | 优点 | 缺点 |
|------|------|------|
| **本实现**：分层 + 非阻塞轮询 | 灵活；握手失败不卡主；易于调试 | 需定期调用 poll；需检查返回值 |
| 方案 B：阻塞握手 + 中断模式 | 事件驱动；省轮询 | 握手卡住会死机；接线问题难排查 |
| 方案 C：完全事件驱动 | 零轮询开销 | 回调上下文有限；状态管理复杂 |

**为什么选择分层？**

1. **传输层独立性**：`transfer_frame` 只做底层收发，可复用、测试、替换（如 USART bit-bang SPI）
2. **握手失败容错**：允许握手失败，`poll()` 仍可继续尝试——便于调试接线问题
3. **延迟可控**：轮询频率由应用决定（10ms/20ms/50ms），不被中断打扰

**为什么不用中断？**

- PS2 无中断信号（只有 SPI 时钟），必须主动轮询
- 中断模式需靠 GPIO 变化（如 ATT 脉冲），但多数接收器不提供

### 回包有效性判断

回包整帧为全 `0xFF` 或全 `0x00` 时，认为无效（`transfer_packet` 返回 false）。这通常代表：

- 未接线或接线松动
- ATT/CS 时序错误
- SPI 配置不当（CPOL/CPHA 反了）
- 设备未初始化或故障

---

## 注意事项

### ⚠️ SPI 配置由主工程决定
本模块不设置 SPI 的 CPOL/CPHA/分频。若通信失败：
1. 检查 CubeMX SPI 配置（Mode/Frequency）
2. 尝试降低 SPI 频率（PS2 一般 1~2 MHz 即可）
3. 调整 ATT 延迟（当前 `delay_ms(1)`）

### ⚠️ delay_ms 的默认实现
若未提供 delay 回调，内部使用默认实现：
- RTOS 运行时：`osDelay(ms)`
- RTOS 未启动：无延时（返回）

建议在 RTOS 启动后使用，或显式传入自己的 delay。

### ⚠️ 握手失败不影响 poll()
握手失败时，`handshake()` 返回 false，但 `poll()` 仍可调用——允许稍后重试或看日志排查。

### ⚠️ 回包格式因设备而异
部分 2.4G 接收器的回包顺序或位定义可能不同。若实测数据对不上：
1. 打印整个 9 字节回包：`for (int i=0; i<9; i++) printf("%02X ", rx[i]);`
2. 对照设备文档调整 `parse_state()` 中的索引

### ⚠️ 摇杆中心值不一定是 128
不同设备的摇杆中心值可能在 120～135 之间。应用层应做死区处理：
```cpp
auto centered_x = (st.left_x > 110 && st.left_x < 145) ? 128 : st.left_x;
```

---

## 常见坑

### 坑 1：忘记调用 poll()
**症状**：状态总是不变，按键无反应  
**原因**：只调用了 `init()`，没有定期 `poll()`  
**解决**：在主循环或 Task 中每 10～20ms 调用一次 `poll()`

```cpp
// ❌ 错误
ps2.init();
auto st = ps2.read_state();  // 永不更新

// ✅ 正确
ps2.init();
for (;;) {
  ps2.poll();  // 定期轮询
  auto st = ps2.read_state();
  osDelay(10);
}
```

### 坑 2：回包全 0xFF 或全 0x00
**症状**：`poll()` 总是返回 false  
**原因**：通常是 SPI 配置、ATT 时序或接线问题  
**调试**：
```cpp
// 在 transfer_packet 中加 printf，看实际回包
printf("RX: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
       rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7], rx[8]);

// 若全是 0xFF，说明 SPI MISO 未被拉低，检查：
// - 接线 MOSI/MISO/CLK/ATT
// - SPI 极性（CPOL/CPHA）
// - ATT 延迟（可试试改成 delay_ms(5)）
```

### 坑 3：握手后仍无数据
**症状**：`handshake()` 返回 true，但 `poll()` 返回 false  
**原因**：握手序列不完整或设备不支持该模式  
**解决**：
```cpp
// 手动重试握手
for (int i = 0; i < 3; i++) {
  if (ps2.handshake()) break;
  osDelay(100);
}

// 若仍失败，查看是否需要调整握手序列
// （见 bsp_ps2.cpp 中的握手字节序列）
```

### 坑 4：摇杆值在跳跃，不稳定
**症状**：`st.left_x` / `st.left_y` 不停抖动  
**原因**：可能是 SPI 噪声、接线质量差、或设备故障  
**解决**：
```cpp
// 在应用层做简单滤波（如中值滤波或低通滤波）
struct {
  uint8_t hist[3];  // 历史值环形缓冲
  int idx = 0;
} lx_filter;

uint8_t get_filtered_lx(uint8_t raw_lx) {
  lx_filter.hist[lx_filter.idx] = raw_lx;
  lx_filter.idx = (lx_filter.idx + 1) % 3;
  // 简单平均
  return (lx_filter.hist[0] + lx_filter.hist[1] + lx_filter.hist[2]) / 3;
}
```

---

## 总结

**ps2 模块三个核心点**：

1. **分层设计**：底层 SPI 收发 → 中层 ATT 时序 → 上层状态管理
   - 便于复用、测试、排查问题

2. **非阻塞轮询**：定期 `poll()`，握手失败不卡住
   - 便于接线/兼容性问题的渐进式调试

3. **回包有效性**：全 0xFF/0x00 认为无效，帮助快速发现硬件问题
   - 若实际数据不同，需检查 SPI/ATT 配置或设备兼容性

**使用建议**：

- 初始化时允许握手失败，看 log 诊断
- 主循环定期 `poll()`（10～20ms），不要忘记
- 摇杆值需要应用层死区/滤波，不要直接用原始值
- 若多次握手失败，check SPI 频率/CPOL/CPHA 以及接线

---

## 与代码规范的对应

- **不复制策略**：`ps2_controller` 禁用拷贝构造（阻止 SPI 句柄重复持有），遵循 RAII 原则
- **命名约定**：`transfer_frame` / `transfer_packet` 清晰分层；`on_change` 遵循回调命名
- **回调类型**：使用 `gdut::function<void(const ps2_state&)>`，与项目其他模块一致

---

## 调试技巧

### 技巧 1：打印回包进行对比
```cpp
// 在 transfer_packet 返回前，根据调试开关打印
#ifdef PS2_DEBUG_PACKET
  printf("[PS2] TX: ");
  for (int i = 0; i < 9; i++) printf("%02X ", tx[i]);
  printf("| RX: ");
  for (int i = 0; i < 9; i++) printf("%02X ", rx[i]);
  printf("\n");
#endif
```

### 技巧 2：验证握手成功
```cpp
// 握手后立即轮询几次，看回包是否稳定
bool verify_handshake() {
  uint8_t rx_prev[9];
  for (int i = 0; i < 5; i++) {
    if (!ps2.poll()) return false;
    // 比对回包，若变化太大说明设备不稳定
    osDelay(20);
  }
  return true;
}
```

### 技巧 3：隔离硬件问题
```cpp
// 临时修改 SPI 配置，尝试不同的时序
// 如改 hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
// 然后重新握手，看是否改善

// 或尝试不同的 ATT 延迟
// delay_ms(1) → delay_ms(5) → delay_ms(10)
```

### 技巧 4：单步测试 transfer_frame
```cpp
// 若怀疑中层 transfer_packet 有问题，可直接测试底层
uint8_t tx_test[9] = {0x01, 0x42, /* ... */};
uint8_t rx_test[9];
ps2_spi.transmit_receive(tx_test, rx_test, 9, 100);  // 不走 ATT 时序
printf("Raw SPI result: ");
for (int i = 0; i < 9; i++) printf("%02X ", rx_test[i]);
```

---

## 相关源码

- [bsp_ps2.hpp](../../../BSP/bsp_ps2.hpp)
- [bsp_ps2.cpp](../../../BSP/bsp_ps2.cpp)
- [bsp_spi.hpp](../../../BSP/bsp_spi.hpp)
