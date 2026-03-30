# BSP PS2 通讯模块（bsp_ps2.hpp / bsp_ps2.cpp）

## 简介
该模块用于通过 **SPI** 与 PS2 手柄/2.4G PS2 接收模块进行通讯，提供：

- `ps2_controller`：手柄控制器类
- **握手初始化**（`handshake()`）：尝试进入/退出配置模式并确认设备在线
- **轮询读取**（`poll()`）：读取按键与摇杆状态
- 回调通知（`on_change`）：状态变化时触发

> 模块不绑定具体 SPI 外设（SPI1/2/3）与具体 GPIO，引脚与 SPI 句柄由主工程注入。

---

## 核心设计

### 1) 传输分层：`transfer_frame` vs `transfer_packet`
模块内部把“纯 SPI 传输”和“设备事务（含 ATT/CS 时序）”拆成两层：

- `transfer_frame(tx, rx)`
  - 只负责一次 SPI 全双工收发（9 字节）
  - **不操作** ATT/CS 引脚
  - 通过 `gdut::spi_proxy::transmit_receive` 实现
  - 使用固定超时 `k_spi_timeout = 5ms`

- `transfer_packet(tx, rx)`
  - 完整的 PS2 事务封装：
    1. `ATT = 0`（select）
    2. `delay_ms(1)`
    3. `transfer_frame(...)`
    4. `ATT = 1`（release）
    5. `delay_ms(1)`
  - 对回包做最基本有效性判断：若整帧全 `0xFF` 或全 `0x00`，认为无效（返回 `false`）

### 2) 握手（`handshake()`）
握手用于“尽可能把设备切到可稳定读数据的状态”，并验证设备在线。

当前握手序列（兼容性优先）：

1. **probe**：`0x42` 读一次，确认设备会回包
2. **enter config**：`0x43 ... 0x01` 进入配置模式
3. **set analog + lock（可选）**：`0x44 ...`（部分 2.4G 接收器可能不支持，不强制成功）
4. **exit config（保守）**：`0x43 ... 0x00`（退出配置模式，后续字节全 0）
5. **confirm read**：再次 `0x42` 确认仍可读

`init()` 中会自动调用一次 `(void)handshake()`；握手失败不会阻止后续 `poll()`（便于调试兼容性与接线）。

---

## 如何使用

### 1) 基本接入（阻塞 SPI）
```cpp
#include "bsp_ps2.hpp"
#include "bsp_spi.hpp"

// CubeMX 生成（主工程提供）
extern SPI_HandleTypeDef hspi2;

// 1) 准备 SPI 代理（指向你选择的 hspiX）
static gdut::spi_proxy ps2_spi(&hspi2);

// 2) 准备 ATT 引脚（示例：你自己的 gpio pin 类，需提供 write(bool)）
gdut::gpio_pin att_pin(/* ... */);

// 3) 创建 PS2 控制器
auto ps2 = gdut::make_ps2_controller(
    att_pin,
    [](uint32_t ms) { osDelay(ms); },  // 也可以传 nullptr 使用默认 delay_ms
    &ps2_spi);

// 4) 初始化（会自动尝试握手）
ps2.init();

// 5) 轮询
for (;;) {
  if (ps2.poll()) {
    auto st = ps2.read_state();
    // st.buttons / st.left_x / st.left_y / st.right_x / st.right_y
  }
  osDelay(10);
}
```

### 2) 状态变化回调
```cpp
ps2.on_change([](const gdut::ps2_state& st) {
  // 发生变化才触发
});
```

---

## 数据解析说明（当前实现）
当前实现假设 `poll()` 收到的 9 字节回包布局为：

- `rx[3..4]`：按键位图（16-bit，小端）
- `rx[5]`：left_x
- `rx[6]`：left_y
- `rx[7]`：right_x
- `rx[8]`：right_y

按键位图（`buttons`）的 bit 定义：

- bit0=Select, bit1=L3, bit2=R3, bit3=Start  
- bit4=Up, bit5=Right, bit6=Down, bit7=Left  
- bit8=L2, bit9=R2, bit10=L1, bit11=R1  
- bit12=Triangle, bit13=Circle, bit14=Cross, bit15=Square  

> 不同 2.4G 接收模块的回包格式可能存在差异；如果实测不一致，需要根据实际回包调整 `parse_state()`。

---

## 超时与稳定性
- SPI 传输超时固定为 `5ms`（`k_spi_timeout`）
- 回包若整帧为全 `0xFF` 或全 `0x00`，认为无效（通常代表未选中设备、未接线或模式不匹配）

---

## 注意事项 / 常见问题

### 1) `delay_ms` 的默认实现
若用户未提供 `delay_ms`，内部默认实现为：
- RTOS 运行时：使用 `osDelay(ms)`
- RTOS 未运行时：不延时（`default_delay_ms` 直接返回）

建议主工程在 RTOS 已启动后使用，或显式传入自己的 delay 实现。

### 2) SPI 配置由主工程决定
本模块不负责设置 SPI 的 CPOL/CPHA/分频等参数。若通信失败，需在主工程调整：
- SPI Mode（CPOL/CPHA）
- SPI 频率
- ATT/CS 时序（延时）

---

## 相关源码
- `BSP/bsp_ps2.hpp`
- `BSP/bsp_ps2.cpp`
- `BSP/bsp_spi.hpp`
