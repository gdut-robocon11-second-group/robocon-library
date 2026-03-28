# BSP PS/2 控制器驱动（bsp_ps2.hpp / bsp_ps2.cpp）

## 原理

本驱动用于与 PS/2 手柄或通过转换模块（PS/2 → SPI）的外设通信。驱动以 `spi_proxy` 为传输层，通过 `ATT` GPIO 控制帧边界并使用 SPI 全双工传输一个字节（TX/RX）。

驱动解析手柄返回的数据帧并将按钮与模拟摇杆值转换为 `ps2_state` 供上层使用。

## 核心设计

- 强制使用硬件 SPI（`spi_proxy::transmit_receive`），提高时序可靠性；去除软件 bit‑bang 回退。
- `pins_interface` 注入 ATT 控制与延时函数，保持与仓库 GPIO 封装的兼容性。
- 轮询函数 `poll()` 发送标准轮询帧并解析返回字节到 `ps2_state`。

## API 说明

- 类型：`gdut::ps2_controller`
- 构造函数：

  `ps2_controller(pins_interface pins, spi_proxy *spi)` — 必须传入已初始化并配置好的 `spi_proxy*`；`pins_interface.set_att` 用于控制 ATT（/CS）。

- 主要方法：

  - `void init()`：初始化 ATT/CMD/CLK 的空闲态。
  - `bool poll()`：发送轮询命令并解析回复，返回是否状态发生变化。
  - `ps2_state read_state() const`：获取最新解析的手柄状态。
  - `void on_change(std::function<void(const ps2_state&)>)`：注册状态变化回调。

- `pins_interface` 字段：

  - `set_att(bool)`：控制 ATT（必须实现）。
  - `write_cmd(bool)`, `set_clk(bool)`, `read_dat()`：SPI-only 模式下不使用，但保留接口。
  - `delay_us(uint32_t)`：提供微秒级延时函数。

## 如何使用（简要示例）

1. 初始化 HAL 层的 `SPI_HandleTypeDef`（例如 `hspi1`），并创建 `spi_proxy`：

```cpp
extern SPI_HandleTypeDef hspi1; // 在 HAL 初始化代码中定义
gdut::spi_proxy spi(&hspi1);
spi.set_first_bit(gdut::spi_first_bit::lsb);
spi.set_mode(gdut::spi_mode::master);
spi.set_data_size(gdut::spi_data_size::bits8);
spi.set_baud_rate_prescaler(gdut::spi_baud_rate_prescaler::div64);
spi.init();
```

2. 定义 `pins_interface` 并绑定 `ATT` 对应的 GPIO（使用仓库新的 `gpio_proxy` API）：

```cpp
GPIO_InitTypeDef att_init{};
att_init.Pin = GPIO_PIN_4;
att_init.Mode = GPIO_MODE_OUTPUT_PP;
att_init.Pull = GPIO_NOPULL;
att_init.Speed = GPIO_SPEED_FREQ_LOW;

gdut::gpio_proxy att(gdut::get_gpio_port_ptr(gdut::gpio_port::A), &att_init);
att.initialize();

ps2_controller::pins_interface pins;
pins.set_att = [&att](bool v){ att.write(v); };
pins.delay_us = [](uint32_t us){
  using namespace gdut;
  if (us == 0) return;
  auto start = steady_clock::now();
  auto target = start + steady_clock::duration(us);
  if (us >= 2000) std::this_thread::sleep_for(std::chrono::microseconds(us - 1000));
  while (steady_clock::now() < target) { }
};
```

3. 构造并使用 `ps2_controller`：

```cpp
gdut::ps2_controller ctl(std::move(pins), &spi);
ctl.init();
while(true){
  if (ctl.poll()) {
    auto st = ctl.read_state();
    // 处理 st
  }
  HAL_Delay(10);
}
```
