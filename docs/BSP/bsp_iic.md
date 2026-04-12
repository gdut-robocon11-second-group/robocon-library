# BSP I2C 模块（bsp_iic.hpp）

## 原理
这个模块是对 STM32 HAL I2C 的一层 C++ 封装。它的作用不是“重新发明 I2C”，而是把 HAL 的 C 风格接口包装成更适合项目使用的形式：

- 统一管理一个 `I2C_HandleTypeDef`
- 提供阻塞、中断、存储器读写、从机模式等接口
- 把中断回调集中转发到对象里，方便上层注册 lambda 或函数对象

如果把 I2C 比作“公交车”，那这个类就是“车站管理员”：它不负责传感器逻辑，只负责把数据安全地送到对的地方。

## 核心设计

### 类 `i2c`
`i2c` 直接绑定一个 HAL I2C 句柄，提供：
- `init()` / `deinit()`：初始化和反初始化
- `transmit()` / `receive()`：阻塞模式主机收发
- `transmit_it()` / `receive_it()`：中断模式主机收发
- `mem_write()` / `mem_read()`：寄存器型器件最常用的读写方式
- `slave_*` 接口：从机模式收发
- 回调注册：把 HAL 中断完成事件转发给上层

### 类 `i2c_irq_handler`
它负责把“哪个 I2C 实例触发了中断”这件事分辨清楚，然后调用对应对象注册的回调。

这样设计的好处是：
- 中断入口统一，代码不容易散落一地
- 上层只需要关心“某个传输完成了”，不用管 HAL 的底层细节

### 设计优点
- **功能完整**：主机、从机、阻塞、中断都覆盖了
- **适合传感器**：对 MPU6050、EEPROM、OLED 这类寄存器设备很顺手
- **回调友好**：支持把完成事件转成 C++ 风格函数对象

### 设计缺点
- 接口较多，新同学第一次看会觉得“为什么这么全”
- 中断模式比阻塞模式更灵活，但调试门槛也更高
- 如果回调里做太多事，容易影响中断响应

## 如何使用

### 创建并初始化
```cpp
#include "bsp_iic.hpp"

extern I2C_HandleTypeDef hi2c1;

gdut::i2c bus(&hi2c1);
if (bus.init() != HAL_OK) {
    // 初始化失败
}
```

### 阻塞模式读写
```cpp
uint8_t tx_data[] = {0x01, 0x02};
uint8_t rx_data[2] = {};

bus.transmit(0x50 << 1, tx_data, sizeof(tx_data));
bus.receive(0x50 << 1, rx_data, sizeof(rx_data));
```

### 寄存器读写
```cpp
uint8_t buffer[8] = {};
bus.mem_write(0x68 << 1, 0x6B, I2C_MEMADD_SIZE_8BIT, buffer, sizeof(buffer));
bus.mem_read(0x68 << 1, 0x3B, I2C_MEMADD_SIZE_8BIT, buffer, sizeof(buffer));
```

### 中断模式
```cpp
bus.register_master_rx_cplt_callback([]() {
    printf("rx done\n");
});

bus.receive_it(0x50 << 1, buffer, sizeof(buffer));
```

### 设备就绪检测
```cpp
if (bus.is_device_ready(0x68 << 1) == HAL_OK) {
    // 设备在线
}
```

## 实际应用示例

### 读取 MPU6050
```cpp
#include "bsp_mpu6050.hpp"

gdut::mpu6050 imu(&bus, [](std::uint32_t ms) { osDelay(ms); });
if (imu.init()) {
    auto accel = imu.get_accel();
    auto gyro = imu.get_gyro();
}
```

### 读取 EEPROM
```cpp
uint8_t id = 0;
bus.mem_read(0x50 << 1, 0x00, I2C_MEMADD_SIZE_8BIT, &id, 1);
```

### 作为从机设备
```cpp
bus.enable_listen();
bus.register_slave_rx_cplt_callback([]() {
    // 从机接收完成
});
```

## 为什么这样设计

### 和直接调用 HAL 比较
#### 优点
- 代码更统一
- 传感器驱动更容易复用
- 中断回调管理更清楚

#### 缺点
- 比直接用 HAL 多一层封装
- 有时排查问题需要同时看 HAL 和封装层

## 注意事项
- I2C 地址在 HAL 里通常要传 8 位地址，很多新同学会把 7 位地址直接塞进去
- `mem_read()` / `mem_write()` 的 `mem_addr_size` 一定要和器件手册一致
- 中断回调里不要做太重的业务逻辑，最好只做“置标志”
- 设备没有上拉电阻、线太长或速度太高时，I2C 会非常不稳定

## 常见坑
- **坑 1：** 设备地址左右移错误，表现为一直读不到器件
- **坑 2：** 以为中断模式会自动帮你处理所有异常，实际上错误码还是要查
- **坑 3：** 回调注册了，但没有在对应的 HAL IRQ 里调用 `i2c_irq_handler`

## 总结
如果你在项目里要接很多 I2C 设备，这个模块很适合做统一底座：上层只关心“读什么、写什么”，底层统一处理 HAL 句柄和中断分发。

## 与代码规范的对应
- 句柄集中管理，减少全局变量散落
- 禁止拷贝和移动，避免同一外设被重复封装
- 使用回调对象而不是裸函数指针，便于类型安全传参
- 蛇形命名与现有库保持一致

## 注意事项/坑点
- ⚠️ **地址格式**：HAL 常用 8 位地址，很多设备手册写的是 7 位地址
- ⚠️ **回调别做重活**：中断回调里尽量只置位或转发，不要长时间阻塞
- ⚠️ **存储器地址大小**：`mem_addr_size` 不匹配时，读写会直接失败
- ⚠️ **总线稳定性**：上拉、电平、线长、速度都会影响可靠性
- ⚠️ **IRQ 入口要接好**：只注册回调不调用 `i2c_irq_handler` 没有用

## 与裸 HAL 的对比

| 特性 | `gdut::i2c` | 直接用 HAL |
|------|-------------|------------|
| 接口统一性 | ✅ 高 | ❌ 低 |
| 回调管理 | ✅ 统一封装 | ❌ 分散 |
| 学习成本 | ✅ 中等 | ✅ 低 |
| 复用性 | ✅ 高 | ❌ 一般 |
| 调试灵活性 | ✅ 较高 | ✅ 最高 |

## 调试技巧

### 检查设备是否在线
```cpp
if (bus.is_device_ready(0x68 << 1) != HAL_OK) {
    printf("I2C device not ready\n");
}
```

### 先用阻塞模式验证连线
```cpp
uint8_t id = 0;
bus.mem_read(0x68 << 1, 0x75, I2C_MEMADD_SIZE_8BIT, &id, 1);
printf("WHO_AM_I = 0x%02X\n", id);
```

### 看错误码
```cpp
printf("i2c error = 0x%08lX\n", static_cast<unsigned long>(bus.get_error()));
```

相关源码：[Middlewares/robocon-library/BSP/bsp_iic.hpp](../BSP/bsp_iic.hpp)
