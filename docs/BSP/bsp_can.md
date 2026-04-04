# BSP CAN 通讯模块（bsp_can.hpp）

## 原理
该模块对 STM32 HAL 的 CAN 通讯功能进行面向对象的封装，提供基于代理（Proxy）模式的类型安全的 CAN 总线通讯接口。

## 核心设计

### CAN 类型与配置
```cpp
enum class can_type : uint8_t { standard_type = 1, extended_type = 2 };
enum class can_mailbox : uint32_t { mailbox0 = 1, mailbox1 = 2, mailbox2 = 4 };
enum class can_fifo : uint8_t { fifo0 = 0, fifo1 = 1 };
```

### 基类 `base_can_proxy`
- 全局实例注册表，支持每条总线最多 10 个 CAN 代理实例
- STM32F407 支持 CAN1/CAN2 两条总线
- 支持实例自动注册/注销（避免悬空指针）
- 静态分发函数：从接收中断回调中调用，二分查找对应实例

### 模板类 `can_proxy<Type, CanId, MailboxMask>`
- 编译期检查 CAN ID 合法性（标准帧 11 位，扩展帧 29 位）
- 自动生成发送帧头模板
- 继承自 `base_can_proxy`，支持多个不同 ID 的代理共存

## 如何使用

### 基础使用示例
```cpp
#include "bsp_can.hpp"

// 定义一个标准帧 CAN 代理，ID 为 0x123，使用所有邮箱
using my_can_proxy = gdut::can_proxy<gdut::can_type::standard_type, 0x123>;

CAN_HandleTypeDef hcan1;  // STM32 HAL CAN 句柄

// 创建 CAN 实例并注册
my_can_proxy can_device(hcan1);
can_device.register_self(0);  // 注册到 CAN1（bus_index=0）

// 启动 CAN
can_device.start();

// 发送数据
uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
if (can_device.transmit(data)) {
    // 发送成功
}

// 停止 CAN
can_device.stop();

// 注销实例
can_device.unregister_self();
```

### 扩展帧使用
```cpp
// 定义扩展帧代理，ID 为 0x18FF1234
using ext_can_proxy = gdut::can_proxy<gdut::can_type::extended_type, 0x18FF1234>;
ext_can_proxy can_ext(hcan1);
```

### 自定义邮箱掩码
```cpp
// 只使用邮箱 0 和邮箱 1
using limited_can_proxy = gdut::can_proxy<
    gdut::can_type::standard_type, 0x456,
    gdut::can_mailbox::mailbox0 | gdut::can_mailbox::mailbox1>;
```

### 接收回调扩展
```cpp
class my_can_device : public gdut::can_proxy<gdut::can_type::standard_type, 0x789> {
protected:
    bool receive(CAN_RxHeaderTypeDef *rxh, uint8_t data[8]) override {
        // 处理接收到的 CAN 数据
        if (rxh->DLC == 8) {
            process_message(data);
        }
        return true;  // 表示处理成功
    }

private:
    void process_message(const uint8_t *data) {
        // 实现自定义数据处理逻辑
    }
};
```

## 与代码规范的对应
- 强类型枚举，避免魔法数
- 编译期检查 CAN ID 合法性，确保类型安全
- RAII 模式自动管理 CAN 实例生命周期
- 蛇形命名约定，`m_` 前缀表示私有成员
- 基于二分查找的高效实例分发机制

## 中断集成

在 STM32 HAL 的 CAN 接收中断处理中集成：
```cpp
// 在 CAN RX 中断处理函数中调用
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxh;
    uint8_t data[8];
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxh, data) == HAL_OK) {
        // 调用全局分发函数，自动查找对应的 CAN 代理实例
        gdut::base_can_proxy::dispatch(0, &rxh, data);  // bus_index=0 表示 CAN1
    }
}
```

## 注意事项/坑点
- 使用前必须确保 CAN 外设时钟已启用并正确初始化
- 同一个 CAN ID 在同一条总线上只能注册一个代理实例
- 发送数据固定为 8 字节格式
- `register_self()` 需要传入正确的 `bus_index`（0 for CAN1, 1 for CAN2）
- 接收回调运行在中断上下文，避免耗时操作
- 实例析构时会自动调用 `unregister_self()`，但显式调用会搜索所有总线更安全

相关源码：[BSP/bsp_can.hpp](../../BSP/bsp_can.hpp)
