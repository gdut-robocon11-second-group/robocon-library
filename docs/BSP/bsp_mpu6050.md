# BSP MPU6050 模块（bsp_mpu6050.hpp）

## 原理
MPU6050 是一个六轴 IMU，包含加速度计和陀螺仪。这个模块的目标不是“把所有姿态算法一次性写死”，而是把原始数据读取、量程配置、互补滤波和一些常用功能整理成易用接口。

简单说：
- 底层负责和 MPU6050 通信
- 中层负责把原始寄存器数据变成物理量
- 上层可以拿这些数据去做姿态估计、传感器融合、运动检测

## 核心设计

### 枚举与数据结构
- `mpu6050_addr`：选择器件地址，取决于 AD0 引脚
- `accel_range`：加速度计量程，决定 LSB/g 的换算关系
- `gyro_range`：陀螺仪量程，决定 LSB/(°/s) 的换算关系
- `dlpf_cfg`：数字低通滤波器带宽
- `clk_src`：时钟源选择
- `mpu6050_data`：三轴原始数据结构

### 类 `mpu6050`
`mpu6050` 依赖 `gdut::i2c` 做寄存器访问，初始化时还需要一个延时回调，因为复位和配置寄存器都需要等待硬件稳定。

### 设计优点
- **接口完整**：原始数据、温度、姿态、运动检测都能读
- **适合教学**：可以直接看到“寄存器 → 原始值 → 物理量”的过程
- **有一定封装性**：比裸寄存器操作更容易维护

### 设计缺点
- 功能多，初学者第一次看会觉得长
- 互补滤波只能解决“部分姿态问题”，不是万能姿态解算器
- Yaw 主要靠陀螺仪积分，长期会漂移

## 如何使用

### 创建并初始化
```cpp
#include "bsp_iic.hpp"
#include "bsp_mpu6050.hpp"

extern I2C_HandleTypeDef hi2c1;

gdut::i2c bus(&hi2c1);

auto delay_ms = [](std::uint32_t ms) {
    osDelay(ms);
};

gdut::mpu6050 imu(&bus, delay_ms, gdut::mpu6050_addr::low);
if (!imu.init()) {
    // 初始化失败
}
```

### 读取原始数据
```cpp
auto accel = imu.get_accel();
auto gyro = imu.get_gyro();
float temperature = imu.get_temperature();
```

### 读取互补滤波结果
```cpp
auto angles = imu.get_filtered_euler_angles(0.01f);
float yaw = imu.get_yaw();
```

### 获取真实加速度
```cpp
auto real_accel = imu.get_real_acceleration();
```

### 调整量程和滤波
```cpp
imu.set_sample_rate(100);
imu.set_dlpf(gdut::dlpf_cfg::bandwidth_44hz);
imu.set_accel_range(gdut::accel_range::range_8g);
imu.set_gyro_range(gdut::gyro_range::range_500dps);
imu.set_filter_alpha(0.98f);
```

## 实际应用示例

### 姿态估计
```cpp
auto angles = imu.get_filtered_euler_angles(0.01f);
printf("roll=%.2f pitch=%.2f yaw=%.2f\n",
       angles.roll, angles.pitch, angles.yaw);
```

### 航向角复位
```cpp
imu.reset_yaw(0.0f);
```

### 运动检测
```cpp
imu.enable_motion_detection(50);
if (imu.get_interrupt_status()) {
    // 发生了运动事件
}
```

### 读取完整 IMU 数据
```cpp
auto data = imu.read_imu_data();
```

## 为什么这样设计

### 和“只读原始寄存器”相比
#### 优点
- 上手更快
- 不用每次都重新写寄存器解析逻辑
- 能直接拿到角速度、加速度和温度

#### 缺点
- 封装后文件比较长
- 姿态算法不是特别复杂，但也不是完全“零成本”

### 和“把所有融合都写死”相比
#### 优点
- 更灵活
- 上层还能自己换算法

#### 缺点
- 不能直接拿来当完整姿态导航系统

## 使用建议
- 初始化时优先确认 `WHO_AM_I` 读数正确
- 固定周期调用 `get_filtered_euler_angles(dt)`，效果会更稳定
- 如果只想看原始传感器数据，可以直接用 `get_accel()` 和 `get_gyro()`
- 如果要做融合控制，建议把这份数据交给上层单独处理

## 注意事项
- `Yaw` 会漂移，这是陀螺仪积分的天然问题，不是代码写错
- `set_sample_rate()` 和 `set_dlpf()` 要配套考虑，别让滤波器和采样率互相打架
- 不同量程下的 LSB 不一样，换了量程就要重新理解数值大小
- 传感器安装方向变了，坐标轴含义也会跟着变

## 常见坑
- **坑 1：** 以为 `get_yaw()` 是磁航向，实际上它没有磁力计校正
- **坑 2：** 互补滤波只调用一次，结果角度几乎没意义
- **坑 3：** 量程调大以后，以为数据“变小了就是坏了”，其实只是换算系数变了

## 总结
这个模块适合做“IMU 原始数据 + 常用预处理”的底座。如果你后面要做姿态解算、定位或者控制，建议把它当作数据源，而不是最终控制器。

## 与代码规范的对应
- 枚举统一描述硬件配置，避免裸数字到处飞
- 类职责集中在“读取、换算、滤波、配置”
- 接口按功能拆分，原始数据和处理后数据分开提供
- 命名保持蛇形风格，和库内其他模块一致

## 注意事项/坑点
- ⚠️ **Yaw 会漂移**：这是陀螺仪积分的天然问题，不是 bug
- ⚠️ **量程和换算要对应**：改了量程不等于数据错了，先看 LSB 系数
- ⚠️ **固定周期更稳**：互补滤波最好稳定周期调用
- ⚠️ **坐标轴方向**：板子安装角度变了，roll/pitch/yaw 的意义也变
- ⚠️ **`WHO_AM_I` 检查**：初始化先确认器件身份，别盲目往下读

## 与“只读原始寄存器”的对比

| 特性 | `gdut::mpu6050` | 手写寄存器读取 |
|------|------------------|------------------|
| 上手难度 | ✅ 低 | ❌ 高 |
| 可读性 | ✅ 高 | ❌ 一般 |
| 灵活性 | ✅ 较高 | ✅ 最高 |
| 代码量 | ✅ 更少 | ❌ 更多 |

## 调试技巧

### 先确认器件是否正常响应
```cpp
uint8_t id = imu.get_device_id();
printf("device id = 0x%02X\n", id);
```

### 先看原始数据，再看滤波后数据
```cpp
auto accel = imu.get_accel();
auto gyro = imu.get_gyro();
auto angles = imu.get_filtered_euler_angles(0.01f);
```

### 通过静止测试检查零漂
- 把传感器平放在桌面上
- 观察 `roll` / `pitch` 是否接近 0
- 观察 `yaw` 是否持续变化

相关源码：[Middlewares/robocon-library/BSP/bsp_mpu6050.hpp](../BSP/bsp_mpu6050.hpp)
