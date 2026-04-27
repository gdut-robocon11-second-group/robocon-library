# Modules 底盘运动学模块（chassis_kinematics.hpp）

## 原理
该模块封装了四轮全向底盘（常见 X 布局）的线速度运动学映射，提供：
- **正运动学**：机器人速度 `->` 四轮轮速
- **逆运动学**：四轮轮速 `->` 机器人速度

核心类型为 `gdut::universal_wheel_kinematics<Radius>`，使用编译期常量矩阵完成计算。

## 核心设计

### 1) 编译期参数与约束
- 模板参数 `Radius` 表示轮子到底盘中心距离。
- `static_assert(Radius > 0)` 在编译期阻止非法配置。

### 2) 矩阵形式
设机器人速度向量为：

$$
\mathbf{v} = \begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}
$$

轮速向量为：

$$
\mathbf{w} = \begin{bmatrix} v_1 \\ v_2 \\ v_3 \\ v_4 \end{bmatrix}
$$

则有：

$$
\mathbf{w} = \mathbf{F}\mathbf{v}, \qquad \mathbf{v} = \mathbf{I}\mathbf{w}
$$

其中 `F`、`I` 分别对应 `forward_kinematics_matrix` 与 `inverse_kinematics_matrix`。

### 3) 数值常量
矩阵中使用了 `1/sqrt(2)` 与 `1/(4*sqrt(2))` 的浮点近似值，兼顾性能与可读性。

## 接口说明

### `forward_kinematics(const vector<float, 3>& velocities)`
- 输入：`(vx, vy, omega)`
- 输出：`vector<float, 4>`（四轮线速度）

### `inverse_kinematics(const vector<float, 4>& wheel_velocities)`
- 输入：`(v1, v2, v3, v4)`
- 输出：`vector<float, 3>`（底盘速度）

## 如何使用

### 基础示例
```cpp
#include "universal_wheel_kinematics.hpp"

using kinematics_t = gdut::universal_wheel_kinematics<1>;

gdut::vector<float, 3> cmd{1.0f, 0.0f, 0.3f}; // vx, vy, omega
auto wheel_vel = kinematics_t::forward_kinematics(cmd);
```

### 里程计反解示例
```cpp
gdut::vector<float, 4> measured_wheels{2.3f, 1.9f, 2.1f, 2.0f};
auto robot_vel = kinematics_t::inverse_kinematics(measured_wheels);

float vx = robot_vel[0];
float vy = robot_vel[1];
float omega = robot_vel[2];
(void)vx; (void)vy; (void)omega;
```

### 控制环中的常见用法
```cpp
void chassis_control_step(float vx_cmd, float vy_cmd, float wz_cmd) {
    gdut::vector<float, 3> body_cmd{vx_cmd, vy_cmd, wz_cmd};
    auto wheels = kinematics_t::forward_kinematics(body_cmd);

    set_wheel_target_speed(0, wheels[0]);
    set_wheel_target_speed(1, wheels[1]);
    set_wheel_target_speed(2, wheels[2]);
    set_wheel_target_speed(3, wheels[3]);
}
```

## 工程建议
- 统一轮序定义（电机 ID 与矩阵行顺序一致）。
- 统一单位：`vx/vy` 用 m/s、`omega` 用 rad/s。
- 若底盘几何参数变化，优先校核矩阵系数而非在上层“打补丁”。

## 常见问题与排查
- **症状：前进命令出现横移**
  - 检查轮序映射是否错位。
- **症状：旋转方向相反**
  - 检查坐标系约定（右手系/左手系）与电机正方向。
- **症状：正逆解不一致**
  - 检查 `Radius` 配置和单位，及编码器速度正负号。

## 注意事项
- `Radius` 是模板参数（`std::size_t`），语义上是几何缩放常量。
- 当前实现固定为四轮模型，不直接适用于三轮/麦克纳姆差异布局。
- 模块只负责运动学映射，不包含动力学补偿与摩擦补偿。

相关源码：
- [Modules/universal_wheel_kinematics.hpp](../../Modules/universal_wheel_kinematics.hpp)
