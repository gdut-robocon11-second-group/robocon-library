# Components 卡尔曼滤波模块（kalman_filter.hpp）

## 概览

该模块提供一个编译期固定维度的线性卡尔曼滤波器：`gdut::kalman_filter<N, L, M, T>`。

- `N`：状态维度
- `L`：控制输入维度
- `M`：观测维度
- `T`：数值类型（默认 `float`）

实现依赖 [DSP/matrix.hpp](../../DSP/matrix.hpp) 的固定维矩阵运算，适合 MCU / RTOS 场景。

> 注意：该类内部包含多个定长矩阵对象，实例栈占用与 `N/L/M` 成正相关。在线程/任务栈较小场景下，请评估 `sizeof(kalman_filter<...>)` 并适当增加栈空间。

## 数学模型

预测步骤：

$$
\hat{x}_{k|k-1} = A\hat{x}_{k-1|k-1} + Bu_k
$$

$$
P_{k|k-1} = AP_{k-1|k-1}A^T + Q
$$

校正步骤：

$$
S_k = HP_{k|k-1}H^T + R
$$

$$
K_k = P_{k|k-1}H^TS_k^{-1}
$$

$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k(z_k - H\hat{x}_{k|k-1})
$$

$$
P_{k|k} = (I - K_kH)P_{k|k-1}
$$

代码里额外执行了协方差对称化：

$$
P \leftarrow \frac{P + P^T}{2}
$$

用于减小浮点误差导致的非对称问题。

## 接口说明

### 构造

- `kalman_filter()`：默认初始化
  - `A = I`
  - `B = 0`
  - `H = 0`
  - `Q = I`
  - `R = I`
  - `P = I`
  - `x = 0`

- `kalman_filter(A, B, H, Q, R, x0, P0)`：自定义初始化。

### 核心更新

- `update_prediction(const matrix<T, L, 1>& u)`：执行预测并返回预测后状态 `x`。
- `update_correction(const matrix<T, M, 1>& z)`：执行校正并返回校正后状态 `x`。

### 参数设置

- `set_state_transition(A)`
- `set_control_input(B)`
- `set_observation_model(H)`
- `set_process_noise(Q)`
- `set_measurement_noise(R)`
- `set_estimation_error_covariance(P)`
- `set_initial_state(x0)`

### 参数读取

- `get_state_transition()`
- `get_control_input()`
- `get_observation_model()`
- `get_process_noise()`
- `get_measurement_noise()`
- `get_estimation_error_covariance()`
- `get_state_estimate()`

## 使用示例（1 维）

```cpp
#include "kalman_filter.hpp"

using gdut::kalman_filter;
using gdut::matrix;

kalman_filter<1, 1, 1, float> kf;

// 可选：按实际系统重设参数
kf.set_state_transition(matrix<float, 1, 1>{1.0f});
kf.set_control_input(matrix<float, 1, 1>{1.0f});
kf.set_observation_model(matrix<float, 1, 1>{1.0f});
kf.set_process_noise(matrix<float, 1, 1>{0.01f});
kf.set_measurement_noise(matrix<float, 1, 1>{0.1f});

auto u = matrix<float, 1, 1>{0.0f};
auto z = matrix<float, 1, 1>{10.0f};

auto x_pred = kf.update_prediction(u);
auto x_corr = kf.update_correction(z);

// 读取当前估计与协方差
auto x_now = kf.get_state_estimate();
auto p_now = kf.get_estimation_error_covariance();
```

## 工程建议

- `Q` 越大：更相信模型变化（响应更快，抖动可能增加）。
- `R` 越大：更不信任测量（响应更慢，更平滑）。
- 初始 `P0` 较大可加快初期收敛，但也会引入更大更新幅度。
- 若观测缺失，可只调用 `update_prediction()`。

相关源码：
- [Components/kalman_filter.hpp](../../Components/kalman_filter.hpp)
