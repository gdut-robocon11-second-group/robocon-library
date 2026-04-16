#ifndef COMPONENTS_KALMAN_FILTER_HPP
#define COMPONENTS_KALMAN_FILTER_HPP

#include "matrix.hpp"
#include "memory_resource.hpp"
#include "uncopyable.hpp"
#include <cstddef>
#include <deque>
#include <memory_resource>


namespace gdut {

// 卡尔曼滤波器类模板
// N: 状态维度, L: 控制输入维度, M: 观测维度, T: 数据类型
// 注意：此类模板需要的栈空间较大，使用时请确保有足够的内存
// 特别是在RTOS的任务中使用时，建议增加任务的栈大小 sizeof(kalman_filter)
template <::std::size_t N, ::std::size_t L, ::std::size_t M, typename T = float>
class kalman_filter : uncopyable {
  static_assert(N > 0 && L > 0 && M > 0,
                "kalman_filter requires fixed compile-time dimensions");

public:
  kalman_filter()
      : A(matrix<T, N, N>::identity()), B(matrix<T, N, L>::zeros()),
        H(matrix<T, M, N>::zeros()), Q(matrix<T, N, N>::identity()),
        R(matrix<T, M, M>::identity()), P(matrix<T, N, N>::identity()),
        x(matrix<T, N, 1>::zeros()) {
    // 默认构造函数，使用单位矩阵和零矩阵初始化
  }

  kalman_filter(const matrix<T, N, N> &A, const matrix<T, N, L> &B,
                const matrix<T, M, N> &H, const matrix<T, N, N> &Q,
                const matrix<T, M, M> &R,
                const matrix<T, N, 1> &x0 = matrix<T, N, 1>::zeros(),
                const matrix<T, N, N> &P0 = matrix<T, N, N>::identity())
      : A(A), B(B), H(H), Q(Q), R(R), P(P0), x(x0) {
    // 初始化状态估计和误差协方差矩阵
  }

  ~kalman_filter() = default;

  matrix<T, N, 1> update_prediction(const matrix<T, L, 1> &u) {
    // 预测状态
    x = A * x + B * u;
    // 预测误差协方差
    P = A * P * A.transpose() + Q;
    return x;
  }

  [[nodiscard]] matrix<T, N, 1> update_correction(const matrix<T, M, 1> &z) {
    // 计算卡尔曼增益
    matrix<T, M, M> S = H * P * H.transpose() + R;
    matrix<T, N, M> K = P * H.transpose() * S.inverse();
    // 更新状态估计
    x = x + K * (z - H * x);
    // 更新误差协方差
    P = (matrix<T, N, N>::identity() - K * H) * P;
    P = (P + P.transpose()) * 0.5;
    return x;
  }

  void set_state_transition(const matrix<T, N, N> &A_) { this->A = A_; }

  void set_control_input(const matrix<T, N, L> &B_) { this->B = B_; }

  void set_observation_model(const matrix<T, M, N> &H_) { this->H = H_; }

  void set_process_noise(const matrix<T, N, N> &Q_) { this->Q = Q_; }

  void set_measurement_noise(const matrix<T, M, M> &R_) { this->R = R_; }

  void set_estimation_error_covariance(const matrix<T, N, N> &P_) {
    this->P = P_;
  }

  void set_initial_state(const matrix<T, N, 1> &x0) { this->x = x0; }

  [[nodiscard]] matrix<T, N, N> get_state_transition() const { return A; }

  [[nodiscard]] matrix<T, N, L> get_control_input() const { return B; }

  [[nodiscard]] matrix<T, M, N> get_observation_model() const { return H; }

  [[nodiscard]] matrix<T, N, N> get_process_noise() const { return Q; }

  [[nodiscard]] matrix<T, M, M> get_measurement_noise() const { return R; }

  [[nodiscard]] matrix<T, N, N> get_estimation_error_covariance() const {
    return P;
  }

  [[nodiscard]] matrix<T, N, 1> get_state_estimate() const { return x; }

private:
  matrix<T, N, N> A; // 状态转移矩阵
  matrix<T, N, L> B; // 控制输入矩阵
  matrix<T, M, N> H; // 观测矩阵
  matrix<T, N, N> Q; // 过程噪声协方差矩阵
  matrix<T, M, M> R; // 观测噪声协方差矩阵
  matrix<T, N, N> P; // 估计误差协方差矩阵
  matrix<T, N, 1> x; // 状态估计
};

template <std::size_t N, std::size_t L, std::size_t M, typename T = float>
class adaptive_kalman_filter : uncopyable {
public:
  // 构造函数，增加自适应参数
  adaptive_kalman_filter()
      : A(matrix<T, N, N>::identity()), B(matrix<T, N, L>::zeros()),
        H(matrix<T, M, N>::zeros()), Q(matrix<T, N, N>::identity()),
        R(matrix<T, M, M>::identity()), P(matrix<T, N, N>::identity()),
        x(matrix<T, N, 1>::zeros()),
        // 自适应参数初始化
        window_size_(20),         // 默认窗口大小
        forgetting_factor_(0.98), // 遗忘因子（用于递推估计）
        enable_adapt_R_(true),    // 默认开启 R 自适应
        enable_adapt_Q_(false) // 默认关闭 Q 自适应（Q 自适应需要更复杂的逻辑）
  {
    // 初始化新息滑动窗口
    innovation_window_.clear();
  }

  // 带参数构造（与标准 KF 相同，附加自适应参数）
  adaptive_kalman_filter(
      const matrix<T, N, N> &A, const matrix<T, N, L> &B,
      const matrix<T, M, N> &H, const matrix<T, N, N> &Q,
      const matrix<T, M, M> &R,
      const matrix<T, N, 1> &x0 = matrix<T, N, 1>::zeros(),
      const matrix<T, N, N> &P0 = matrix<T, N, N>::identity())
      : A(A), B(B), H(H), Q(Q), R(R), P(P0), x(x0), window_size_(20),
        forgetting_factor_(0.98), enable_adapt_R_(true),
        enable_adapt_Q_(false) {
    innovation_window_.clear();
  }

  ~adaptive_kalman_filter() = default;

  // 预测步骤（与标准 KF 完全一致）
  matrix<T, N, 1> update_prediction(const matrix<T, L, 1> &u) {
    x = A * x + B * u;
    P = A * P * A.transpose() + Q; // 这里使用当前（可能自适应后）的 Q
    return x;
  }

  // 校正步骤（增加自适应逻辑）
  [[nodiscard]] matrix<T, N, 1> update_correction(const matrix<T, M, 1> &z) {
    // ---------- 1. 计算新息和卡尔曼增益 ----------
    matrix<T, M, 1> y = z - H * x;                 // 新息
    matrix<T, M, M> S = H * P * H.transpose() + R; // 新息协方差理论值
    matrix<T, N, M> K = P * H.transpose() * S.inverse();

    // ---------- 2. 执行状态和协方差更新 ----------
    x = x + K * y;
    P = (matrix<T, N, N>::identity() - K * H) * P;
    P = (P + P.transpose()) * 0.5; // 强制对称

    // ---------- 3. 自适应更新噪声协方差 ----------
    adapt_noise_covariances(y, S);

    return x;
  }

  // -------------- 自适应噪声估计核心方法 --------------
  void adapt_noise_covariances(const matrix<T, M, 1> &innovation,
                               const matrix<T, M, M> &S_theoretical) {
    if (!enable_adapt_R_ && !enable_adapt_Q_)
      return;

    // 将当前新息存入滑动窗口
    innovation_window_.push_back(innovation);
    if (innovation_window_.size() > window_size_) {
      innovation_window_.pop_front();
    }

    // 计算新息的实际协方差（基于滑动窗口）
    matrix<T, M, M> C_innovation = matrix<T, M, M>::zeros();
    for (const auto &v : innovation_window_) {
      C_innovation = C_innovation + v * v.transpose();
    }
    C_innovation =
        C_innovation * (1.0 / static_cast<T>(innovation_window_.size()));

    // ---------- 自适应 R ----------
    if (enable_adapt_R_) {
      // 实际协方差 - 理论预测协方差（不含R的部分）≈ R的真实值
      // 理论公式：C_innovation ≈ H*P*H^T + R
      matrix<T, M, M> estimated_R = C_innovation - H * P * H.transpose();

      // 确保对角元素为正，非对角元素合理（简单处理：只更新对角元素）
      for (std::size_t i = 0; i < M; ++i) {
        if (estimated_R[i, i] > 0) {
          // 使用一阶低通滤波平滑更新，避免突变
          R[i, i] = forgetting_factor_ * R[i, i] +
                    (1 - forgetting_factor_) * estimated_R[i, i];
        }
      }
      // 保持对称性
      R = (R + R.transpose()) * 0.5;
    }

    // ---------- 自适应 Q（可选，示例采用简单方法）----------
    if (enable_adapt_Q_) {
      // Q 的估计更复杂，一种近似是：Q ≈ K * C_innovation * K^T
      // 但需要谨慎使用，这里给出示意，实际使用时需根据具体模型调整
      matrix<T, N, M> K_approx = P * H.transpose() * S_theoretical.inverse();
      matrix<T, N, N> estimated_Q =
          K_approx * C_innovation * K_approx.transpose();

      for (std::size_t i = 0; i < N; ++i) {
        if (estimated_Q[i, i] > 0) {
          Q[i, i] = forgetting_factor_ * Q[i, i] +
                    (1 - forgetting_factor_) * estimated_Q[i, i];
        }
      }
      Q = (Q + Q.transpose()) * 0.5;
    }
  }

  // -------------- 自适应参数设置接口 --------------
  void set_window_size(std::size_t ws) {
    window_size_ = ws;
    // 清空窗口，重新积累
    innovation_window_.clear();
  }

  void set_forgetting_factor(T ff) { forgetting_factor_ = ff; }

  void enable_adaptation(bool enable_R, bool enable_Q) {
    enable_adapt_R_ = enable_R;
    enable_adapt_Q_ = enable_Q;
  }

  // -------------- 标准 KF 原有接口保持不变 --------------
  void set_state_transition(const matrix<T, N, N> &A_) { A = A_; }
  void set_control_input(const matrix<T, N, L> &B_) { B = B_; }
  void set_observation_model(const matrix<T, M, N> &H_) { H = H_; }
  void set_process_noise(const matrix<T, N, N> &Q_) { Q = Q_; }
  void set_measurement_noise(const matrix<T, M, M> &R_) { R = R_; }
  void set_estimation_error_covariance(const matrix<T, N, N> &P_) { P = P_; }
  void set_initial_state(const matrix<T, N, 1> &x0) { x = x0; }

  [[nodiscard]] matrix<T, N, N> get_state_transition() const { return A; }
  [[nodiscard]] matrix<T, N, L> get_control_input() const { return B; }
  [[nodiscard]] matrix<T, M, N> get_observation_model() const { return H; }
  [[nodiscard]] matrix<T, N, N> get_process_noise() const { return Q; }
  [[nodiscard]] matrix<T, M, M> get_measurement_noise() const { return R; }
  [[nodiscard]] matrix<T, N, N> get_estimation_error_covariance() const {
    return P;
  }
  [[nodiscard]] matrix<T, N, 1> get_state_estimate() const { return x; }

private:
  // ---------- 标准 KF 成员 ----------
  matrix<T, N, N> A;
  matrix<T, N, L> B;
  matrix<T, M, N> H;
  matrix<T, N, N> Q;
  matrix<T, M, M> R;
  matrix<T, N, N> P;
  matrix<T, N, 1> x;

  // ---------- 自适应新增成员 ----------
  std::deque<matrix<T, M, 1>, std::pmr::polymorphic_allocator<matrix<T, M, 1>>>
      innovation_window_{
          pmr::portable_resource::get_instance()}; // 滑动窗口存储新息
  std::size_t window_size_;                        // 窗口大小
  T forgetting_factor_;                            // 遗忘因子 (0,1]
  bool enable_adapt_R_;                            // 是否自适应 R
  bool enable_adapt_Q_;                            // 是否自适应 Q
};

} // namespace gdut

#endif // COMPONENTS_KALMAN_FILTER_HPP
