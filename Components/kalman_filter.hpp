#ifndef COMPONENTS_KALMAN_FILTER_HPP
#define COMPONENTS_KALMAN_FILTER_HPP

#include "matrix.hpp"
#include "uncopyable.hpp"

namespace gdut {

// 卡尔曼滤波器类模板
// N: 状态维度, L: 控制输入维度, M: 观测维度, T: 数据类型
// 注意：此类模板需要的栈空间较大，使用时请确保有足够的内存
// 特别是在rtos的任务中使用时，建议增加任务的栈大小 sizeof(kalman_filter)
template <std::size_t N, std::size_t L, std::size_t M, typename T = float>
class kalman_filter : uncopyable {
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

} // namespace gdut

#endif // COMPONENTS_KALMAN_FILTER_HPP
