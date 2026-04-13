#ifndef COMPONENTS_KALMAN_FILTER_HPP
#define COMPONENTS_KALMAN_FILTER_HPP

#include "matrix.hpp"
#include "uncopyable.hpp"
#include <cstddef>

namespace gdut {

// 卡尔曼滤波器类模板
// N: 状态维度, L: 控制输入维度, M: 观测维度, T: 数据类型
// 注意：此类模板需要的栈空间较大，使用时请确保有足够的内存
// 特别是在RTOS的任务中使用时，建议增加任务的栈大小 sizeof(kalman_filter)
template <::std::size_t N, ::std::size_t L, ::std::size_t M,
          typename T = float>
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

// 自适应卡尔曼滤波器类模板
// N: 状态维度, L: 控制输入维度, M: 观测维度, T: 数据类型
// 注意：此类模板需要的栈空间较大，使用时请确保有足够的内存
// 特别是在RTOS的任务中使用时，建议增加任务的栈大小 sizeof(dynamic_kalman_filter)
template <::std::size_t N, ::std::size_t L, ::std::size_t M,
          typename T = float>
class dynamic_kalman_filter : uncopyable {
public:
  static_assert(N > 0 && L > 0 && M > 0,
                "dynamic_kalman_filter requires fixed compile-time dimensions");

  using state_matrix_t = matrix<T, N, N>;
  using control_matrix_t = matrix<T, N, L>;
  using observation_matrix_t = matrix<T, M, N>;
  using measurement_cov_t = matrix<T, M, M>;
  using state_vector_t = matrix<T, N, 1>;
  using control_vector_t = matrix<T, L, 1>;
  using measurement_vector_t = matrix<T, M, 1>;

  dynamic_kalman_filter()
      : A_(state_matrix_t::identity()), B_(control_matrix_t::zeros()),
      H_(observation_matrix_t::zeros()), Q_(state_matrix_t::identity()),
      R_(measurement_cov_t::identity()), P_(state_matrix_t::identity()),
      x_(state_vector_t::zeros()) {}

  dynamic_kalman_filter(const state_matrix_t &A, const control_matrix_t &B,
                        const observation_matrix_t &H, const state_matrix_t &Q,
                        const measurement_cov_t &R,
                        const state_vector_t &x0 = state_vector_t::zeros(),
                        const state_matrix_t &P0 = state_matrix_t::identity())
    : A_(A), B_(B), H_(H), Q_(Q), R_(R), P_(P0), x_(x0), x_prior_(x0),
      P_prior_(P0) {}

  [[nodiscard]] constexpr ::std::size_t state_size() const { return N; }
  [[nodiscard]] constexpr ::std::size_t control_size() const { return L; }
  [[nodiscard]] constexpr ::std::size_t measurement_size() const { return M; }

  /**
   * 预测步骤：
   * x^-_k = A x_{k-1} + B u_k
   * P^-_k = A P_{k-1} A^T + Q
   */
  state_vector_t update_prediction(const control_vector_t &u) {
    x_prior_ = A_ * x_ + B_ * u;
    P_prior_ = A_ * P_ * A_.transpose() + Q_;
    return x_prior_;
  }

  /**
   * 校正步骤 + 自适应协方差更新：
   * e_k = z_k - H x^-_k
   * S_k = H P^-_k H^T + R_k
   * K_k = P^-_k H^T S_k^{-1}
   * x_k = x^-_k + K_k e_k
   * P_k = (I-KH)P^-(I-KH)^T + K R K^T
   *
   * 自适应更新采用对角项 EWMA：
   * R_k <- (1-rho_r) R_{k-1} + rho_r * diag(e_k e_k^T - H P^-_k H^T)
   * Q_k <- (1-rho_q) Q_{k-1} + rho_q * diag((x_k - x^-_k)(x_k - x^-_k)^T)
   */
  [[nodiscard]] state_vector_t update_correction(const measurement_vector_t &z) {
    const state_vector_t innovation = z - H_ * x_prior_;
    measurement_cov_t S = H_ * P_prior_ * H_.transpose() + R_;
    matrix<T, N, M> K = P_prior_ * H_.transpose() * S.inverse();

    x_ = x_prior_ + K * innovation;

    const state_matrix_t I = state_matrix_t::identity();
    const state_matrix_t I_minus_KH = I - K * H_;
    P_ = I_minus_KH * P_prior_ * I_minus_KH.transpose() +
         K * R_ * K.transpose();
    P_ = (P_ + P_.transpose()) * static_cast<T>(0.5);

    adapt_measurement_noise(innovation, H_, P_prior_);
    adapt_process_noise(x_, x_prior_);

    return x_;
  }

  void set_adaptation_rates(T process_rate, T measurement_rate) {
    q_adapt_rate_ = clamp_rate(process_rate);
    r_adapt_rate_ = clamp_rate(measurement_rate);
  }

  [[nodiscard]] T get_process_adaptation_rate() const { return q_adapt_rate_; }
  [[nodiscard]] T get_measurement_adaptation_rate() const {
    return r_adapt_rate_;
  }

  void enable_adaptation(bool enable) { adaptive_enabled_ = enable; }
  [[nodiscard]] bool adaptation_enabled() const { return adaptive_enabled_; }

  void set_state_transition(const state_matrix_t &A) { A_ = A; }
  void set_control_input(const control_matrix_t &B) { B_ = B; }
  void set_observation_model(const observation_matrix_t &H) { H_ = H; }
  void set_process_noise(const state_matrix_t &Q) { Q_ = Q; }
  void set_measurement_noise(const measurement_cov_t &R) { R_ = R; }
  void set_estimation_error_covariance(const state_matrix_t &P) { P_ = P; }
  void set_initial_state(const state_vector_t &x0) { x_ = x0; }

  [[nodiscard]] state_matrix_t get_state_transition() const { return A_; }
  [[nodiscard]] control_matrix_t get_control_input() const { return B_; }
  [[nodiscard]] observation_matrix_t get_observation_model() const { return H_; }
  [[nodiscard]] state_matrix_t get_process_noise() const { return Q_; }
  [[nodiscard]] measurement_cov_t get_measurement_noise() const { return R_; }
  [[nodiscard]] state_matrix_t get_estimation_error_covariance() const {
    return P_;
  }
  [[nodiscard]] state_vector_t get_state_estimate() const { return x_; }

  ~dynamic_kalman_filter() = default;

private:
  static T clamp_value(T value, T lower, T upper) {
    return (value < lower) ? lower : ((value > upper) ? upper : value);
  }

  static T clamp_rate(T value) { return clamp_value(value, static_cast<T>(0), static_cast<T>(1)); }

  static T abs_value(T value) {
    return (value < static_cast<T>(0)) ? static_cast<T>(-value) : value;
  }

  static void set_diagonal_from_matrix(state_matrix_t &target,
                                       const state_matrix_t &source,
                                       T lower_bound) {
    for (::std::size_t i = 0; i < N; ++i) {
      target(i, i) = clamp_value(source(i, i), lower_bound,
                                 static_cast<T>(1e9));
    }
  }

  void adapt_measurement_noise(const measurement_vector_t &innovation,
                               const observation_matrix_t &H,
                               const state_matrix_t &P_prior) {
    if (!adaptive_enabled_) {
      return;
    }

    measurement_cov_t hpht = H * P_prior * H.transpose();
    measurement_cov_t updated = R_;
    for (::std::size_t i = 0; i < M; ++i) {
      const T innovation_energy = innovation(i, 0) * innovation(i, 0);
      const T estimate = clamp_value(innovation_energy - hpht(i, i),
                                     r_noise_floor_, r_noise_ceiling_);
      updated(i, i) = (static_cast<T>(1) - r_adapt_rate_) * R_(i, i) +
                      r_adapt_rate_ * estimate;
      updated(i, i) = clamp_value(updated(i, i), r_noise_floor_, r_noise_ceiling_);
    }
    R_ = updated;
  }

  void adapt_process_noise(const state_vector_t &x_post,
                           const state_vector_t &x_prior) {
    if (!adaptive_enabled_) {
      return;
    }

    state_vector_t delta = x_post - x_prior;
    state_matrix_t updated = Q_;
    for (::std::size_t i = 0; i < N; ++i) {
      const T estimate = clamp_value(delta(i, 0) * delta(i, 0), q_noise_floor_,
                                     q_noise_ceiling_);
      updated(i, i) = (static_cast<T>(1) - q_adapt_rate_) * Q_(i, i) +
                      q_adapt_rate_ * estimate;
      updated(i, i) = clamp_value(updated(i, i), q_noise_floor_, q_noise_ceiling_);
    }
    Q_ = updated;
  }

  state_matrix_t A_;       // 状态转移矩阵
  control_matrix_t B_;     // 控制输入矩阵
  observation_matrix_t H_; // 观测矩阵
  state_matrix_t Q_;       // 过程噪声协方差矩阵
  measurement_cov_t R_;    // 观测噪声协方差矩阵
  state_matrix_t P_;       // 估计误差协方差矩阵
  state_vector_t x_;       // 状态估计
  state_vector_t x_prior_; // 预测状态
  state_matrix_t P_prior_; // 预测协方差

  bool adaptive_enabled_{true};
  T q_adapt_rate_{static_cast<T>(0.02)};
  T r_adapt_rate_{static_cast<T>(0.05)};
  T q_noise_floor_{static_cast<T>(1e-6)};
  T q_noise_ceiling_{static_cast<T>(1e6)};
  T r_noise_floor_{static_cast<T>(1e-6)};
  T r_noise_ceiling_{static_cast<T>(1e6)};
};

} // namespace gdut

#endif // COMPONENTS_KALMAN_FILTER_HPP
