#ifndef COMPONENTS_PID_CONTROLLER_HPP
#define COMPONENTS_PID_CONTROLLER_HPP

#include <algorithm>
#include <limits>
#include <type_traits>

namespace gdut {

template <typename T> class pid_controller {
  static_assert(std::is_floating_point_v<T>,
                "Template parameter T must be a floating-point type");

public:
  pid_controller() = default;

  pid_controller(T Kp, T Ki, T Kd, T DeadZone = T{},
                 T IntegralWindupLimit = T{},
                 T MinOutput = std::numeric_limits<T>::lowest(),
                 T MaxOutput = std::numeric_limits<T>::max(),
                 T Alpha = static_cast<T>(0.1)) {
    (void)set_parameters(Kp, Ki, Kd, DeadZone, IntegralWindupLimit, MinOutput,
                         MaxOutput);
  }
  ~pid_controller() = default;

  [[nodiscard]] bool set_Kp(T Kp) {
    if (Kp < T{}) {
      return false; // Proportional gain must be non-negative
    }
    this->Kp = Kp;
    return true;
  }
  [[nodiscard]] bool set_Ki(T Ki) {
    if (Ki < T{}) {
      return false; // Integral gain must be non-negative
    }
    this->Ki = Ki;
    return true;
  }
  [[nodiscard]] bool set_Kd(T Kd) {
    if (Kd < T{}) {
      return false; // Derivative gain must be non-negative
    }
    this->Kd = Kd;
    return true;
  }
  [[nodiscard]] bool set_dead_zone(T DeadZone) {
    if (DeadZone < T{}) {
      return false; // Dead zone must be non-negative
    }
    this->DeadZone = DeadZone;
    return true;
  }
  [[nodiscard]] bool set_integral_windup_limit(T IntegralWindupLimit) {
    if (IntegralWindupLimit < T{}) {
      return false; // Integral windup limit must be non-negative
    }
    this->IntegralWindupLimit = IntegralWindupLimit;
    return true;
  }

  [[nodiscard]] bool set_output_limits(T MinOutput, T MaxOutput) {
    if (MinOutput >= MaxOutput) {
      return false; // Minimum output must be less than maximum output
    }
    this->MinOutput = MinOutput;
    this->MaxOutput = MaxOutput;
    return true;
  }

  [[nodiscard]] bool set_alpha(T Alpha) {
    if (Alpha < T{} || Alpha > static_cast<T>(1)) {
      return false; // Alpha must be in the range [0, 1]
    }
    this->Alpha = Alpha;
    return true;
  }
  [[nodiscard]] bool set_integral(T integral)
  {
    if (integral < MinOutput || integral > MaxOutput) {
      return false;
    }
    m_integral = integral;
    return true;
  }
  [[nodiscard]] bool
  set_parameters(T Kp, T Ki, T Kd, T DeadZone = T{},
                 T IntegralWindupLimit = T{},
                 T MinOutput = std::numeric_limits<T>::lowest(),
                 T MaxOutput = std::numeric_limits<T>::max(),
                 T Alpha = static_cast<T>(0.1)) {
    bool result = true;
    result = result && set_Kp(Kp);
    result = result && set_Ki(Ki);
    result = result && set_Kd(Kd);
    result = result && set_dead_zone(DeadZone);
    result = result && set_integral_windup_limit(IntegralWindupLimit);
    result = result && set_output_limits(MinOutput, MaxOutput);
    result = result && set_alpha(Alpha);
    return result;
  }

  // error = target - current
  [[nodiscard]] T update(T error, T dt) {
    if (DeadZone > T{} && std::abs(error) < DeadZone) {
      m_prev_error = error; // Reset previous error to prevent derivative kick
      return m_output;      // No change in output if within dead zone
    }
    if (IntegralWindupLimit > T{}) {
      m_integral = std::clamp(m_integral + error * dt, -IntegralWindupLimit,
                              IntegralWindupLimit);
    } else {
      m_integral += error * dt;
    }
    T derivative = (error - m_prev_error) / dt;
    // 滤波处理：使用指数移动平均滤波器来平滑导数项
    m_deriv_filter = Alpha * derivative + (1 - Alpha) * m_deriv_filter;
    m_prev_error = error;
    return m_output =
               std::clamp(Kp * error + Ki * m_integral + Kd * m_deriv_filter,
                          MinOutput, MaxOutput);
  }

private:
  T Kp{};
  T Ki{};
  T Kd{};
  T DeadZone{};
  T IntegralWindupLimit{};
  T MinOutput = std::numeric_limits<T>::lowest();
  T MaxOutput = std::numeric_limits<T>::max();
  T Alpha{}; // 滤波系数

  T m_integral{};
  T m_prev_error{};
  T m_output{};
  T m_deriv_filter{}; // 用于滤波的变量
};

template <
    typename T, T Kp, T Ki, T Kd, T DeadZone = T{}, T IntegralWindupLimit = T{},
    T MinOutput = std::numeric_limits<T>::lowest(),
    T MaxOutput = std::numeric_limits<T>::max(), T Alpha = static_cast<T>(0.1)>
pid_controller<T> make_pid_controller() {
  static_assert(Kp >= 0, "Proportional gain must be non-negative");
  static_assert(Ki >= 0, "Integral gain must be non-negative");
  static_assert(Kd >= 0, "Derivative gain must be non-negative");
  static_assert(Kp > 0 || Ki > 0 || Kd > 0,
                "At least one gain must be positive");
  static_assert(DeadZone >= T{}, "Dead zone must be non-negative");
  static_assert(IntegralWindupLimit >= T{},
                "Integral windup limit must be non-negative");
  static_assert(MinOutput < MaxOutput,
                "Minimum output must be less than maximum output");
  static_assert(std::is_floating_point_v<T>,
                "Template parameter T must be a floating-point type");
  return pid_controller<T>{
      Kp, Ki, Kd, DeadZone, IntegralWindupLimit, MinOutput, MaxOutput, Alpha};
}

} // namespace gdut

#endif // COMPONENTS_PID_CONTROLLER_HPP
