#ifndef COMPONENTS_PID_CONTROLLER_HPP
#define COMPONENTS_PID_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace gdut {

template <typename T> class pid_controller {
  static_assert(std::is_floating_point_v<T>,
                "Template parameter T must be a floating-point type");

public:
  pid_controller() = default;

  pid_controller(const pid_controller &other) = default;
  pid_controller &operator=(const pid_controller &other) = default;
  pid_controller(pid_controller &&other) noexcept = default;
  pid_controller &operator=(pid_controller &&other) noexcept = default;

  pid_controller(T Kp, T Ki, T Kd, T DeadZone = T{},
                 T IntegralWindupLimit = T{},
                 T MinOutput = std::numeric_limits<T>::lowest(),
                 T MaxOutput = std::numeric_limits<T>::max()) {
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

  void set_integral(T integral) {
    if (IntegralWindupLimit > T{}) {
      m_integral =
          std::clamp(integral, -IntegralWindupLimit, IntegralWindupLimit);
    } else {
      m_integral = integral;
    }
  }

  [[nodiscard]] bool
  set_parameters(T Kp, T Ki, T Kd, T DeadZone = T{},
                 T IntegralWindupLimit = T{},
                 T MinOutput = std::numeric_limits<T>::lowest(),
                 T MaxOutput = std::numeric_limits<T>::max()) {
    bool result = true;
    result = result && set_Kp(Kp);
    result = result && set_Ki(Ki);
    result = result && set_Kd(Kd);
    result = result && set_dead_zone(DeadZone);
    result = result && set_integral_windup_limit(IntegralWindupLimit);
    result = result && set_output_limits(MinOutput, MaxOutput);
    return result;
  }

  void set_target(T target) { this->m_target = target; }

  [[nodiscard]] T get_target() const { return m_target; }

  [[nodiscard]] T update(T current, T dt) {
    T error = m_target - current;
    if (std::abs(dt) < static_cast<T>(1e-6)) {
      dt = static_cast<T>(1e-6);
    }
    if (DeadZone > T{}) {
      const T error_abs_delta = std::abs(error - m_prev_error);
      const T error_dead_zone = DeadZone * dt;
      // 停车模式：只有目标和当前都足够接近 0 时,
      // 同时也满足导数条件，才直接归零； 如果目标已是 0 但电机还在动，则继续让
      // PID 刹到接近 0。
      if (std::abs(m_target) < DeadZone && std::abs(current) < DeadZone &&
          error_abs_delta < error_dead_zone) {
        m_prev_error = T{};
        m_integral = T{};
        return m_output = std::clamp(T{}, MinOutput, MaxOutput);
      }
      if (std::abs(m_target) >= DeadZone && std::abs(error) < DeadZone &&
          error_abs_delta < error_dead_zone) {
        // 非零目标时，死区内保持最后一次有效输出，避免来回抖动
        m_prev_error = error;
        return m_output = std::clamp(m_output, MinOutput, MaxOutput);
      }
    }
    if (IntegralWindupLimit > T{}) {
      m_integral = std::clamp(m_integral + error * dt, -IntegralWindupLimit,
                              IntegralWindupLimit);
    } else {
      m_integral += error * dt;
    }
    T derivative = (error - m_prev_error) / dt;
    m_prev_error = error;
    return m_output = std::clamp(Kp * error + Ki * m_integral + Kd * derivative,
                                 MinOutput, MaxOutput);
  }

  void reset() {
    m_integral = T{};
    m_prev_error = T{};
    m_output = T{};
  }

private:
  T Kp{};
  T Ki{};
  T Kd{};
  T DeadZone{};
  T IntegralWindupLimit{};
  T MinOutput = std::numeric_limits<T>::lowest();
  T MaxOutput = std::numeric_limits<T>::max();

  T m_target{};
  T m_integral{};
  T m_prev_error{};
  T m_output{};
};

template <typename T, T Kp, T Ki, T Kd, T DeadZone = T{},
          T IntegralWindupLimit = T{},
          T MinOutput = std::numeric_limits<T>::lowest(),
          T MaxOutput = std::numeric_limits<T>::max()>
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
      Kp, Ki, Kd, DeadZone, IntegralWindupLimit, MinOutput, MaxOutput};
}

} // namespace gdut

#endif // COMPONENTS_PID_CONTROLLER_HPP
