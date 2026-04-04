#ifndef MODULES_CHASSIS_KINEMATICS_HPP
#define MODULES_CHASSIS_KINEMATICS_HPP

#include "matrix.hpp"
#include <cstddef>

namespace gdut {

/*
 * @brief 机械臂运动学类，提供正运动学和逆运动学的计算方法
 * @tparam Radius 轮子到机器人中心的距离，单位为米
 */
template <float Radius> class chassis_kinematics {
public:
  static_assert(Radius > 0, "Radius must be greater than 0");

  static constexpr float radius = Radius;

  // 0.707106781f == 1 / sqrt(2)
  static constexpr matrix<float, 4, 3> forward_kinematics_matrix{
      -0.707106781f, 0.707106781f,  radius,        0.707106781f,
      0.707106781f,  radius,        0.707106781f,  -0.707106781f,
      radius,        -0.707106781f, -0.707106781f, radius};

  //  0.176776695f == 1 / (4 * sqrt(2))
  static constexpr matrix<float, 3, 4> inverse_kinematics_matrix{
      -0.176776695f,  0.176776695f,   0.176776695f,   -0.176776695f,
      0.176776695f,   0.176776695f,   -0.176776695f,  -0.176776695f,
      0.25f / radius, 0.25f / radius, 0.25f / radius, 0.25f / radius};

  /*
   * @brief 计算机器人速度到轮速的转换
   * @param velocities 机器人速度，格式为 (vx, vy, omega)，单位为 m/s 和 rad/s
   * @return 轮速，格式为 (v1, v2, v3, v4)，单位为 m/s
   */
  static vector<float, 4>
  forward_kinematics(const vector<float, 3> &velocities) {
    return forward_kinematics_matrix * velocities;
  };

  /*
   * @brief 计算轮速到机器人速度的转换
   * @param wheel_velocities 轮速，格式为 (v1, v2, v3, v4)，单位为 m/s
   * @return 机器人速度，格式为 (vx, vy, omega)，单位为 m/s 和 rad/s
   */
  static vector<float, 3>
  inverse_kinematics(const vector<float, 4> &wheel_velocities) {
    return inverse_kinematics_matrix * wheel_velocities;
  }
};

} // namespace gdut

#endif // MODULES_CHASSIS_KINEMATICS_HPP
