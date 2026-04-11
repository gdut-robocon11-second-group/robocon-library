#ifndef BSP_SERVO_HPP
#define BSP_SERVO_HPP

#include "function.hpp"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "uncopyable.hpp"

#include <cstdint>
#include <functional>
#include <utility> //因使用到std::move，为提升可移植性，增加

namespace gdut {

/**
 * @brief 舵机参数结构体
 */
struct ServoConfig {
  uint16_t min_pulse;    //!< 0度对应的脉宽 (定时器计数)
  uint16_t max_pulse;    //!< 180度对应的脉宽 (定时器计数)
  uint8_t min_angle;     //!< 最小角度限制
  uint8_t max_angle;     //!< 最大角度限制
  uint8_t default_angle; //!< 默认角度
};

/**
 * @brief 舵机控制类
 *
 * 封装舵机控制功能，提供角度设置、角度限制、回调机制等。
 * 与现有代码完全兼容，可直接替换原函数。
 */
class servo : private uncopyable {
public:
  /// 舵机**命令发送**完成时的回调函数类型（参数为当前设置的角度）
  /// @attention 该回调在寄存器写入后立即触发，不代表舵机已物理转动到位

  using move_complete_callback_t = gdut::function<void(uint8_t current_angle)>;
  /// 舵机错误回调函数类型
  using error_callback_t = gdut::function<void(HAL_StatusTypeDef error)>;
  using delay_callback_t = gdut::function<void(uint16_t ms)>;

  /**
   * @brief 构造函数
   * @param htim 定时器句柄 (如 &htim4_pwm)
   * @param channel 定时器通道 (如 TIM_CHANNEL_1)
   * @param config 舵机配置参数
   *
   * @note 构造函数不会修改硬件，需随后调用 set_angle() 设置初始位置
   */
  servo(TIM_HandleTypeDef *htim, uint32_t channel, delay_callback_t delay_cb,
        const ServoConfig &config = ServoConfig{1000, 5000, 0, 180, 90})
      : m_htim(htim), m_channel(channel), m_config(config),
        m_current_angle(config.default_angle) {
    m_callbacks.delay_cb = std::move(delay_cb);
  }

  /**
   * @brief 设置舵机角度
   * @param angle 目标角度 (0~180)
   * @return HAL_StatusTypeDef
   *
   * @note 与现有代码中的 set_servo_angle() 功能相同
   */
  HAL_StatusTypeDef set_angle(uint8_t angle) {
    if (m_htim == nullptr) {
      if (m_callbacks.error_cb) {
        std::invoke(m_callbacks.error_cb, HAL_ERROR);
      }
      return HAL_ERROR;
    }

    // 角度限制
    if (angle < m_config.min_angle)
      angle = m_config.min_angle;
    if (angle > m_config.max_angle)
      angle = m_config.max_angle;

    m_current_angle = angle;

    // 线性映射：角度 -> 脉宽
    // 原公式：pulse = 1000 + angle * 4000 / 180
    uint32_t pulse =
        m_config.min_pulse + (static_cast<uint32_t>(angle) *
                              (m_config.max_pulse - m_config.min_pulse)) /
                                 180U;

    // 设置比较寄存器
    __HAL_TIM_SET_COMPARE(m_htim, m_channel, pulse);

    // 调用运动完成回调
    if (m_callbacks.move_complete_cb) {
      std::invoke(m_callbacks.move_complete_cb, m_current_angle);
    }

    return HAL_OK;
  }

  /**
   * @brief 获取当前设置的角度
   * @return uint8_t 当前角度
   */
  uint8_t get_angle() const { return m_current_angle; }

  /**
   * @brief 获取当前PWM脉宽对应的比较寄存器值
   * @return uint32_t CCR值
   */
  uint32_t get_current_pulse() const {
    if (m_htim == nullptr) {
      return 0;
    }
    return __HAL_TIM_GET_COMPARE(m_htim, m_channel);
  }

  /**
   * @brief 平滑移动到指定角度 (阻塞式，软件延时)
   * @param target_angle 目标角度
   * @param step 单步角度增量
   * @param delay_ms 每步之间的延时(ms)
   * @return HAL_StatusTypeDef
   *
   * @note 在RTOS线程中，建议使用 osDelay 替换 HAL_Delay
   */
  HAL_StatusTypeDef move_smooth(uint8_t target_angle, uint8_t step = 1,
                                uint16_t delay_ms = 20) {
    if (m_htim == nullptr || step == 0) {
      if (m_callbacks.error_cb) {
        std::invoke(m_callbacks.error_cb, HAL_ERROR);
      }
      return HAL_ERROR;
    }

    if (target_angle < m_config.min_angle)
      target_angle = m_config.min_angle;
    if (target_angle > m_config.max_angle)
      target_angle = m_config.max_angle;

    if (m_current_angle == target_angle) {
      return HAL_OK;
    }

    int8_t direction = (target_angle > m_current_angle) ? 1 : -1;
    uint8_t current = m_current_angle;

    while (current != target_angle) {
      int16_t next = static_cast<int16_t>(current) + direction * step;
      if ((direction > 0 && next > target_angle) ||
          (direction < 0 && next < target_angle)) {
        next = target_angle;
      }
      if (next < 0)
        next = 0;
      if (next > 180)
        next = 180;

      current = static_cast<uint8_t>(next);
      HAL_StatusTypeDef ret = set_angle(current);
      if (ret != HAL_OK) {
        return ret;
      }

      if (m_callbacks.delay_cb) {
        std::invoke(m_callbacks.delay_cb, delay_ms);
      }
    }
    return HAL_OK;
  }

  /**
   * @brief 直接设置PWM脉宽
   * @param pulse 脉宽值 (定时器计数)
   * @return HAL_StatusTypeDef
   *
   * @note 高级功能，用于直接控制脉宽
   */
  HAL_StatusTypeDef set_pulse(uint16_t pulse) {
    if (m_htim == nullptr) {
      if (m_callbacks.error_cb) {
        std::invoke(m_callbacks.error_cb, HAL_ERROR);
      }
      return HAL_ERROR;
    }
    __HAL_TIM_SET_COMPARE(m_htim, m_channel, pulse);
    return HAL_OK;
  }

  // --- 配置获取与回调注册接口 ---
  const ServoConfig &get_config() const { return m_config; }
  void update_config(const ServoConfig &config) { m_config = config; }

  void register_move_complete_callback(move_complete_callback_t cb) {
    m_callbacks.move_complete_cb = std::move(cb);
  }
  void register_error_callback(error_callback_t cb) {
    m_callbacks.error_cb = std::move(cb);
  }

protected:
  struct Callbacks {
    move_complete_callback_t move_complete_cb{nullptr};
    error_callback_t error_cb{nullptr};
    delay_callback_t delay_cb{nullptr};
  };

private:
  TIM_HandleTypeDef *m_htim; //!< 定时器句柄
  uint32_t m_channel;        //!< 定时器通道
  ServoConfig m_config;      //!< 舵机配置参数
  uint8_t m_current_angle;   //!< 当前角度

  Callbacks m_callbacks;
};

} // namespace gdut

#endif // BSP_SERVO_HPP