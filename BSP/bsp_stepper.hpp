#ifndef BSP_STEPPER_HPP
#define BSP_STEPPER_HPP

#include "bsp_gpio_pin.hpp"
#include "bsp_timer.hpp"
#include "uncopyable.hpp"

#include <cstdint>
#include <atomic>

namespace gdut {

/**
 * @brief 硬件PWM版 42步进电机驱动
 *
 * 使用定时器的PWM通道直接输出STEP脉冲（50%占空比），
 * Update中断仅用于精确计数步数（不影响脉冲精度）。
 *
 * CubeMX 配置要求：
 *   1. STEP引脚 → 对应定时器的 PWM Generation 通道（AF模式）
 *   2. 定时器时钟 + PSC 配置为 1us 分辨率（即计数频率 1MHz）
 *   3. 必须开启 Update Interrupt（NVIC）
 *   4. DIR 和 ENABLE 为普通 GPIO Output
 */
class pwm_stepper_motor : private uncopyable {
public:
  /**
   * @param dir_pin       DIR 引脚
   * @param enable_pin    ENABLE 引脚（低电平有效）
   * @param step_timer    已配置为PWM模式的 timer 对象
   * @param pwm_channel   PWM通道（TIM_CHANNEL_1 ~ TIM_CHANNEL_4）
   */
  pwm_stepper_motor(gpio_proxy& dir_pin,
                    gpio_proxy& enable_pin,
                    timer& step_timer,
                    uint32_t pwm_channel = TIM_CHANNEL_1)
      : m_dir_pin(dir_pin),
        m_enable_pin(enable_pin),
        m_step_timer(step_timer),
        m_pwm_channel(pwm_channel),
        m_remaining_steps(0) {

    m_step_timer.register_period_elapsed_callback(
        [this]() { handle_step_isr(); });
  }

  ~pwm_stepper_motor() {
    stop();
  }

  void set_direction(bool clockwise) {
    m_dir_pin.write(clockwise ? 1 : 0);
  }

  void enable(bool en = true) {
    m_enable_pin.write(en ? 0 : 1);   // 低电平使能
  }

  void disable() {
    enable(false);
  }

  /**
   * @brief 设置速度（单位：steps/s）
   * @note 要求定时器分辨率为 1us（计数频率 1MHz）
   */
  void set_speed(uint32_t steps_per_sec) {
    if (steps_per_sec == 0) {
      stop();
      return;
    }

    // 限制最大速度，防止 ARR 过小导致不稳定
    if (steps_per_sec > 50000) steps_per_sec = 50000;

    auto* htim = m_step_timer.get_htim();
    if (!htim) return;

    // 定时器计数频率为 1MHz → 周期单位为 us
    uint32_t period_us = 1000000UL / steps_per_sec;
    if (period_us == 0) period_us = 1;

    __HAL_TIM_SET_AUTORELOAD(htim, period_us - 1);

    // 50% 占空比
    uint32_t ccr = (period_us + 1) / 2;

    timer::timer_pwm pwm_helper(&m_step_timer);
    pwm_helper.set_duty(m_pwm_channel, ccr);

    // 启动 PWM
    if (HAL_TIM_PWM_GetState(htim) != HAL_TIM_STATE_BUSY) {
      pwm_helper.pwm_start(m_pwm_channel);
    }

    m_step_timer.enable_it(TIM_IT_UPDATE);
  }

  /**
   * @brief 非阻塞移动指定步数，完成后自动停止
   */
  void move_steps(uint32_t num_steps, uint32_t steps_per_sec) {
    if (num_steps == 0) return;

    m_remaining_steps = num_steps;
    set_speed(steps_per_sec);
  }

  /** 立即停止运动 */
  void stop() {
    auto* htim = m_step_timer.get_htim();
    if (htim) {
      timer::timer_pwm pwm_helper(&m_step_timer);
      pwm_helper.pwm_stop(m_pwm_channel);
      m_step_timer.disable_it(TIM_IT_UPDATE);
    }
    m_remaining_steps = 0;
  }

  bool is_moving() const noexcept {
    return m_remaining_steps > 0;
  }

  uint32_t get_remaining_steps() const noexcept {
    return m_remaining_steps;
  }

private:
  void handle_step_isr() {
    if (m_remaining_steps > 0) {
      --m_remaining_steps;
      if (m_remaining_steps == 0) {
        stop();
      }
    }
  }

private:
  gpio_proxy& m_dir_pin;
  gpio_proxy& m_enable_pin;
  timer& m_step_timer;
  uint32_t m_pwm_channel;

  std::atomic<uint32_t> m_remaining_steps{0};   // 剩余步数，Update中断递减
};

} // namespace gdut

#endif // BSP_STEPPER_HPP