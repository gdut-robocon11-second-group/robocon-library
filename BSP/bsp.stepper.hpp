#ifndef BSP_STEPPER_HPP
#define BSP_STEPPER_HPP

#include "bsp_gpio_pin.hpp"
#include "bsp_timer.hpp"
#include "bsp_uncopyable.hpp"

#include <chrono>
#include <cstdint>

namespace gdut {

/**
 * @brief 硬件PWM版 42步进电机驱动
 * 使用定时器的PWM通道直接输出STEP脉冲（50%占空比），
 * Update中断仅用于精确计数步数（不影响脉冲精度）。
 *
 * CubeMX 配置要求：
 *   1. STEP引脚 → 对应定时器的 PWM Generation 通道（AF模式）
 *   2. 定时器时钟建议设为 1MHz（PSC = APB时钟/1M - 1），方便计算
 *   3. 必须开启 Update Interrupt（NVIC）
 *   4. DIR / ENABLE 仍为普通GPIO Output PP
 */
template <typename DirPinType, typename EnablePinType>
class pwm_stepper_motor : private uncopyable {
public:
  /**
   * @param dir_pin      DIR 引脚（gpio_pin）
   * @param enable_pin   ENABLE 引脚（低电平有效）
   * @param step_timer   已配置为PWM的 timer 对象
   * @param pwm_channel  使用的PWM通道（TIM_CHANNEL_1~4）
   */
  pwm_stepper_motor(DirPinType& dir_pin,
                    EnablePinType& enable_pin,
                    timer& step_timer,
                    uint32_t pwm_channel = TIM_CHANNEL_1)
      : m_dir_pin(dir_pin),
        m_enable_pin(enable_pin),
        m_step_timer(step_timer),
        m_pwm_channel(pwm_channel),
        m_remaining_steps(0) {
    // 注册Update中断回调（每输出一个完整脉冲触发一次）
    m_step_timer.register_period_elapsed_callback(
        [this]() { handle_step_isr(); });
  }

  void set_direction(bool clockwise) { m_dir_pin.write(clockwise); }

  void enable(bool en = true) { m_enable_pin.write(!en); }  // 大多数驱动器低电平使能
  void disable() { enable(false); }

  /**
   * @brief 设置恒速（单位：steps/s）
   * @note 内部自动把PWM频率设为 steps_per_sec（50%占空比）
   */
  void set_speed(uint32_t steps_per_sec) {
    if (steps_per_sec == 0) {
      stop();
      return;
    }
    if (steps_per_sec > 50000) steps_per_sec = 50000;

    // 假设定时器分辨率为 1us（CubeMX 中已配置好PSC）
    uint32_t interval_us = 1000000ULL / steps_per_sec;

    auto* htim = m_step_timer.get_htim();
    if (!htim) return;

    __HAL_TIM_SET_AUTORELOAD(htim, interval_us ? interval_us - 1 : 0);

    // 50% 占空比
    uint32_t ccr = interval_us / 2;
    timer::timer_pwm pwm_helper(&m_step_timer);
    pwm_helper.set_duty(m_pwm_channel, ccr);

    // 启动PWM（如果还没启动）
    if (HAL_TIM_PWM_GetState(htim) != HAL_TIM_STATE_BUSY) {
      pwm_helper.pwm_start(m_pwm_channel);
    }

    // 开启Update中断用于步数计数
    m_step_timer.enable_it(TIM_IT_UPDATE);
  }

  /**
   * @brief 运动指定步数（非阻塞，自动停止）
   */
  void move_steps(uint32_t num_steps, uint32_t steps_per_sec) {
    if (num_steps == 0) return;
    m_remaining_steps = num_steps;
    set_speed(steps_per_sec);
  }

  /** 立即停止 */
  void stop() {
    auto* htim = m_step_timer.get_htim();
    if (htim) {
      timer::timer_pwm pwm_helper(&m_step_timer);
      pwm_helper.pwm_stop(m_pwm_channel);
      m_step_timer.disable_it(TIM_IT_UPDATE);
    }
    m_remaining_steps = 0;
  }

  bool is_moving() const noexcept { return m_remaining_steps > 0; }

private:
  void handle_step_isr() {
    // 硬件PWM每产生一个完整脉冲，Update中断触发一次 → 计一步
    if (m_remaining_steps > 0) {
      --m_remaining_steps;
      if (m_remaining_steps == 0) {
        stop();
      }
    }
  }

  DirPinType& m_dir_pin;
  EnablePinType& m_enable_pin;
  timer& m_step_timer;
  uint32_t m_pwm_channel;
  uint32_t m_remaining_steps{0};
};

} // namespace gdut

#endif // BSP_STEPPER_HPP