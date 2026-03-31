#ifndef BSP_STEPPER_HPP
#define BSP_STEPPER_HPP

#include "bsp_gpio_pin.hpp"
#include "bsp_timer.hpp"
#include "uncopyable.hpp"

#include <atomic>
#include <cstdint>
#include <algorithm>

namespace gdut {

/**
 * @brief 硬件PWM步进电机驱动（支持任意定时器时钟）
 *
 * 使用定时器PWM通道生成STEP脉冲（约50%占空比）。
 * Update中断仅用于步数计数，实现非阻塞 move_steps 并自动停止。
 */
class pwm_stepper_motor : private uncopyable {
public:
  /**
   * @param dir_pin       DIR 引脚
   * @param enable_pin    ENABLE 引脚（低电平有效）
   * @param step_timer    已配置好的 timer 对象
   * @param pwm_channel   PWM通道（TIM_CHANNEL_1~4）
   * @param is_apb2       该定时器是否挂在 APB2 总线上（TIM1/TIM8/TIM9~TIM11 为 true，其余通常为 false）
   */
  pwm_stepper_motor(gpio_proxy& dir_pin,
                    gpio_proxy& enable_pin,
                    timer& step_timer,
                    uint32_t pwm_channel = TIM_CHANNEL_1,
                    bool is_apb2 = false)
      : m_dir_pin(dir_pin),
        m_enable_pin(enable_pin),
        m_step_timer(step_timer),
        m_pwm_channel(pwm_channel),
        m_is_apb2(is_apb2),
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
    m_enable_pin.write(en ? 0 : 1);  // 低电平使能
  }

  void disable() { enable(false); }

  bool is_enabled() const {
    return m_enable_pin.read() == 0;
  }

  /**
   * @brief 设置速度（单位：steps/s）
   */
  void set_speed(uint32_t steps_per_sec) {
    if (steps_per_sec == 0) {
      stop();
      return;
    }
    if (steps_per_sec > 100000) steps_per_sec = 100000;

    auto* htim = m_step_timer.get_htim();
    if (!htim) return;

    uint32_t timer_clk = get_timer_clock();   // 内部自动计算
    if (timer_clk == 0) return;

    // 目标计数器频率控制在 1MHz~2MHz 左右（分辨率与范围平衡）
    uint64_t target_cnt_freq = std::clamp<uint64_t>(
        static_cast<uint64_t>(steps_per_sec) * 2ULL, 1000000ULL, 2000000ULL);

    uint64_t max_arr = (htim->Instance == TIM2 || htim->Instance == TIM5) 
                       ? 0xFFFFFFFFULL : 0xFFFFULL;

    // 计算 PSC 和 ARR
    uint32_t psc = 0;
    uint32_t arr = 0;

    uint64_t ideal_psc = (timer_clk / target_cnt_freq);
    if (ideal_psc > 0) ideal_psc -= 1;
    if (ideal_psc > 0xFFFF) ideal_psc = 0xFFFF;

    psc = static_cast<uint32_t>(ideal_psc);

    uint64_t actual_cnt_freq = timer_clk / (psc + 1);
    arr = static_cast<uint32_t>(actual_cnt_freq / steps_per_sec);

    if (arr == 0) arr = 1;
    if (arr > max_arr) arr = static_cast<uint32_t>(max_arr);

    // 设置预分频和周期
    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, arr);

    // 50% 占空比
    uint32_t ccr = (arr + 1) / 2;

    timer::timer_pwm pwm_helper(&m_step_timer);
    pwm_helper.set_duty(m_pwm_channel, ccr);

    // 启动 PWM
    if (HAL_TIM_PWM_GetState(htim) != HAL_TIM_STATE_BUSY) {
      pwm_helper.pwm_start(m_pwm_channel);
    }

    m_step_timer.enable_it(TIM_IT_UPDATE);
  }

  void move_steps(uint32_t num_steps, uint32_t steps_per_sec) {
    if (num_steps == 0) return;
    m_remaining_steps.store(num_steps, std::memory_order_relaxed);
    set_speed(steps_per_sec);
  }

  void stop() {
    auto* htim = m_step_timer.get_htim();
    if (htim) {
      timer::timer_pwm pwm_helper(&m_step_timer);
      pwm_helper.pwm_stop(m_pwm_channel);
      m_step_timer.disable_it(TIM_IT_UPDATE);
    }
    m_remaining_steps.store(0, std::memory_order_relaxed);
  }

  bool is_moving() const noexcept {
    return m_remaining_steps.load(std::memory_order_relaxed) > 0;
  }

  uint32_t get_remaining_steps() const noexcept {
    return m_remaining_steps.load(std::memory_order_relaxed);
  }

private:
  /**
   * @brief 内部获取定时器实际输入时钟频率（Hz）
   */
  uint32_t get_timer_clock() const {
    uint32_t pclk = m_is_apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    if (pclk == 0) return 0;

    // STM32F4 定时器时钟规则：若对应 APB 预分频 > 1，则定时器时钟 = PCLK × 2
    if (m_is_apb2) {
      if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0) {  // PPRE2 不为 1
        return pclk * 2;
      }
    } else {
      if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0) {  // PPRE1 不为 1
        return pclk * 2;
      }
    }
    return pclk;
  }

  void handle_step_isr() {
    auto remaining = m_remaining_steps.load(std::memory_order_relaxed);
    if (remaining > 0) {
      --remaining;
      m_remaining_steps.store(remaining, std::memory_order_relaxed);
      if (remaining == 0) {
        stop();
      }
    }
  }

private:
  gpio_proxy& m_dir_pin;
  gpio_proxy& m_enable_pin;
  timer& m_step_timer;
  uint32_t m_pwm_channel;
  bool m_is_apb2;                     

  std::atomic<uint32_t> m_remaining_steps{0};
};

} // namespace gdut

#endif // BSP_STEPPER_HPP