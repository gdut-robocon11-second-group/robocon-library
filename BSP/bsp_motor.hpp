#ifndef BSP_MOTOR_HPP
#define BSP_MOTOR_HPP

#include "bsp_timer.hpp"
#include "stm32f407xx.h"
#include "uncopyable.hpp"
#include <algorithm>
#include <atomic>
#include <cstdint>

namespace gdut {

class motor : private gdut::uncopyable {
  static_assert(std::atomic<uint32_t>::is_always_lock_free);
  static_assert(std::atomic<float>::is_always_lock_free);

public:
  // pwm_timer:用于 PWM 输出的timer对象指针
  // pwm_channel_A:正转通道
  // direction_gpio_port:用于控制电机转向的 GPIO 端口
  // direction_gpio_pin:用于控制电机转向的 GPIO 引脚
  // encoder_timer:编码器计数的timer对象指针
  // ppr: 编码器一圈的脉冲数
  motor(gdut::timer *pwm_timer, uint32_t pwm_channel_A,
        GPIO_TypeDef *direction_gpio_port, uint16_t direction_gpio_pin,
        gdut::timer *encoder_timer, float ppr)
      : pwm_timer_(pwm_timer), encoder_timer_(encoder_timer),
        pwm_channel_A_(pwm_channel_A),
        direction_gpio_port_(direction_gpio_port),
        direction_gpio_pin_(direction_gpio_pin), ppr_(ppr > 0.0f ? ppr : 1.0f),
        current_encoder_count_(0), total_revolutions_(0.0f),
        current_speed_(0.0f) {
    init_encoder_state();
    if (pwm_timer_) {
      pwm_timer_->start();
      gdut::timer::timer_pwm pwm(pwm_timer_);
      pwm.pwm_start(pwm_channel_A_);
    }
  }

  // 移动构造
  motor(motor &&other) noexcept
      : pwm_timer_(other.pwm_timer_), encoder_timer_(other.encoder_timer_),
        pwm_channel_A_(other.pwm_channel_A_),
        direction_gpio_port_(other.direction_gpio_port_),
        direction_gpio_pin_(other.direction_gpio_pin_), ppr_(other.ppr_),
        current_encoder_count_(
            other.current_encoder_count_.load(std::memory_order_acquire)),
        total_revolutions_(
            other.total_revolutions_.load(std::memory_order_acquire)),
        current_speed_(other.current_speed_.load(std::memory_order_acquire)) {
    other.pwm_timer_ = nullptr;
    other.encoder_timer_ = nullptr;
  }

  // ----- 获取状态 -----
  float get_current_speed() const {
    return current_speed_.load(std::memory_order_acquire);
  } // 当前转速

  float get_total_revolutions() const {
    return total_revolutions_.load(std::memory_order_acquire);
  } // 累计转动圈数

  uint32_t get_current_encoder_count() const {
    return current_encoder_count_.load(std::memory_order_acquire);
  }

  // ----- 控制 -----

  // 刷新编码器状态（建议在定时器中断中周期调用）
  void refresh_encoder_state(float control_period_sec) {
    if (!encoder_timer_)
      return;

    if (control_period_sec <= 0.0f)
      return;

    // 读取编码器计数值
    gdut::timer::timer_proxy encoder_proxy(encoder_timer_);
    const uint32_t previous_encoder_count =
        current_encoder_count_.load(std::memory_order_acquire);
    uint32_t current_counter = encoder_proxy.get_counter();
    const uint64_t counter_width =
        static_cast<uint64_t>(encoder_proxy.get_arr()) + 1ULL;
    const int64_t half_range = static_cast<int64_t>(counter_width / 2ULL);
    int64_t delta_count = static_cast<int64_t>(current_counter) -
                          static_cast<int64_t>(previous_encoder_count);

    // 该解包方法要求：单个控制周期内编码器增量绝对值小于计数器范围的一半
    if (delta_count > half_range) {
      // 发生了向下溢出
      delta_count -= static_cast<int64_t>(counter_width);
    } else if (delta_count < -half_range) {
      // 发生了向上溢出
      delta_count += static_cast<int64_t>(counter_width);
    }

    current_encoder_count_.store(current_counter, std::memory_order_release);

    // 累计圈数：按增量累加，保留真实累计语义
    total_revolutions_.fetch_add(static_cast<float>(delta_count) / ppr_,
                                 std::memory_order_relaxed);

    // 计算当前转速（转/秒）
    current_speed_.store(static_cast<float>(delta_count) /
                             (ppr_ * control_period_sec),
                         std::memory_order_release);
  }

  // 通过 GPIO 控制方向，并设置单个 PWM 通道的占空比
  void set_pwm_duty(float duty) {
    if (!pwm_timer_)
      return;

    // 获取定时器的自动重载值
    gdut::timer::timer_proxy proxy(pwm_timer_);
    uint32_t arr = proxy.get_arr();
    uint32_t max_compare = (arr > 0) ? arr + 1 : 1; // 占空比最大值对应的比较值
    const float clamped_duty = std::clamp(duty, -1.0f, 1.0f);
    const float duty_abs = std::abs(clamped_duty);
    uint32_t compare_A = static_cast<uint32_t>(duty_abs * max_compare);
    gdut::timer::timer_pwm pwm(pwm_timer_);

    // 注意：此处方向由 GPIO 决定，同时GPIO与PWM作差得到最终比较值
    // 所以负方向的占空比需要通过 max_compare - compare_A
    // 来实现反转，而不是直接使用 compare_A
    if (clamped_duty >= 0.0f) {
      HAL_GPIO_WritePin(direction_gpio_port_, direction_gpio_pin_,
                        GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(direction_gpio_port_, direction_gpio_pin_,
                        GPIO_PIN_SET);
      compare_A = max_compare - compare_A;
    }
    pwm.set_duty(pwm_channel_A_, compare_A);
  }

protected:
  void init_encoder_state() {
    if (!encoder_timer_)
      return;
    gdut::timer::timer_encoder encoder(encoder_timer_);
    (void)encoder.encoder_start(TIM_CHANNEL_ALL);
    gdut::timer::timer_proxy proxy(encoder_timer_);
    current_encoder_count_.store(proxy.get_counter(), std::memory_order_release);
    total_revolutions_.store(0.0f, std::memory_order_release);
    current_speed_.store(0.0f, std::memory_order_release);
  }

private:
  // 硬件句柄
  gdut::timer *pwm_timer_;
  gdut::timer *encoder_timer_;
  uint32_t pwm_channel_A_;
  GPIO_TypeDef *direction_gpio_port_;
  uint16_t direction_gpio_pin_;

  // 编码器参数
  float ppr_; // 一圈脉冲数

  // 状态变量
  std::atomic<uint32_t> current_encoder_count_;
  std::atomic<float> total_revolutions_; // 累计圈数
  std::atomic<float> current_speed_;     // 转/秒
};

} // namespace gdut

#endif // BSP_MOTOR_HPP
