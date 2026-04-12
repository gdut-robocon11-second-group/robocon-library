#ifndef BSP_MOTOR_HPP
#define BSP_MOTOR_HPP

#include "bsp_timer.hpp"
#include "stm32f407xx.h"
#include "uncopyable.hpp"
#include <algorithm>
#include <cstdint>

namespace gdut {

class motor : private gdut::uncopyable {
public:
  // pwm_timer:用于 PWM 输出的timer对象指针
  // pwm_channel_A:正转通道
  // direction_gpio_port:用于控制电机转向的 GPIO 端口
  // direction_gpio_pin:用于控制电机转向的 GPIO 引脚
  // encoder_timer:编码器计数的timer对象指针
  // ppr: 编码器一圈的脉冲数
  // control_period_sec:控制周期（秒）
  motor(gdut::timer *pwm_timer, uint32_t pwm_channel_A,
        GPIO_TypeDef *direction_gpio_port, uint16_t direction_gpio_pin,
        gdut::timer *encoder_timer, float ppr, float control_period_sec = 0.01f)
      : pwm_timer_(pwm_timer), encoder_timer_(encoder_timer),
        pwm_channel_A_(pwm_channel_A),
        direction_gpio_port_(direction_gpio_port),
        direction_gpio_pin_(direction_gpio_pin), ppr_(ppr),
        control_period_(control_period_sec), current_encoder_count_(0),
        total_revolutions_(0.0f), current_speed_(0.0f), enabled_(true) {
    init_encoder_state();
  }

  // 移动构造
  motor(motor &&other) noexcept
      : pwm_timer_(other.pwm_timer_), encoder_timer_(other.encoder_timer_),
        pwm_channel_A_(other.pwm_channel_A_),
        direction_gpio_port_(other.direction_gpio_port_),
        direction_gpio_pin_(other.direction_gpio_pin_), ppr_(other.ppr_),
        control_period_(other.control_period_),
        current_encoder_count_(other.current_encoder_count_),
        total_revolutions_(other.total_revolutions_),
        current_speed_(other.current_speed_), enabled_(other.enabled_) {
    other.pwm_timer_ = nullptr;
    other.encoder_timer_ = nullptr;
  }

  // ----- 获取状态 -----
  float get_current_speed() const { return current_speed_; } // 当前转速

  float get_total_revolutions() const {
    return total_revolutions_;
  } // 累计转动圈数

  int32_t get_current_encoder_count() const { return current_encoder_count_; }

  // ----- 控制 -----
  void enable(bool enable) { // 使能或禁用输出
    enabled_ = enable;
    if (!enabled_) {
      set_pwm_duty(0.0f);
    }
  }

  // 刷新编码器状态（建议在定时器中断中周期调用）
  void refresh_encoder_state() {
    if (!encoder_timer_)
      return;

    // 读取编码器计数值
    gdut::timer::timer_proxy encoder_proxy(encoder_timer_);
    const int32_t previous_encoder_count = current_encoder_count_;
    current_encoder_count_ = static_cast<int32_t>(encoder_proxy.get_counter());
    int32_t delta_count = current_encoder_count_ - previous_encoder_count;

    // 更新累计圈数
    total_revolutions_ = static_cast<float>(current_encoder_count_) / ppr_;

    // 计算当前转速（转/秒）
    current_speed_ = static_cast<float>(delta_count) / (ppr_ * control_period_);
  }

protected:
  void init_encoder_state() {
    if (!encoder_timer_)
      return;
    gdut::timer::timer_proxy proxy(encoder_timer_);
    current_encoder_count_ = static_cast<int32_t>(proxy.get_counter());
    total_revolutions_ = static_cast<float>(current_encoder_count_) / ppr_;
  }

  void set_pwm_duty(float duty) { // 设置两个通道的占空比
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

    if (clamped_duty >= 0.0f) {
      direction_gpio_port_->BSRR = direction_gpio_pin_;
    } else {
      direction_gpio_port_->BSRR = static_cast<uint32_t>(direction_gpio_pin_)
                                   << 16U;
    }
    pwm.set_duty(pwm_channel_A_, compare_A);
  }

private:
  // 硬件句柄
  gdut::timer *pwm_timer_;
  gdut::timer *encoder_timer_;
  uint32_t pwm_channel_A_;
  GPIO_TypeDef *direction_gpio_port_;
  uint16_t direction_gpio_pin_;

  // 编码器参数
  float ppr_;            // 一圈脉冲数
  float control_period_; // 控制周期

  // 状态变量
  int32_t current_encoder_count_;
  float total_revolutions_;
  float current_speed_; // 转/秒

  bool enabled_;
};

} // namespace gdut

#endif // BSP_MOTOR_HPP
