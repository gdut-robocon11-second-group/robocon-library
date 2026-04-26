#ifndef BSP_PCA9685_HPP
#define BSP_PCA9685_HPP

#include "bsp_iic.hpp"
#include "clock.hpp"
#include <array>
#include <chrono>
#include <cstdint>

namespace gdut {

class pca9685 {
public:
  static constexpr std::uint8_t channel_count = 16; // 16路输出
  static constexpr std::uint16_t resolution = 4096; // 4096步分辨率
  static constexpr float internal_oscillator_hz = 25000000.0f; // 25MHz内部振荡器

  // PCA9685 默认 7bit 地址是 0x40（不含读写位）
  // 设备地址要左移 1 位，留读写位
  static constexpr std::uint8_t default_address_7bit = 0x40;
  static constexpr std::uint16_t default_address = (default_address_7bit << 1);

  explicit pca9685(gdut::i2c &bus,
                   std::uint16_t dev_addr = default_address) noexcept
      : m_bus(&bus), m_dev_addr(dev_addr) {}

  ~pca9685() noexcept = default;

  pca9685(const pca9685 &) = delete;
  pca9685 &operator=(const pca9685 &) = delete;
  pca9685(pca9685 &&) = delete;
  pca9685 &operator=(pca9685 &&) = delete;

  // 初始化：配置 MODE1 / MODE2，并设置 PWM 频率
  HAL_StatusTypeDef init(float pwm_freq_hz = 50.0f);

  // 探测设备是否存在
  HAL_StatusTypeDef
  is_ready(std::uint32_t trials = 3,
           // 应该是这样用的，我忘了，可能会错
           std::chrono::milliseconds timeout = std::chrono::milliseconds(10));

  // 频率配置，所有的pwm都是同一个频率
  HAL_StatusTypeDef set_pwm_freq(float freq_hz);
  float get_pwm_freq() const noexcept { return m_pwm_freq_hz; }

  // 基础寄存器读写函数
  HAL_StatusTypeDef write_register(std::uint8_t reg, std::uint8_t value);
  HAL_StatusTypeDef read_register(std::uint8_t reg, std::uint8_t &value);

  // 多字节连续写
  HAL_StatusTypeDef write_registers(std::uint8_t reg, const std::uint8_t *data,
                                    std::uint16_t size);

  // 通道原始 PWM 设置
  HAL_StatusTypeDef set_pwm(std::uint8_t channel, std::uint16_t on,
                            std::uint16_t off);

  // 直接按 0~4095 设置占空值
  HAL_StatusTypeDef set_duty(std::uint8_t channel, std::uint16_t duty,
                             bool invert = false);

  // 全通道统一输出
  HAL_StatusTypeDef set_all_pwm(std::uint16_t on, std::uint16_t off);

  // 全关 / 全开
  HAL_StatusTypeDef all_off();
  HAL_StatusTypeDef all_on();

  // 舵机控制接口
  HAL_StatusTypeDef set_servo_pulse_us(std::uint8_t channel, float pulse_us);
  HAL_StatusTypeDef set_servo_angle(std::uint8_t channel, float angle_deg,
                                    float min_pulse_us = 500.0f,
                                    float max_pulse_us = 2500.0f,
                                    float max_angle_deg = 180.0f);

  // 休眠 / 唤醒
  HAL_StatusTypeDef sleep();
  HAL_StatusTypeDef wakeup();

  // 软件复位当前芯片常用方式：恢复寄存器到上电配置不太适合直接做普通成员函数，
  // 一般工程里更常用重新 init；这里先不封 SWRST General Call。

private:
  // 寄存器地址
  enum class reg : std::uint8_t {
    mode1 = 0x00,
    mode2 = 0x01,
    subadr1 = 0x02,
    subadr2 = 0x03,
    subadr3 = 0x04,
    allcalladr = 0x05,

    led0_on_l = 0x06,
    led0_on_h = 0x07,
    led0_off_l = 0x08,
    led0_off_h = 0x09,

    all_led_on_l = 0xFA,
    all_led_on_h = 0xFB,
    all_led_off_l = 0xFC,
    all_led_off_h = 0xFD,

    pre_scale = 0xFE
  };

  // MODE1 bits
  static constexpr std::uint8_t mode1_allcall = 0x01;
  static constexpr std::uint8_t mode1_sub3 = 0x02;
  static constexpr std::uint8_t mode1_sub2 = 0x04;
  static constexpr std::uint8_t mode1_sub1 = 0x08;
  static constexpr std::uint8_t mode1_sleep = 0x10;
  static constexpr std::uint8_t mode1_ai = 0x20;
  static constexpr std::uint8_t mode1_extclk = 0x40;
  static constexpr std::uint8_t mode1_restart = 0x80;

  // MODE2 bits
  static constexpr std::uint8_t mode2_outne_0 = 0x01;
  static constexpr std::uint8_t mode2_outne_1 = 0x02;
  static constexpr std::uint8_t mode2_outdrv = 0x04;
  static constexpr std::uint8_t mode2_och = 0x08;
  static constexpr std::uint8_t mode2_invrt = 0x10;

private:
  static constexpr std::uint16_t clamp_u12(std::uint16_t value) noexcept {
    return (value > 4095U) ? 4095U : value;
  }

  static float clamp_freq(float freq_hz) noexcept;
  static std::uint8_t calc_prescale(float freq_hz) noexcept;

  HAL_StatusTypeDef update_mode1_bits(std::uint8_t set_mask,
                                      std::uint8_t clear_mask);

private:
  gdut::i2c *m_bus{nullptr};
  std::uint16_t m_dev_addr{default_address};
  float m_pwm_freq_hz{50.0f};
};

} // namespace gdut

#endif // BSP_PCA9685_HPP