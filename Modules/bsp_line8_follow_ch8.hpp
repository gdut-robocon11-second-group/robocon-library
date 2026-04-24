#ifndef BSP_LINE_FOLLOWER_8CH_HPP
#define BSP_LINE_FOLLOWER_8CH_HPP

#include "bsp_iic.hpp"
#include "stm32f4xx_hal.h"

#include <array>
#include <chrono>
#include <cstdint>

namespace gdut {

class line_follower_8ch {
public:
  // 对控制方式的读写
  enum class comm_mode : uint8_t {
    null = 0,
    i2c,
    gpio,
  };

  // 总共8个通道
  static constexpr uint8_t channel_count = 8;
  // I2C 地址
  static constexpr uint8_t i2c_addr_7bit = 0x5D;
  // HAL I2C 接口通常要求传入左移 1 位后的地址。
  static constexpr uint16_t i2c_addr_hal =
      static_cast<uint16_t>(i2c_addr_7bit << 1);

  // 协议寄存器
  // 5寄存器是线检测结果寄存器
  // 6是通道一的灰度模拟值寄存器
  static constexpr uint8_t reg_line_result = 0x05;
  static constexpr uint8_t reg_gray_ch1 = 0x06;
  static constexpr uint8_t reg_threshold_ch1 = 0x16;
  // gpio口
  struct gpio_pin {
    GPIO_TypeDef *port{nullptr};
    uint16_t pin{0};
  };

  using delay_us_callback_t = void (*)(uint32_t us);

  struct gpio_bus {
    // 三个选择器，一个输出
    gpio_pin ad0{};
    gpio_pin ad1{};
    gpio_pin ad2{};
    gpio_pin out{};

    // 可选：用于切换 AD0/AD1/AD2 后给模块一点稳定时间。
    // 若有自己的 us 延时函数，可传进来；没有也能用。
    delay_us_callback_t delay_us{nullptr};
    uint32_t settle_us{5};
  };

  struct snapshot_gpio {
    // 统一语义：1 = 检测到目标线；0 = 未检测到目标线。
    std::array<uint8_t, channel_count> line{};
  };

  struct snapshot_i2c {
    // 统一语义：1 = 检测到目标线；0 = 未检测到目标线。
    std::array<uint8_t, channel_count> line{};
    std::array<uint16_t, channel_count> gray{};
    std::array<uint16_t, channel_count> threshold{};
  };

  line_follower_8ch() = default;
  ~line_follower_8ch() = default;

  // 设置参数，支持两种方式：I2C 或 GPIO
  void set_parameters(gdut::i2c &bus, uint16_t hal_addr = i2c_addr_hal) {
    m_mode = comm_mode::i2c;
    m_i2c = &bus;
    m_i2c_hal_addr = hal_addr;
  }

  void set_parameters(const gpio_bus &bus) {
    m_mode = comm_mode::gpio;
    m_i2c = nullptr;
    m_gpio = bus;
  }

  // 两种方式的构造函数
  explicit line_follower_8ch(gdut::i2c &bus, uint16_t hal_addr = i2c_addr_hal);
  explicit line_follower_8ch(const gpio_bus &bus);
  // 模式的识别
  comm_mode mode() const { return m_mode; }
  bool is_i2c_mode() const { return m_mode == comm_mode::i2c; }
  bool is_gpio_mode() const { return m_mode == comm_mode::gpio; }

  // 探测设备是否在线、是否能正常通信
  HAL_StatusTypeDef
  probe(std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  // 统一返回：bit=1 表示检测到目标线，一次性读取所有的数组
  HAL_StatusTypeDef read_line_mask(
      uint8_t &mask,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  // 放进数组
  HAL_StatusTypeDef read_line_array(
      std::array<uint8_t, channel_count> &states,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  // 单通道的识别
  HAL_StatusTypeDef read_channel_line(
      uint8_t channel, uint8_t &detected,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  // 仅 I2C 模式支持
  // 读取单通道的灰度模拟值
  HAL_StatusTypeDef read_channel_gray(
      uint8_t channel, uint16_t &gray,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  // 读取单通道的阈值
  HAL_StatusTypeDef read_channel_threshold(
      uint8_t channel, uint16_t &threshold,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  // 读取所有通道的灰度模拟值
  HAL_StatusTypeDef read_all_gray(
      std::array<uint16_t, channel_count> &gray,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  // 读取所有通道的阈值
  HAL_StatusTypeDef read_all_threshold(
      std::array<uint16_t, channel_count> &threshold,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  // 读取所有通道的灰度模拟值和阈值
  HAL_StatusTypeDef read_snapshot(
      snapshot_gpio &data, bool with_gray = true, bool with_threshold = false,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  HAL_StatusTypeDef read_snapshot(
      snapshot_i2c &data, bool with_gray = true, bool with_threshold = false,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

private:
  static bool valid_channel(uint8_t channel) {
    return channel >= 1 && channel <= channel_count;
  }

  static uint8_t gray_reg(uint8_t channel) {
    return static_cast<uint8_t>(reg_gray_ch1 + (channel - 1U) * 2U);
  }

  static uint8_t threshold_reg(uint8_t channel) {
    return static_cast<uint8_t>(reg_threshold_ch1 + (channel - 1U) * 2U);
  }

  static uint8_t
  make_mask_from_array(const std::array<uint8_t, channel_count> &states);

  HAL_StatusTypeDef i2c_read_u8(
      uint8_t reg, uint8_t &value,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  HAL_StatusTypeDef i2c_read_u16_le(
      uint8_t reg, uint16_t &value,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(20));

  HAL_StatusTypeDef gpio_select_channel(uint8_t channel);
  HAL_StatusTypeDef gpio_read_out(GPIO_PinState &pin_state);

private:
  comm_mode m_mode{comm_mode::null};
  gdut::i2c *m_i2c{nullptr};
  uint16_t m_i2c_hal_addr{i2c_addr_hal};
  gpio_bus m_gpio{};
};

} // namespace gdut

#endif // BSP_LINE_FOLLOWER_8CH_HPP
