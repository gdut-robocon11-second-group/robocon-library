#ifndef BSP_PS2_HPP
#define BSP_PS2_HPP

#include "bsp_gpio_pin.hpp"
#include "bsp_spi.hpp"
#include "uncopyable.hpp"
#include "function.hpp"

#include <utility>
#include <array>
#include <cstdint>

namespace gdut {

/**
 * @brief PS2 手柄状态
 *
 * - buttons：16 位按键位图，1 表示按下，0 表示松开
 *   bit0=Select, bit1=L3, bit2=R3, bit3=Start,
 *   bit4=Up, bit5=Right, bit6=Down, bit7=Left,
 *   bit8=L2, bit9=R2, bit10=L1, bit11=R1,
 *   bit12=Triangle, bit13=Circle, bit14=Cross, bit15=Square
 *
 * - left_x / left_y：左摇杆原始坐标
 * - right_x / right_y：右摇杆原始坐标
 */
struct ps2_state {
  uint16_t buttons{0};
  uint8_t left_x{0};
  uint8_t left_y{0};
  uint8_t right_x{0};
  uint8_t right_y{0};
};

class ps2_controller : private uncopyable{
public:
  struct pins_interface {
    gdut::function<void(bool)> set_att;
    gdut::function<void(uint32_t)> delay_ms;
  };

  explicit ps2_controller(pins_interface pins, spi_proxy *spi);
  ~ps2_controller() = default;

  // init: 做基本引脚状态初始化 + 尝试握手（握手失败不会崩溃）
  void init();

  // handshake: 发送 PS2 配置序列，成功返回 true
  bool handshake();

// poll: 读一次状态（握手失败时也允许你继续 poll，看模块是否仍会回数据）
  bool poll();
  ps2_state read_state() const;
  
  void on_change(gdut::function<void(const ps2_state &)> cb);

private:
  bool transfer_frame(std::array<uint8_t, 9> &tx, std::array<uint8_t, 9> &rx);
  void parse_state(const std::array<uint8_t, 9> &rx);
  bool transfer_packet(const std::array<uint8_t, 9> &tx, std::array<uint8_t, 9> &rx);

  spi_proxy *m_spi{nullptr};
  pins_interface m_pins;
  ps2_state m_state{};
  gdut::function<void(const ps2_state &)> m_on_change;
};

template <typename Att>
ps2_controller make_ps2_controller(
    Att &att, gdut::function<void(uint32_t)> delay_ms, spi_proxy *spi) {
  ps2_controller::pins_interface pins;
  pins.set_att = [&att](bool v) { att.write(v); };
  pins.delay_ms = std::move(delay_ms);
  return ps2_controller(std::move(pins), spi);
}

} // namespace gdut

#endif // BSP_PS2_HPP