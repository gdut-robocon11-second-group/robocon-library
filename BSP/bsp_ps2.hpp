#ifndef BSP_PS2_HPP
#define BSP_PS2_HPP

#include "bsp_spi.hpp"
#include "function.hpp"
#include "uncopyable.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <span>

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
 * - right_x / right_y：右摇杆原始坐标（回包字节 5/6）
 * - left_x / left_y：左摇杆原始坐标（回包字节 7/8）
 */
struct ps2_bottons {
  inline static constexpr uint16_t k_select = 0x0001;
  inline static constexpr uint16_t k_l3 = 0x0002;
  inline static constexpr uint16_t k_r3 = 0x0004;
  inline static constexpr uint16_t k_start = 0x0008;
  inline static constexpr uint16_t k_up = 0x0010;
  inline static constexpr uint16_t k_right = 0x0020;
  inline static constexpr uint16_t k_down = 0x0040;
  inline static constexpr uint16_t k_left = 0x0080;
  inline static constexpr uint16_t k_l2 = 0x0100;
  inline static constexpr uint16_t k_r2 = 0x0200;
  inline static constexpr uint16_t k_l1 = 0x0400;
  inline static constexpr uint16_t k_r1 = 0x0800;
  inline static constexpr uint16_t k_triangle = 0x1000;
  inline static constexpr uint16_t k_circle = 0x2000;
  inline static constexpr uint16_t k_cross = 0x4000;
  inline static constexpr uint16_t k_square = 0x8000;
};

struct ps2_state {
  uint16_t buttons{0};
  uint8_t left_x{0};
  uint8_t left_y{0};
  uint8_t right_x{0};
  uint8_t right_y{0};

  bool is_button_pressed(uint16_t keymask) const {
    return (buttons & keymask) != 0;
  }

  bool select_is_pressed() const {
    return is_button_pressed(ps2_bottons::k_select);
  }

  bool l3_is_pressed() const { return is_button_pressed(ps2_bottons::k_l3); }

  bool r3_is_pressed() const { return is_button_pressed(ps2_bottons::k_r3); }

  bool start_is_pressed() const {
    return is_button_pressed(ps2_bottons::k_start);
  }

  bool up_is_pressed() const { return is_button_pressed(ps2_bottons::k_up); }

  bool right_is_pressed() const {
    return is_button_pressed(ps2_bottons::k_right);
  }

  bool down_is_pressed() const {
    return is_button_pressed(ps2_bottons::k_down);
  }

  bool left_is_pressed() const {
    return is_button_pressed(ps2_bottons::k_left);
  }
  bool l2_is_pressed() const { return is_button_pressed(ps2_bottons::k_l2); }

  bool r2_is_pressed() const { return is_button_pressed(ps2_bottons::k_r2); }

  bool l1_is_pressed() const { return is_button_pressed(ps2_bottons::k_l1); }

  bool r1_is_pressed() const { return is_button_pressed(ps2_bottons::k_r1); }

  bool triangle_is_pressed() const {
    return is_button_pressed(ps2_bottons::k_triangle);
  }

  bool circle_is_pressed() const {
    return is_button_pressed(ps2_bottons::k_circle);
  }

  bool cross_is_pressed() const {
    return is_button_pressed(ps2_bottons::k_cross);
  }

  bool square_is_pressed() const {
    return is_button_pressed(ps2_bottons::k_square);
  }
};

class ps2_controller : private uncopyable {
public:
  struct pins_interface {
    gdut::function<void(bool)> set_att;
    gdut::function<void(uint32_t)> delay_ms;
    gdut::function<void(uint32_t)> delay_us;
  };

  struct config {
    std::chrono::milliseconds spi_timeout{20};
    uint32_t att_guard_us{20U};
  };

  ps2_controller(pins_interface pins, spi_proxy *spi);
  ps2_controller(pins_interface pins, spi_proxy *spi, config cfg);
  ~ps2_controller() = default;

  // init: 做基本引脚状态初始化 + 尝试握手（握手失败不会崩溃）
  bool init();

  // handshake: 发送 PS2 配置序列，成功返回 true
  bool handshake();

  // poll: 读一次状态（握手失败时也允许你继续 poll，看模块是否仍会回数据）
  bool poll();
  ps2_state read_state() const;

  void on_change(gdut::function<void(const ps2_state &)> cb);

protected:
  static bool frame_all_eq(std::span<const uint8_t, 9> rx, uint8_t v);
  static bool frame_looks_dead(std::span<const uint8_t, 9> rx);
  static bool is_valid_mode(uint8_t mode);
  static bool frame_is_valid(std::span<const uint8_t, 9> rx);
  static void delay_us(uint32_t us);

private:
  using frame_t = std::array<uint8_t, 9>;

  inline static constexpr uint8_t k_ps2_ack = 0x5A;
  inline static constexpr uint8_t k_ps2_mode_digital = 0x41;
  inline static constexpr uint8_t k_ps2_mode_analog_red = 0x73;
  inline static constexpr uint8_t k_ps2_mode_analog_pressure = 0x79;

  inline static constexpr frame_t k_cmd_poll = {0x01, 0x42, 0x00, 0x00, 0x00,
                                                0x00, 0x00, 0x00, 0x00};
  inline static constexpr frame_t k_cmd_enter_config = {
      0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  inline static constexpr frame_t k_cmd_set_analog_lock = {
      0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
  inline static constexpr frame_t k_cmd_exit_config = {
      0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

  bool transfer_frame(std::span<const uint8_t, 9> tx, std::span<uint8_t, 9> rx);
  void parse_state(std::span<const uint8_t, 9> rx);
  bool transfer_packet(std::span<const uint8_t, 9> tx,
                       std::span<uint8_t, 9> rx,
                       bool validate_frame = true);

  spi_proxy *m_spi{nullptr};
  pins_interface m_pins;
  config m_cfg{};
  ps2_state m_state{};
  gdut::function<void(const ps2_state &)> m_on_change;
};

} // namespace gdut

#endif // BSP_PS2_HPP
