#ifndef BSP_PS2_HPP
#define BSP_PS2_HPP

#include "bsp_gpio_pin.hpp"
#include "uncopyable.hpp"
#include <array>
#include <cstdint>
#include <functional>

namespace gdut {

class spi_proxy; // forward declare optional SPI proxy

/**
 * @brief PPS2手柄状态表示
 */
struct ps2_state {
  uint16_t buttons{0};
  int8_t lx{0};
  int8_t ly{0};
  int8_t rx{0};
  int8_t ry{0};
  bool analog{false};
};

class ps2_controller : private uncopyable {
public:
  struct pins_interface {
    std::function<void(bool)> set_att;
    std::function<void(bool)> write_cmd;
    std::function<void(bool)> set_clk;
    std::function<bool()> read_dat;
    std::function<void(uint32_t)> delay_us;
  };

  explicit ps2_controller(pins_interface pins, spi_proxy *spi);
  ~ps2_controller() = default;

  void init();
  bool poll();
  ps2_state read_state() const;

  void set_mode_digital();
  void set_mode_analog();

  void on_change(std::function<void(const ps2_state &)> cb);

private:
  uint8_t exchange_byte(uint8_t tx);

  spi_proxy *m_spi{nullptr};

  pins_interface m_pins;
  ps2_state m_state{};
  std::function<void(const ps2_state &)> m_on_change;
};

template <typename Att, typename Cmd, typename Clk, typename Dat>
ps2_controller
make_ps2_controller(Att &att, Cmd &cmd, Clk &clk, Dat &dat,
                    std::function<void(uint32_t)> delay_us,
                    spi_proxy *spi) {
  ps2_controller::pins_interface pins;
  pins.set_att = [&att](bool v) { att.write(v); };
  pins.write_cmd = [&cmd](bool v) { cmd.write(v); };
  pins.set_clk = [&clk](bool v) { clk.write(v); };
  pins.read_dat = [&dat]() { return dat.read(); };
  pins.delay_us = std::move(delay_us);
  return ps2_controller(std::move(pins), spi);
}

} // namespace gdut

#endif // BSP_PS2_HPP