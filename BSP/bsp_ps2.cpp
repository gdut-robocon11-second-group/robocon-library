#include "bsp_ps2.hpp"
#include "stm32f4xx_hal.h"
#include "../Components/clock.hpp"
#include <array>
#include <chrono>
#include <thread>

namespace gdut {

static void default_delay_us(uint32_t us) {
  using namespace gdut;
  if (us == 0)
    return;
  auto start = steady_clock::now();
  auto target = start + steady_clock::duration(us);
  // 对较大延时，适当让出调度以减少 CPU 占用
  if (us >= 2000) {
    std::this_thread::sleep_for(std::chrono::microseconds(us - 1000));
  }
  while (steady_clock::now() < target) {
    // 忙等短循环，保证微秒级精度
  }
}

ps2_controller::ps2_controller(pins_interface pins, spi_proxy *spi)
    : m_pins(std::move(pins)), m_on_change(nullptr), m_spi(spi) {
  if (!m_pins.delay_us) {
    m_pins.delay_us = default_delay_us;
  }
}

void ps2_controller::init() {
  if (m_pins.set_att)
    m_pins.set_att(true);
  if (m_pins.write_cmd)
    m_pins.write_cmd(true);
  if (m_pins.set_clk)
    m_pins.set_clk(true);

  m_pins.delay_us(10);
}

uint8_t ps2_controller::exchange_byte(uint8_t tx) {
  // Hardware SPI-only path. Transmit and receive a single byte.
  if (!m_spi) {
    // Controller requires spi_proxy in constructor — return error sentinel.
    return 0xff;
  }

  uint8_t rx = 0;
  // SPI proxy expects pointers; use a modest timeout
  constexpr auto timeout = std::chrono::milliseconds(10);
  if (m_spi->transmit_receive(&tx, &rx, 1, timeout)) {
    return rx;
  }

  // If hardware SPI fails, return error sentinel
  return 0xff;
}
}

bool ps2_controller::poll() {
  std::array<uint8_t, 9> tx = {0x01, 0x42, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00};
  std::array<uint8_t, 9> rx{};

  if (m_pins.set_att)
    m_pins.set_att(false);
  m_pins.delay_us(20);

  for (size_t i = 0; i < tx.size(); ++i) {
    rx[i] = exchange_byte(tx[i]);
  }

  if (m_pins.set_att)
    m_pins.set_att(true);

  ps2_state new_state{};
  if (rx.size() >= 5) {
    new_state.buttons =
        static_cast<uint16_t>(rx[3]) | (static_cast<uint16_t>(rx[4]) << 8);
  }
  if (rx.size() >= 2 && (rx[1] == 0x73 || rx[1] == 0x7F)) {
    new_state.analog = true;
    if (rx.size() >= 9) {
      new_state.lx = static_cast<int8_t>(rx[5]);
      new_state.ly = static_cast<int8_t>(rx[6]);
      new_state.rx = static_cast<int8_t>(rx[7]);
      new_state.ry = static_cast<int8_t>(rx[8]);
    }
  }

  bool changed = (new_state.buttons != m_state.buttons) ||
                 (new_state.lx != m_state.lx) || (new_state.ly != m_state.ly) ||
                 (new_state.analog != m_state.analog);
  m_state = new_state;
  if (changed && m_on_change)
    m_on_change(m_state);
  return changed;
}

ps2_state ps2_controller::read_state() const { return m_state; }

void ps2_controller::on_change(std::function<void(const ps2_state &)> cb) {
  m_on_change = std::move(cb);
}

} // namespace gdut
