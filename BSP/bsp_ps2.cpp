#include "bsp_ps2.hpp"
#include "bsp_spi.hpp"
#include "cmsis_os2.h"

#include <utility>
#include <chrono>
#include <array>

namespace gdut {

static void default_delay_ms(uint32_t ms) {
  if (ms == 0U) {
    return;
  }

  if (osKernelGetState() == osKernelRunning) {
    osDelay(ms);
  }
}

ps2_controller::ps2_controller(pins_interface pins, spi_proxy *spi)
    : m_spi(spi), m_pins(std::move(pins)), m_state{}, m_on_change(nullptr) {
  if (!m_pins.delay_ms) {
    m_pins.delay_ms = default_delay_ms;
  }
}

void ps2_controller::init() {
  if (m_pins.set_att) {
    m_pins.set_att(true);
  }

  if (m_pins.delay_ms) {
    m_pins.delay_ms(1U);
  }
}

bool ps2_controller::transfer_frame(std::array<uint8_t, 9> &tx,
                                    std::array<uint8_t, 9> &rx) {
  if (m_spi == nullptr) {
    return false;
  }

  return m_spi->transmit_receive(tx.data(), rx.data(),
                               static_cast<uint16_t>(tx.size()),
                               std::chrono::milliseconds(0));
}

void ps2_controller::parse_state(const std::array<uint8_t, 9> &rx) {
  ps2_state new_state{};

  new_state.buttons =
   static_cast<uint16_t>(rx[3]) | (static_cast<uint16_t>(rx[4]) << 8);

  new_state.left_x = rx[5];
  new_state.left_y = rx[6];
  new_state.right_x = rx[7];
  new_state.right_y = rx[8];

  new_state.analog = true;

  if (new_state.buttons != m_state.buttons ||
      new_state.left_x != m_state.left_x ||
      new_state.left_y != m_state.left_y ||
      new_state.right_x != m_state.right_x ||
      new_state.right_y != m_state.right_y ||
      new_state.analog != m_state.analog) {
    m_state = new_state;
    if (m_on_change) {
      m_on_change(m_state);
    }
  }
}

bool ps2_controller::poll() {
  if (m_spi == nullptr) {
    return false;
  }

  std::array<uint8_t, 9> tx{0x01, 0x42, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00};
  std::array<uint8_t, 9> rx{};

  if (m_pins.set_att) {
    m_pins.set_att(false);
  }

  if (m_pins.delay_ms) {
    m_pins.delay_ms(1U);
  }

  const bool ok = transfer_frame(tx, rx);

  if (m_pins.set_att) {
    m_pins.set_att(true);
  }

  if (!ok) {
    return false;
  }

  parse_state(rx);
  return true;
}

ps2_state ps2_controller::read_state() const {
  return m_state;
}

void ps2_controller::on_change(gdut::function<void(const ps2_state &)> cb) {
  m_on_change = std::move(cb);
}

} // namespace gdut
