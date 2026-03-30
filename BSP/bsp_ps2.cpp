#include "bsp_ps2.hpp"
#include "bsp_spi.hpp"
#include "cmsis_os2.h"

#include <array>
#include <chrono>
#include <utility>

namespace gdut {

static constexpr auto k_spi_timeout = std::chrono::milliseconds(5);

static void default_delay_ms(uint32_t ms) {
  if (ms == 0U) {
    return;
  }

  if (osKernelGetState() == osKernelRunning) {
    osDelay(ms);
  }
}

static bool frame_all_eq(const std::array<uint8_t, 9> &rx, uint8_t v) {
  for (auto b : rx) {
    if (b != v)
      return false;
  }
  return true;
}

static bool frame_looks_dead(const std::array<uint8_t, 9> &rx) {
  return frame_all_eq(rx, 0xFF) || frame_all_eq(rx, 0x00);
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

  // 自动尝试握手；失败也不阻止后续 poll
  (void)handshake();
}

bool ps2_controller::transfer_frame(std::array<uint8_t, 9> &tx,
                                    std::array<uint8_t, 9> &rx) {
  if (m_spi == nullptr) {
    return false;
  }

  return m_spi->transmit_receive(tx.data(), rx.data(),
                                 static_cast<uint16_t>(tx.size()),
                                 k_spi_timeout);
}

bool ps2_controller::transfer_packet(const std::array<uint8_t, 9> &tx,
                                     std::array<uint8_t, 9> &rx) {
  if (m_spi == nullptr) {
    return false;
  }

  std::array<uint8_t, 9> tx_copy = tx;

  if (m_pins.set_att) {
    m_pins.set_att(false);
  }
  if (m_pins.delay_ms) {
    m_pins.delay_ms(1U);
  }

  const bool ok = transfer_frame(tx_copy, rx);

  if (m_pins.set_att) {
    m_pins.set_att(true);
  }
  if (m_pins.delay_ms) {
    m_pins.delay_ms(1U);
  }

  if (!ok) {
    return false;
  }

  if (frame_looks_dead(rx)) {
    return false;
  }

  return true;
}

bool ps2_controller::handshake() {
  if (m_spi == nullptr) {
    return false;
  }

  std::array<uint8_t, 9> rx{};

  // 1) 确认设备在线
  {
    const std::array<uint8_t, 9> tx{0x01, 0x42, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00};
    if (!transfer_packet(tx, rx)) {
      return false;
    }
  }

  // 2) 进入配置模式
  {
    const std::array<uint8_t, 9> tx{0x01, 0x43, 0x00, 0x01, 0x00,
                                    0x00, 0x00, 0x00, 0x00};
    if (!transfer_packet(tx, rx)) {
      return false;
    }
  }

  // 3) 尝试设为模拟并锁定

  {
    const std::array<uint8_t, 9> tx{0x01, 0x44, 0x00, 0x01, 0x03,
                                    0x00, 0x00, 0x00, 0x00};
    (void)transfer_packet(tx, rx);
  }

  // 4) 退出配置模式
  {
    const std::array<uint8_t, 9> tx{0x01, 0x43, 0x00, 0x00, 0x5A,
                                    0x5A, 0x5A, 0x5A, 0x5A};
    if (!transfer_packet(tx, rx)) {
      return false;
    }
  }

  // 5) 再读一次状态确认
  {
    const std::array<uint8_t, 9> tx{0x01, 0x42, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00};
    if (!transfer_packet(tx, rx)) {
      return false;
    }
  }

  return true;
}

void ps2_controller::parse_state(const std::array<uint8_t, 9> &rx) {
  ps2_state new_state{};

  // 这里仍然假设 9 字节布局：buttons in rx[3..4], sticks in rx[5..8]
  new_state.buttons =
      static_cast<uint16_t>(rx[3]) | (static_cast<uint16_t>(rx[4]) << 8);

  new_state.left_x = rx[5];
  new_state.left_y = rx[6];
  new_state.right_x = rx[7];
  new_state.right_y = rx[8];

  if (new_state.buttons != m_state.buttons ||
      new_state.left_x != m_state.left_x ||
      new_state.left_y != m_state.left_y ||
      new_state.right_x != m_state.right_x ||
      new_state.right_y != m_state.right_y) {
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

  const std::array<uint8_t, 9> tx{0x01, 0x42, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00};
  std::array<uint8_t, 9> rx{};

  if (!transfer_packet(tx, rx)) {
    return false;
  }

  parse_state(rx);
  return true;
}

ps2_state ps2_controller::read_state() const { return m_state; }

void ps2_controller::on_change(gdut::function<void(const ps2_state &)> cb) {
  m_on_change = std::move(cb);
}

} // namespace gdut
