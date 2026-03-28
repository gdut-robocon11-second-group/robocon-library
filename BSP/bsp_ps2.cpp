#include "bsp_ps2.hpp"
#include "bsp_spi.hpp"
#include "stm32f4xx_hal.h"
#include <array>
#include <cmsis_os2.h>

namespace gdut {

static void default_delay_us(uint32_t us) {
  if (us == 0)
    return;
  // 非阻塞优先：内核运行时仅使用 osDelay 让出 CPU。
  // 小于 1ms 的延时在该路径下直接跳过，避免忙等占用线程。
#if defined(__CMSIS_OS2)
  if (osKernelGetState() == osKernelRunning) {
    uint32_t sleep_ms = us / 1000;
    if (sleep_ms > 0) {
      osDelay(sleep_ms);
    }
    return;
  }
#endif

  // 若 RTOS 未运行，默认实现不做忙等，避免阻塞。
  (void)us;
}

ps2_controller::ps2_controller(pins_interface pins, spi_proxy *spi)
    : m_spi(spi), m_pins(pins), m_on_change(nullptr) {
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
  if (!m_spi) {
    return 0xff;
  }

  uint8_t rx = 0;
  // 非阻塞尝试：0ms 超时，SPI 忙时立即返回失败。
  const auto timeout = std::chrono::milliseconds(0);
  if (m_spi->transmit_receive(&tx, &rx, 1, timeout)) {
    return rx;
  }

  // If hardware SPI fails, return error sentinel
  return 0xff;
}

bool ps2_controller::poll() {
  std::array<uint8_t, 9> tx = {0x01, 0x42, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00};
  std::array<uint8_t, 9> rx{};

  if (!m_spi) {
    return false;
  }

  if (m_pins.set_att)
    m_pins.set_att(false);
  m_pins.delay_us(20);

  // 非阻塞尝试：单次整帧传输，失败立即退出。
  const auto timeout = std::chrono::milliseconds(0);
  if (!m_spi->transmit_receive(tx.data(), rx.data(), static_cast<uint16_t>(tx.size()),
                               timeout)) {
    if (m_pins.set_att)
      m_pins.set_att(true);
    return false;
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

void ps2_controller::on_change(gdut::function<void(const ps2_state &)> cb) {
  m_on_change = cb;
}

} // namespace gdut
