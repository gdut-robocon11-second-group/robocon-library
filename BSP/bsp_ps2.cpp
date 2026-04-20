#include "bsp_ps2.hpp"
#include "bsp_spi.hpp"

#include <span>
#include <utility>

namespace gdut {

ps2_controller::ps2_controller(pins_interface pins, spi_proxy *spi)
    : ps2_controller(std::move(pins), spi, config{}) {}

ps2_controller::ps2_controller(pins_interface pins, spi_proxy *spi, config cfg)
    : m_spi(spi), m_pins(std::move(pins)), m_cfg(std::move(cfg)), m_state{},
      m_on_change(nullptr) {
  if (!m_pins.delay_us) {
    m_pins.delay_us = &ps2_controller::delay_us;
  }
}

void ps2_controller::init() {
  if (m_pins.set_att) {
    std::invoke(m_pins.set_att, true); // 空闲状态 ATT 拉高
  }

  if (m_pins.delay_ms) {
    std::invoke(m_pins.delay_ms, 20);
  }

  // 自动尝试握手；失败也不阻止后续 poll
  static_cast<void>(handshake());
}

bool ps2_controller::transfer_frame(std::span<const uint8_t, 9> tx,
                                    std::span<uint8_t, 9> rx) {
  if (m_spi == nullptr) {
    return false;
  }

  // 逐字节发送并接收，防止PS2接收不到，但是一般情况下不需要
  /* for (uint8_t i = 0; i < static_cast<uint8_t>(tx.size()); ++i) {
    uint8_t rx_byte = 0U;
    if (!m_spi->transmit_receive(&tx[i], &rx_byte, 1U, m_cfg.spi_timeout)) {
      return false;
    }
    rx[i] = rx_byte;
  }
  return true; */

  return m_spi->transmit_receive(tx.data(), rx.data(),
                                 static_cast<uint8_t>(tx.size()),
                                 m_cfg.spi_timeout * 9);
}

bool ps2_controller::transfer_packet(std::span<const uint8_t, 9> tx,
                                     std::span<uint8_t, 9> rx,
                                     bool validate_frame) {
  if (m_spi == nullptr) {
    return false;
  }

  // 1) 通信开始前先拉低 ATT 并等待保护时间
  if (m_pins.set_att) {
    std::invoke(m_pins.set_att, false);
  }
  // 2) ATT 拉低后至少要等待 m_cfg.att_guard_us 微秒才能开始通信
  std::invoke(m_pins.delay_us, m_cfg.att_guard_us);

  // 3) 发送并接收数据
  const bool ok = transfer_frame(tx, rx);

  // 4) 无论成功与否，通信结束后都要拉高 ATT 并等待保护时间
  if (m_pins.set_att) {
    std::invoke(m_pins.set_att, true);
  }
  // ATT 拉高后至少要等待 m_cfg.att_guard_us 微秒才能进行下一次通信
  std::invoke(m_pins.delay_us, m_cfg.att_guard_us);

  // 5) 如果通信成功还要验证数据帧是否合法
  if (!ok)
    return false;
  if (validate_frame && !frame_is_valid(rx))
    return false;
  return true;
}

bool ps2_controller::handshake() {
  if (m_spi == nullptr) {
    return false;
  }

  uint8_t rx[9]{};

  // 1) 确认设备在线
  if (!transfer_packet(k_cmd_poll, rx, true)) {
    return false;
  }

  // 2) 进入配置模式
  if (!transfer_packet(k_cmd_enter_config, rx, false)) {
    return false;
  }

  // 3) 尝试设为模拟并锁定（非关键步骤，失败不立刻返回）
  (void)transfer_packet(k_cmd_set_analog_lock, rx, false);

  // 4) 退出配置模式
  if (!transfer_packet(k_cmd_exit_config, rx, false)) {
    return false;
  }

  // 5) 再读一次状态确认
  if (!transfer_packet(k_cmd_poll, rx, true)) {
    return false;
  }

  return true;
}

void ps2_controller::parse_state(std::span<const uint8_t, 9> rx) {
  ps2_state new_state{};

  // PS2 按键位是低有效：0=按下，1=松开，因此需按位取反后再组合。
  new_state.buttons =
      static_cast<uint16_t>(static_cast<uint8_t>(~rx[3])) |
      (static_cast<uint16_t>(static_cast<uint8_t>(~rx[4])) << 8);
  if (new_state.buttons == 65535U) {
    // 按键全松开时某些 2.4G 接收器会回传 0xFF 0xFF，但这时其实是等同于没有按键被按下的，因此把它当成0处理。
    new_state.buttons = 0;
  }

  // rx[5..6] 为右摇杆，rx[7..8] 为左摇杆。
  new_state.right_x = rx[5];
  new_state.right_y = rx[6];
  new_state.left_x = rx[7];
  new_state.left_y = rx[8];

  // 这里PS2手柄在红色灯不亮时摇杆会回传0xFF（而不是127），
  // 但这时其实是等同于摇杆处于中心位置的，因此把它们都当成127处理。
  // 但是如果两个遥杆的四个轴都是255的话会被误判，但是危险性不大，先不做特殊处理了。
  if (new_state.left_x == 255 && new_state.left_y == 255 &&
      new_state.right_x == 255 && new_state.right_y == 255) {
    new_state.left_x = 127;
    new_state.left_y = 127;
    new_state.right_x = 127;
    new_state.right_y = 127;
  }

  const bool changed = new_state.buttons != m_state.buttons ||
                       new_state.left_x != m_state.left_x ||
                       new_state.left_y != m_state.left_y ||
                       new_state.right_x != m_state.right_x ||
                       new_state.right_y != m_state.right_y;
  m_state = new_state;
  if (changed) {
    if (m_on_change) {
      std::invoke(m_on_change, m_state);
    }
  }
}

bool ps2_controller::poll() {
  if (m_spi == nullptr) {
    return false;
  }
  uint8_t rx[9]{};

  if (!transfer_packet(k_cmd_poll, rx, true)) {
    return false;
  }

  parse_state(rx);
  return true;
}

ps2_state ps2_controller::read_state() const { return m_state; }

void ps2_controller::on_change(gdut::function<void(const ps2_state &)> cb) {
  m_on_change = std::move(cb);
}

bool ps2_controller::frame_all_eq(std::span<const uint8_t, 9> rx, uint8_t v) {
  for (auto b : rx) {
    if (b != v)
      return false;
  }
  return true;
}

bool ps2_controller::frame_looks_dead(std::span<const uint8_t, 9> rx) {
  return frame_all_eq(rx, 0xFF) || frame_all_eq(rx, 0x00);
}

bool ps2_controller::is_valid_mode(uint8_t mode) {
  return mode == k_ps2_mode_digital || mode == k_ps2_mode_analog_red ||
         mode == k_ps2_mode_analog_pressure;
}

bool ps2_controller::frame_is_valid(std::span<const uint8_t, 9> rx) {
  if (frame_looks_dead(rx)) {
    return false;
  }

  // 某些 2.4G 接收器 ACK/mode 字节不严格遵循标准，先只过滤死帧提高兼容性。
  return true;
}

void ps2_controller::delay_us(uint32_t us) {
  if (us == 0U) {
    return;
  }

  const uint64_t iterations_per_us =
      static_cast<uint64_t>((SystemCoreClock / 1000000U) / 5U);
  const uint64_t count = static_cast<uint64_t>(us) *
                         (iterations_per_us == 0U ? 1U : iterations_per_us);
  for (uint64_t i = 0; i < count; ++i) {
    __NOP();
  }
}

} // namespace gdut
