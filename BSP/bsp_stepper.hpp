#ifndef BSP_STEPPER_HPP
#define BSP_STEPPER_HPP

#include "bsp_gpio_pin.hpp"
#include "bsp_timer.hpp"
#include "bsp_uart.hpp"
#include "uncopyable.hpp"
#include "verification_algorithm.hpp"
#include <atomic>
#include <cstdint>
#include <ctime>

namespace gdut {

/**
 * @brief 硬件PWM版 42步进电机驱动
 *
 * 使用定时器的PWM通道直接输出STEP脉冲（50%占空比），
 * Update中断仅用于精确计数步数（不影响脉冲精度）。
 *
 * CubeMX 配置要求：
 *   1. STEP引脚 → 对应定时器的 PWM Generation 通道（AF模式）
 *   2. 定时器时钟 + PSC 配置为 1us 分辨率（即计数频率 1MHz）
 *   3. 必须开启 Update Interrupt（NVIC）
 *   4. DIR 为普通 GPIO Output
 */
class stepper_motor : private uncopyable {
public:
  /**
   * @param dir_pin       DIR 引脚
   * @param step_timer    已配置为PWM模式的 timer 对象
   * @param pwm_channel   PWM通道（TIM_CHANNEL_1 ~ TIM_CHANNEL_4）
   */
  stepper_motor(gpio_proxy *dir_pin, timer *step_timer, uint32_t pwm_channel)
      : m_dir_pin(dir_pin), m_step_timer(step_timer),
        m_pwm_channel(pwm_channel), m_remaining_steps(0) {

    m_step_timer->register_period_elapsed_callback(
        [this]() { handle_step_isr(); });
  }

  ~stepper_motor() {
    stop();
    m_step_timer->register_period_elapsed_callback(timer::callback_t{});
  }

  void set_direction(bool clockwise) { m_dir_pin->write(clockwise); }

  /**
   * @brief 设置速度（单位：steps/s）
   * @note 基于实际定时器 PSC 配置计算周期
   *
   * 注意：如果步数为0或速度为0，则会立即停止运动
   * ARR如果过小可能导致定时器不稳定，函数内部会限制最大速度以避免这种情况
   */
  void set_speed(uint32_t steps_per_sec) {
    if (steps_per_sec == 0) {
      stop();
      return;
    }

    auto *htim = m_step_timer->get_htim();
    if (!htim)
      return;

    gdut::timer::timer_proxy timer_proxy{m_step_timer};
    const uint32_t psc = timer_proxy.get_psc();
    const uint32_t apb_clk = timer_proxy.get_apb_clock();

    // 限制最大速度，防止 ARR 过小导致不稳定
    if (steps_per_sec > 50000) {
      steps_per_sec = 50000;
    }

    // 计算定时器分辨率（纳秒）
    // resolution_ns = (PSC + 1) / APB_CLK * 1e9
    uint64_t timer_resolution_ns =
        static_cast<uint64_t>(psc + 1) * 1000000000ULL / apb_clk;

    // 计算所需周期（纳秒）
    uint64_t period_ns = 1000000000ULL / steps_per_sec;

    // 周期对应的计数器值 = period_ns / timer_resolution_ns
    uint32_t period_counts =
        static_cast<uint32_t>(period_ns / timer_resolution_ns);
    if (period_counts == 0)
      period_counts = 1;

    // ARR = 周期计数 - 1（硬件计数器是从0开始）
    __HAL_TIM_SET_AUTORELOAD(htim, period_counts - 1);

    // 50% 占空比：CCR = period_counts / 2
    uint32_t ccr = (period_counts + 1) / 2;

    timer::timer_pwm pwm_helper(m_step_timer);
    pwm_helper.set_duty(m_pwm_channel, ccr);

    // 启动 PWM
    if (HAL_TIM_PWM_GetState(htim) != HAL_TIM_STATE_BUSY) {
      pwm_helper.pwm_start(m_pwm_channel);
    }

    m_step_timer->enable_it(TIM_IT_UPDATE);
  }

  /**
   * @brief 非阻塞移动指定步数，完成后自动停止
   */
  void move_steps(uint32_t num_steps, uint32_t steps_per_sec) {
    if (num_steps == 0)
      return;

    m_remaining_steps = num_steps;
    set_speed(steps_per_sec);
  }

  /** 立即停止运动 */
  void stop() {
    auto *htim = m_step_timer->get_htim();
    if (htim) {
      timer::timer_pwm pwm_helper(m_step_timer);
      pwm_helper.pwm_stop(m_pwm_channel);
      m_step_timer->disable_it(TIM_IT_UPDATE);
    }
    m_remaining_steps = 0;
  }

  bool is_moving() const noexcept { return m_remaining_steps > 0; }

  uint32_t get_remaining_steps() const noexcept { return m_remaining_steps; }

private:
  void handle_step_isr() {
    if (m_remaining_steps > 0) {
      --m_remaining_steps;
      if (m_remaining_steps == 0) {
        stop();
      }
    }
  }

private:
  gpio_proxy *m_dir_pin;
  timer *m_step_timer;
  uint32_t m_pwm_channel;

  std::atomic<uint32_t> m_remaining_steps{0}; // 剩余步数，Update中断递减
};

enum class tmc2209_register : uint8_t {
  GCONF_REG_ADDR = 0x00,        // 全局配置寄存器地址
  GSTAT_REG_ADDR = 0x01,        // 全局状态寄存器地址
  IFCNT_REG_ADDR = 0x02,        // 接口计数寄存器地址
  NODECONF_REG_ADDR = 0x03,     // 节点配置寄存器地址
  OTP_PROG_REG_ADDR = 0x04,     // OTP编程寄存器地址
  OTP_READ_REG_ADDR = 0x05,     // OTP读取寄存器地址
  IOIN_REG_ADDR = 0x06,         // 输入引脚状态寄存器地址
  FACTORY_CONF_REG_ADDR = 0x07, // 出厂配置寄存器地址

  // Velocity Dependent Control
  IHOLD_IRUN_REG_ADDR = 0x10, // 电流配置寄存器地址
  TPOWERDOWN_REG_ADDR = 0x11, // 电源管理寄存器地址
  TSTEP_REG_ADDR = 0x12,      // 步进时间寄存器地址
  TPWMTHRS_REG_ADDR = 0x13,   // PWM阈值寄存器地址
  VACTUAL_REG_ADDR = 0x22,    // 实际速度寄存器地址

  // StallGuard Control
  TCOOLTHRS_REG_ADDR = 0x14, // TCOOLTHRS寄存器地址
  SGTHRS_REG_ADDR = 0x40,    // SGTHRS寄存器地址
  SG_RESULT_REG_ADDR = 0x41, // SG_RESULT寄存器地址
  COOLCONF_REG_ADDR = 0x42   // COOLCONF寄存器地址
};

struct tmc2209_packet {
  using verify_algorithm_t = gdut::crc8_algorithm;

  struct write_packet {
    uint8_t header = 0x55;    // 固定帧头0x55 or 0xA0
    uint8_t node_address;     // TMC2209地址 (只支持四个地址)
    uint8_t register_address; // 7位寄存器地址，最低为0
    uint8_t data[4];          // 4字节数据
    uint8_t crc;              // crc8校验码，覆盖 crc 前的所有字节（包含 data）
  } __attribute__((packed));

  struct read_packet {
    uint8_t header = 0x55;    // 固定帧头0x55 or 0xA0
    uint8_t node_address;     // TMC2209地址 (只支持四个地址)
    uint8_t register_address; // 7位寄存器地址，最低为0
    uint8_t crc;              // crc8校验码
  } __attribute__((packed));

  struct received_packet {
    uint8_t header;           // 固定帧头0x90
    uint8_t master_address;   // 主机地址，默认为0xFF
    uint8_t register_address; // 7位寄存器地址，最低为0
    uint8_t data[4];          // 4字节数据 (注意不能直接读取，存在字节序问题)
    uint8_t crc;              // crc8校验码
  } __attribute__((packed));

  [[nodiscard]] static write_packet
  build_write_packet(uint8_t node_address, tmc2209_register register_address,
                     uint32_t data) {
    write_packet packet;
    packet.node_address = node_address;
    // 写操作：寄存器地址占7位，最低位固定为0
    packet.register_address = std::to_underlying(register_address) << 1;
    // 数据按大端格式存储
    uint8_t *data_bytes = packet.data;
    data_bytes[0] = (data >> 24) & 0xFF;
    data_bytes[1] = (data >> 16) & 0xFF;
    data_bytes[2] = (data >> 8) & 0xFF;
    data_bytes[3] = data & 0xFF;

    // 计算CRC8校验码
    verify_algorithm_t crc_algo;
    packet.crc = crc_algo.calculate(
        reinterpret_cast<const uint8_t *>(&packet),
        reinterpret_cast<const uint8_t *>(&packet) + sizeof(packet),
        reinterpret_cast<const uint8_t *>(&packet.crc));

    return packet;
  }

  [[nodiscard]] static read_packet
  build_read_packet(uint8_t node_address, tmc2209_register register_address) {
    read_packet packet;
    packet.node_address = node_address;
    // 读操作：最高位为0，寄存器地址占7位，剩下为0
    packet.register_address = std::to_underlying(register_address) << 1;
    // 计算CRC8校验码
    verify_algorithm_t crc_algo;
    packet.crc = crc_algo.calculate(
        reinterpret_cast<const uint8_t *>(&packet),
        reinterpret_cast<const uint8_t *>(&packet) + sizeof(packet),
        reinterpret_cast<const uint8_t *>(&packet.crc));

    return packet;
  }

  [[nodiscard]] static bool check_crc(const received_packet &packet) {
    verify_algorithm_t crc_algo;
    return crc_algo.verify(reinterpret_cast<const uint8_t *>(&packet),
                           reinterpret_cast<const uint8_t *>(&packet) +
                               sizeof(packet),
                           reinterpret_cast<const uint8_t *>(&packet.crc));
  }

  [[nodiscard]] static bool
  validate_response(const received_packet &packet,
                    tmc2209_register expected_register) {
    // 校验帧头、寄存器地址和CRC
    if (packet.header != 0x90) {
      return false;
    }
    if (packet.register_address !=
        (std::to_underlying(expected_register) << 1)) {
      return false;
    }
    if (!check_crc(packet)) {
      return false;
    }
    return true;
  }

  [[nodiscard]] static uint32_t parse_data(const received_packet &packet) {
    // 数据按大端格式存储
    const uint8_t *data_bytes = packet.data;
    uint32_t data = (static_cast<uint32_t>(data_bytes[0]) << 24) |
                    (static_cast<uint32_t>(data_bytes[1]) << 16) |
                    (static_cast<uint32_t>(data_bytes[2]) << 8) |
                    static_cast<uint32_t>(data_bytes[3]);
    return data;
  }
};

class tmc2209_uart_controller : private uncopyable {
public:
  static constexpr std::chrono::milliseconds default_timeout = std::chrono::milliseconds(50);

  tmc2209_uart_controller(gdut::uart *uart, uint8_t node_address)
      : m_uart(uart), m_node_address(node_address) {}

  ~tmc2209_uart_controller() = default;

  // 通过 UART 发送写寄存器命令
  bool write_register(
      tmc2209_register register_address, uint32_t data,
      std::chrono::milliseconds delay_ms = default_timeout) {
    tmc2209_packet::write_packet packet = tmc2209_packet::build_write_packet(
        m_node_address, register_address, data);
    return m_uart->send(reinterpret_cast<const uint8_t *>(&packet),
                        sizeof(packet), delay_ms) == HAL_OK;
  }

  // 通过 UART 发送读寄存器命令并等待响应
  // 注意：此函数会阻塞直到收到响应，实际使用时建议在单独的线程中调用
  // 返回值包含原始响应数据，调用者需要自行验证和解析
  // 特别是data字段需要按大端格式解析
  [[nodiscard]] tmc2209_packet::received_packet read_register(
      tmc2209_register register_address,
      std::chrono::milliseconds delay_ms = default_timeout) {
    tmc2209_packet::read_packet packet =
        tmc2209_packet::build_read_packet(m_node_address, register_address);
    tmc2209_packet::received_packet response{};
    if (m_uart->send(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet),
                     delay_ms) != HAL_OK) {
      // 发送失败，返回一个无效的响应
      return response;
    }
    m_uart->receive(reinterpret_cast<uint8_t *>(&response), sizeof(response),
                    delay_ms);
    return response;
  }

  // 读取TMC2209的TSTEP寄存器（步进时间，n=20）
  uint32_t get_tstep(std::chrono::milliseconds delay_ms = default_timeout) {
    auto response = read_register(tmc2209_register::TSTEP_REG_ADDR, delay_ms);
    if (tmc2209_packet::validate_response(response,
                                          tmc2209_register::TSTEP_REG_ADDR)) {
      return tmc2209_packet::parse_data(response);
    }
    return 0;
  }

  // 设置TMC2209的TCOOLTHRS寄存器（速度阈值，n=20）
  void set_tcoolthrs(uint32_t threshold, std::chrono::milliseconds delay_ms = default_timeout) {
    write_register(tmc2209_register::TCOOLTHRS_REG_ADDR, threshold & 0xFFFFF, delay_ms);
  }

  // 设置TMC2209的SGTHRS寄存器（过流阈值，n=8）
  void set_stallguard_threshold(uint8_t threshold, std::chrono::milliseconds delay_ms = default_timeout) {
    write_register(tmc2209_register::SGTHRS_REG_ADDR, threshold, delay_ms);
  }

  // 读取TMC2209的SG_RESULT寄存器（StallGuard结果，n=10）
  [[nodiscard]] uint16_t get_stallguard_result(std::chrono::milliseconds delay_ms = default_timeout) {
    auto result = read_register(tmc2209_register::SG_RESULT_REG_ADDR, delay_ms);
    if (tmc2209_packet::validate_response(
            result, tmc2209_register::SG_RESULT_REG_ADDR)) {
      return static_cast<uint16_t>(tmc2209_packet::parse_data(result) & 0x03FF);
    }
    return 0;
  }

  // 设置TMC2209的COOLCONF寄存器（散热配置，n=16）
  // 参数说明：
  void set_coolconf(uint8_t seimin, uint8_t sedn, uint8_t semax, uint8_t seup,
                    uint8_t semin, std::chrono::milliseconds delay_ms = default_timeout) {
    uint32_t coolconf_value = ((static_cast<uint32_t>(seimin) & 0x1) << 15) |
                              ((static_cast<uint32_t>(sedn) & 0x3) << 13) |
                              ((static_cast<uint32_t>(semax) & 0xF) << 8) |
                              ((static_cast<uint32_t>(seup) & 0x3) << 5) |
                              (static_cast<uint32_t>(semin) & 0xF);
    write_register(tmc2209_register::COOLCONF_REG_ADDR, coolconf_value, delay_ms);
  }

  void init() { set_tcoolthrs(10000); }

private:
  gdut::uart *m_uart;
  uint8_t m_node_address; // TMC2209地址
};

} // namespace gdut

#endif // BSP_STEPPER_HPP
