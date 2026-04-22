#include "bsp_pca9685.hpp"

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"


namespace gdut {
// 芯片有指定的频率范围，这里对频率进行裁剪
float pca9685::clamp_freq(float freq_hz) noexcept {
  if (freq_hz < 24.0f) {
    return 24.0f;
  }
  if (freq_hz > 1526.0f) {
    return 1526.0f;
  }
  return freq_hz;
}
uint8_t pca9685::calc_prescale(float freq_hz) noexcept {
  freq_hz = clamp_freq(freq_hz);

  // prescale = round(osc / (4096 * freq) - 1)
  const float prescale_f =
      (internal_oscillator_hz / (static_cast<float>(resolution) * freq_hz)) -
      1.0f;

  float rounded = prescale_f + 0.5f; // 因为要写入整数要四舍入五
  // 寄存器保护
  if (rounded < 0.0f) {
    rounded = 0.0f;
  }
  if (rounded > 255.0f) {
    rounded = 255.0f;
  }

  return static_cast<uint8_t>(rounded);
}
// 是否已经好了
HAL_StatusTypeDef pca9685::is_ready(uint32_t trials,
                                    std::chrono::milliseconds timeout) {
  if (m_bus == nullptr) {
    return HAL_ERROR;
  }
  return m_bus->is_device_ready(m_dev_addr, trials, timeout);
}

HAL_StatusTypeDef pca9685::write_register(uint8_t reg_addr, uint8_t value) {
  if (m_bus == nullptr) {
    return HAL_ERROR;
  }

  return m_bus->mem_write(m_dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &value, 1,
                          std::chrono::milliseconds(20));
}

HAL_StatusTypeDef pca9685::read_register(uint8_t reg_addr, uint8_t &value) {
  if (m_bus == nullptr) {
    return HAL_ERROR;
  }

  return m_bus->mem_read(m_dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &value, 1,
                         std::chrono::milliseconds(20));
}

HAL_StatusTypeDef pca9685::write_registers(uint8_t reg_addr,
                                           const uint8_t *data, uint16_t size) {
  if ((m_bus == nullptr) || (data == nullptr) || (size == 0U)) {
    return HAL_ERROR;
  }

  return m_bus->mem_write(m_dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data,
                          size, std::chrono::milliseconds(20));
}

HAL_StatusTypeDef pca9685::update_mode1_bits(uint8_t set_mask,
                                             uint8_t clear_mask) {
  uint8_t mode1 = 0;
  HAL_StatusTypeDef status =
      read_register(static_cast<uint8_t>(reg::mode1), mode1);
  if (status != HAL_OK) {
    return status;
  }

  mode1 = static_cast<uint8_t>((mode1 | set_mask) &
                               static_cast<uint8_t>(~clear_mask));
  return write_register(static_cast<uint8_t>(reg::mode1), mode1);
}

HAL_StatusTypeDef pca9685::init(float pwm_freq_hz) {
  HAL_StatusTypeDef status = is_ready();
  if (status != HAL_OK) {
    return status;
  }

  // MODE1:
  // AI = 1，开启自动地址递增
  // ALLCALL 保持默认响应也可以；常规驱动时不强依赖
  status = write_register(static_cast<uint8_t>(reg::mode1), mode1_ai);
  if (status != HAL_OK) {
    return status;
  }

  // MODE2:
  // OUTDRV = 1 -> 推挽输出
  // OCH = 0 -> 在 STOP 时更新输出（默认、稳妥）
  status = write_register(static_cast<std::uint8_t>(reg::mode2), mode2_outdrv);
  if (status != HAL_OK) {
    return status;
  }

  if (m_delay_callback) {
    m_delay_callback(1);
  } else {
    osDelay(1);
  }

  status = set_pwm_freq(pwm_freq_hz);
  if (status != HAL_OK) {
    return status;
  }

  return all_off();
}

HAL_StatusTypeDef pca9685::sleep() { return update_mode1_bits(mode1_sleep, 0); }

HAL_StatusTypeDef pca9685::wakeup() {
  HAL_StatusTypeDef status = update_mode1_bits(0, mode1_sleep);
  if (status != HAL_OK) {
    return status;
  }

  // 官方手册给的是振荡器起来最多 500us，这里留 1ms 更稳
  if (m_delay_callback) {
    m_delay_callback(1);
  } else {
    osDelay(1);
  }

  // 写 1 到 RESTART 位以重启 PWM 逻辑
  return update_mode1_bits(mode1_restart, 0);
}

HAL_StatusTypeDef pca9685::set_pwm_freq(float freq_hz) {
  freq_hz = clamp_freq(freq_hz);
  const uint8_t prescale = calc_prescale(freq_hz);

  uint8_t old_mode1 = 0;
  HAL_StatusTypeDef status =
      read_register(static_cast<uint8_t>(reg::mode1), old_mode1);
  if (status != HAL_OK) {
    return status;
  }

  // 先进入 sleep，再写 PRE_SCALE
  // 手册明确要求：MODE1.SLEEP=0 时，写 PRE_SCALE 会被阻塞
  uint8_t sleep_mode = static_cast<uint8_t>(
      (old_mode1 & static_cast<uint8_t>(~mode1_restart)) | mode1_sleep);

  status = write_register(static_cast<uint8_t>(reg::mode1), sleep_mode);
  if (status != HAL_OK) {
    return status;
  }

  status = write_register(static_cast<uint8_t>(reg::pre_scale), prescale);
  if (status != HAL_OK) {
    return status;
  }

  // 恢复原模式，退出休眠
  status = write_register(static_cast<std::uint8_t>(reg::mode1), old_mode1);
  if (status != HAL_OK) {
    return status;
  }

  HAL_Delay(1);

  // RESTART + 保持 AI
  status = write_register(
      static_cast<uint8_t>(reg::mode1),
      static_cast<uint8_t>(old_mode1 | mode1_restart | mode1_ai));
  if (status != HAL_OK) {
    return status;
  }

  m_pwm_freq_hz = freq_hz;
  return HAL_OK;
}

HAL_StatusTypeDef pca9685::set_pwm_opened(uint8_t channel, uint16_t on, uint16_t off) {
  if (channel >= channel_count) {
    return HAL_ERROR;
  }

  uint8_t data[4] = {0};

  // 允许 full on / full off 特殊编码使用 bit4，所以这里只不过滤 4096
  // 普通 12bit 数据应在 0~4095
  data[0] = static_cast<uint8_t>(on & 0xFFU);
  data[1] = static_cast<uint8_t>((on >> 8) & 0x1FU);
  data[2] = static_cast<uint8_t>(off & 0xFFU);
  data[3] = static_cast<uint8_t>((off >> 8) & 0x1FU);

  const uint8_t base =
      static_cast<uint8_t>(static_cast<uint8_t>(reg::led0_on_l) + 4U * channel);

  return write_registers(base, data, 4);
}

HAL_StatusTypeDef pca9685::set_all_pwm(uint16_t on, uint16_t off) {
  uint8_t data[4] = {0};

  data[0] = static_cast<uint8_t>(on & 0xFFU);
  data[1] = static_cast<uint8_t>((on >> 8) & 0x1FU);
  data[2] = static_cast<uint8_t>(off & 0xFFU);
  data[3] = static_cast<uint8_t>((off >> 8) & 0x1FU);

  return write_registers(static_cast<uint8_t>(reg::all_led_on_l), data, 4);
}

HAL_StatusTypeDef pca9685::all_off() {
  // FULL_OFF: ALL_LED_OFF_H bit4 = 1
  return set_all_pwm(0, 4096);
}

HAL_StatusTypeDef pca9685::all_on() {
  // FULL_ON: ALL_LED_ON_H bit4 = 1
  return set_all_pwm(4096, 0);
}

HAL_StatusTypeDef pca9685::set_duty(std::uint8_t channel, std::uint16_t duty,
                                    bool invert) {
  if (channel >= channel_count) {
    return HAL_ERROR;
  }

  duty = clamp_u12(duty);

  if (!invert) {
    if (duty == 0U) {
      return set_pwm_opened(channel, 0, 4096); // full off
    }
    if (duty >= 4095U) {
      return set_pwm_opened(channel, 4096, 0); // full on
    }
    return set_pwm_opened(channel, 0, duty);
  }

  // 反相模式
  if (duty == 0U) {
    return set_pwm_opened(channel, 4096, 0);
  }
  if (duty >= 4095U) {
    return set_pwm_opened(channel, 0, 4096);
  }
  return set_pwm_opened(channel, 0, static_cast<uint16_t>(4095U - duty));
}

HAL_StatusTypeDef pca9685::set_servo_pulse_us(uint8_t channel, float pulse_us) {
  if (channel >= channel_count) {
    return HAL_ERROR;
  }
  if (m_pwm_freq_hz <= 0.0f) {
    return HAL_ERROR;
  }

  // 周期 us = 1e6 / f
  const float period_us = 1000000.0f / m_pwm_freq_hz;
  float ticks_f = (pulse_us / period_us) * static_cast<float>(resolution);

  if (ticks_f < 0.0f) {
    ticks_f = 0.0f;
  }
  if (ticks_f > 4095.0f) {
    ticks_f = 4095.0f;
  }

  const uint16_t ticks = static_cast<uint16_t>(ticks_f + 0.5f);
  return set_pwm_opened(channel, 0, ticks);
}
// 度数接口,min_pulse_us 和 max_pulse_us 分别对应 0° 和 max_angle_deg° 的脉宽，线性插值
//min_pulse_us,500us，最大的对应 2500us，但实际使用时可能需要微调以适配舵机的实际范围和性能
HAL_StatusTypeDef pca9685::set_servo_angle(std::uint8_t channel,
                                           float angle_deg, float min_pulse_us,
                                           float max_pulse_us,
                                           float max_angle_deg) {
  if (channel >= channel_count) {
    return HAL_ERROR;
  }
  if (max_angle_deg <= 0.0f) {
    return HAL_ERROR;
  }
  if (max_pulse_us < min_pulse_us) {
    return HAL_ERROR;
  }

  if (angle_deg < 0.0f) {
    angle_deg = 0.0f;
  }
  if (angle_deg > max_angle_deg) {
    angle_deg = max_angle_deg;
  }

  const float pulse_us = min_pulse_us + (angle_deg / max_angle_deg) *
                                            (max_pulse_us - min_pulse_us);

  return set_servo_pulse_us(channel, pulse_us);
}

} // namespace gdut