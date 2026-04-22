#include "bsp_line8_follow_ch8.hpp"

namespace gdut {

  //两种初始化
line_follower_8ch::line_follower_8ch(gdut::i2c &bus, uint16_t hal_addr)
    : m_mode(comm_mode::i2c), m_i2c(&bus), m_i2c_hal_addr(hal_addr) {}

line_follower_8ch::line_follower_8ch(const gpio_bus &bus)
    : m_mode(comm_mode::gpio), m_i2c(nullptr), m_gpio(bus) {}

    //检查有没有连接上
HAL_StatusTypeDef line_follower_8ch::probe(std::chrono::milliseconds timeout) {
  if (is_i2c_mode()) {
    if (m_i2c == nullptr) {
      return HAL_ERROR;
    }
    return m_i2c->is_device_ready(m_i2c_hal_addr, 3, timeout);
  }

  // GPIO 模式没有设备应答，这里检查引脚是否配置完整：
  // 既要求 port 非空，也要求 pin 掩码非 0，避免后续读写时实际不操作任何引脚。
  if (m_gpio.ad0.port == nullptr || m_gpio.ad0.pin == 0 ||
      m_gpio.ad1.port == nullptr || m_gpio.ad1.pin == 0 ||
      m_gpio.ad2.port == nullptr || m_gpio.ad2.pin == 0 ||
      m_gpio.out.port == nullptr || m_gpio.out.pin == 0) {
    return HAL_ERROR;
  }
  return HAL_OK;
}
//读取 统一返回：bit=1 表示检测到目标线，一次性读取所有的数组
HAL_StatusTypeDef line_follower_8ch::read_line_mask(
    uint8_t &mask, std::chrono::milliseconds timeout) {
  mask = 0;
//I2C 模式
  if (is_i2c_mode()) {
    // 文档规定：I2C 读取“全部传感器巡线识别结果”时，bit=1 表示检测到目标线。
    return i2c_read_u8(reg_line_result, mask, timeout);
  }
//GPIO 模式，一个个读
  std::array<uint8_t, channel_count> states{};
  HAL_StatusTypeDef status = read_line_array(states, timeout);
  if (status != HAL_OK) {
    return status;
  }
  // 将数组转换成位掩码
  mask = make_mask_from_array(states);
  return HAL_OK;
}
//放进数组，一次性读
HAL_StatusTypeDef line_follower_8ch::read_line_array(
    std::array<uint8_t, channel_count> &states,
    std::chrono::milliseconds timeout) {
      //把states数组初始化为0
  for (auto &v : states) {
    v = 0;
  }
//i2c模式
  if (is_i2c_mode()) {
    uint8_t mask = 0;
    HAL_StatusTypeDef status = i2c_read_u8(reg_line_result, mask, timeout);
    if (status != HAL_OK) {
      return status;
    }
    // 将位掩码转换为数组，存入里面
    for (uint8_t i = 0; i < channel_count; ++i) {
      states[i] = static_cast<uint8_t>((mask >> i) & 0x01U);
    }
    return HAL_OK;
  }

  for (uint8_t ch = 1; ch <= channel_count; ++ch) {
    uint8_t detected = 0;
    HAL_StatusTypeDef status = read_channel_line(ch, detected, timeout);
    if (status != HAL_OK) {
      return status;
    }
    states[ch - 1U] = detected;
  }
  return HAL_OK;
}
//读取单个通道的巡线识别结果
HAL_StatusTypeDef line_follower_8ch::read_channel_line(
    uint8_t channel, uint8_t &detected, std::chrono::milliseconds timeout) {
  detected = 0;

  if (!valid_channel(channel)) {
    return HAL_ERROR;
  }

  if (is_i2c_mode()) {
    uint8_t mask = 0;
    HAL_StatusTypeDef status = i2c_read_u8(reg_line_result, mask, timeout);
    if (status != HAL_OK) {
      return status;
    }

    detected = static_cast<uint8_t>((mask >> (channel - 1U)) & 0x01U);
    return HAL_OK;
  }

  HAL_StatusTypeDef status = gpio_select_channel(channel);
  if (status != HAL_OK) {
    return status;
  }

  GPIO_PinState out_state = GPIO_PIN_SET;
  status = gpio_read_out(out_state);
  if (status != HAL_OK) {
    return status;
  }

  // 文档规定：GPIO 模式下，检测到线段时 OUT 输出低电平。
  detected = (out_state == GPIO_PIN_RESET) ? 1U : 0U;
  return HAL_OK;
}
//读取单个通道的灰度值
HAL_StatusTypeDef line_follower_8ch::read_channel_gray(
    uint8_t channel, uint16_t &gray, std::chrono::milliseconds timeout) {
  gray = 0;
  if (!valid_channel(channel) || !is_i2c_mode()) {
    return HAL_ERROR;
  }
  return i2c_read_u16_le(gray_reg(channel), gray, timeout);
}
//读取单个通道的阈值
HAL_StatusTypeDef line_follower_8ch::read_channel_threshold(
    uint8_t channel, uint16_t &threshold, std::chrono::milliseconds timeout) {
  threshold = 0;
  if (!valid_channel(channel) || !is_i2c_mode()) {
    return HAL_ERROR;
  }
  return i2c_read_u16_le(threshold_reg(channel), threshold, timeout);
}
//读取所有灰度值
HAL_StatusTypeDef line_follower_8ch::read_all_gray(
    std::array<uint16_t, channel_count> &gray,
    std::chrono::milliseconds timeout) {
  if (!is_i2c_mode()) {
    return HAL_ERROR;
  }

  for (uint8_t ch = 1; ch <= channel_count; ++ch) {
    HAL_StatusTypeDef status = read_channel_gray(ch, gray[ch - 1U], timeout);
    if (status != HAL_OK) {
      return status;
    }
  }
  return HAL_OK;
}
//读取所有阈值
HAL_StatusTypeDef line_follower_8ch::read_all_threshold(
    std::array<uint16_t, channel_count> &threshold,
    std::chrono::milliseconds timeout) {
  if (!is_i2c_mode()) {
    return HAL_ERROR;
  }

  for (uint8_t ch = 1; ch <= channel_count; ++ch) {
    HAL_StatusTypeDef status =
        read_channel_threshold(ch, threshold[ch - 1U], timeout);
    if (status != HAL_OK) {
      return status;
    }
  }
  return HAL_OK;
}
//读取一个快照，把当前传感器的一组状态读出来，存进 snapshot data
HAL_StatusTypeDef line_follower_8ch::read_snapshot(
    snapshot &data, bool with_gray, bool with_threshold,
    std::chrono::milliseconds timeout) {
  HAL_StatusTypeDef status = read_line_array(data.line, timeout);
  if (status != HAL_OK) {
    return status;
  }
  // 将数组转换成位掩码
  data.line_mask = make_mask_from_array(data.line);

  if (with_gray) {
    if (!is_i2c_mode()) {
      return HAL_ERROR;
    }
    status = read_all_gray(data.gray, timeout);
    if (status != HAL_OK) {
      return status;
    }
  }

  if (with_threshold) {
    if (!is_i2c_mode()) {
      return HAL_ERROR;
    }
    status = read_all_threshold(data.threshold, timeout);
    if (status != HAL_OK) {
      return status;
    }
  }

  return HAL_OK;
}
//将数组转换成位掩码
uint8_t line_follower_8ch::make_mask_from_array(
    const std::array<uint8_t, channel_count> &states) {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < channel_count; ++i) {
    if (states[i] != 0U) {
      mask |= static_cast<uint8_t>(1U << i);
    }
  }
  return mask;
}

//reg时寄存器的地址，value时读取到的值，timeout时超时时间
HAL_StatusTypeDef line_follower_8ch::i2c_read_u8(uint8_t reg, uint8_t &value,
                               std::chrono::milliseconds timeout) {
  if (m_i2c == nullptr) {
    return HAL_ERROR;
  }

  return m_i2c->mem_read(m_i2c_hal_addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1,
                         timeout);
}
//读取一个16位的数据
HAL_StatusTypeDef line_follower_8ch::i2c_read_u16_le(uint8_t reg, uint16_t &value,
                                   std::chrono::milliseconds timeout) {
  if (m_i2c == nullptr) {
    return HAL_ERROR;
  }

  uint8_t raw[2] = {0, 0};
  HAL_StatusTypeDef status =
      m_i2c->mem_read(m_i2c_hal_addr, reg, I2C_MEMADD_SIZE_8BIT, raw, 2,
                      timeout);
  if (status != HAL_OK) {
    return status;
  }

  value = static_cast<uint16_t>(static_cast<uint16_t>(raw[0]) |
                                (static_cast<uint16_t>(raw[1]) << 8U));
  return HAL_OK;
}
//选择通道
HAL_StatusTypeDef line_follower_8ch::gpio_select_channel(uint8_t channel) {
  if (!valid_channel(channel)) {
    return HAL_ERROR;
  }
  if (m_gpio.ad0.port == nullptr || m_gpio.ad1.port == nullptr ||
      m_gpio.ad2.port == nullptr) {
    return HAL_ERROR;
  }

  const uint8_t index = static_cast<uint8_t>(channel - 1U);
  // 计算每个选择器的状态
  const GPIO_PinState ad0_state =
      ((index >> 0U) & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  const GPIO_PinState ad1_state =
      ((index >> 1U) & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  const GPIO_PinState ad2_state =
      ((index >> 2U) & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
//写入引脚状态
  HAL_GPIO_WritePin(m_gpio.ad0.port, m_gpio.ad0.pin, ad0_state);
  HAL_GPIO_WritePin(m_gpio.ad1.port, m_gpio.ad1.pin, ad1_state);
  HAL_GPIO_WritePin(m_gpio.ad2.port, m_gpio.ad2.pin, ad2_state);

  if (m_gpio.delay_us != nullptr && m_gpio.settle_us > 0U) {
    m_gpio.delay_us(m_gpio.settle_us);
  } else {
    // 没有 us 延时函数时，给一个很短的空转等待，避免刚切通道就立刻读取。
    for (volatile uint32_t i = 0; i < 64U; ++i) {
      __NOP();
    }
  }

  return HAL_OK;
}
  //读取引脚状态
HAL_StatusTypeDef line_follower_8ch::gpio_read_out(GPIO_PinState &pin_state) {
  if (m_gpio.out.port == nullptr) {
    return HAL_ERROR;
  }
  pin_state = HAL_GPIO_ReadPin(m_gpio.out.port, m_gpio.out.pin);
  return HAL_OK;
}

} // namespace gdut
