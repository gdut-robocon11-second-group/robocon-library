#ifndef GPIO_PIN_HPP
#define GPIO_PIN_HPP

#include "bsp_utility.hpp"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"

#include "uncopyable.hpp"
#include <cstdint>
#include <utility>

namespace gdut {

class gpio_proxy : private uncopyable {
public:
  gpio_proxy(GPIO_TypeDef *Port, GPIO_InitTypeDef *InitStruct)
      : m_gpio_port(Port), m_init_struct(InitStruct) {
    // initialize();
  }

  gpio_proxy(gpio_proxy &&pin) noexcept
      : m_gpio_port(pin.m_gpio_port), m_init_struct(pin.m_init_struct) {
    pin.m_gpio_port = nullptr;
    pin.m_init_struct = nullptr;
  }

  gpio_proxy &operator=(gpio_proxy &&pin) noexcept {
    if (this != &pin) {
      m_gpio_port = pin.m_gpio_port;
      m_init_struct = pin.m_init_struct;
      pin.m_gpio_port = nullptr;
      pin.m_init_struct = nullptr;
    }
    return *this;
  }

  void initialize() { HAL_GPIO_Init(m_gpio_port, m_init_struct); }

  void deinitialize() { HAL_GPIO_DeInit(m_gpio_port, m_init_struct->Pin); }

  void set_mode(gpio_mode mode) {
    m_init_struct->Mode = std::to_underlying(mode);
    HAL_GPIO_Init(m_gpio_port, m_init_struct);
  }

  void set_pull(gpio_pull pull) {
    m_init_struct->Pull = std::to_underlying(pull);
    HAL_GPIO_Init(m_gpio_port, m_init_struct);
  }

  void set_alternate(uint32_t alternate) {
    m_init_struct->Pull = alternate;
    HAL_GPIO_Init(m_gpio_port, m_init_struct);
  }

  void set_speed(gpio_speed speed) {
    m_init_struct->Speed = std::to_underlying(speed);
    HAL_GPIO_Init(m_gpio_port, m_init_struct);
  }

  void set_pin(gpio_pin pin) {
    m_init_struct->Pin = std::to_underlying(pin);
    HAL_GPIO_Init(m_gpio_port, m_init_struct);
  }

  ~gpio_proxy() noexcept {
    // deinitialize();
  }

  void write(bool state) {
    HAL_GPIO_WritePin(m_gpio_port, m_init_struct->Pin,
                      static_cast<GPIO_PinState>(state));
  }

  bool read() const noexcept {
    return HAL_GPIO_ReadPin(m_gpio_port, m_init_struct->Pin);
  }

  void toggle() { HAL_GPIO_TogglePin(m_gpio_port, m_init_struct->Pin); }

  GPIO_TypeDef *m_gpio_port;
  GPIO_InitTypeDef *m_init_struct;
};

} // namespace gdut

#endif // GPIO_PIN_HPP
