#ifndef BSP_IIC_HPP
#define BSP_IIC_HPP

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "bsp_type_traits.hpp"
#include "bsp_uncopyable.hpp"
#include "bsp_functions.hpp"

#include <cstdint>
#include <chrono>
#include <functional>
#include <utility>
#include <algorithm>

namespace gdut{
class i2c: private uncopyable {
public:


  using master_tx_cplt_callback_t = gdut::function<void()>;
  using master_rx_cplt_callback_t = gdut::function<void()>;
  using mem_tx_cplt_callback_t = gdut::function<void()>;
  using mem_rx_cplt_callback_t = gdut::function<void()>;
  using error_callback_t = gdut::function<void(uint32_t error)>;
  using abort_cplt_callback_t = gdut::function<void()>;
  using listen_cplt_callback_t = gdut::function<void()>;
  using addr_callback_t = gdut::function<void(uint16_t addr, uint32_t direction)>;
  using slave_tx_cplt_callback_t = gdut::function<void()>;
  using slave_rx_cplt_callback_t = gdut::function<void()>;

  explicit i2c(I2C_HandleTypeDef* hi2c) : m_hi2c(hi2c) {}
  ~i2c() noexcept = default;
  // 禁止拷贝和移动
  i2c(const i2c&) = delete;
  i2c& operator=(const i2c&) = delete;
  i2c(i2c&&) = delete;
  i2c& operator=(i2c&&) = delete;

    // 初始化
  HAL_StatusTypeDef init() {
    return m_hi2c ? HAL_I2C_Init(m_hi2c) : HAL_ERROR;
  }
  HAL_StatusTypeDef deinit() {
    return m_hi2c ? HAL_I2C_DeInit(m_hi2c) : HAL_ERROR;
  }

  // 阻塞模式发送
  HAL_StatusTypeDef transmit(uint16_t dev_addr, const uint8_t *data, uint16_t size,
                               std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
    return m_hi2c ? HAL_I2C_Master_Transmit(m_hi2c, dev_addr, const_cast<uint8_t*>(data), size,
                                      timeout_cast(timeout)) : HAL_ERROR;
  }

  // 阻塞模式接收
  HAL_StatusTypeDef receive(uint16_t dev_addr, uint8_t *data, uint16_t size,
                              std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
    return m_hi2c ? HAL_I2C_Master_Receive(m_hi2c, dev_addr, data, size,
                                     timeout_cast(timeout)) : HAL_ERROR;
  }

  // 中断模式发送
  HAL_StatusTypeDef transmit_it(uint16_t dev_addr, const uint8_t *data, uint16_t size) {
    return m_hi2c ? HAL_I2C_Master_Transmit_IT(m_hi2c, dev_addr, const_cast<uint8_t*>(data), size) : HAL_ERROR;
  }
  

  // 中断模式接收
  HAL_StatusTypeDef receive_it(uint16_t dev_addr, uint8_t *data, uint16_t size) {
    return m_hi2c ? HAL_I2C_Master_Receive_IT(m_hi2c, dev_addr, data, size) : HAL_ERROR;
  }

  // 存储器写（阻塞）
  HAL_StatusTypeDef mem_write(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_addr_size,
                         const uint8_t *data, uint16_t size,
                         std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
    return m_hi2c ? HAL_I2C_Mem_Write(m_hi2c, dev_addr, mem_addr, mem_addr_size,
                                const_cast<uint8_t*>(data), size, timeout_cast(timeout)) : HAL_ERROR;
  }

  // 存储器读（阻塞）
  HAL_StatusTypeDef mem_read(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_addr_size,
                        uint8_t *data, uint16_t size,
                        std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
    return m_hi2c ? HAL_I2C_Mem_Read(m_hi2c, dev_addr, mem_addr, mem_addr_size,
                               data, size, timeout_cast(timeout)) : HAL_ERROR;
  }

  // 存储器写（中断）
  HAL_StatusTypeDef mem_write_it(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_addr_size,
                            const uint8_t *data, uint16_t size) {
    return m_hi2c ? HAL_I2C_Mem_Write_IT(m_hi2c, dev_addr, mem_addr, mem_addr_size,
                                   const_cast<uint8_t*>(data), size) : HAL_ERROR;
  }
  
  // 存储器读（中断）
  HAL_StatusTypeDef mem_read_it(uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_addr_size,
                           uint8_t *data, uint16_t size) {
    return m_hi2c ? HAL_I2C_Mem_Read_IT(m_hi2c, dev_addr, mem_addr, mem_addr_size,
                                  data, size) : HAL_ERROR;
  }
  
   // 从模式发送（阻塞）
  HAL_StatusTypeDef slave_transmit(const uint8_t *data, uint16_t size,
                              std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
    return m_hi2c ? HAL_I2C_Slave_Transmit(m_hi2c, const_cast<uint8_t*>(data), size,
                                     timeout_cast(timeout)) : HAL_ERROR;
  }

  // 从模式接收（阻塞）
  HAL_StatusTypeDef slave_receive(uint8_t *data, uint16_t size,
                             std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
    return m_hi2c ? HAL_I2C_Slave_Receive(m_hi2c, data, size, timeout_cast(timeout)) : HAL_ERROR;
  }
  
  // 从模式发送（中断）
  HAL_StatusTypeDef slave_transmit_it(const uint8_t *data, uint16_t size) {
    return m_hi2c ? HAL_I2C_Slave_Transmit_IT(m_hi2c, const_cast<uint8_t*>(data), size) : HAL_ERROR;
  }
  
  // 从模式接收（中断）
  HAL_StatusTypeDef slave_receive_it(uint8_t *data, uint16_t size) {
    return m_hi2c ? HAL_I2C_Slave_Receive_IT(m_hi2c, data, size) : HAL_ERROR;
  }

   // 检查设备是否就绪
  HAL_StatusTypeDef is_device_ready(uint16_t dev_addr, uint32_t trials = 3,
                               std::chrono::milliseconds timeout = std::chrono::milliseconds(10)) {
    return m_hi2c ? HAL_I2C_IsDeviceReady(m_hi2c, dev_addr, trials, timeout_cast(timeout)) : HAL_ERROR;
  }
  
  // 启用监听模式
  HAL_StatusTypeDef enable_listen() {
    return m_hi2c ? HAL_I2C_EnableListen_IT(m_hi2c) : HAL_ERROR;
  }
  
  // 禁用监听模式
  HAL_StatusTypeDef disable_listen() {
    return m_hi2c ? HAL_I2C_DisableListen_IT(m_hi2c) : HAL_ERROR;
  }
  
  // 中止传输
  HAL_StatusTypeDef abort() {
    return m_hi2c ? HAL_I2C_Master_Abort_IT(m_hi2c, 0x00) : HAL_ERROR;
  }
  
  // 获取状态
  HAL_I2C_StateTypeDef get_state() const {
    return m_hi2c ? m_hi2c->State : HAL_I2C_STATE_RESET;
  }

  // 获取错误码
  uint32_t get_error() const {
    return m_hi2c ? m_hi2c->ErrorCode : HAL_I2C_ERROR_INVALID_CALLBACK;
  }

   // 获取I2C实例索引
  uint8_t get_i2c_index() const {
    if (!m_hi2c) return 0xFF;
    
    I2C_TypeDef* instance = m_hi2c->Instance;
    if (instance == I2C1) return 0;
    if (instance == I2C2) return 1;
    if (instance == I2C3) return 2;
    return 0xFF;
  }
  
  // 获取HAL句柄
  I2C_HandleTypeDef *get_hi2c() const { return m_hi2c; }

  // 配置接口
  void set_clock_speed(uint32_t clock_speed) {
    if (m_hi2c) m_hi2c->Init.ClockSpeed = clock_speed;
  }
  void set_duty_cycle(uint32_t duty_cycle) {
    if (m_hi2c) m_hi2c->Init.DutyCycle = duty_cycle;
  }
  void set_addressing_mode(uint32_t addressing_mode) {
    if (m_hi2c) m_hi2c->Init.AddressingMode = addressing_mode;
  }
  void set_dual_addressing_mode(uint32_t dual_addressing_mode) {
    if (m_hi2c) m_hi2c->Init.DualAddressMode = dual_addressing_mode;
  }
  void set_own_address1(uint32_t own_address1) {
    if (m_hi2c) m_hi2c->Init.OwnAddress1 = own_address1;
  }
  void set_own_address2(uint32_t own_address2) {
    if (m_hi2c) m_hi2c->Init.OwnAddress2 = own_address2;
  }
  void set_general_call_mode(uint32_t general_call_mode) {
    if (m_hi2c) m_hi2c->Init.GeneralCallMode = general_call_mode;
  }
  void set_no_stretch_mode(uint32_t no_stretch_mode) {
    if (m_hi2c) m_hi2c->Init.NoStretchMode = no_stretch_mode;
  }

   // 应用配置
  HAL_StatusTypeDef apply_config() {
    return m_hi2c ? HAL_I2C_Init(m_hi2c) : HAL_ERROR;
  }

  //回调注册接口
void register_master_tx_cplt_callback(master_tx_cplt_callback_t cb) {
    m_callbacks.master_tx_cplt_cb = std::move(cb);
  }
  void register_master_rx_cplt_callback(master_rx_cplt_callback_t cb) {
    m_callbacks.master_rx_cplt_cb = std::move(cb);
  }
  void register_mem_tx_cplt_callback(mem_tx_cplt_callback_t cb) {
    m_callbacks.mem_tx_cplt_cb = std::move(cb);
  }
  void register_mem_rx_cplt_callback(mem_rx_cplt_callback_t cb) {
    m_callbacks.mem_rx_cplt_cb = std::move(cb);
  }
  void register_error_callback(error_callback_t cb) {
    m_callbacks.error_cb = std::move(cb);
  }
  void register_abort_cplt_callback(abort_cplt_callback_t cb) {
    m_callbacks.abort_cplt_cb = std::move(cb);
  }
  void register_listen_cplt_callback(listen_cplt_callback_t cb) {
    m_callbacks.listen_cplt_cb = std::move(cb);
  }
  
  void register_addr_callback(addr_callback_t cb) {
    m_callbacks.addr_cb = std::move(cb);
  }
  
  void register_slave_tx_cplt_callback(slave_tx_cplt_callback_t cb) {
    m_callbacks.slave_tx_cplt_cb = std::move(cb);
  }
  
  void register_slave_rx_cplt_callback(slave_rx_cplt_callback_t cb) {
    m_callbacks.slave_rx_cplt_cb = std::move(cb);
  }

  //回调调用接口 
  void call_master_tx_cplt_callback() {
    if (m_callbacks.master_tx_cplt_cb) {
      std::invoke(m_callbacks.master_tx_cplt_cb);
    }
  }
  void call_master_rx_cplt_callback() {
    if (m_callbacks.master_rx_cplt_cb) {
      std::invoke(m_callbacks.master_rx_cplt_cb);
    }
  }
  void call_mem_tx_cplt_callback() {
    if (m_callbacks.mem_tx_cplt_cb) {
      std::invoke(m_callbacks.mem_tx_cplt_cb);
    }
  }
  void call_mem_rx_cplt_callback() {
    if (m_callbacks.mem_rx_cplt_cb) {
      std::invoke(m_callbacks.mem_rx_cplt_cb);
    }
  }
  void call_error_callback(uint32_t error) {
    if (m_callbacks.error_cb) {
      std::invoke(m_callbacks.error_cb, error);
    }
  }
  void call_abort_cplt_callback() {
    if (m_callbacks.abort_cplt_cb) {
      std::invoke(m_callbacks.abort_cplt_cb);
    }
  }
  void call_listen_cplt_callback() {
    if (m_callbacks.listen_cplt_cb) {
      std::invoke(m_callbacks.listen_cplt_cb);
    }
  }
  void call_addr_callback(uint16_t addr, uint32_t direction) {
    if (m_callbacks.addr_cb) {
      std::invoke(m_callbacks.addr_cb, addr, direction);
    }
  }
  
  void call_slave_tx_cplt_callback() {
    if (m_callbacks.slave_tx_cplt_cb) {
      std::invoke(m_callbacks.slave_tx_cplt_cb);
    }
  }
  void call_slave_rx_cplt_callback() {
    if (m_callbacks.slave_rx_cplt_cb) {
      std::invoke(m_callbacks.slave_rx_cplt_cb);
    }
  }
private:
  I2C_HandleTypeDef *m_hi2c{nullptr};

  struct i2c_callbacks {
    master_tx_cplt_callback_t master_tx_cplt_cb;   // 主模式发送完成回调
    master_rx_cplt_callback_t master_rx_cplt_cb;   // 主模式接收完成回调
    mem_tx_cplt_callback_t mem_tx_cplt_cb;         // 存储器发送完成回调
    mem_rx_cplt_callback_t mem_rx_cplt_cb;         // 存储器接收完成回调
    error_callback_t error_cb;                     // 错误回调
    abort_cplt_callback_t abort_cplt_cb;           // 中止完成回调
    listen_cplt_callback_t listen_cplt_cb;         // 监听完成回调
    addr_callback_t addr_cb;                       // 地址回调 
    slave_tx_cplt_callback_t slave_tx_cplt_cb;     // 从模式发送完成回调
    slave_rx_cplt_callback_t slave_rx_cplt_cb;     // 从模式接收完成回调
  };
  i2c_callbacks m_callbacks{};

  // 超时转换
  static uint32_t timeout_cast(std::chrono::milliseconds timeout) {
    if (timeout.count() > std::numeric_limits<uint32_t>::max()) {
      return HAL_MAX_DELAY;
    }
    return static_cast<uint32_t>(timeout.count());
  }
};

class i2c_irq_handler {
public:
  i2c_irq_handler() = default;
  ~i2c_irq_handler()= default;
  i2c_irq_handler(const i2c_irq_handler &) = delete;
  i2c_irq_handler &operator=(const i2c_irq_handler &) = delete;
  i2c_irq_handler(i2c_irq_handler &&) = delete;
  i2c_irq_handler &operator=(i2c_irq_handler &&) = delete;

  static bool register_i2c(i2c *i2c_obj) {
    if (!i2c_obj) return false;
    if (!i2c_obj->get_hi2c()) return false;
    
    uint8_t idx = i2c_obj->get_i2c_index();
    if (idx < 3) {
      m_i2cs[idx] = i2c_obj;
      return true;
    }
    return false;
  }

  static void unregister_i2c(i2c *i2c_obj) {
    if (!i2c_obj) return;
    if (!i2c_obj->get_hi2c()) return;
    
    uint8_t idx = i2c_obj->get_i2c_index();
    if (idx < 3) {
      m_i2cs[idx] = nullptr;
    }
  }

    // 主模式接收完成中断
  static void handle_master_rx_cplt(I2C_TypeDef *instance) {
    uint8_t idx = get_i2c_index(instance);
    if (idx < 3 && m_i2cs[idx]) {
      m_i2cs[idx]->call_master_rx_cplt_callback();
    }
  }
  
  // 存储器写完成中断
  static void handle_mem_tx_cplt(I2C_TypeDef *instance) {
    uint8_t idx = get_i2c_index(instance);
    if (idx < 3 && m_i2cs[idx]) {
      m_i2cs[idx]->call_mem_tx_cplt_callback();
    }
  }
  
  // 存储器读完成中断
  static void handle_mem_rx_cplt(I2C_TypeDef *instance) {
    uint8_t idx = get_i2c_index(instance);
    if (idx < 3 && m_i2cs[idx]) {
      m_i2cs[idx]->call_mem_rx_cplt_callback();
    }
  }
  
  // 错误中断
  static void handle_error(I2C_TypeDef *instance, uint32_t error) {
    uint8_t idx = get_i2c_index(instance);
    if (idx < 3 && m_i2cs[idx]) {
      m_i2cs[idx]->call_error_callback(error);
    }
  }
  
  // 中止完成中断
  static void handle_abort_cplt(I2C_TypeDef *instance) {
    uint8_t idx = get_i2c_index(instance);
    if (idx < 3 && m_i2cs[idx]) {
      m_i2cs[idx]->call_abort_cplt_callback();
    }
  }
  
  // 监听完成中断
  static void handle_listen_cplt(I2C_TypeDef *instance) {
    uint8_t idx = get_i2c_index(instance);
    if (idx < 3 && m_i2cs[idx]) {
      m_i2cs[idx]->call_listen_cplt_callback();
    }
  }
  
  // 地址匹配中断
  static void handle_addr(I2C_TypeDef *instance, uint16_t addr, uint32_t direction) {
    uint8_t idx = get_i2c_index(instance);
    if (idx < 3 && m_i2cs[idx]) {
      m_i2cs[idx]->call_addr_callback(addr, direction);
    }
  }
  
  // 从模式发送完成中断
  static void handle_slave_tx_cplt(I2C_TypeDef *instance) {
    uint8_t idx = get_i2c_index(instance);
    if (idx < 3 && m_i2cs[idx]) {
      m_i2cs[idx]->call_slave_tx_cplt_callback();
    }
  }
  
  // 从模式接收完成中断
  static void handle_slave_rx_cplt(I2C_TypeDef *instance) {
    uint8_t idx = get_i2c_index(instance);
    if (idx < 3 && m_i2cs[idx]) {
      m_i2cs[idx]->call_slave_rx_cplt_callback();
    }
  }

private:
  inline static i2c *m_i2cs[3] = {};  // 支持I2C1, I2C2, I2C3
  
  static uint8_t get_i2c_index(I2C_TypeDef *instance) {
    if (instance == I2C1) return 0;
    if (instance == I2C2) return 1;
    if (instance == I2C3) return 2;
    return 0xFF;
  }

};
};

#endif // BSP_IIC_HPP