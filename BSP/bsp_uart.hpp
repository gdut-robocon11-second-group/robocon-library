#ifndef BSP_UART_HPP
#define BSP_UART_HPP

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_uart.h"

#include "bsp_dma.hpp"
#include "bsp_utility.hpp"
#include "function.hpp"
#include "uncopyable.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <utility>

namespace gdut {
/**
 * @brief 针对 UART 外设的 DMA 操作封装
 *
 * 通过 CRTP 继承 dma_transfer_base，将 TX/RX dma_proxy 与 UART HAL 句柄关联，
 * 并手动配置 DMA 发起传输。传输完成或出错时，由内部静态回调恢复 UART HAL 状态
 * 后再通过 dma_proxy::call_dma_callback() 转发用户回调。
 *
 * @note address 参数对 UART 无意义，所有传输均忽略该参数。
 * @note 不直接调用 HAL_UART_Transmit_DMA / HAL_UART_Receive_DMA，
 *       以防止 HAL 内部覆盖 dma_proxy 已注册的回调函数。
 */
class dma_uart : public dma_transfer_base<dma_uart> {
public:
  explicit dma_uart(UART_HandleTypeDef *m_uart) : m_uart(m_uart) {}

  ~dma_uart() = default;

  dma_uart(dma_uart &&other) noexcept
      : m_uart(std::exchange(other.m_uart, nullptr)),
        m_tx_dma(std::exchange(other.m_tx_dma, nullptr)),
        m_rx_dma(std::exchange(other.m_rx_dma, nullptr)) {}

  dma_uart &operator=(dma_uart &&other) noexcept {
    if (this != std::addressof(other)) {
      m_uart = std::exchange(other.m_uart, nullptr);
      m_tx_dma = std::exchange(other.m_tx_dma, nullptr);
      m_rx_dma = std::exchange(other.m_rx_dma, nullptr);
    }
    return *this;
  }

private:
  friend class dma_transfer_base<dma_uart>;

  void do_bind_tx(dma_proxy *tx_dma) {
    if (tx_dma && m_uart) {
      m_tx_dma = tx_dma;
      __HAL_LINKDMA(m_uart, hdmatx, *tx_dma->get_handle());
    }
  }

  void do_bind_rx(dma_proxy *rx_dma) {
    if (rx_dma && m_uart) {
      m_rx_dma = rx_dma;
      __HAL_LINKDMA(m_uart, hdmarx, *rx_dma->get_handle());
    }
  }

  bool do_transmit(const uint8_t *data, std::size_t size, uint16_t address) {
    (void)address; // UART 不使用地址参数，忽略以避免编译器警告

    // 传输大小超过 HAL uint16_t 范围则拒绝，避免截断导致错误传输
    if (m_uart == nullptr || m_tx_dma == nullptr || data == nullptr ||
        size == 0U || size > 65535U) {
      return false;
    }

    if (m_uart->gState != HAL_UART_STATE_READY) {
      return false;
    }

    // 初始化 DMA 代理（调用 HAL_DMA_Init 初始化 DMA 句柄）
    m_tx_dma->init();

    // 覆盖 DMA 句柄的 Parent 与完成/错误回调，以便在 DMA 中断里恢复 UART 状态
    // 并通过 dma_proxy::call_dma_callback() 转发到用户回调。
    // 注意：不能直接调用 HAL_UART_Transmit_DMA，否则 HAL 会重新覆盖这些回调。
    DMA_HandleTypeDef *hdma_tx = m_tx_dma->get_handle();
    if (hdma_tx == nullptr) {
      m_uart->ErrorCode = HAL_UART_ERROR_DMA;
      return false;
    }
    hdma_tx->Parent = this;
    hdma_tx->XferCpltCallback = tx_dma_cplt_cb;
    hdma_tx->XferErrorCallback = tx_dma_error_cb;

    // 参考 HAL_UART_Transmit_DMA 内部实现，手动配置 UART 状态与 DMA
    m_uart->pTxBuffPtr = const_cast<uint8_t *>(data);
    m_uart->TxXferSize = static_cast<uint16_t>(size);
    m_uart->TxXferCount = static_cast<uint16_t>(size);
    m_uart->ErrorCode = HAL_UART_ERROR_NONE;
    m_uart->gState = HAL_UART_STATE_BUSY_TX;

    const auto src_addr =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(data));
    const auto dst_addr = static_cast<uint32_t>(
        reinterpret_cast<uintptr_t>(&m_uart->Instance->DR));
    if (HAL_DMA_Start_IT(hdma_tx, src_addr, dst_addr,
                         static_cast<uint16_t>(size)) != HAL_OK) {
      m_uart->ErrorCode = HAL_UART_ERROR_DMA;
      m_uart->gState = HAL_UART_STATE_READY;
      return false;
    }

    __HAL_UART_CLEAR_FLAG(m_uart, UART_FLAG_TC);
    ATOMIC_SET_BIT(m_uart->Instance->CR3, USART_CR3_DMAT);
    return true;
  }

  bool do_receive(uint8_t *buffer, std::size_t size, uint16_t address) {
    (void)address; // UART 不使用地址参数，忽略以避免编译器警告

    // 传输大小超过 HAL uint16_t 范围则拒绝，避免截断导致错误传输
    if (m_uart == nullptr || m_rx_dma == nullptr || buffer == nullptr ||
        size == 0U || size > 65535U) {
      return false;
    }

    if (m_uart->RxState == HAL_UART_STATE_BUSY_RX) {
      return false; // 已有接收在进行中
    }

    // 初始化 DMA 代理（调用 HAL_DMA_Init 初始化 DMA 句柄）
    m_rx_dma->init();

    // 覆盖 DMA 句柄的 Parent 与完成/错误回调，以便在 DMA 中断里恢复 UART 状态
    // 并通过 dma_proxy::call_dma_callback() 转发到用户回调。
    DMA_HandleTypeDef *hdma_rx = m_rx_dma->get_handle();
    if (hdma_rx == nullptr) {
      m_uart->ErrorCode = HAL_UART_ERROR_DMA;
      return false;
    }
    hdma_rx->Parent = this;
    hdma_rx->XferCpltCallback = rx_dma_cplt_cb;
    hdma_rx->XferErrorCallback = rx_dma_error_cb;

    // 参考 HAL_UART_Receive_DMA 内部实现，手动配置 UART 状态与 DMA
    m_uart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
    m_uart->pRxBuffPtr = buffer;
    m_uart->RxXferSize = static_cast<uint16_t>(size);
    m_uart->ErrorCode = HAL_UART_ERROR_NONE;
    m_uart->RxState = HAL_UART_STATE_BUSY_RX;

    const auto src_addr = static_cast<uint32_t>(
        reinterpret_cast<uintptr_t>(&m_uart->Instance->DR));
    const auto dst_addr =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buffer));
    if (HAL_DMA_Start_IT(hdma_rx, src_addr, dst_addr,
                         static_cast<uint16_t>(size)) != HAL_OK) {
      m_uart->ErrorCode = HAL_UART_ERROR_DMA;
      m_uart->RxState = HAL_UART_STATE_READY;
      return false;
    }

    __HAL_UART_CLEAR_OREFLAG(m_uart);
    if (m_uart->Init.Parity != UART_PARITY_NONE) {
      ATOMIC_SET_BIT(m_uart->Instance->CR1, USART_CR1_PEIE);
    }
    ATOMIC_SET_BIT(m_uart->Instance->CR3, USART_CR3_EIE);
    ATOMIC_SET_BIT(m_uart->Instance->CR3, USART_CR3_DMAR);
    return true;
  }

  // TX DMA 传输完成回调：清除 DMAT 位、恢复 gState，然后转发用户回调
  static void tx_dma_cplt_cb(DMA_HandleTypeDef *hdma) {
    if (!hdma)
      return;
    auto *self = static_cast<dma_uart *>(hdma->Parent);
    if (!self || !self->m_uart)
      return;
    ATOMIC_CLEAR_BIT(self->m_uart->Instance->CR3, USART_CR3_DMAT);
    self->m_uart->gState = HAL_UART_STATE_READY;
    if (self->m_tx_dma)
      self->m_tx_dma->call_dma_callback({});
  }

  // TX DMA 错误回调：清除 DMAT 位、恢复 gState，然后上报错误
  static void tx_dma_error_cb(DMA_HandleTypeDef *hdma) {
    if (!hdma)
      return;
    auto *self = static_cast<dma_uart *>(hdma->Parent);
    if (!self || !self->m_uart)
      return;
    ATOMIC_CLEAR_BIT(self->m_uart->Instance->CR3, USART_CR3_DMAT);
    self->m_uart->ErrorCode = HAL_UART_ERROR_DMA;
    self->m_uart->gState = HAL_UART_STATE_READY;
    if (self->m_tx_dma)
      self->m_tx_dma->call_dma_callback(
          std::error_code(hdma->ErrorCode, dma_error_category::instance()));
  }

  // RX DMA 传输完成回调：清除 DMAR/EIE/PEIE 位、恢复 RxState，然后转发用户回调
  static void rx_dma_cplt_cb(DMA_HandleTypeDef *hdma) {
    if (!hdma)
      return;
    auto *self = static_cast<dma_uart *>(hdma->Parent);
    if (!self || !self->m_uart)
      return;
    ATOMIC_CLEAR_BIT(self->m_uart->Instance->CR3,
                     USART_CR3_DMAR | USART_CR3_EIE);
    if (self->m_uart->Init.Parity != UART_PARITY_NONE) {
      ATOMIC_CLEAR_BIT(self->m_uart->Instance->CR1, USART_CR1_PEIE);
    }
    self->m_uart->RxState = HAL_UART_STATE_READY;
    if (self->m_rx_dma)
      self->m_rx_dma->call_dma_callback({});
  }

  // RX DMA 错误回调：清除 DMAR/EIE/PEIE 位、恢复 RxState，然后上报错误
  static void rx_dma_error_cb(DMA_HandleTypeDef *hdma) {
    if (!hdma)
      return;
    auto *self = static_cast<dma_uart *>(hdma->Parent);
    if (!self || !self->m_uart)
      return;
    ATOMIC_CLEAR_BIT(self->m_uart->Instance->CR3,
                     USART_CR3_DMAR | USART_CR3_EIE);
    if (self->m_uart->Init.Parity != UART_PARITY_NONE) {
      ATOMIC_CLEAR_BIT(self->m_uart->Instance->CR1, USART_CR1_PEIE);
    }
    self->m_uart->ErrorCode = HAL_UART_ERROR_DMA;
    self->m_uart->RxState = HAL_UART_STATE_READY;
    if (self->m_rx_dma)
      self->m_rx_dma->call_dma_callback(
          std::error_code(hdma->ErrorCode, dma_error_category::instance()));
  }

  UART_HandleTypeDef *m_uart{nullptr};
  dma_proxy *m_tx_dma{nullptr};
  dma_proxy *m_rx_dma{nullptr};
};

class uart : private uncopyable {
public:
  using rx_callback_t =
      gdut::function<void(const uint8_t *data, uint16_t size)>;
  using tx_callback_t = gdut::function<void()>;
  using error_callback_t = gdut::function<void(uint32_t error)>;
  using idle_callback_t = gdut::function<void()>;
  using dma_rx_cplt_callback_t = gdut::function<void()>;
  using dma_tx_cplt_callback_t = gdut::function<void()>;
  using dma_error_callback_t = gdut::function<void()>;

  uart(UART_HandleTypeDef *huart, bool use_half_duplex = false,
       DMA_HandleTypeDef *hdma_rx = nullptr,
       DMA_HandleTypeDef *hdma_tx = nullptr)
      : m_huart(huart), m_hdma_rx(nullptr), m_hdma_tx(nullptr),
        m_use_half_duplex(use_half_duplex) {
    init(hdma_rx, hdma_tx);
  }

  ~uart() noexcept { deinit(); }

  // 初始化
  HAL_StatusTypeDef init(DMA_HandleTypeDef *hdma_rx = nullptr,
                         DMA_HandleTypeDef *hdma_tx = nullptr) {
    if (m_huart) {
      attach_dma_rx(hdma_rx);
      attach_dma_tx(hdma_tx);
    }
    if (is_half_duplex_mode()) {
      return HAL_HalfDuplex_Init(m_huart);
    }
    return HAL_UART_Init(m_huart);
  }

  HAL_StatusTypeDef deinit() {
    if (HAL_UART_DeInit(m_huart) != HAL_OK) {
      return HAL_ERROR;
    }
    if (m_hdma_rx) {
      m_hdma_rx->Parent = nullptr;
      m_hdma_rx->XferCpltCallback = nullptr;
      m_hdma_rx->XferHalfCpltCallback = nullptr;
      m_hdma_rx->XferErrorCallback = nullptr;
      m_hdma_rx->XferAbortCallback = nullptr;
    }
    if (m_hdma_tx) {
      m_hdma_tx->Parent = nullptr;
      m_hdma_tx->XferCpltCallback = nullptr;
      m_hdma_tx->XferHalfCpltCallback = nullptr;
      m_hdma_tx->XferErrorCallback = nullptr;
      m_hdma_tx->XferAbortCallback = nullptr;
    }
    return HAL_OK;
  }

  uart(uart &&other) noexcept
      : m_huart(std::exchange(other.m_huart, nullptr)),
        m_hdma_rx(std::exchange(other.m_hdma_rx, nullptr)),
        m_hdma_tx(std::exchange(other.m_hdma_tx, nullptr)),
        m_callbacks(std::move(other.m_callbacks)) {}
  uart &operator=(uart &&other) noexcept {
    if (this != std::addressof(other)) {
      deinit();
      m_huart = std::exchange(other.m_huart, nullptr);
      m_hdma_rx = std::exchange(other.m_hdma_rx, nullptr);
      m_hdma_tx = std::exchange(other.m_hdma_tx, nullptr);
      m_callbacks = std::move(other.m_callbacks);
    }
    return *this;
  }

  // 发送函数（阻塞模式）
  HAL_StatusTypeDef
  send(const uint8_t *data, uint16_t size,
       std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
    if (is_half_duplex_mode()) {
      if (HAL_HalfDuplex_EnableTransmitter(m_huart) != HAL_OK) {
        return HAL_ERROR;
      }
    }
    return HAL_UART_Transmit(m_huart, const_cast<uint8_t *>(data), size,
                             timeout.count() >
                                     std::numeric_limits<uint32_t>::max()
                                 ? std::numeric_limits<uint32_t>::max()
                                 : static_cast<uint32_t>(timeout.count()));
  }

  // 接收函数（阻塞模式）
  HAL_StatusTypeDef receive(
      uint8_t *data, uint16_t size,
      std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
    if (is_half_duplex_mode()) {
      if (HAL_HalfDuplex_EnableReceiver(m_huart) != HAL_OK) {
        return HAL_ERROR;
      }
      __HAL_UART_CLEAR_OREFLAG(m_huart);
    }
    return HAL_UART_Receive(m_huart, data, size,
                            timeout.count() >
                                    std::numeric_limits<uint32_t>::max()
                                ? std::numeric_limits<uint32_t>::max()
                                : static_cast<uint32_t>(timeout.count()));
  }

  // 发送函数（中断模式）
  HAL_StatusTypeDef send_it(const uint8_t *data, uint16_t size) {
    return HAL_UART_Transmit_IT(m_huart, const_cast<uint8_t *>(data), size);
  }

  // 接收函数（中断模式）
  HAL_StatusTypeDef receive_it(uint8_t *data, uint16_t size) {
    return HAL_UART_Receive_IT(m_huart, data, size);
  }

  // 发送函数（DMA模式）
  HAL_StatusTypeDef send_dma(const uint8_t *data, uint16_t size) {
    if (!m_hdma_tx) {
      return HAL_ERROR;
    }
    return HAL_UART_Transmit_DMA(m_huart, const_cast<uint8_t *>(data), size);
  }

  // 接收函数（DMA模式）
  HAL_StatusTypeDef receive_dma(uint8_t *data, uint16_t size) {
    if (!m_hdma_rx) {
      return HAL_ERROR;
    }
    return HAL_UART_Receive_DMA(m_huart, data, size);
  }

  // 中断控制
  HAL_StatusTypeDef enable_it(uint32_t interrupt) {
    __HAL_UART_ENABLE_IT(m_huart, interrupt);
    return HAL_OK;
  }
  HAL_StatusTypeDef disable_it(uint32_t interrupt) {
    __HAL_UART_DISABLE_IT(m_huart, interrupt);
    return HAL_OK;
  }

  // 获取HAL句柄
  UART_HandleTypeDef *get_huart() const { return m_huart; }

  // 获取DMA句柄
  DMA_HandleTypeDef *get_hdma_rx() const { return m_hdma_rx; }
  DMA_HandleTypeDef *get_hdma_tx() const { return m_hdma_tx; }

  // 检查发送是否完成
  bool is_tx_complete() const {
    return __HAL_UART_GET_FLAG(m_huart, UART_FLAG_TC) != RESET;
  }

  // 检查是否有数据可读
  bool is_rx_ready() const {
    return __HAL_UART_GET_FLAG(m_huart, UART_FLAG_RXNE) != RESET;
  }

  // 配置接口
  void set_baudrate(uint32_t baudrate) { m_huart->Init.BaudRate = baudrate; }
  void set_word_length(uint32_t word_length) {
    m_huart->Init.WordLength = word_length;
  }
  void set_stop_bits(uint32_t stop_bits) { m_huart->Init.StopBits = stop_bits; }
  void set_parity(uint32_t parity) { m_huart->Init.Parity = parity; }
  void set_mode(uint32_t mode) { m_huart->Init.Mode = mode; }
  void set_hw_flow_ctl(uint32_t hw_flow_ctl) {
    m_huart->Init.HwFlowCtl = hw_flow_ctl;
  }
  void set_over_sampling(uint32_t over_sampling) {
    m_huart->Init.OverSampling = over_sampling;
  }
  HAL_StatusTypeDef apply_config() {
    if (is_half_duplex_mode()) {
      return HAL_HalfDuplex_Init(m_huart);
    }
    return HAL_UART_Init(m_huart);
  }

  // DMA关联函数
  void attach_dma_rx(DMA_HandleTypeDef *hdma_rx) {
    m_hdma_rx = hdma_rx;
    if (!m_hdma_rx) {
      return;
    }

    // 关联DMA回调
    hdma_rx->Parent = this;
    hdma_rx->XferCpltCallback = &uart::dma_rx_xfer_cplt_cb;
    hdma_rx->XferHalfCpltCallback = &uart::dma_rx_xfer_half_cplt_cb;
    hdma_rx->XferErrorCallback = &uart::dma_error_cb;
    hdma_rx->XferAbortCallback = &uart::dma_abort_cb;

    // 设置UART的DMA接收句柄
    __HAL_LINKDMA(m_huart, hdmarx, *m_hdma_rx);
  }

  void attach_dma_tx(DMA_HandleTypeDef *hdma_tx) {
    m_hdma_tx = hdma_tx;
    if (!m_hdma_tx) {
      return;
    }
    // 关联DMA回调
    hdma_tx->Parent = this;
    hdma_tx->XferCpltCallback = &uart::dma_tx_xfer_cplt_cb;
    hdma_tx->XferHalfCpltCallback = nullptr; // 发送通常不需要半传输完成回调
    hdma_tx->XferErrorCallback = &uart::dma_error_cb;
    hdma_tx->XferAbortCallback = &uart::dma_abort_cb;
    // 设置UART的DMA发送句柄
    __HAL_LINKDMA(m_huart, hdmatx, *m_hdma_tx);
  }

  // DMA静态回调函数
  static void dma_rx_xfer_cplt_cb(DMA_HandleTypeDef *hdma) {
    if (!hdma) {
      return;
    }
    uart *u = static_cast<uart *>(hdma->Parent);
    if (u && u->m_callbacks.dma_rx_cplt_cb) {
      std::invoke(u->m_callbacks.dma_rx_cplt_cb);
    }
  }
  static void dma_rx_xfer_half_cplt_cb(DMA_HandleTypeDef *hdma) {
    if (!hdma) {
      return;
    }
    uart *u = static_cast<uart *>(hdma->Parent);
    // 半传输完成时复用接收完成回调，使上层可通过同一回调处理两种事件
    if (u && u->m_callbacks.dma_rx_cplt_cb) {
      std::invoke(u->m_callbacks.dma_rx_cplt_cb);
    }
  }
  static void dma_tx_xfer_cplt_cb(DMA_HandleTypeDef *hdma) {
    if (!hdma) {
      return;
    }
    uart *u = static_cast<uart *>(hdma->Parent);
    if (u && u->m_callbacks.dma_tx_cplt_cb) {
      std::invoke(u->m_callbacks.dma_tx_cplt_cb);
    }
  }
  static void dma_error_cb(DMA_HandleTypeDef *hdma) {
    if (!hdma)
      return;
    uart *u = static_cast<uart *>(hdma->Parent);
    if (u && u->m_callbacks.dma_error_cb) {
      std::invoke(u->m_callbacks.dma_error_cb);
    }
  }
  static void dma_abort_cb(DMA_HandleTypeDef *hdma) {
    if (!hdma)
      return;
    uart *u = static_cast<uart *>(hdma->Parent);
    if (u && u->m_callbacks.dma_error_cb) {
      std::invoke(u->m_callbacks.dma_error_cb);
    }
  }

  // 回调注册接口
  void register_rx_callback(rx_callback_t cb) {
    m_callbacks.rx_cb = std::move(cb);
  }
  void register_tx_callback(tx_callback_t cb) {
    m_callbacks.tx_cb = std::move(cb);
  }
  void register_error_callback(error_callback_t cb) {
    m_callbacks.error_cb = std::move(cb);
  }
  void register_idle_callback(idle_callback_t cb) {
    m_callbacks.idle_cb = std::move(cb);
  }
  void register_dma_rx_cplt_callback(dma_rx_cplt_callback_t cb) {
    m_callbacks.dma_rx_cplt_cb = std::move(cb);
  }
  void register_dma_tx_cplt_callback(dma_tx_cplt_callback_t cb) {
    m_callbacks.dma_tx_cplt_cb = std::move(cb);
  }
  void register_dma_error_callback(dma_error_callback_t cb) {
    m_callbacks.dma_error_cb = std::move(cb);
  }

  // 回调调用接口（供中断服务例程使用）
  void call_rx_callback(const uint8_t *data, uint16_t size) {
    if (m_callbacks.rx_cb) {
      std::invoke(m_callbacks.rx_cb, data, size);
    }
  }
  void call_tx_callback() {
    if (m_callbacks.tx_cb) {
      std::invoke(m_callbacks.tx_cb);
    }
  }
  void call_error_callback(uint32_t error) {
    if (m_callbacks.error_cb) {
      std::invoke(m_callbacks.error_cb, error);
    }
  }
  void call_idle_callback() {
    if (m_callbacks.idle_cb) {
      std::invoke(m_callbacks.idle_cb);
    }
  }
  void call_dma_rx_cplt_callback() {
    if (m_callbacks.dma_rx_cplt_cb) {
      std::invoke(m_callbacks.dma_rx_cplt_cb);
    }
  }
  void call_dma_tx_cplt_callback() {
    if (m_callbacks.dma_tx_cplt_cb) {
      std::invoke(m_callbacks.dma_tx_cplt_cb);
    }
  }
  void call_dma_error_callback() {
    if (m_callbacks.dma_error_cb) {
      std::invoke(m_callbacks.dma_error_cb);
    }
  }

protected:
  struct uart_callbacks {
    rx_callback_t rx_cb;                   // 接收完成回调
    tx_callback_t tx_cb;                   // 发送完成回调
    error_callback_t error_cb;             // 错误回调
    idle_callback_t idle_cb;               // 空闲中断回调
    dma_rx_cplt_callback_t dma_rx_cplt_cb; // DMA接收完成回调
    dma_tx_cplt_callback_t dma_tx_cplt_cb; // DMA发送完成回调
    dma_error_callback_t dma_error_cb;     // DMA错误回调
  };

private:
  [[nodiscard]] bool is_half_duplex_mode() const { return m_use_half_duplex; }

  UART_HandleTypeDef *m_huart{nullptr};  // UART句柄
  DMA_HandleTypeDef *m_hdma_rx{nullptr}; // 接收DMA句柄
  DMA_HandleTypeDef *m_hdma_tx{nullptr}; // 发送DMA句柄
  uart_callbacks m_callbacks{};          // 回调管理器
  bool m_use_half_duplex{false};         // 是否使用半双工模式
};

/**
 * @brief UART 中断分发器与实例注册管理类。
 *
 * 维护一个静态的 @ref uart 实例注册表，并为 HAL/中断回调提供统一的分发入口。
 * 每个底层 `UART_HandleTypeDef::Instance`（如 USART1、USART2 等）被映射到
 * 对应的 @ref uart 对象指针，从而在中断服务程序中根据硬件句柄找到上层封装
 * 对象并调用其回调。
 *
 * @section uart_irq_handler_usage 使用方式
 * - 创建并初始化 @ref uart 对象后，调用 @ref register_uart 注册该实例。
 * - 在 @ref uart 对象被销毁或硬件外设反初始化之前，必须调用
 *   @ref unregister_uart 取消注册，以避免注册表中残留悬空指针。
 * - HAL 层中断服务函数应调用本类提供的静态分发函数，由本类根据
 *   `UART_HandleTypeDef::Instance` 查找对应的 @ref uart 实例并触发其回调。
 *
 * @section uart_irq_handler_thread_safety 线程安全说明
 * - 本类不包含任何同步原语，本身不是线程安全类。
 * - 建议在调度器启动前或临界区内调用 @ref register_uart / @ref unregister_uart
 *   进行静态配置，不要在 ISR 中修改注册表。
 */
class uart_irq_handler {
public:
  uart_irq_handler() = default;
  ~uart_irq_handler() noexcept = default;
  uart_irq_handler(const uart_irq_handler &) = delete;
  uart_irq_handler &operator=(const uart_irq_handler &) = delete;
  uart_irq_handler(uart_irq_handler &&) = delete;
  uart_irq_handler &operator=(uart_irq_handler &&) = delete;

  static bool register_uart(uart *uart_obj) {
    if (!uart_obj)
      return false;
    if (!uart_obj->get_huart())
      return false;
    uint8_t idx = get_uart_index(uart_obj->get_huart()->Instance);
    if (idx < 6) {
      m_uarts[idx] = uart_obj;
      return true;
    }
    return false;
  }
  static void unregister_uart(uart *uart_obj) {
    if (!uart_obj)
      return;
    if (!uart_obj->get_huart())
      return;
    uint8_t idx = get_uart_index(uart_obj->get_huart()->Instance);
    if (idx < 6) {
      m_uarts[idx] = nullptr;
    }
  }
  // 接收完成中断
  static void handle_rx_cplt(USART_TypeDef *instance, const uint8_t *data,
                             uint16_t size) {
    uint8_t idx = get_uart_index(instance);
    if (idx < 6 && m_uarts[idx]) {
      m_uarts[idx]->call_rx_callback(data, size);
    }
  }

  // 发送完成中断
  static void handle_tx_cplt(USART_TypeDef *instance) {
    uint8_t idx = get_uart_index(instance);
    if (idx < 6 && m_uarts[idx]) {
      m_uarts[idx]->call_tx_callback();
    }
  }

  // 错误中断
  static void handle_error(USART_TypeDef *instance, uint32_t error) {
    uint8_t idx = get_uart_index(instance);
    if (idx < 6 && m_uarts[idx]) {
      m_uarts[idx]->call_error_callback(error);
    }
  }

  // 空闲中断
  static void handle_idle(USART_TypeDef *instance) {
    uint8_t idx = get_uart_index(instance);
    if (idx < 6 && m_uarts[idx]) {
      m_uarts[idx]->call_idle_callback();
    }
  }

  // 接收中断（字节接收）
  static void handle_rx_byte(USART_TypeDef *instance, uint8_t data) {
    uint8_t idx = get_uart_index(instance);
    if (idx < 6 && m_uarts[idx]) {
      // 这里可以添加环形缓冲区处理
      // m_uarts[idx]->m_rx_buffer.push_back(data);
      // 或者直接回调
      m_uarts[idx]->call_rx_callback(&data, 1);
    }
  }

private:
  inline static uart *m_uarts[6] =
      {}; // 支持USART1, USART2, USART3, UART4, UART5, USART6
};

} // namespace gdut

#endif // BSP_UART_HPP