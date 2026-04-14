#ifndef BSP_TIMER_HPP
#define BSP_TIMER_HPP

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_tim.h"

#include "function.hpp"
#include "uncopyable.hpp"

#include <functional>
#include <utility>

namespace gdut {

class timer : private uncopyable {
public:
  using callback_t = gdut::function<void()>;

  // 构造函数
  timer(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma = nullptr)
      : m_htim(htim), m_hdma(nullptr) {
    attach_dma(hdma);
  }

  ~timer() { deinit(); }

  // 移动构造函数
  timer(timer &&other) noexcept
      : m_htim(other.m_htim), m_hdma(other.m_hdma),
        m_callbacks(std::move(other.m_callbacks)) {
    if (m_hdma) {
      m_hdma->Parent = this;
    }
    other.m_htim = nullptr;
    other.m_hdma = nullptr;
  }

  // 移动赋值运算符
  timer &operator=(timer &&other) noexcept {
    if (this != std::addressof(other)) {
      deinit();
      m_htim = other.m_htim;
      m_hdma = other.m_hdma;
      m_callbacks = std::move(other.m_callbacks);
      if (m_hdma) {
        m_hdma->Parent = this;
      }
      other.m_htim = nullptr;
      other.m_hdma = nullptr;
    }
    return *this;
  }

  // 初始化和去初始化定时器
  HAL_StatusTypeDef init() {
    if (!m_htim) {
      return HAL_ERROR;
    }
    return HAL_TIM_Base_Init(m_htim);
  }
  HAL_StatusTypeDef deinit() {
    if (m_hdma != nullptr) {
      m_hdma->XferCpltCallback = nullptr;
      m_hdma->XferHalfCpltCallback = nullptr;
      m_hdma->XferErrorCallback = nullptr;
      m_hdma->XferAbortCallback = nullptr;
      m_hdma->Parent = nullptr;
      m_hdma = nullptr;
    }
    if (!m_htim) {
      return HAL_OK;
    }
    return HAL_TIM_Base_DeInit(m_htim);
  }

  // 启动和停止定时器
  HAL_StatusTypeDef start() {
    if (!m_htim) {
      return HAL_ERROR;
    }
    return HAL_TIM_Base_Start(m_htim);
  }
  HAL_StatusTypeDef stop() {
    if (!m_htim) {
      return HAL_ERROR;
    }
    return HAL_TIM_Base_Stop(m_htim);
  }

  // 中断和 DMA 控制
  HAL_StatusTypeDef enable_it(uint32_t interrupt) {
    if (!m_htim) {
      return HAL_ERROR;
    }
    __HAL_TIM_ENABLE_IT(m_htim, interrupt);
    return HAL_OK;
  }
  HAL_StatusTypeDef disable_it(uint32_t interrupt) {
    if (!m_htim) {
      return HAL_ERROR;
    }
    __HAL_TIM_DISABLE_IT(m_htim, interrupt);
    return HAL_OK;
  }
  // 回调注册接口（委托给 callback_manager）
  void register_period_elapsed_callback(callback_t cb) {
    m_callbacks.period_elapsed_cb = std::move(cb);
  }

  [[nodiscard]] bool register_capture_callback(uint32_t channel,
                                               callback_t cb) {
    if (channel >= 1 && channel <= 4) {
      m_callbacks.capture_cbs[channel - 1] = std::move(cb);
      return true;
    }
    return false;
  }

  void register_error_callback(callback_t cb) {
    m_callbacks.error_cb = std::move(cb);
  }

  // 回调调用接口（供中断服务例程使用）
  void call_period_elapsed_callback() {
    if (m_callbacks.period_elapsed_cb) {
      std::invoke(m_callbacks.period_elapsed_cb);
    }
  }

  void call_capture_callback(uint32_t channel) {
    if (channel >= 1 && channel <= 4 && m_callbacks.capture_cbs[channel - 1]) {
      std::invoke(m_callbacks.capture_cbs[channel - 1]);
    }
  }

  void call_error_callback() {
    if (m_callbacks.error_cb) {
      std::invoke(m_callbacks.error_cb);
    }
  }

  // DMA 注册接口（委托给 callback_mgr_）
  void register_dma_xfer_cplt_callback(callback_t cb) {
    m_callbacks.dma_xfer_cplt_cb = std::move(cb);
  }
  void register_dma_xfer_half_callback(callback_t cb) {
    m_callbacks.dma_xfer_half_cb = std::move(cb);
  }
  void register_dma_error_callback(callback_t cb) {
    m_callbacks.dma_error_cb = std::move(cb);
  }

  TIM_HandleTypeDef *get_htim() { return m_htim; }
  const TIM_HandleTypeDef *get_htim() const { return m_htim; }

  // timer提供时钟基功能
  class timer_proxy {
  public:
    timer_proxy(timer *t) : m_htim(t ? t->get_htim() : nullptr) {}
    ~timer_proxy() noexcept = default;
    timer_proxy(const timer_proxy &) = default;
    timer_proxy &operator=(const timer_proxy &other) = default;
    timer_proxy(timer_proxy &&other) noexcept
        : m_htim(std::exchange(other.m_htim, nullptr)) {}
    timer_proxy &operator=(timer_proxy &&other) noexcept {
      if (this != std::addressof(other)) {
        m_htim = std::exchange(other.m_htim, nullptr);
      }
      return *this;
    }

    TIM_HandleTypeDef *get_htim() { return m_htim; }
    const TIM_HandleTypeDef *get_htim() const { return m_htim; }

    HAL_StatusTypeDef base_init() { return HAL_TIM_Base_Init(m_htim); }
    HAL_StatusTypeDef base_deinit() { return HAL_TIM_Base_DeInit(m_htim); }

    void set_arr(uint32_t arr) { __HAL_TIM_SET_AUTORELOAD(m_htim, arr); }
    uint32_t get_arr() const { return __HAL_TIM_GET_AUTORELOAD(m_htim); }

    void set_psc(uint32_t psc) { __HAL_TIM_SET_PRESCALER(m_htim, psc); }
    uint32_t get_psc() const { return m_htim->Instance->PSC; }

    void set_counter(uint32_t counter) {
      __HAL_TIM_SET_COUNTER(m_htim, counter);
    }
    uint32_t get_counter() const { return __HAL_TIM_GET_COUNTER(m_htim); }

    /**
     * @brief 获取定时器所在总线的时钟频率（Hz）
     * STM32F407: TIM1, TIM8 在 APB2 (168MHz); 其他在 APB1 (84MHz)
     */
    uint32_t get_apb_clock() const {
      if (!m_htim)
        return 0;
      if (m_htim->Instance == TIM1 || m_htim->Instance == TIM8) {
        return 168000000U;  // APB2
      }
      return 84000000U;  // APB1 (default for TIM2-13)
    }

  private:
    TIM_HandleTypeDef *m_htim;
  };

  // timer提供pwm
  class timer_pwm {
  public:
    timer_pwm(timer *t) : m_proxy(t) {}
    ~timer_pwm() = default;
    timer_pwm(const timer_pwm &) = default;
    timer_pwm &operator=(const timer_pwm &) = default;
    timer_pwm(timer_pwm &&other) noexcept : m_proxy(std::move(other.m_proxy)) {}
    timer_pwm &operator=(timer_pwm &&other) noexcept {
      if (this != std::addressof(other)) {
        m_proxy = std::move(other.m_proxy);
      }
      return *this;
    }

    HAL_StatusTypeDef pwm_start(uint32_t channel) {
      return HAL_TIM_PWM_Start(m_proxy.get_htim(), channel);
    }
    HAL_StatusTypeDef pwm_stop(uint32_t channel) {
      return HAL_TIM_PWM_Stop(m_proxy.get_htim(), channel);
    }

    HAL_StatusTypeDef pwm_start_it(uint32_t channel) {
      return HAL_TIM_PWM_Start_IT(m_proxy.get_htim(), channel);
    }
    HAL_StatusTypeDef pwm_stop_it(uint32_t channel) {
      return HAL_TIM_PWM_Stop_IT(m_proxy.get_htim(), channel);
    }

    HAL_StatusTypeDef pwm_start_dma(uint32_t channel, const uint32_t *pData,
                                    uint16_t Length) {
      return HAL_TIM_PWM_Start_DMA(m_proxy.get_htim(), channel, pData, Length);
    }
    HAL_StatusTypeDef pwm_stop_dma(uint32_t channel) {
      return HAL_TIM_PWM_Stop_DMA(m_proxy.get_htim(), channel);
    }

    HAL_StatusTypeDef pwm_start_complementary(uint32_t channel) {
      return HAL_TIMEx_PWMN_Start(m_proxy.get_htim(), channel);
    }
    HAL_StatusTypeDef pwm_stop_complementary(uint32_t channel) {
      return HAL_TIMEx_PWMN_Stop(m_proxy.get_htim(), channel);
    }

    HAL_StatusTypeDef set_duty(uint32_t channel, uint32_t duty) {
      switch (channel) {
      case TIM_CHANNEL_1:
        __HAL_TIM_SET_COMPARE(m_proxy.get_htim(), TIM_CHANNEL_1, duty);
        break;
      case TIM_CHANNEL_2:
        __HAL_TIM_SET_COMPARE(m_proxy.get_htim(), TIM_CHANNEL_2, duty);
        break;
      case TIM_CHANNEL_3:
        __HAL_TIM_SET_COMPARE(m_proxy.get_htim(), TIM_CHANNEL_3, duty);
        break;
      case TIM_CHANNEL_4:
        __HAL_TIM_SET_COMPARE(m_proxy.get_htim(), TIM_CHANNEL_4, duty);
        break;
      default:
        return HAL_ERROR;
      }
      return HAL_OK;
    }

  private:
    timer_proxy m_proxy;
  };

  // timer提供编码器接口
  class timer_encoder {
  public:
    timer_encoder(timer *t) : m_proxy(t) {}
    ~timer_encoder() = default;
    timer_encoder(const timer_encoder &) = default;
    timer_encoder &operator=(const timer_encoder &) = default;
    timer_encoder(timer_encoder &&other) noexcept
        : m_proxy(std::move(other.m_proxy)) {}
    timer_encoder &operator=(timer_encoder &&other) noexcept {
      if (this != std::addressof(other)) {
        m_proxy = std::move(other.m_proxy);
      }
      return *this;
    }

    HAL_StatusTypeDef encoder_start(uint32_t channel) {
      return HAL_TIM_Encoder_Start(m_proxy.get_htim(), channel);
    }
    HAL_StatusTypeDef encoder_stop(uint32_t channel) {
      return HAL_TIM_Encoder_Stop(m_proxy.get_htim(), channel);
    }

    HAL_StatusTypeDef encoder_start_it(uint32_t channel) {
      return HAL_TIM_Encoder_Start_IT(m_proxy.get_htim(), channel);
    }
    HAL_StatusTypeDef encoder_stop_it(uint32_t channel) {
      return HAL_TIM_Encoder_Stop_IT(m_proxy.get_htim(), channel);
    }

    HAL_StatusTypeDef encoder_start_dma(uint32_t channel, uint32_t *pData1,
                                        uint32_t *pData2, uint16_t Length) {
      return HAL_TIM_Encoder_Start_DMA(m_proxy.get_htim(), channel, pData1,
                                       pData2, Length);
    }
    HAL_StatusTypeDef encoder_stop_dma(uint32_t channel) {
      return HAL_TIM_Encoder_Stop_DMA(m_proxy.get_htim(), channel);
    }

    bool is_counting_down() const {
      return __HAL_TIM_IS_TIM_COUNTING_DOWN(m_proxy.get_htim());
    }

    HAL_StatusTypeDef enable_index_interrupt(uint32_t interrupt) {
      __HAL_TIM_ENABLE_IT(m_proxy.get_htim(), interrupt);
      return HAL_OK;
    }
    HAL_StatusTypeDef disable_index_interrupt(uint32_t interrupt) {
      __HAL_TIM_DISABLE_IT(m_proxy.get_htim(), interrupt);
      return HAL_OK;
    }

  private:
    timer_proxy m_proxy;
  };

  // timer提供输出比较接口
  class timer_oc {
  public:
    timer_oc(timer *t) : m_proxy(t) {}
    ~timer_oc() = default;
    timer_oc(const timer_oc &) = default;
    timer_oc &operator=(const timer_oc &) = default;
    timer_oc(timer_oc &&other) noexcept : m_proxy(std::move(other.m_proxy)) {}
    timer_oc &operator=(timer_oc &&other) noexcept {
      if (this != std::addressof(other)) {
        m_proxy = std::move(other.m_proxy);
      }
      return *this;
    }

    HAL_StatusTypeDef oc_start(uint32_t channel) {
      return HAL_TIM_OC_Start(m_proxy.get_htim(), channel);
    }
    HAL_StatusTypeDef oc_stop(uint32_t channel) {
      return HAL_TIM_OC_Stop(m_proxy.get_htim(), channel);
    }

    HAL_StatusTypeDef oc_start_it(uint32_t channel) {
      return HAL_TIM_OC_Start_IT(m_proxy.get_htim(), channel);
    }
    HAL_StatusTypeDef oc_stop_it(uint32_t channel) {
      return HAL_TIM_OC_Stop_IT(m_proxy.get_htim(), channel);
    }

    HAL_StatusTypeDef set_compare(uint32_t channel, uint32_t compare) {
      __HAL_TIM_SET_COMPARE(m_proxy.get_htim(), channel, compare);
      return HAL_OK;
    }
    uint32_t get_compare(uint32_t channel) {
      return __HAL_TIM_GET_COMPARE(m_proxy.get_htim(), channel);
    }

  private:
    timer_proxy m_proxy;
  };

  // timer提供输入捕获接口
  class timer_ic {
  public:
    timer_ic(timer *t) : m_proxy(t) {}
    ~timer_ic() = default;
    timer_ic(const timer_ic &) = default;
    timer_ic &operator=(const timer_ic &) = default;
    timer_ic(timer_ic &&other) noexcept : m_proxy(std::move(other.m_proxy)) {}
    timer_ic &operator=(timer_ic &&other) noexcept {
      if (this != std::addressof(other)) {
        m_proxy = std::move(other.m_proxy);
      }
      return *this;
    }

    HAL_StatusTypeDef ic_start(uint32_t channel) {
      return HAL_TIM_IC_Start(m_proxy.get_htim(), channel);
    }
    HAL_StatusTypeDef ic_stop(uint32_t channel) {
      return HAL_TIM_IC_Stop(m_proxy.get_htim(), channel);
    }

    HAL_StatusTypeDef ic_start_it(uint32_t channel) {
      return HAL_TIM_IC_Start_IT(m_proxy.get_htim(), channel);
    }
    HAL_StatusTypeDef ic_stop_it(uint32_t channel) {
      return HAL_TIM_IC_Stop_IT(m_proxy.get_htim(), channel);
    }

    HAL_StatusTypeDef ic_start_dma(uint32_t channel, uint32_t *pData,
                                   uint16_t Length) {
      return HAL_TIM_IC_Start_DMA(m_proxy.get_htim(), channel, pData, Length);
    }
    HAL_StatusTypeDef ic_stop_dma(uint32_t channel) {
      return HAL_TIM_IC_Stop_DMA(m_proxy.get_htim(), channel);
    }
    uint32_t get_capture(uint32_t channel) {
      return __HAL_TIM_GET_COMPARE(m_proxy.get_htim(), channel);
    }

  private:
    timer_proxy m_proxy;
  };

  // 将 HAL DMA 回调桥接到 timer_manager
  void attach_dma(DMA_HandleTypeDef *hdma) {
    this->m_hdma = hdma;
    if (!hdma)
      return;
    hdma->Parent = this;
    hdma->XferCpltCallback = &timer::dma_xfer_cplt_cb;
    hdma->XferHalfCpltCallback = &timer::dma_xfer_half_cplt_cb;
    hdma->XferErrorCallback = &timer::dma_error_cb;
    hdma->XferAbortCallback = &timer::dma_abort_cb;
  }

  static void dma_xfer_cplt_cb(DMA_HandleTypeDef *hdma) {
    if (!hdma)
      return;
    timer *t = static_cast<timer *>(hdma->Parent);
    if (t && t->m_callbacks.dma_xfer_cplt_cb) {
      std::invoke(t->m_callbacks.dma_xfer_cplt_cb);
    }
  }
  static void dma_xfer_half_cplt_cb(DMA_HandleTypeDef *hdma) {
    if (!hdma)
      return;
    timer *t = static_cast<timer *>(hdma->Parent);
    if (t && t->m_callbacks.dma_xfer_half_cb) {
      std::invoke(t->m_callbacks.dma_xfer_half_cb);
    }
  }
  static void dma_error_cb(DMA_HandleTypeDef *hdma) {
    if (!hdma)
      return;
    timer *t = static_cast<timer *>(hdma->Parent);
    if (t && t->m_callbacks.dma_error_cb) {
      std::invoke(t->m_callbacks.dma_error_cb);
    }
  }
  static void dma_abort_cb(DMA_HandleTypeDef *hdma) {
    if (!hdma)
      return;
    timer *t = static_cast<timer *>(hdma->Parent);
    if (t && t->m_callbacks.dma_error_cb) {
      std::invoke(t->m_callbacks.dma_error_cb);
    }
  }

protected:
  struct timer_callbacks {
    callback_t period_elapsed_cb{}; // 更新中断回调
    callback_t capture_cbs[4]{};    // 捕获中断回调（最多4个通道）
    callback_t error_cb{};
    callback_t dma_xfer_cplt_cb{};
    callback_t dma_xfer_half_cb{};
    callback_t dma_error_cb{};
  };

private:
  TIM_HandleTypeDef *m_htim{nullptr};
  DMA_HandleTypeDef *m_hdma{nullptr};
  timer_callbacks m_callbacks{};
};
} // namespace gdut
#endif // BSP_TIMER_HPP
