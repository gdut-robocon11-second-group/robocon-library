#ifndef BSP_UTILITY_HPP
#define BSP_UTILITY_HPP

#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dma.h"
#include <chrono>
#include <cmsis_os2.h>
#include <cstdint>

/**
 * @brief 将对象放置到核心耦合存储器（.ccmram）的属性
 *
 * @note 在 STM32F407 上，CCM RAM 不能被任何 DMA 控制器访问。
 *       线程栈、控制块和函数对象可以安全地放在 CCM RAM。
 *       但任何直接或间接作为 DMA 源/目的的数据缓冲区都
 *       不能放在 CCM RAM 中（即不要使用 GDUT_CCMRAM 标注、
 *       不要从 CCMRAM 池分配，也不要捕获到存放在 CCMRAM 的 lambda）。
 */
#define GDUT_CCMRAM __attribute__((section(".ccmram")))

namespace gdut {

/**
 * @brief 类型安全的 GPIO 端口枚举
 */
enum class gpio_port : uint8_t { A = 1, B, C, D, E, F, G, H, I };

enum class gpio_mode : uint32_t {
  input = GPIO_MODE_INPUT,
  output = GPIO_MODE_OUTPUT_PP,
  alternate_function_push_pull = GPIO_MODE_AF_PP,
  alternate_function_open_drain = GPIO_MODE_AF_OD,
  analog = GPIO_MODE_ANALOG
};

enum class gpio_pull : uint32_t {
  no_pull = GPIO_NOPULL,
  pull_up = GPIO_PULLUP,
  pull_down = GPIO_PULLDOWN
};

enum class gpio_speed : uint32_t {
  low = GPIO_SPEED_FREQ_LOW,
  medium = GPIO_SPEED_FREQ_MEDIUM,
  high = GPIO_SPEED_FREQ_HIGH,
  very_high = GPIO_SPEED_FREQ_VERY_HIGH
};

enum class gpio_pin : uint32_t {
  pin_0 = GPIO_PIN_0,
  pin_1 = GPIO_PIN_1,
  pin_2 = GPIO_PIN_2,
  pin_3 = GPIO_PIN_3,
  pin_4 = GPIO_PIN_4,
  pin_5 = GPIO_PIN_5,
  pin_6 = GPIO_PIN_6,
  pin_7 = GPIO_PIN_7,
  pin_8 = GPIO_PIN_8,
  pin_9 = GPIO_PIN_9,
  pin_10 = GPIO_PIN_10,
  pin_11 = GPIO_PIN_11,
  pin_12 = GPIO_PIN_12,
  pin_13 = GPIO_PIN_13,
  pin_14 = GPIO_PIN_14,
  pin_15 = GPIO_PIN_15
};

[[nodiscard]] constexpr GPIO_TypeDef *get_gpio_port_ptr(uint32_t port) {
  switch (port) {
  case GPIOA_BASE:
    return GPIOA;
  case GPIOB_BASE:
    return GPIOB;
  case GPIOC_BASE:
    return GPIOC;
  case GPIOD_BASE:
    return GPIOD;
  case GPIOE_BASE:
    return GPIOE;
  case GPIOF_BASE:
    return GPIOF;
  case GPIOG_BASE:
    return GPIOG;
  case GPIOH_BASE:
    return GPIOH;
  case GPIOI_BASE:
    return GPIOI;
  default:
    return nullptr; // 端口非法
  }
}

[[nodiscard]] constexpr GPIO_TypeDef *get_gpio_port_ptr(gpio_port port) {
  switch (port) {
  case gpio_port::A:
    return GPIOA;
  case gpio_port::B:
    return GPIOB;
  case gpio_port::C:
    return GPIOC;
  case gpio_port::D:
    return GPIOD;
  case gpio_port::E:
    return GPIOE;
  case gpio_port::F:
    return GPIOF;
  case gpio_port::G:
    return GPIOG;
  case gpio_port::H:
    return GPIOH;
  case gpio_port::I:
    return GPIOI;
  default:
    return nullptr; // 端口非法
  }
}

enum class timer_id : uint8_t {
  tim1 = 1,
  tim2,
  tim3,
  tim4,
  tim5,
  tim9,
  tim10,
  tim11
};

[[nodiscard]] constexpr TIM_TypeDef *get_timer_ptr(timer_id id) {
  switch (id) {
  case timer_id::tim1:
    return TIM1;
  case timer_id::tim2:
    return TIM2;
  case timer_id::tim3:
    return TIM3;
  case timer_id::tim4:
    return TIM4;
  case timer_id::tim5:
    return TIM5;
  case timer_id::tim9:
    return TIM9;
  case timer_id::tim10:
    return TIM10;
  case timer_id::tim11:
    return TIM11;
  default:
    return nullptr; // 定时器 ID 非法
  }
}

// 获取UART实例索引
[[nodiscard]] constexpr uint8_t get_uart_index(USART_TypeDef *uart_instance) {
  switch (reinterpret_cast<uintptr_t>(uart_instance)) {
  case USART1_BASE:
    return 0;
  case USART2_BASE:
    return 1;
  case USART3_BASE:
    return 2;
  case UART4_BASE:
    return 3;
  case UART5_BASE:
    return 4;
  case USART6_BASE:
    return 5;
  default:
    return 0xFF;
  }
}

enum class dma_stream_type : uint8_t {
  dma1_stream0,
  dma1_stream1,
  dma1_stream2,
  dma1_stream3,
  dma1_stream4,
  dma1_stream5,
  dma1_stream6,
  dma1_stream7,
  dma2_stream0,
  dma2_stream1,
  dma2_stream2,
  dma2_stream3,
  dma2_stream4,
  dma2_stream5,
  dma2_stream6,
  dma2_stream7
};

[[nodiscard]] constexpr DMA_Stream_TypeDef *
get_dma_stream(dma_stream_type type) {
  switch (type) {
  case dma_stream_type::dma1_stream0:
    return DMA1_Stream0;
  case dma_stream_type::dma1_stream1:
    return DMA1_Stream1;
  case dma_stream_type::dma1_stream2:
    return DMA1_Stream2;
  case dma_stream_type::dma1_stream3:
    return DMA1_Stream3;
  case dma_stream_type::dma1_stream4:
    return DMA1_Stream4;
  case dma_stream_type::dma1_stream5:
    return DMA1_Stream5;
  case dma_stream_type::dma1_stream6:
    return DMA1_Stream6;
  case dma_stream_type::dma1_stream7:
    return DMA1_Stream7;
  case dma_stream_type::dma2_stream0:
    return DMA2_Stream0;
  case dma_stream_type::dma2_stream1:
    return DMA2_Stream1;
  case dma_stream_type::dma2_stream2:
    return DMA2_Stream2;
  case dma_stream_type::dma2_stream3:
    return DMA2_Stream3;
  case dma_stream_type::dma2_stream4:
    return DMA2_Stream4;
  case dma_stream_type::dma2_stream5:
    return DMA2_Stream5;
  case dma_stream_type::dma2_stream6:
    return DMA2_Stream6;
  case dma_stream_type::dma2_stream7:
    return DMA2_Stream7;
  default:
    return nullptr; // 非法DMA类型
  }
}

enum class dma_channel : uint32_t {
  channel_0 = DMA_CHANNEL_0,
  channel_1 = DMA_CHANNEL_1,
  channel_2 = DMA_CHANNEL_2,
  channel_3 = DMA_CHANNEL_3,
  channel_4 = DMA_CHANNEL_4,
  channel_5 = DMA_CHANNEL_5,
  channel_6 = DMA_CHANNEL_6,
  channel_7 = DMA_CHANNEL_7
};

enum class dma_direction : uint32_t {
  peripheral_to_memory = DMA_PERIPH_TO_MEMORY,
  memory_to_peripheral = DMA_MEMORY_TO_PERIPH,
  memory_to_memory = DMA_MEMORY_TO_MEMORY
};

enum class dma_peripheral_data_alignment : uint32_t {
  byte = DMA_PDATAALIGN_BYTE,
  half_word = DMA_PDATAALIGN_HALFWORD,
  word = DMA_PDATAALIGN_WORD
};

enum class dma_memory_data_alignment : uint32_t {
  byte = DMA_MDATAALIGN_BYTE,
  half_word = DMA_MDATAALIGN_HALFWORD,
  word = DMA_MDATAALIGN_WORD
};

enum class dma_mode : uint32_t {
  normal = DMA_NORMAL,
  circular = DMA_CIRCULAR,
  peripheral_flow_controller = DMA_PFCTRL
};

enum class dma_priority : uint32_t {
  low = DMA_PRIORITY_LOW,
  medium = DMA_PRIORITY_MEDIUM,
  high = DMA_PRIORITY_HIGH,
  very_high = DMA_PRIORITY_VERY_HIGH
};

enum class dma_fifo_mode : uint32_t {
  disable = DMA_FIFOMODE_DISABLE,
  enable = DMA_FIFOMODE_ENABLE
};

enum class dma_fifo_threshold : uint32_t {
  quarter_full = DMA_FIFO_THRESHOLD_1QUARTERFULL,
  half_full = DMA_FIFO_THRESHOLD_HALFFULL,
  three_quarters_full = DMA_FIFO_THRESHOLD_3QUARTERSFULL,
  full = DMA_FIFO_THRESHOLD_FULL
};

enum class dma_memory_burst : uint32_t {
  single = DMA_MBURST_SINGLE,
  inc4 = DMA_MBURST_INC4,
  inc8 = DMA_MBURST_INC8,
  inc16 = DMA_MBURST_INC16
};

enum class dma_peripheral_burst : uint32_t {
  single = DMA_PBURST_SINGLE,
  inc4 = DMA_PBURST_INC4,
  inc8 = DMA_PBURST_INC8,
  inc16 = DMA_PBURST_INC16
};

enum class spi_mode : uint32_t {
  master = SPI_MODE_MASTER,
  slave = SPI_MODE_SLAVE
};

enum class spi_direction : uint32_t {
  two_lines = SPI_DIRECTION_2LINES,
  two_lines_rx_only = SPI_DIRECTION_2LINES_RXONLY,
  one_line = SPI_DIRECTION_1LINE
};

enum class spi_data_size : uint32_t {
  data_size_8bit = SPI_DATASIZE_8BIT,
  data_size_16bit = SPI_DATASIZE_16BIT
};

enum class spi_clock_polarity : uint32_t {
  low = SPI_POLARITY_LOW,
  high = SPI_POLARITY_HIGH
};

enum class spi_clock_phase : uint32_t {
  first_edge = SPI_PHASE_1EDGE,
  second_edge = SPI_PHASE_2EDGE
};

enum class spi_nss : uint32_t {
  soft = SPI_NSS_SOFT,
  hard_input = SPI_NSS_HARD_INPUT,
  hard_output = SPI_NSS_HARD_OUTPUT
};

enum class spi_baud_rate_prescaler : uint32_t {
  div2 = SPI_BAUDRATEPRESCALER_2,
  div4 = SPI_BAUDRATEPRESCALER_4,
  div8 = SPI_BAUDRATEPRESCALER_8,
  div16 = SPI_BAUDRATEPRESCALER_16,
  div32 = SPI_BAUDRATEPRESCALER_32,
  div64 = SPI_BAUDRATEPRESCALER_64,
  div128 = SPI_BAUDRATEPRESCALER_128,
  div256 = SPI_BAUDRATEPRESCALER_256
};

enum class spi_first_bit : uint32_t {
  msb = SPI_FIRSTBIT_MSB,
  lsb = SPI_FIRSTBIT_LSB
};

enum class spi_ti_mode : uint32_t {
  disable = SPI_TIMODE_DISABLE,
  enable = SPI_TIMODE_ENABLE
};

enum class spi_crc_calculation : uint32_t {
  disable = SPI_CRCCALCULATION_DISABLE,
  enable = SPI_CRCCALCULATION_ENABLE
};

enum class spi_crc_polynomial : uint16_t {
  crc_polynomial_7 = 7,
  crc_polynomial_8 = 8,
  crc_polynomial_16 = 16,
  crc_polynomial_32 = 32
};

template <typename Rep, typename Period>
uint32_t time_to_ticks(const std::chrono::duration<Rep, Period> &timeout) {
  uint32_t ticks;
  if (timeout == std::chrono::duration<Rep, Period>::max()) {
    ticks = osWaitForever;
  } else {
    // 转换为毫秒（亚毫秒精度会被截断）
    auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
    // 处理负数时长（非法状态）
    if (ms < 0) {
      return 0;
    }

    // 处理 0 或正数时长
    if (ms == 0) {
      ticks = 0;
    } else {
      // 将毫秒转换为 tick（tickFreq 单位为 Hz）
      // tick 计算：ticks = (ms * tickFreq) / 1000
      uint32_t tick_freq = osKernelGetTickFreq();
      // 夹紧到 UINT32_MAX-1 以避免溢出（UINT32_MAX 保留给 osWaitForever）
      // 为避免溢出，计算 max_ms：max_ms = (UINT32_MAX - 1) * 1000 / tick_freq
      uint64_t max_ms =
          static_cast<uint64_t>(UINT32_MAX - 1) * 1000ULL / tick_freq;
      if (static_cast<uint64_t>(ms) >= max_ms) {
        ticks = UINT32_MAX - 1;
      } else {
        ticks = static_cast<uint32_t>((static_cast<uint64_t>(ms) * tick_freq) /
                                      1000);
      }
    }
  }
  return ticks;
}

// 针对 std::chrono::milliseconds 的特化重载，避免不必要的转换
inline uint32_t time_to_ticks(std::chrono::milliseconds timeout) {
  if (timeout == std::chrono::milliseconds::max()) {
    return osWaitForever;
  }

  auto ms = timeout.count();
  // 处理负数时长（非法状态）
  if (ms < 0) {
    return 0;
  }

  // 将毫秒转换为 tick（tickFreq 单位为 Hz）
  uint32_t tick_freq = osKernelGetTickFreq();
  // 夹紧到 UINT32_MAX-1 以避免溢出（UINT32_MAX 保留给 osWaitForever）
  uint64_t max_ms = static_cast<uint64_t>(UINT32_MAX - 1) * 1000ULL / tick_freq;
  if (static_cast<uint64_t>(ms) >= max_ms) {
    return UINT32_MAX - 1;
  }
  return static_cast<uint32_t>((static_cast<uint64_t>(ms) * tick_freq) / 1000);
}

} // namespace gdut

#endif // BSP_UTILITY_HPP
