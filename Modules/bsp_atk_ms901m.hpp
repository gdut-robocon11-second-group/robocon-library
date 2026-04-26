#ifndef BSP_ATK_MS901M_HPP
#define BSP_ATK_MS901M_HPP

#include "function.hpp"
#include "memory_resource.hpp"
#include "message_queue.hpp"
#include "stm32f4xx_hal_uart.h"
#include "thread.hpp"
#include "uncopyable.hpp"
#include <algorithm>
#include <array>
#include <span>
#include <vector>

namespace gdut {

// ATK MS901M 六轴传感器寄存器地址
enum class atk_ms901m_reg : std::uint8_t {
  SAVE = 0x00,       // 保存当前设置到 flash
  SENCAL = 0x01,     // 设置传感器校准
  SENSTA = 0x02,     // 读取传感器校准状态
  GYROFSR = 0x03,    // 陀螺仪满量程
  ACCFSR = 0x04,     // 加速度计满量程
  GYROBW = 0x05,     // 陀螺仪带宽
  ACCBW = 0x06,      // 加速度计带宽
  BAUD = 0x07,       // UART 通讯波特率
  RETURNSET = 0x08,  // 主动上报内容
  RETURNSET2 = 0x09, // 主动上报内容 2（保留）
  RETURNRATE = 0x0A, // 主动上报速率
  ALG = 0x0B,        // 算法
  ASM = 0x0C,        // 安装方向
  GAUCAL = 0x0D,     // 陀螺仪自校准开关
  BAUCAL = 0x0E,     // 气压计自校准开关
  LEDOFF = 0x0F,     // LED 开关
  RESET = 0x7F       // 复位
};

enum class atk_ms901m_return_id : std::uint8_t {
  EULER = 0x01,          // 欧拉角
  QUATERNION = 0x02,     // 四元数
  GYRO_AND_ACC = 0x03,   // 陀螺仪和加速度计数据
  MAG_AND_TEMP = 0x04,   // 磁力计和温度计数据
  ATMOS_AND_TEMP = 0x05, // 大气压强和温度数据
  PORT_STATUS = 0x06,    // 端口状态
};

enum class atk_ms901m_gyro_fsr : uint8_t {
  DPS250 = 0x00,
  DPS500 = 0x01,
  DPS1000 = 0x02,
  DPS2000 = 0x03,
};

enum class atk_ms901m_acc_fsr : uint8_t {
  G2 = 0x00,
  G4 = 0x01,
  G8 = 0x02,
  G16 = 0x03,
};

// ATK MS901M 请求帧头
struct atk_ms901m_request_header {
  uint8_t header[2] = {0x55, 0xAF};
  uint8_t id;     // 读 id | 0x80，写 id
  uint8_t length; // 数据长度
  // uint8_t data[length];
  // uint8_t checksum;
} __attribute__((packed));

// ATK MS901M 响应帧头
struct atk_ms901m_response_header {
  uint8_t header[2] = {0x55, 0xAF};
  uint8_t id;
  uint8_t length; // 数据长度
  // uint8_t data[length];
  // uint8_t checksum;
} __attribute__((packed));

struct atk_ms901m_retrunset_data {
  static constexpr std::uint8_t SET_EULER_BIT = 1u << 0;
  static constexpr std::uint8_t SET_QUATERNION_BIT = 1u << 1;
  static constexpr std::uint8_t SET_GYRO_AND_ACC_BIT = 1u << 2;
  static constexpr std::uint8_t SET_MAG_AND_TEMP_BIT = 1u << 3;
  static constexpr std::uint8_t SET_ATMOS_AND_TEMP_BIT = 1u << 4;
  static constexpr std::uint8_t SET_PORT_STATUS_BIT = 1u << 5;
  static constexpr std::uint8_t SET_UPLOAD_DATA_BIT = 1u << 6;
  static constexpr std::uint8_t EMPTY_BIT = 1u << 7;

  std::uint8_t flags = 0;

  void set_euler(bool enabled) {
    if (enabled) {
      flags |= SET_EULER_BIT;
    } else {
      flags &= static_cast<std::uint8_t>(~SET_EULER_BIT);
    }
  }

  [[nodiscard]] bool get_euler() const { return (flags & SET_EULER_BIT) != 0; }

  void set_quaternion(bool enabled) {
    if (enabled) {
      flags |= SET_QUATERNION_BIT;
    } else {
      flags &= static_cast<std::uint8_t>(~SET_QUATERNION_BIT);
    }
  }

  [[nodiscard]] bool get_quaternion() const {
    return (flags & SET_QUATERNION_BIT) != 0;
  }

  void set_gyro_and_acc(bool enabled) {
    if (enabled) {
      flags |= SET_GYRO_AND_ACC_BIT;
    } else {
      flags &= static_cast<std::uint8_t>(~SET_GYRO_AND_ACC_BIT);
    }
  }

  [[nodiscard]] bool get_gyro_and_acc() const {
    return (flags & SET_GYRO_AND_ACC_BIT) != 0;
  }

  void set_mag_and_temp(bool enabled) {
    if (enabled) {
      flags |= SET_MAG_AND_TEMP_BIT;
    } else {
      flags &= static_cast<std::uint8_t>(~SET_MAG_AND_TEMP_BIT);
    }
  }

  [[nodiscard]] bool get_mag_and_temp() const {
    return (flags & SET_MAG_AND_TEMP_BIT) != 0;
  }

  void set_atmos_and_temp(bool enabled) {
    if (enabled) {
      flags |= SET_ATMOS_AND_TEMP_BIT;
    } else {
      flags &= static_cast<std::uint8_t>(~SET_ATMOS_AND_TEMP_BIT);
    }
  }

  [[nodiscard]] bool get_atmos_and_temp() const {
    return (flags & SET_ATMOS_AND_TEMP_BIT) != 0;
  }

  void set_port_status(bool enabled) {
    if (enabled) {
      flags |= SET_PORT_STATUS_BIT;
    } else {
      flags &= static_cast<std::uint8_t>(~SET_PORT_STATUS_BIT);
    }
  }

  [[nodiscard]] bool get_port_status() const {
    return (flags & SET_PORT_STATUS_BIT) != 0;
  }

  void set_upload_data(bool enabled) {
    if (enabled) {
      flags |= SET_UPLOAD_DATA_BIT;
    } else {
      flags &= static_cast<std::uint8_t>(~SET_UPLOAD_DATA_BIT);
    }
  }

  [[nodiscard]] bool get_upload_data() const {
    return (flags & SET_UPLOAD_DATA_BIT) != 0;
  }

  void set_empty(bool enabled) {
    if (enabled) {
      flags |= EMPTY_BIT;
    } else {
      flags &= static_cast<std::uint8_t>(~EMPTY_BIT);
    }
  }

  [[nodiscard]] bool get_empty() const { return (flags & EMPTY_BIT) != 0; }
} __attribute__((packed));

class atk_ms901m : private uncopyable {
public:
  atk_ms901m() = default;
  ~atk_ms901m() = default;

  atk_ms901m(atk_ms901m &&other) noexcept = delete;
  atk_ms901m &operator=(atk_ms901m &&other) noexcept = delete;
  void set_uart(UART_HandleTypeDef *uart) { m_uart = uart; }
  void
  set_send_func(function<void(const uint8_t *data, uint16_t size)> send_func) {
    m_send_func = std::move(send_func);
  }
  void set_euler_callback(
      function<void(float roll, float pitch, float yaw)> callback) {
    m_callbacks.euler_callback = std::move(callback);
  }
  void set_quaternion_callback(
      function<void(float q0, float q1, float q2, float q3)> callback) {
    m_callbacks.quaternion_callback = std::move(callback);
  }
  void set_gyro_and_acc_callback(
      function<void(float gyro_x, float gyro_y, float gyro_z, float acc_x,
                    float acc_y, float acc_z)>
          callback) {
    m_callbacks.gyro_and_acc_callback = std::move(callback);
  }

  void handle_uart_rx(uint16_t size) {
    if (!m_message_queue) {
      if (m_uart) {
        HAL_UARTEx_ReceiveToIdle_DMA(m_uart, m_rx_buffer.data(),
                                     m_rx_buffer.size());
      }
      return;
    }
    message_buffer buf;
    auto iter = m_rx_buffer.begin();
    while (size > 0) {
      uint16_t chunk_size =
          std::min(size, static_cast<uint16_t>(sizeof(buf.data)));
      buf.size = static_cast<uint8_t>(chunk_size);
      std::copy(iter, iter + chunk_size, buf.data);
      if (!m_message_queue.send_from_isr(buf)) {
        // 发送失败，丢弃数据
        break;
      }
      iter += chunk_size;
      size -= chunk_size;
    }
    if (m_uart) {
      HAL_UARTEx_ReceiveToIdle_DMA(m_uart, m_rx_buffer.data(),
                                   m_rx_buffer.size());
    }
  }

  void handle_error_rx() {
    if (m_uart) {
      HAL_UARTEx_ReceiveToIdle_DMA(m_uart, m_rx_buffer.data(),
                                   m_rx_buffer.size());
    }
  }

  void start() {
    if (m_processing_thread.joinable()) {
      return;
    }
    if (!m_uart) {
      return;
    }
    m_message_queue = message_queue<message_buffer>(10);
    std::pmr::vector<std::uint8_t>(pmr::portable_resource::get_instance())
        .swap(m_message_buffer);
    m_processing_thread =
        thread<2048, osPriorityHigh>("atk_ms901m", [this]() {
          while (true) {
            message_buffer buf;
            if (m_message_queue.receive(buf)) {
              m_message_buffer.insert(m_message_buffer.end(), buf.data,
                                      buf.data + buf.size);
              process_message();
            }
          }
        });
    HAL_UARTEx_ReceiveToIdle_DMA(m_uart, m_rx_buffer.data(),
                                 m_rx_buffer.size());

    {
      // 复位
      uint8_t empty{0};
      send_frame<1, false, atk_ms901m_reg::RESET>(
          std::span<const uint8_t, 1>{&empty, &empty + 1});
    }

    // 设置主动上报内容为欧拉角和陀螺仪加速度数据
    {
      atk_ms901m_retrunset_data retrunset_data;
      retrunset_data.set_euler(true);
      retrunset_data.set_quaternion(false);
      retrunset_data.set_gyro_and_acc(true);
      retrunset_data.set_mag_and_temp(false);
      retrunset_data.set_atmos_and_temp(false);
      retrunset_data.set_port_status(false);
      retrunset_data.set_upload_data(false);
      send_frame<sizeof(retrunset_data), false, atk_ms901m_reg::RETURNSET>(
          std::span<const uint8_t, sizeof(retrunset_data)>{
              reinterpret_cast<const uint8_t *>(&retrunset_data),
              reinterpret_cast<const uint8_t *>(&retrunset_data) +
                  sizeof(retrunset_data)});
    }

    // 设置陀螺仪满量程为 2000 dps
    send_frame<1, false, atk_ms901m_reg::GYROFSR>(std::span<const uint8_t, 1>{
        reinterpret_cast<const uint8_t *>(&m_gyro_fsr),
        reinterpret_cast<const uint8_t *>(&m_gyro_fsr) + 1});

    // 设置加速度计满量程为 4 g
    send_frame<1, false, atk_ms901m_reg::ACCFSR>(std::span<const uint8_t, 1>{
        reinterpret_cast<const uint8_t *>(&m_acc_fsr),
        reinterpret_cast<const uint8_t *>(&m_acc_fsr) + 1});

    // 使用九轴融合算法
    {
      std::uint8_t alg = 0x01; // 0x00: 六轴融合，0x01: 九轴融合
      send_frame<1, false, atk_ms901m_reg::ALG>(std::span<const uint8_t, 1>{
          reinterpret_cast<const uint8_t *>(&alg),
          reinterpret_cast<const uint8_t *>(&alg) + 1});
    }
  }

  std::size_t get_gyro_fsr() const {
    switch (m_gyro_fsr) {
    case atk_ms901m_gyro_fsr::DPS250:
      return 250;
    case atk_ms901m_gyro_fsr::DPS500:
      return 500;
    case atk_ms901m_gyro_fsr::DPS1000:
      return 1000;
    case atk_ms901m_gyro_fsr::DPS2000:
      return 2000;
    }
    return 0;
  }

  std::size_t get_acc_fsr() const {
    switch (m_acc_fsr) {
    case atk_ms901m_acc_fsr::G2:
      return 2;
    case atk_ms901m_acc_fsr::G4:
      return 4;
    case atk_ms901m_acc_fsr::G8:
      return 8;
    case atk_ms901m_acc_fsr::G16:
      return 16;
    }
    return 0;
  }

protected:
  struct message_buffer {
    uint8_t size;
    uint8_t data[32];
  };

  struct callback_functions_t {
    function<void(float roll, float pitch, float yaw)> euler_callback;
    function<void(float w, float x, float y, float z)> quaternion_callback;
    function<void(float gx, float gy, float gz, float ax, float ay, float az)>
        gyro_and_acc_callback;
  };

  void process_message() {
    while (m_message_buffer.size() >= sizeof(atk_ms901m_response_header)) {
      atk_ms901m_response_header response_header;
      std::copy(m_message_buffer.begin(),
                m_message_buffer.begin() + sizeof(response_header),
                reinterpret_cast<uint8_t *>(&response_header));
      atk_ms901m_response_header *header = &response_header;
      if (header->header[0] != 0x55 ||
          (header->header[1] != 0xAF && header->header[1] != 0x55)) {
        // 无效帧头，丢弃第一个字节
        m_message_buffer.erase(m_message_buffer.begin());
        continue;
      }
      const size_t frame_size =
          sizeof(atk_ms901m_response_header) + header->length + 1;
      if (m_message_buffer.size() < frame_size) {
        // 不完整帧，等待更多数据
        break;
      }
      uint8_t checksum = 0;
      for (size_t i = 0; i < frame_size - 1; ++i) {
        checksum += m_message_buffer[i];
      }
      if (checksum != m_message_buffer[frame_size - 1]) {
        // 校验失败，丢弃这个帧
        m_message_buffer.erase(m_message_buffer.begin(),
                               m_message_buffer.begin() + frame_size);
        continue;
      }
      // 处理有效帧
      handle_frame(header->header[1] == 0x55, header->id,
                   m_message_buffer.data() + sizeof(atk_ms901m_response_header),
                   header->length);
      m_message_buffer.erase(m_message_buffer.begin(),
                             m_message_buffer.begin() + frame_size);
      osDelay(5); // 适当延时，让出处理器给其他任务，避免长时间占用 CPU
    }
  }

  template <std::size_t Length, bool Read, atk_ms901m_reg Reg>
  void send_frame(std::span<const uint8_t, Length> data) {
    static_assert(Length <= 255, "Data length exceeds maximum allowed size");
    if (!m_send_func) {
      return;
    }
    atk_ms901m_request_header header{
        .id = std::to_underlying(Reg),
        .length = Length,
    };
    if constexpr (Read) {
      header.id |= 0x80;
    }
    std::array<uint8_t, sizeof(header) + Length + 1> frame{};
    std::copy(reinterpret_cast<const uint8_t *>(&header),
              reinterpret_cast<const uint8_t *>(&header) + sizeof(header),
              frame.data());
    std::copy(data.begin(), data.end(), frame.data() + sizeof(header));
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(header) + Length; ++i) {
      checksum += frame[i];
    }
    frame[sizeof(header) + Length] = checksum;
    m_send_func(frame.data(), sizeof(header) + Length + 1);
  }

  void handle_frame(bool is_ret, uint8_t id, const uint8_t *data,
                    uint8_t length) {
    // 这里可以根据 id 和 data 解析具体的传感器数据或响应
    if (is_ret) {
      switch (static_cast<atk_ms901m_return_id>(id)) {
      case atk_ms901m_return_id::EULER:
        parse_euler(data, length);
        break;
      case atk_ms901m_return_id::QUATERNION:
        parse_quaternion(data, length);
        break;
      case atk_ms901m_return_id::GYRO_AND_ACC:
        parse_gyro_and_acc(data, length);
        break;
      default:
        break;
      };
    } else {
      // 响应帧
      switch (static_cast<atk_ms901m_reg>(id & 0x7F)) {
      default:
        break;
      }
    }
  }

  void parse_euler(const uint8_t *data, uint8_t length) {
    // 解析欧拉角数据
    if (!m_callbacks.euler_callback) {
      return;
    }
    if (length < 6) {
      // 数据长度不足，丢弃
      return;
    }
    float roll =
        (static_cast<int16_t>(data[1] << 8) | data[0]) / 32768.0f * 180.0f;
    float pitch =
        (static_cast<int16_t>(data[3] << 8) | data[2]) / 32768.0f * 180.0f;
    float yaw =
        (static_cast<int16_t>(data[5] << 8) | data[4]) / 32768.0f * 180.0f;
    m_callbacks.euler_callback(roll, pitch, yaw);
  }

  void parse_quaternion(const uint8_t *data, uint8_t length) {
    // 解析四元数数据
    if (!m_callbacks.quaternion_callback) {
      return;
    }
    if (length < 8) {
      // 数据长度不足，丢弃
      return;
    }
    float w = (static_cast<int16_t>(data[1] << 8) | data[0]) / 32768.0f;
    float x = (static_cast<int16_t>(data[3] << 8) | data[2]) / 32768.0f;
    float y = (static_cast<int16_t>(data[5] << 8) | data[4]) / 32768.0f;
    float z = (static_cast<int16_t>(data[7] << 8) | data[6]) / 32768.0f;
    m_callbacks.quaternion_callback(w, x, y, z);
  }

  void parse_gyro_and_acc(const uint8_t *data, uint8_t length) {
    // 解析陀螺仪和加速度计数据
    if (!m_callbacks.gyro_and_acc_callback) {
      return;
    }
    if (length < 12) {
      // 数据长度不足，丢弃
      return;
    }
    float accx = (static_cast<int16_t>(data[1] << 8) | data[0]) / 32768.0f *
                 get_acc_fsr();
    float accy = (static_cast<int16_t>(data[3] << 8) | data[2]) / 32768.0f *
                 get_acc_fsr();
    float accz = (static_cast<int16_t>(data[5] << 8) | data[4]) / 32768.0f *
                 get_acc_fsr();
    float gyrox = (static_cast<int16_t>(data[7] << 8) | data[6]) / 32768.0f *
                  get_gyro_fsr();
    float gyroy = (static_cast<int16_t>(data[9] << 8) | data[8]) / 32768.0f *
                  get_gyro_fsr();
    float gyroz = (static_cast<int16_t>(data[11] << 8) | data[10]) / 32768.0f *
                  get_gyro_fsr();
    m_callbacks.gyro_and_acc_callback(gyrox, gyroy, gyroz, accx, accy, accz);
  }

private:
  UART_HandleTypeDef *m_uart{nullptr};

  atk_ms901m_gyro_fsr m_gyro_fsr{atk_ms901m_gyro_fsr::DPS500};
  atk_ms901m_acc_fsr m_acc_fsr{atk_ms901m_acc_fsr::G4};

  function<void(const uint8_t *data, uint16_t size)> m_send_func;
  callback_functions_t m_callbacks;

  std::array<std::uint8_t, 512> m_rx_buffer{};
  std::pmr::vector<std::uint8_t> m_message_buffer;
  message_queue<message_buffer> m_message_queue{empty_message_queue};
  thread<2048, osPriorityHigh> m_processing_thread{empty_thread};
};

} // namespace gdut

#endif // BSP_ATK_MS901M_HPP
