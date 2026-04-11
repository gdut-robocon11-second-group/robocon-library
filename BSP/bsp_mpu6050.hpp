#ifndef BSP_MPU6050_HPP
#define BSP_MPU6050_HPP

#include "bsp_iic.hpp"
#include "clock.hpp"
#include "function.hpp"
#include "stm32f4xx_hal.h"
#include "uncopyable.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <utility>


namespace gdut {

/*
MPU6050 I2C 地址
MPU6050 的I2C从机地址由AD0引脚决定：
AD0连接到GND时，地址为0x68
AD0连接到VCC时，地址为0x69
*/
enum class mpu6050_addr : uint8_t {
  low = 0x68, // AD0 接 GND (地面)，I2C地址为 0x68
  high = 0x69 // AD0 接 VCC (电源)，I2C地址为 0x69
};

/*
 * MPU6050 寄存器地址定义
 */
namespace mpu6050_reg {
// 自检相关寄存器
constexpr uint8_t SELF_TEST_X =
    0x0D; // 自检控制 X，用于验证加速度计和陀螺仪X轴自检功能
constexpr uint8_t SELF_TEST_Y =
    0x0E; // 自检控制 Y，用于验证加速度计和陀螺仪Y轴自检功能
constexpr uint8_t SELF_TEST_Z =
    0x0F; // 自检控制 Z，用于验证加速度计和陀螺仪Z轴自检功能
constexpr uint8_t SELF_TEST_A = 0x10; // 自检控制 A，用于验证陀螺仪自检功能

// 采样率和信号处理配置
constexpr uint8_t SMPLRT_DIV =
    0x19; // 采样率分频器，采样率=1000Hz/(1+SMPLRT_DIV)，范围0-255，默认0(1000Hz)
constexpr uint8_t CONFIG =
    0x1A; // 配置寄存器，配置DLPF(数字低通滤波器)带宽，范围0-7
constexpr uint8_t GYRO_CONFIG =
    0x1B; // 陀螺仪配置，设置陀螺仪满量程范围：±250/500/1000/2000°/s，自检使能
constexpr uint8_t ACCEL_CONFIG =
    0x1C; // 加速度计配置，设置加速度计满量程范围：±2/4/8/16g，自检使能

// 低功耗和运动检测
constexpr uint8_t LP_ACCEL_ODR =
    0x1E; // 低功耗加速度计输出数据速率，配置低功耗模式下的采样速率
constexpr uint8_t WOM_THR =
    0x1F; // 唤醒动作(Wake-On-Motion)门限，用于运动检测，范围0-255(1mg/LSB)

// FIFO 缓存控制
constexpr uint8_t FIFO_EN =
    0x23; // FIFO使能控制，选择哪些数据写入FIFO(温度、陀螺仪、加速度计等)

// I2C 主模式和从模式配置(用于外接磁力计等)
constexpr uint8_t I2C_MST_CTRL =
    0x24; // I2C主控制，配置I2C主模式总线速率和从机数量
constexpr uint8_t I2C_SLV0_ADDR = 0x25; // I2C从0地址，配置从设备0的I2C地址
constexpr uint8_t I2C_SLV0_REG =
    0x26; // I2C从0寄存器，配置从设备0要读取的寄存器地址
constexpr uint8_t I2C_SLV0_CTRL =
    0x27; // I2C从0控制，配置从设备0的数据长度和字节交换方式
constexpr uint8_t I2C_SLV1_ADDR = 0x28; // I2C从1地址，配置从设备1的I2C地址
constexpr uint8_t I2C_SLV1_REG =
    0x29; // I2C从1寄存器，配置从设备1要读取的寄存器地址
constexpr uint8_t I2C_SLV1_CTRL =
    0x2A; // I2C从1控制，配置从设备1的数据长度和字节交换方式

// 中断配置
constexpr uint8_t INT_PIN_CFG =
    0x37; // 中断引脚配置，配置INT引脚的电平极性、推挽/开漏、缩脉模式等
constexpr uint8_t INT_ENABLE =
    0x38; // 中断使能，选择并使能哪些中断类型(数据就绪、FIFO溢出、运动检测等)
constexpr uint8_t INT_STATUS =
    0x3A; // 中断状态，读取当前的中断标志(读取后自动清除对应标志)

// 加速度计数据输出(6字节：X_HIGH, X_LOW, Y_HIGH, Y_LOW, Z_HIGH, Z_LOW)
constexpr uint8_t ACCEL_XOUT_H =
    0x3B; // 加速度计X轴高字节(MSB)，16位有符号整数的高8位
constexpr uint8_t ACCEL_XOUT_L =
    0x3C; // 加速度计X轴低字节(LSB)，16位有符号整数的低8位
constexpr uint8_t ACCEL_YOUT_H =
    0x3D; // 加速度计Y轴高字节(MSB)，16位有符号整数的高8位
constexpr uint8_t ACCEL_YOUT_L =
    0x3E; // 加速度计Y轴低字节(LSB)，16位有符号整数的低8位
constexpr uint8_t ACCEL_ZOUT_H =
    0x3F; // 加速度计Z轴高字节(MSB)，16位有符号整数的高8位
constexpr uint8_t ACCEL_ZOUT_L =
    0x40; // 加速度计Z轴低字节(LSB)，16位有符号整数的低8位

// 温度数据输出(2字节：TEMP_HIGH, TEMP_LOW)
constexpr uint8_t TEMP_OUT_H =
    0x41; // 温度输出高字节(MSB)，16位有符号整数的高8位，温度=raw/340+36.53℃
constexpr uint8_t TEMP_OUT_L =
    0x42; // 温度输出低字节(LSB)，16位有符号整数的低8位

// 陀螺仪数据输出(6字节：X_HIGH, X_LOW, Y_HIGH, Y_LOW, Z_HIGH, Z_LOW)
constexpr uint8_t GYRO_XOUT_H =
    0x43; // 陀螺仪X轴高字节(MSB)，16位有符号整数的高8位
constexpr uint8_t GYRO_XOUT_L =
    0x44; // 陀螺仪X轴低字节(LSB)，16位有符号整数的低8位
constexpr uint8_t GYRO_YOUT_H =
    0x45; // 陀螺仪Y轴高字节(MSB)，16位有符号整数的高8位
constexpr uint8_t GYRO_YOUT_L =
    0x46; // 陀螺仪Y轴低字节(LSB)，16位有符号整数的低8位
constexpr uint8_t GYRO_ZOUT_H =
    0x47; // 陀螺仪Z轴高字节(MSB)，16位有符号整数的高8位
constexpr uint8_t GYRO_ZOUT_L =
    0x48; // 陀螺仪Z轴低字节(LSB)，16位有符号整数的低8位

// 复位和其他控制
constexpr uint8_t SIGNAL_PATH_RESET =
    0x68; // 信号路径复位，重置陀螺仪、加速度计和温度传感器的模拟和数字信号通路
constexpr uint8_t ACCEL_INTEL_CTRL =
    0x69; // 加速度计智能控制，配置WOM(唤醒动作)智能输出功能和低功耗加速度计模式

// 用户控制
constexpr uint8_t USER_CTRL =
    0x6A; // 用户控制，使能I2C主模式、重置FIFO、复位I2C主接口、使能DMP(动作处理单元)等

// 电源管理
constexpr uint8_t PWR_MGMT_1 =
    0x6B; // 电源管理1，设置时钟源、复位、睡眠模式控制等(bit7:DEVICE_RESET,
          // bit6:SLEEP, bits2-0:CLKSEL)
constexpr uint8_t PWR_MGMT_2 =
    0x6C; // 电源管理2，控制各传感器的工作/停用状态(bit5:STBY_XA,
          // bit4:STBY_YA等)

// FIFO 缓存大小和数据
constexpr uint8_t FIFO_COUNTH =
    0x72; // FIFO计数器高字节，表示FIFO中字节数的高8位(14位计数器，范围0-8191)
constexpr uint8_t FIFO_COUNTL =
    0x73; // FIFO计数器低字节，表示FIFO中字节数的低8位
constexpr uint8_t FIFO_R_W =
    0x74; // FIFO读写数据，读写FIFO缓存中的数据，允许突发访问

// 设备识别
constexpr uint8_t WHO_AM_I =
    0x75; // 设备ID识别寄存器(只读)，默认值为0x68或0x69(取决于AD0引脚状态)
} // namespace mpu6050_reg

/**
 * @brief 加速度计量程选择
 * - ±2g:   16384 LSB/g (精度最高)
 * - ±4g:   8192 LSB/g
 * - ±8g:   4096 LSB/g
 * - ±16g:  2048 LSB/g (范围最大)
 */
enum class accel_range : uint8_t {
  range_2g = 0x00, // ±2g，量程±2g
  range_4g = 0x08, // ±4g，量程±4g
  range_8g = 0x10, // ±8g，量程±8g
  range_16g = 0x18 // ±16g，量程±16g
};

/**
 * 陀螺仪量程选择
 * - ±250°/s:  131 LSB/°/s (最高精度)
 * - ±500°/s:  65.5 LSB/°/s
 * - ±1000°/s: 32.8 LSB/°/s
 * - ±2000°/s: 16.4 LSB/°/s (最大范围)
 */
enum class gyro_range : uint8_t {
  range_250dps = 0x00,  // ±250°/s，
  range_500dps = 0x08,  // ±500°/s，
  range_1000dps = 0x10, // ±1000°/s，
  range_2000dps = 0x18  // ±2000°/s，
};

/**
 * @brief 数字低通滤波器(DLPF)带宽
 *
 * DLPF用于抑制高频噪声。更低的带宽有更强的滤波效果但延迟更大。
 * 采样率必须至少是滤波器带宽的2倍(Nyquist采样定理)。
 *
 * 使用建议：
 * - 260Hz: 最小滤波，适合高动态应用(避免延迟)
 * - 94Hz:  中等滤波，平衡性能和噪声(推荐通用设置)
 * - 44Hz:  较强滤波，适合导航和定位
 * - 5Hz:   极强滤波，适合低频测量和数据稳定化
 */
enum class dlpf_cfg : uint8_t {
  bandwidth_260hz = 0x00, // 260Hz带宽，最小滤波，适合高速动态应用，延迟0ms
  bandwidth_184hz = 0x01, // 184Hz带宽，轻度滤波，适合中等应用
  bandwidth_94hz =
      0x02, // 94Hz带宽，中等滤波，推荐通用选择，平衡滤波效果和延迟(2ms)
  bandwidth_44hz = 0x03, // 44Hz带宽，较强滤波，适合机器人导航，延迟4.9ms
  bandwidth_21hz = 0x04, // 21Hz带宽，强滤波，适合低速应用
  bandwidth_10hz = 0x05, // 10Hz带宽，更强滤波，用于精确姿态估计
  bandwidth_5hz =
      0x06, // 5Hz带宽，极强滤波，数据平稳但延迟大(13.4ms)，用于静态测量
  bandwidth_reserved = 0x07 // 保留值，不应使用
};

/**
 * @brief 时钟源选择
 *
 * 选择MPU6050用于内部时钟参考的时钟源。陀螺仪PLL可提供更稳定的时钟。
 *
 * - pll_z_gyro: 最常用，使用Z轴陀螺仪作为PLL参考，精度高且成本低
 * - internal_8mhz: 调试和初期启动时使用，精度相对低
 * - 外部时钟: 需要外部时钟输入到CLK_IN引脚
 */
enum class clk_src : uint8_t {
  internal_8mhz = 0x00, // 内部RC振荡器，8MHz，精度一般，用于初始化阶段
  pll_x_gyro = 0x01,    // PLL参考X轴陀螺仪，精度高(±5%)
  pll_y_gyro = 0x02,    // PLL参考Y轴陀螺仪，精度高(±5%)
  pll_z_gyro = 0x03,    // PLL参考Z轴陀螺仪，精度高(±5%)★推荐首选，应用最广泛
  pll_external_32khz = 0x04,   // PLL参考外部32kHz时钟，需要外部晶振驱动
  pll_external_19_2mhz = 0x05, // PLL参考外部19.2MHz时钟，需要外部晶振驱动
  reserved = 0x06,             // 保留值，不应使用
  stop = 0x07 // 停止时钟，进入超低功耗模式，不应在正常工作时使用
};

/**
 * @brief MPU6050 3轴数据结构
 */
struct mpu6050_data {
  int16_t x;
  int16_t y;
  int16_t z;
};

class mpu6050 : private uncopyable {
public:
  // c_obj I2C 驱动对象  addr:mpu的地址
  explicit mpu6050(i2c *i2c_obj, gdut::function<void(std::uint32_t)> delay_func,
                   mpu6050_addr addr = mpu6050_addr::low)
      : m_i2c(i2c_obj), m_i2c_addr(static_cast<uint8_t>(addr) << 1),
        m_delay_func(std::move(delay_func)) {}

  ~mpu6050() = default;

  /**
   * @brief 初始化 MPU6050
   * @return true 初始化成功
   * @return false 初始化失败（设备未响应或 WHO_AM_I 不符）
   */
  bool init() {
    if (!m_i2c) {
      return false;
    }

    // 检查设备
    uint8_t who_am_i;
    if (!read_register(mpu6050_reg::WHO_AM_I, &who_am_i, 1)) {
      return false;
    }
    if (who_am_i != 0x68 && who_am_i != 0x69) {
      return false;
    }

    // 复位设备
    reset();

    // 等待复位完成
    uint8_t pwr_mgmt;
    for (int i = 0; i < 10; ++i) {
      if (!read_register(mpu6050_reg::PWR_MGMT_1, &pwr_mgmt, 1)) {
        return false;
      }
      if ((pwr_mgmt & 0x80) == 0) { // DEVICE_RESET 清除
        break;
      }
      if (m_delay_func) {
        std::invoke(m_delay_func, 10);
      }
    }
    if ((pwr_mgmt & 0x80) != 0) { // 复位未完成
      return false;
    }

    // 设置时钟源为 PLL (Z轴陀螺仪) 并禁用睡眠模式
    // PWR_MGMT_1: bit6=0(禁用睡眠), bits2-0=011(Z轴陀螺仪PLL)
    if (!write_register(mpu6050_reg::PWR_MGMT_1, 0x03)) {
      return false;
    }

    // 启用所有传感器
    if (!write_register(mpu6050_reg::PWR_MGMT_2, 0x00)) {
      return false;
    }

    // 设置采样率
    set_sample_rate(100); // 默认 100Hz

    // 设置 DLPF
    set_dlpf(dlpf_cfg::bandwidth_44hz);

    // 设置加速度计和陀螺仪量程
    set_accel_range(accel_range::range_16g);
    set_gyro_range(gyro_range::range_2000dps);

    // 清除中断状态
    uint8_t int_status;
    read_register(mpu6050_reg::INT_STATUS, &int_status, 1);

    return true;
  }

  /**
   * @brief 复位 MPU6050
   *
   * 执行软件复位，将所有寄存器重置为默认值。
   */
  void reset() {
    write_register(mpu6050_reg::PWR_MGMT_1, 0x80); // 设置 DEVICE_RESET 位
    if (m_delay_func) {
      std::invoke(m_delay_func, 100);
    }
  }

  /*
   * @brief 读取加速度计数据
   *
   * @return 包含 X、Y、Z 轴原始值的结构体
   */
  mpu6050_data get_accel() {
    std::array<uint8_t, 6> buffer;
    if (!read_register(mpu6050_reg::ACCEL_XOUT_H, buffer.data(), 6)) {
      return {0, 0, 0};
    }

    mpu6050_data data;
    data.x = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
    data.y = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
    data.z = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);
    return data;
  }

  // 陀螺仪数据
  mpu6050_data get_gyro() {
    std::array<uint8_t, 6> buffer;
    if (!read_register(mpu6050_reg::GYRO_XOUT_H, buffer.data(), 6)) {
      return {0, 0, 0};
    }

    mpu6050_data data;
    data.x = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
    data.y = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
    data.z = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);
    return data;
  }

  // return 温度值，单位为摄氏度
  // 度 = (原始值 / 340.0) + 36.53
  float get_temperature() {
    std::array<uint8_t, 2> buffer;
    if (!read_register(mpu6050_reg::TEMP_OUT_H, buffer.data(), 2)) {
      return 0.0f;
    }

    int16_t raw = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
    return (raw / 340.0f) + 36.53f;
  }

  // 次性读取加速度计、陀螺仪和温度数据
  struct imu_data {
    mpu6050_data accel; // 加速度原始数据
    mpu6050_data gyro;  // 陀螺仪原始数据
    float temperature;  // 温度数据
    float roll;         // 翻滚角(Roll) 单位:弧度
    float pitch;        // 俯仰角(Pitch) 单位:弧度
    float yaw;          // 偏航角(Yaw) 单位:弧度
    float accel_x;      // 真实加速度X 单位:m/s²
    float accel_y;      // 真实加速度Y 单位:m/s²
    float accel_z;      // 真实加速度Z 单位:m/s²
  };

  imu_data read_imu_data() {
    std::array<uint8_t, 14> buffer;
    if (!read_register(mpu6050_reg::ACCEL_XOUT_H, buffer.data(), 14)) {
      return {{0, 0, 0}, {0, 0, 0}, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    }

    imu_data data;
    // 加速度计
    data.accel.x = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
    data.accel.y = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
    data.accel.z = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);
    // 温度
    int16_t temp_raw = static_cast<int16_t>((buffer[6] << 8) | buffer[7]);
    data.temperature = (temp_raw / 340.0f) + 36.53f;
    // 陀螺仪
    data.gyro.x = static_cast<int16_t>((buffer[8] << 8) | buffer[9]);
    data.gyro.y = static_cast<int16_t>((buffer[10] << 8) | buffer[11]);
    data.gyro.z = static_cast<int16_t>((buffer[12] << 8) | buffer[13]);

    // 转换为物理单位 (使用动态量程转换因子)
    float accel_lsb = get_accel_lsb_per_g();
    float ax_g = data.accel.x / accel_lsb;
    float ay_g = data.accel.y / accel_lsb;
    float az_g = data.accel.z / accel_lsb;

    // 计算欧拉角
    data.roll = std::atan2(ay_g, az_g);
    data.pitch = std::atan2(-ax_g, std::sqrt(ay_g * ay_g + az_g * az_g));
    data.yaw = m_prev_yaw;

    // 计算真实加速度 (去掉重力): a_real = a_accel - a_gravity
    const float g = 9.81f; // 重力加速度
    float gravity_x = g * std::sin(data.pitch);
    float gravity_y = g * std::sin(data.roll) * std::cos(data.pitch);
    float gravity_z = g * std::cos(data.roll) * std::cos(data.pitch);

    data.accel_x = (ax_g * g) - gravity_x;
    data.accel_y = (ay_g * g) - gravity_y;
    data.accel_z = (az_g * g) - gravity_z;

    return data;
  }

  struct euler_angles {
    float roll;  // 翻滚角(Roll) 单位:弧度
    float pitch; // 俯仰角(Pitch) 单位:弧度
    float yaw;   // 偏航角(Yaw) 单位:弧度
  };

  /**
   * @brief 获取通过互补滤波优化的欧拉角（自动计算时间差）
   *
   * 互补滤波器原理：
   * - 陀螺仪：短期精度高，但长期会漂移
   * - 加速度计：长期精度高，但受运动加速度影响
   * - 互补滤波：结合两者优点，过滤噪声同时避免漂移
   *
   * 公式：filtered_angle = alpha * (prev_angle + gyro_rate * dt) + (1-alpha) *
   * accel_angle
   *
   * @return 返回经过互补滤波的roll、pitch角度（单位：弧度）
   *
   * @note 第一次调用会初始化滤波器状态，建议以固定周期调用此函数
   * @note 本版本自动计算两次调用间的时间差，如需手动控制时间差，使用
   * get_filtered_euler_angles(float)
   */
  euler_angles get_filtered_euler_angles() {
    // 计算时间差 dt (秒)
    auto now = gdut::steady_clock::now();
    float dt = 0.01f; // 默认10ms，实则if语句会根据实际时间差调整
    if (m_last_filter_time.time_since_epoch().count() > 0) {
      auto duration = now - m_last_filter_time;
      dt = std::chrono::duration<float>(duration).count();
      if (dt < 0.001f)
        dt = 0.001f; // 最小1ms
      if (dt > 0.1f)
        dt = 0.1f; // 最大100ms
    }
    m_last_filter_time = now;

    // 调用带时间参数的版本
    return get_filtered_euler_angles(dt);
  }

  /**
   *获取通过互补滤波优化的欧拉角（明确指定时间差）
   *
   * 互补滤波器原理：
   * - 陀螺仪：短期精度高，但长期会漂移
   * - 加速度计：长期精度高，但受运动加速度影响
   * - 互补滤波：结合两者优点，过滤噪声同时避免漂移
   *
   * 公式：filtered_angle = alpha * (prev_angle + gyro_rate * dt) + (1-alpha) *
   * accel_angle
   *
   * @param dt 两次调用间的时间差（秒），范围 [0.001, 0.1]（会自动裁剪）
   * @return 返回经过互补滤波的roll、pitch角度（单位：弧度）
   *
   * @note ，适合以下场景：
   *       - 已知固定的采样周期（如10ms）
   *       - 某些事件驱动的更新
   *
   * 使用示例：
   * @code
   * // 100Hz固定采样率（10ms周期）
   * auto angles = imu.get_filtered_euler_angles(0.01f);
   *
   * // 50Hz采样率（20ms周期）
   * auto angles = imu.get_filtered_euler_angles(0.02f);
   * @endcode
   */
  euler_angles get_filtered_euler_angles(float dt) {
    // 裁剪时间差到合理范围
    if (dt < 0.001f)
      dt = 0.001f; // 最小1ms
    if (dt > 0.1f)
      dt = 0.1f; // 最大100ms

    // 获取当前加速度计数据计算参考角度
    auto accel_data = get_accel();
    float accel_lsb = get_accel_lsb_per_g();
    float ax_g = accel_data.x / accel_lsb;
    float ay_g = accel_data.y / accel_lsb;
    float az_g = accel_data.z / accel_lsb;

    euler_angles accel_angles;
    accel_angles.roll = std::atan2(ay_g, az_g);
    accel_angles.pitch =
        std::atan2(-ax_g, std::sqrt(ay_g * ay_g + az_g * az_g));
    accel_angles.yaw = 0.0f; // Yaw需要陀螺仪积分

    // 获取陀螺仪数据（°/s）
    auto gyro_data = get_gyro();
    float gyro_lsb = get_gyro_lsb_per_dps();
    float gyro_x_dps = gyro_data.x / gyro_lsb; // X轴角速率 (°/s)
    float gyro_y_dps = gyro_data.y / gyro_lsb; // Y轴角速率 (°/s)
    float gyro_z_dps = gyro_data.z / gyro_lsb; // Z轴角速率 (°/s) ← 用于Yaw计算

    // 将陀螺仪数据转换为弧度
    const float DEG_TO_RAD = 3.14159265f / 180.0f;
    float gyro_roll_rate = gyro_x_dps * DEG_TO_RAD;  // X轴陀螺仪 -> roll速率
    float gyro_pitch_rate = gyro_y_dps * DEG_TO_RAD; // Y轴陀螺仪 -> pitch速率
    float gyro_yaw_rate = gyro_z_dps * DEG_TO_RAD;   // Z轴陀螺仪 -> yaw速率

    // 互补滤波：gyro_alpha 越大，越信任陀螺仪（更平滑，但可能漂移）
    //          gyro_alpha 越小，越信任加速度计（更稳定，但噪声多）
    euler_angles filtered;
    filtered.roll = m_gyro_alpha * (m_prev_roll + gyro_roll_rate * dt) +
                    (1.0f - m_gyro_alpha) * accel_angles.roll;
    filtered.pitch = m_gyro_alpha * (m_prev_pitch + gyro_pitch_rate * dt) +
                     (1.0f - m_gyro_alpha) * accel_angles.pitch;
    filtered.yaw =
        m_prev_yaw + gyro_yaw_rate * dt; // Yaw: 纯陀螺仪积分（无加速度计修正）

    // 保存当前值供下一次使用
    m_prev_roll = filtered.roll;
    m_prev_pitch = filtered.pitch;
    m_prev_yaw = filtered.yaw; // 保存yaw供下一次积分

    return filtered;
  }

  /**
   * @brief 设置互补滤波器的陀螺仪权重系数
   *
   * @param alpha 权重系数，范围 [0.0, 1.0]
   *              - 0.95: 较强滤波，更稳定，但可能有轻微延迟
   *              - 0.98: 推荐值，平衡滤波效果和响应速度
   *              - 0.99: 轻微滤波，响应快，但噪声稍多
   */
  void set_filter_alpha(float alpha) {
    if (alpha < 0.0f)
      alpha = 0.0f;
    if (alpha > 1.0f)
      alpha = 1.0f;
    m_gyro_alpha = alpha;
  }

  /**
   * @brief 获取当前互补滤波器的陀螺仪权重系数
   *
   * @return 权重系数 [0.0, 1.0]
   */
  float get_filter_alpha() const { return m_gyro_alpha; }

  /**
   * @brief 重置互补滤波器状态
   *
   * 当需要重新初始化滤波器（如设备重启、大幅度姿态改变）时调用此函数
   */
  void reset_filter() {
    m_prev_roll = 0.0f;
    m_prev_pitch = 0.0f;
    m_prev_yaw = 0.0f;
    m_last_filter_time = gdut::steady_clock::time_point{};
  }

  /**
   * @brief 重置偏航角（Yaw）
   *
   * 由于Yaw是陀螺仪积分会长期漂移，可以定期调用此方法重新设定参考方向
   *
   *yaw_angle 新的参考偏航角（弧度），默认0表示当前方向为北方
   *
   * 使用示例：
   * // 初始化时，设定当前方向为参考
   * imu.reset_yaw(0.0f);
   *
   * // 或者在已知方向时调用
   * imu.reset_yaw(3.14159f / 4);  // 设定为45°方向
   */
  void reset_yaw(float yaw_angle = 0.0f) { m_prev_yaw = yaw_angle; }

  /**
   *  获取当前偏航角（Yaw）
   *
   偏航角（弧度），范围[-π, π]
   *
   Yaw通过陀螺仪Z轴积分得到，长期会有漂移，需要外部传感器（如磁力计）校正
   *       或者定期调用 reset_yaw() 重新标定
   */
  float get_yaw() const {
    float yaw = m_prev_yaw;
    // 将yaw限制在[-π, π]范围内
    while (yaw > 3.14159265f)
      yaw -= 2.0f * 3.14159265f;
    while (yaw < -3.14159265f)
      yaw += 2.0f * 3.14159265f;
    return yaw;
  }

  /**
   * @brief 计算真实加速度（去掉重力加速度）
   *
   * @return 返回包含x、y、z真实加速度的结构体，单位m/s²
   */
  struct real_acceleration {
    float x;
    float y;
    float z;
  };

  real_acceleration get_real_acceleration() {
    auto accel_data = get_accel();
    auto angles = get_filtered_euler_angles();

    float accel_lsb = get_accel_lsb_per_g();
    float ax_g = accel_data.x / accel_lsb;
    float ay_g = accel_data.y / accel_lsb;
    float az_g = accel_data.z / accel_lsb;

    const float g = 9.81f; // 重力加速度
    float gravity_x = g * std::sin(angles.pitch);
    float gravity_y = g * std::sin(angles.roll) * std::cos(angles.pitch);
    float gravity_z = g * std::cos(angles.roll) * std::cos(angles.pitch);

    real_acceleration accel;
    accel.x = (ax_g * g) - gravity_x;
    accel.y = (ay_g * g) - gravity_y;
    accel.z = (az_g * g) - gravity_z;

    return accel;
  }

  /**
   * 设置采样率
   * 采样率 = 1000 Hz / (1 + SMPLRT_DIV)
   * sample_rate 采样率，单位为 Hz（1~1000）
   */
  void set_sample_rate(uint16_t sample_rate) {
    if (sample_rate > 1000)
      sample_rate = 1000;
    if (sample_rate < 1)
      sample_rate = 1;
    uint8_t div = static_cast<uint8_t>(1000 / sample_rate - 1);
    write_register(mpu6050_reg::SMPLRT_DIV, div);
  }

  /**
   *设置加速度计量程
   *  量程选择
   */
  void set_accel_range(accel_range range) {
    uint8_t cfg;
    if (!read_register(mpu6050_reg::ACCEL_CONFIG, &cfg, 1)) {
      return;
    }
    cfg = (cfg & 0xE7) | static_cast<uint8_t>(range);
    if (write_register(mpu6050_reg::ACCEL_CONFIG, cfg)) {
      m_current_accel_range = range; // 更新当前量程
    }
  }

  /**
   * @brief 设置陀螺仪量程
   *
   * @param range 量程选择
   */
  void set_gyro_range(gyro_range range) {
    uint8_t cfg;
    if (!read_register(mpu6050_reg::GYRO_CONFIG, &cfg, 1)) {
      return;
    }
    cfg = (cfg & 0xE7) | static_cast<uint8_t>(range);
    if (write_register(mpu6050_reg::GYRO_CONFIG, cfg)) {
      m_current_gyro_range = range; // 更新当前量程
    }
  }

  /**
   *  设置数字低通滤波器
   *  cfg DLPF 配置
   */
  void set_dlpf(dlpf_cfg cfg) {
    uint8_t config;
    if (!read_register(mpu6050_reg::CONFIG, &config, 1)) {
      return;
    }
    config = (config & 0xF8) | static_cast<uint8_t>(cfg);
    write_register(mpu6050_reg::CONFIG, config);
  }

  /**
   *  启用中断
   *  中断类型
   */
  void enable_interrupt(uint8_t interrupt) {
    uint8_t ie;
    if (!read_register(mpu6050_reg::INT_ENABLE, &ie, 1)) {
      return;
    }
    ie |= interrupt;
    write_register(mpu6050_reg::INT_ENABLE, ie);
  }

  /**
   *  禁用中断
   * @param interrupt 中断类型
   */
  void disable_interrupt(uint8_t interrupt) {
    uint8_t ie;
    if (!read_register(mpu6050_reg::INT_ENABLE, &ie, 1)) {
      return;
    }
    ie &= ~interrupt;
    write_register(mpu6050_reg::INT_ENABLE, ie);
  }

  /**
   * 读取中断状态
   * @return 中断状态寄存器值
   */
  uint8_t get_interrupt_status() {
    uint8_t status = 0;
    read_register(mpu6050_reg::INT_STATUS, &status, 1);
    return status;
  }

  /**
   *  启用运动检测
   * @param threshold 运动检测门限（0~255），单位为 1mg，典型值 50
   */
  void enable_motion_detection(uint8_t threshold = 50) {
    // 设置运动检测门限
    write_register(mpu6050_reg::WOM_THR, threshold);
    // 启用运动检测中断
    enable_interrupt(0x40); // WOM_INT_EN
  }

  // 禁用运动检测
  void disable_motion_detection() {
    disable_interrupt(0x40); // WOM_INT_EN
  }

  // 获取设备 ID
  uint8_t get_device_id() {
    uint8_t id = 0;
    read_register(mpu6050_reg::WHO_AM_I, &id, 1);
    return id;
  }

private:
  i2c *m_i2c{nullptr};
  uint8_t m_i2c_addr{
      static_cast<uint8_t>(static_cast<uint8_t>(mpu6050_addr::low) << 1)};
  // HAL 使用的 8 位地址格式，默认对应 7 位从机地址 0x68（左移 1 位后为 0xD0）
  accel_range m_current_accel_range{accel_range::range_16g}; // 当前加速度计量程
  gyro_range m_current_gyro_range{gyro_range::range_2000dps}; // 当前陀螺仪量程
  gdut::function<void(std::uint32_t)> m_delay_func;           // 延迟函数对象

  // 互补滤波器状态变量
  float m_prev_roll{0.0f};   // 上一次的roll角度 (弧度)
  float m_prev_pitch{0.0f};  // 上一次的pitch角度 (弧度)
  float m_prev_yaw{0.0f};    // 上一次的yaw角度 (弧度) ← 陀螺仪积分
  float m_gyro_alpha{0.98f}; // 陀螺仪权重系数 [0.95-0.99]
  gdut::steady_clock::time_point m_last_filter_time; // 上一次滤波的时间

  /**
   *  获取加速度计当前量程的 LSB/g 值
   * @return LSB/g 转换因子
   *
   * 量程对应关系：范围相除
   * - ±2g: 16384 LSB/g
   * - ±4g: 8192 LSB/g
   * - ±8g: 4096 LSB/g
   * - ±16g: 2048 LSB/g
   */
  float get_accel_lsb_per_g() const {
    switch (m_current_accel_range) {
    case accel_range::range_2g:
      return 16384.0f;
    case accel_range::range_4g:
      return 8192.0f;
    case accel_range::range_8g:
      return 4096.0f;
    case accel_range::range_16g:
      return 2048.0f;
    default:
      return 2048.0f; // 默认值
    }
  }

  /**
   *  获取陀螺仪当前量程的 LSB/°/s 值
   * @return LSB/°/s 转换因子
   *
   * 量程对应关系：
   * - ±250°/s: 131.0 LSB/°/s
   * - ±500°/s: 65.5 LSB/°/s
   * - ±1000°/s: 32.8 LSB/°/s
   * - ±2000°/s: 16.4 LSB/°/s
   */
  float get_gyro_lsb_per_dps() const {
    switch (m_current_gyro_range) {
    case gyro_range::range_250dps:
      return 131.0f;
    case gyro_range::range_500dps:
      return 65.5f;
    case gyro_range::range_1000dps:
      return 32.8f;
    case gyro_range::range_2000dps:
      return 16.4f;
    default:
      return 16.4f; // 默认值
    }
  }
  /**
   *  读取寄存器
   * @param reg 寄存器地址
   * @param buffer 读取数据缓冲区
   * @param length 读取字节数
   * @return true 读取成功
   * @return false 读取失败
   */
  bool read_register(uint8_t reg, uint8_t *buffer, uint16_t length) {
    if (!m_i2c || !buffer) {
      return false;
    }
    return m_i2c->mem_read(m_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, buffer,
                           length, std::chrono::milliseconds(1000)) == HAL_OK;
  }

  /**
   * @brief 写入寄存器
   *
   * @param reg 寄存器地址
   * @param value 写入值
   * @return true 写入成功
   * @return false 写入失败
   */
  bool write_register(uint8_t reg, uint8_t value) {
    if (!m_i2c) {
      return false;
    }
    return m_i2c->mem_write(m_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1,
                            std::chrono::milliseconds(1000)) == HAL_OK;
  }
};

} // namespace gdut

#endif // BSP_MPU6050_HPP
