#ifndef BSP_IR_HPP
#define BSP_IR_HPP

#include "stm32f4xx_hal.h"
#include "uncopyable.hpp"
#include <array>
#include <cstddef>  

namespace gdut {

class ir_sensor : private uncopyable {
public:
    // 构造函数：线路传感器端口/引脚数组（固定5路），避障传感器端口/引脚
    ir_sensor(const std::array<GPIO_TypeDef*, 5>& line_ports,
              const std::array<uint16_t, 5>& line_pins,
              GPIO_TypeDef* obstacle_port, uint16_t obstacle_pin)
        : line_ports_(line_ports), line_pins_(line_pins),
          obstacle_port_(obstacle_port), obstacle_pin_(obstacle_pin) {}

    // 初始化所有GPIO为输入模式（上拉）
    void init() {
        GPIO_InitTypeDef gpio_init = {0};
        gpio_init.Mode = GPIO_MODE_INPUT;
        gpio_init.Pull = GPIO_PULLUP;
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

        // 初始化5路线路传感器引脚
        for (std::size_t i = 0; i < line_ports_.size(); ++i) {
            gpio_init.Pin = line_pins_[i];
            HAL_GPIO_Init(line_ports_[i], &gpio_init);
        }

        // 初始化避障传感器引脚
        gpio_init.Pin = obstacle_pin_;
        HAL_GPIO_Init(obstacle_port_, &gpio_init);
    }

    // 读取单路寻迹状态（true = 高电平，通常表示白线；false = 低电平，黑线）
    bool read_line(uint8_t channel) const {
        if (channel >= line_ports_.size()) return false;
        return HAL_GPIO_ReadPin(line_ports_[channel], line_pins_[channel]) == GPIO_PIN_SET;
    }

    // 读取所有寻迹状态，返回5位掩码（bit0 = 通道0，bit1 = 通道1 ...）
    uint8_t read_all_lines() const {
        uint8_t mask = 0;
        for (std::size_t i = 0; i < line_ports_.size(); ++i) {
            if (HAL_GPIO_ReadPin(line_ports_[i], line_pins_[i]) == GPIO_PIN_SET)
                mask |= (1 << i);
        }
        return mask;
    }

    // 读取避障状态（true = 检测到障碍物，具体电平需根据传感器确认）
    bool read_obstacle() const {
        return HAL_GPIO_ReadPin(obstacle_port_, obstacle_pin_) == GPIO_PIN_SET;
    }

private:
    std::array<GPIO_TypeDef*, 5> line_ports_;
    std::array<uint16_t, 5> line_pins_;
    GPIO_TypeDef* obstacle_port_;
    uint16_t obstacle_pin_;
};

} // namespace gdut

#endif // BSP_IR_HPP