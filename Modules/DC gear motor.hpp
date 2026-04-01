#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "async_timer.hpp"  
#include <cstdint>

class Motor 
{
public:
    // pwm_timer:用于 PWM 输出的timer对象指针
    // pwm_ch_A:正转通道
    // pwm_ch_B:反转通道
    // encoder_timer:编码器计数的timer对象指针
    // ppr: 编码器一圈的脉冲数
    // control_period_sec:控制周期（秒）
    Motor(gdut::timer* pwm_timer, uint32_t pwm_ch_A, uint32_t pwm_ch_B,
          gdut::timer* encoder_timer, float ppr, float control_period_sec = 0.01f);

    // 禁止拷贝
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    // 移动构造
    Motor(Motor&& other) noexcept;

    // ----- PID 参数设置 -----
    void setPidGain(float kp, float ki, float kd);
    void setPidOutputLimit(float min, float max);   // 限制输出功率
    void setTargetSpeed(float speed_rps);           // 目标转速

    // ----- 获取状态 -----
    float getCurrentSpeed() const;      // 当前转速
    float getTotalRevolutions() const;  // 累计转动圈数
    float getOutputPower() const;       // 当前输出功率

    // ----- 控制 -----
    void enable(bool enable);   // 使能或禁用 PID 控制（禁用时输出 0）
    void reset();               // 重置 PID 积分项和误差

    // 必须在定时器中断中周期调用
    void update();

private:
    void setPwmPower(float power);  // 设置两个通道的占空比

    // 硬件句柄
    gdut::timer* pwm_timer_;
    gdut::timer* encoder_timer_;
    uint32_t pwm_ch_A_;
    uint32_t pwm_ch_B_;

    // 编码器参数
    float ppr_;                 // 一圈脉冲数
    float control_period_;      // 控制周期

    // 状态变量
    int32_t last_encoder_count_;
    int32_t current_encoder_count_;
    float total_revolutions_;
    float current_speed_;       // 转/秒
    float target_speed_;        // 转/秒
    float output_power_;        // -1..1

    // PID 变量
    float kp_, ki_, kd_;
    float integral_;
    float last_error_;
    float output_min_, output_max_;
    bool enabled_;
};

#endif 