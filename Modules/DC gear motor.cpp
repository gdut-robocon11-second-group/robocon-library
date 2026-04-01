#include "Motor.hpp"
#include <algorithm> 

//  clamp 函数
template<typename T>
static inline T clamp(T value, T min, T max) 
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

Motor::Motor(gdut::timer* pwm_timer, uint32_t pwm_ch_A, uint32_t pwm_ch_B,
             gdut::timer* encoder_timer, float ppr, float control_period_sec)
    : pwm_timer_(pwm_timer)
    , encoder_timer_(encoder_timer)
    , pwm_ch_A_(pwm_ch_A)
    , pwm_ch_B_(pwm_ch_B)
    , ppr_(ppr)
    , control_period_(control_period_sec)
    , last_encoder_count_(0)
    , current_encoder_count_(0)
    , total_revolutions_(0.0f)
    , current_speed_(0.0f)
    , target_speed_(0.0f)
    , output_power_(0.0f)
    , kp_(0.0f), ki_(0.0f), kd_(0.0f)
    , integral_(0.0f)
    , last_error_(0.0f)
    , output_min_(-1.0f)
    , output_max_(1.0f)
    , enabled_(true)
{
    if (encoder_timer_) 
    {
        gdut::timer::timer_proxy proxy(encoder_timer_);
        current_encoder_count_ = static_cast<int32_t>(proxy.get_counter());
        last_encoder_count_ = current_encoder_count_;
        total_revolutions_ = static_cast<float>(current_encoder_count_) / ppr_;
    }
}

Motor::Motor(Motor&& other) noexcept
    : pwm_timer_(other.pwm_timer_)
    , encoder_timer_(other.encoder_timer_)
    , pwm_ch_A_(other.pwm_ch_A_)
    , pwm_ch_B_(other.pwm_ch_B_)
    , ppr_(other.ppr_)
    , control_period_(other.control_period_)
    , last_encoder_count_(other.last_encoder_count_)
    , current_encoder_count_(other.current_encoder_count_)
    , total_revolutions_(other.total_revolutions_)
    , current_speed_(other.current_speed_)
    , target_speed_(other.target_speed_)
    , output_power_(other.output_power_)
    , kp_(other.kp_), ki_(other.ki_), kd_(other.kd_)
    , integral_(other.integral_)
    , last_error_(other.last_error_)
    , output_min_(other.output_min_)
    , output_max_(other.output_max_)
    , enabled_(other.enabled_)
{
    other.pwm_timer_ = nullptr;
    other.encoder_timer_ = nullptr;
}

void Motor::setPidGain(float kp, float ki, float kd) 
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    integral_ = 0.0f;
    last_error_ = 0.0f;
}

void Motor::setPidOutputLimit(float min, float max) 
{
    output_min_ = clamp(min, -1.0f, 1.0f);
    output_max_ = clamp(max, -1.0f, 1.0f);
}

void Motor::setTargetSpeed(float speed_rps) 
{
    target_speed_ = speed_rps;
}

void Motor::enable(bool enable) 
{
    enabled_ = enable;
    if (!enabled_) 
    {
        output_power_ = 0.0f;
        setPwmPower(0.0f);
    }
}

void Motor::reset() 
{
    integral_ = 0.0f;
    last_error_ = 0.0f;
}

float Motor::getCurrentSpeed() const 
{
    return current_speed_;
}

float Motor::getTotalRevolutions() const
{
    return total_revolutions_;
}

float Motor::getOutputPower() const 
{
    return output_power_;
}

void Motor::update() 
{
    if (!encoder_timer_ || !pwm_timer_) return;

    //读取编码器计数值
    gdut::timer::timer_proxy encoder_proxy(encoder_timer_);
    last_encoder_count_ = current_encoder_count_;
    current_encoder_count_ = static_cast<int32_t>(encoder_proxy.get_counter());
    int32_t delta_count = current_encoder_count_ - last_encoder_count_;

    //更新累计圈数
    total_revolutions_ = static_cast<float>(current_encoder_count_) / ppr_;

    //计算当前转速（转/秒）
    current_speed_ = static_cast<float>(delta_count) / (ppr_ * control_period_);

    //PID 控制
    if (enabled_) 
    {
        float error = target_speed_ - current_speed_;
        integral_ += error * control_period_;
        // 积分限幅：防止积分饱和，限制在输出范围内
        float integral_limit = output_max_ / (ki_ + 1e-6f);
        integral_ = clamp(integral_, -integral_limit, integral_limit);

        float derivative = (error - last_error_) / control_period_;
        last_error_ = error;

        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        output_power_ = clamp(output, output_min_, output_max_);
        setPwmPower(output_power_);
    } 
    else 
    {
        output_power_ = 0.0f;
        setPwmPower(0.0f);
    }
}

void Motor::setPwmPower(float power) 
{
    //获取定时器的自动重载值
    gdut::timer::timer_proxy proxy(pwm_timer_);
    uint32_t arr = proxy.get_arr();
    uint32_t max_compare = (arr > 0) ? arr + 1 : 1;  //占空比最大值对应的比较值

    float duty_A = 0.0f;
    float duty_B = 0.0f;

    if (power >= 0) 
    {
        duty_A = power;
        duty_B = 0.0f;
    } 
    else 
    {
        duty_A = 0.0f;
        duty_B = -power;
    }

    uint32_t compare_A = static_cast<uint32_t>(duty_A * max_compare);
    uint32_t compare_B = static_cast<uint32_t>(duty_B * max_compare);

    gdut::timer::timer_pwm pwm(pwm_timer_);
    pwm.set_duty(pwm_ch_A_, compare_A);
    pwm.set_duty(pwm_ch_B_, compare_B);
}