//
// Created by 2b superman on 25-11-21.
//

#include "servo.h"

namespace ega
{
    Servo::Servo(const Config& config) :
        max_angle_(config.max_angle),
        min_pulse_us_(config.min_pulse_us),
        max_pulse_us_(config.max_pulse_us),
        period_us_(config.period_us),
        current_angle_(config.init_angle)
    {
        // 1. 增加活动舵机计数
        ++servo_count_;

        // 2. 配置底层 PWM 驱动
        PWMInstance::Config pwm_cfg;// = config.pwm_config;
        pwm_cfg.handle = config.pwm_handle;
        pwm_cfg.channel = config.pwm_channel;
        pwm_cfg.type = PWMInstance::Type::DIRECT; // 舵机控制通常使用直接寄存器操作
        pwm_cfg.auto_start = false; // 先不开启，等初始化计算完初始角度再开
        pwm_cfg.callback = nullptr;

        // 3. 实例化 PWM 驱动
        pwm_driver_.emplace(pwm_cfg);

        // 4. 设置初始位置
        setAngle(config.init_angle);

        // 5. 启动 PWM 输出
        start();
    }

    Servo::~Servo()
    {
        if (pwm_driver_.has_value())
        {
            pwm_driver_->stop();
            pwm_driver_.reset();
        }
        // 减少活动舵机计数
        --servo_count_;
    }

    void Servo::start() const
    {
        if (pwm_driver_.has_value())
            pwm_driver_->start();
    }

    void Servo::stop() const
    {
        if (pwm_driver_.has_value())
            pwm_driver_->stop();
    }

    void Servo::setAngle(float angle)
    {
        // 1. 角度限幅
        if (angle < 0.0f)
            angle = 0.0f;
        if (angle > max_angle_)
            angle = max_angle_;

        current_angle_ = angle;

        // 2. 计算目标脉宽 (线性映射)
        // Target_Pulse = Min + (Angle / Max_Angle) * (Max - Min)
        float ratio = angle / max_angle_;
        float pulse_width = (float)min_pulse_us_ + ratio * (float)(max_pulse_us_ - min_pulse_us_);

        // 3. 写入脉宽
        writeMicroseconds((uint32_t)pulse_width);
    }

    void Servo::writeMicroseconds(uint32_t us)
    {
        if (!pwm_driver_.has_value())
            return;

        // 安全限幅 (防止舵机堵转烧毁)
        // 有些人喜欢给一点余量，但这里严格按照 Config 限制比较安全
        if (us < min_pulse_us_)
            us = min_pulse_us_;
        if (us > max_pulse_us_)
            us = max_pulse_us_;

        // 4. 计算 PWM 占空比比例
        // Ratio = Pulse_Width_US / Period_US
        // 例如: 1500us / 20000us = 0.075 (7.5% 占空比)
        float duty_ratio = (float)us / (float)period_us_;

        // 5. 调用底层驱动
        pwm_driver_->setDuty(duty_ratio);
    }
}
