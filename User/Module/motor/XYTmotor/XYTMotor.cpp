//
// Created by Codex on 2026/5/19.
//

#include "XYTMotor.h"

#include <algorithm>

namespace
{
    void enableGPIOClock(GPIO_TypeDef* port)
    {
        if (port == GPIOA)
        {
            __HAL_RCC_GPIOA_CLK_ENABLE();
        }
        else if (port == GPIOB)
        {
            __HAL_RCC_GPIOB_CLK_ENABLE();
        }
        else if (port == GPIOC)
        {
            __HAL_RCC_GPIOC_CLK_ENABLE();
        }
        else if (port == GPIOD)
        {
            __HAL_RCC_GPIOD_CLK_ENABLE();
        }
        else if (port == GPIOE)
        {
            __HAL_RCC_GPIOE_CLK_ENABLE();
        }
        else if (port == GPIOF)
        {
            __HAL_RCC_GPIOF_CLK_ENABLE();
        }
        else if (port == GPIOG)
        {
            __HAL_RCC_GPIOG_CLK_ENABLE();
        }
        else if (port == GPIOH)
        {
            __HAL_RCC_GPIOH_CLK_ENABLE();
        }
        else if (port == GPIOI)
        {
            __HAL_RCC_GPIOI_CLK_ENABLE();
        }
    }
}

namespace ega
{
    XYTMotor::XYTMotor(const Config& config)
    {
        direction_ = config.direction;
        output_max_abs_ = OUTPUT_MAX;

        dir_port_ = config.dir_port;
        dir_pin_ = config.dir_pin;
        forward_level_ = config.forward_level;
        reverse_level_ = (forward_level_ == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;

        initDirectionGPIO();

        PWMInstance::Config pwm_cfg = config.pwm_config;
        pwm_cfg.type = PWMInstance::Type::DIRECT;
        pwm_cfg.auto_start = false;
        pwm_cfg.callback = nullptr;
        pwm_.emplace(pwm_cfg);

        setDuty(0.0f);
        if (config.auto_start)
        {
            pwm_->start();
            pwm_started_ = true;
        }

        is_online_ = true;

        if (idx_ < MAX_MOTORS)
        {
            motors_[idx_++] = this;
        }
    }

    XYTMotor::~XYTMotor()
    {
        if (pwm_.has_value())
        {
            setDuty(0.0f);
            pwm_->stop();
            pwm_.reset();
        }

        auto it = std::find(motors_.begin(), motors_.begin() + idx_, this);
        if (it != motors_.begin() + idx_)
        {
            const size_t pos = static_cast<size_t>(std::distance(motors_.begin(), it));
            motors_[pos] = motors_[idx_ - 1];
            motors_[idx_ - 1] = nullptr;
            --idx_;
        }
    }

    void XYTMotor::controlAll()
    {
        sendCommandAll();
    }

    void XYTMotor::enableAll()
    {
        for (size_t i = 0; i < idx_; ++i)
        {
            if (motors_[i])
            {
                motors_[i]->enable();
            }
        }
    }

    void XYTMotor::disableAll()
    {
        for (size_t i = 0; i < idx_; ++i)
        {
            if (motors_[i])
            {
                motors_[i]->disable();
            }
        }
    }

    void XYTMotor::sendCommandAll()
    {
        for (size_t i = 0; i < idx_; ++i)
        {
            if (motors_[i])
            {
                motors_[i]->sendCommand();
            }
        }
    }

    void XYTMotor::syncEnableStateAll()
    {
        for (size_t i = 0; i < idx_; ++i)
        {
            if (motors_[i])
            {
                motors_[i]->syncEnableState();
            }
        }
    }

    bool XYTMotor::hasDisabledMotor()
    {
        for (size_t i = 0; i < idx_; ++i)
        {
            if (motors_[i] && !motors_[i]->is_enabled_)
            {
                return true;
            }
        }
        return false;
    }

    bool XYTMotor::hasOfflineMotor()
    {
        for (size_t i = 0; i < idx_; ++i)
        {
            if (motors_[i] && !motors_[i]->is_online_)
            {
                return true;
            }
        }
        return false;
    }

    void XYTMotor::sendCommand()
    {
        if (!isEnabled() || !isEffortSet())
        {
            setDuty(0.0f);
            return;
        }

        if (!pwm_started_ && pwm_.has_value())
        {
            pwm_->start();
            pwm_started_ = true;
        }

        applyOutput(effort_);
    }

    void XYTMotor::syncEnableState()
    {
        if (!pwm_.has_value())
        {
            return;
        }

        if (isEnabled())
        {
            if (!pwm_started_)
            {
                pwm_->start();
                pwm_started_ = true;
            }
        }
        else
        {
            setDuty(0.0f);
        }
    }

    void XYTMotor::initDirectionGPIO() const
    {
        if (dir_port_ == nullptr || dir_pin_ == 0)
        {
            return;
        }

        enableGPIOClock(dir_port_);

        HAL_GPIO_WritePin(dir_port_, dir_pin_, reverse_level_);

        GPIO_InitTypeDef gpio_init{};
        gpio_init.Pin = dir_pin_;
        gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(dir_port_, &gpio_init);
    }

    void XYTMotor::setForward(bool is_forward) const
    {
        if (dir_port_ == nullptr || dir_pin_ == 0)
        {
            return;
        }

        HAL_GPIO_WritePin(dir_port_, dir_pin_, is_forward ? forward_level_ : reverse_level_);
    }

    void XYTMotor::setDuty(float duty)
    {
        if (!pwm_.has_value())
        {
            return;
        }

        if (duty < 0.0f)
        {
            duty = 0.0f;
        }
        else if (duty > 1.0f)
        {
            duty = 1.0f;
        }

        pwm_->setDuty(duty);
    }

    void XYTMotor::applyOutput(float effort)
    {
        const bool is_forward = (effort >= 0.0f);
        const float abs_effort = is_forward ? effort : -effort;

        if (abs_effort < EFFORT_DEADBAND)
        {
            setDuty(0.0f);
            return;
        }

        const float normalized = abs_effort / EFFORT_MAX_ABS;
        const float duty = DUTY_DEADZONE + normalized * (1.0f - DUTY_DEADZONE);

        setForward(is_forward);
        setDuty(duty);
    }

    void XYTMotor::offlineCallback()
    {
        is_online_ = false;
        unsetEffort();
        disable();
        setDuty(0.0f);
        measure_ = {};
    }
} // namespace ega
