//
// Created by An on 2025/12/21.
//
#include "chassis.h"

#include "logger.h"
#include "tim.h"

namespace ega
{
    Chassis& Chassis::getInstance()
    {
        static Chassis instance;
        return instance;
    }

    Chassis::Config Chassis::makeDefaultConfig()
    {
        Config config{};

        config.motors[toIndex(Wheel::LF)].pwm_config.handle = &htim1;
        config.motors[toIndex(Wheel::LF)].pwm_config.channel = TIM_CHANNEL_1;
        config.motors[toIndex(Wheel::LF)].dir_port = GPIOC;
        config.motors[toIndex(Wheel::LF)].dir_pin = GPIO_PIN_6;

        config.motors[toIndex(Wheel::RF)].pwm_config.handle = &htim1;
        config.motors[toIndex(Wheel::RF)].pwm_config.channel = TIM_CHANNEL_2;
        config.motors[toIndex(Wheel::RF)].dir_port = GPIOI;
        config.motors[toIndex(Wheel::RF)].dir_pin = GPIO_PIN_6;

        config.motors[toIndex(Wheel::LB)].pwm_config.handle = &htim1;
        config.motors[toIndex(Wheel::LB)].pwm_config.channel = TIM_CHANNEL_3;
        config.motors[toIndex(Wheel::LB)].dir_port = GPIOI;
        config.motors[toIndex(Wheel::LB)].dir_pin = GPIO_PIN_7;

        config.motors[toIndex(Wheel::RB)].pwm_config.handle = &htim1;
        config.motors[toIndex(Wheel::RB)].pwm_config.channel = TIM_CHANNEL_4;
        config.motors[toIndex(Wheel::RB)].dir_port = GPIOB;
        config.motors[toIndex(Wheel::RB)].dir_pin = GPIO_PIN_12;

        for (auto& motor : config.motors)
        {
            motor.forward_level = GPIO_PIN_SET;
            motor.auto_start = true;
        }

        config.mixer[toIndex(Wheel::LF)] = { 1.0f, -1.0f, -1.0f };
        config.mixer[toIndex(Wheel::RF)] = { -1.0f,  -1.0f,  -1.0f };
        config.mixer[toIndex(Wheel::LB)] = { 1.0f,  1.0f, -1.0f };
        config.mixer[toIndex(Wheel::RB)] = { -1.0f, 1.0f,  -1.0f };

        config.max_speed = 10.0f;
        config.command_timeout_ms = 200;
        config.control_type = ControlType::OpenLoopSpeed;
        config.init_mode = Mode::Relax;

        return config;
    }

    bool Chassis::init(const Config& config)
    {
        return getInstance().initImpl(config);
    }

    void Chassis::setMode(Mode mode)
    {
        getInstance().setModeImpl(mode);
    }

    Chassis::Mode Chassis::getMode()
    {
        return getInstance().mode_;
    }

    void Chassis::setCommand(const Command& command)
    {
        getInstance().setCommandImpl(command);
    }

    Chassis::Command Chassis::getCommand()
    {
        return getInstance().command_;
    }

    void Chassis::update()
    {
        getInstance().updateImpl();
    }

    void Chassis::stop()
    {
        getInstance().stopImpl();
    }

    void Chassis::debug_printf()
    {
        getInstance().debugPrintfImpl();
    }

    bool Chassis::isInited()
    {
        return getInstance().inited_;
    }

    float Chassis::limit(float value, float min, float max)
    {
        if (value < min)
        {
            return min;
        }
        if (value > max)
        {
            return max;
        }
        return value;
    }

    float Chassis::abs(float value)
    {
        return (value >= 0.0f) ? value : -value;
    }

    bool Chassis::initImpl(const Config& config)
    {
        stopImpl();

        for (auto& motor : motors_)
        {
            motor.reset();
        }

        for (size_t i = 0; i < WHEEL_COUNT; ++i)
        {
            motors_[i].emplace(config.motors[i]);
            motors_[i]->enable();
            motors_[i]->setSpeed(0.0f);
            mixer_[i] = config.mixer[i];
            wheel_speed_[i] = 0.0f;
        }

        max_speed_ = limit(config.max_speed, 0.0f, XYTMotor::SPEED_MAX_ABS);
        command_timeout_ms_ = config.command_timeout_ms;
        control_type_ = config.control_type;
        mode_ = config.init_mode;
        command_ = {};
        last_command_tick_ = HAL_GetTick();
        inited_ = true;

        if (mode_ != Mode::OpenLoop)
        {
            stopImpl();
        }

        return true;
    }

    void Chassis::setModeImpl(Mode mode)
    {
        mode_ = mode;
        if (mode_ != Mode::OpenLoop)
        {
            stopImpl();
        }
    }

    void Chassis::setCommandImpl(const Command& command)
    {
        command_.vx = limit(command.vx, -1.0f, 1.0f);
        command_.vy = limit(command.vy, -1.0f, 1.0f);
        command_.wz = limit(command.wz, -3.0f, 3.0f);
        last_command_tick_ = HAL_GetTick();
    }

    void Chassis::updateImpl()
    {
        if (!inited_)
        {
            return;
        }

        if (mode_ == Mode::Protect || mode_ == Mode::Relax || isCommandTimeout())
        {
            stopImpl();
            return;
        }

        if (control_type_ == ControlType::OpenLoopSpeed)
        {
            runOpenLoop();
        }
        else
        {
            stopImpl();
        }
    }

    void Chassis::stopImpl()
    {
        command_ = {};
        for (size_t i = 0; i < WHEEL_COUNT; ++i)
        {
            wheel_speed_[i] = 0.0f;
            if (motors_[i].has_value())
            {
                motors_[i]->setSpeed(0.0f);
            }
        }
    }

    void Chassis::debugPrintfImpl() const
    {
        static uint32_t last_print_tick = 0;
        const uint32_t now = HAL_GetTick();
        if ((now - last_print_tick) < 200)
        {
            return;
        }
        last_print_tick = now;

        // logger_printf(
        //     "CHASSIS mode=%d cmd=(%.2f,%.2f,%.2f) wheel=(%.2f,%.2f,%.2f,%.2f) effort=(%.2f,%.2f,%.2f,%.2f) max=%.2f\r\n",
        //     static_cast<int>(mode_),
        //     command_.vx,
        //     command_.vy,
        //     command_.wz,
        //     wheel_speed_[toIndex(Wheel::LF)],
        //     wheel_speed_[toIndex(Wheel::RF)],
        //     wheel_speed_[toIndex(Wheel::LB)],
        //     wheel_speed_[toIndex(Wheel::RB)],
        //     wheel_speed_[toIndex(Wheel::LF)] * max_speed_,
        //     wheel_speed_[toIndex(Wheel::RF)] * max_speed_,
        //     wheel_speed_[toIndex(Wheel::LB)] * max_speed_,
        //     wheel_speed_[toIndex(Wheel::RB)] * max_speed_,
        //     max_speed_);
    }

    bool Chassis::isCommandTimeout() const
    {
        if (command_timeout_ms_ == 0)
        {
            return false;
        }

        const uint32_t now = HAL_GetTick();
        return (now - last_command_tick_) > command_timeout_ms_;
    }

    void Chassis::runOpenLoop()
    {
        float max_abs_speed = 0.0f;

        for (size_t i = 0; i < WHEEL_COUNT; ++i)
        {
            wheel_speed_[i] =
                mixer_[i].vx * command_.vx +
                mixer_[i].vy * command_.vy +
                mixer_[i].wz * command_.wz;

            const float current_abs = abs(wheel_speed_[i]);
            if (current_abs > max_abs_speed)
            {
                max_abs_speed = current_abs;
            }
        }

        if (max_abs_speed > 1.0f)
        {
            for (auto& speed : wheel_speed_)
            {
                speed /= max_abs_speed;
            }
        }

        for (size_t i = 0; i < WHEEL_COUNT; ++i)
        {
            setWheelSpeed(static_cast<Wheel>(i), wheel_speed_[i] * max_speed_);
        }
    }

    void Chassis::setWheelSpeed(Wheel wheel, float speed)
    {
        const size_t index = toIndex(wheel);
        if (index >= WHEEL_COUNT || !motors_[index].has_value())
        {
            return;
        }

        motors_[index]->setSpeed(limit(speed, -max_speed_, max_speed_));
    }

} // namespace ega
