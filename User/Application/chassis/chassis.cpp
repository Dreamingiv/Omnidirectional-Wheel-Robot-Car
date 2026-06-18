//
// Created by An on 2025/12/21.
//
#include "chassis.h"

#include "logger.h"
#include "minipc.h"
#include "tim.h"

namespace ega
{
    namespace
    {
        constexpr float CAP_PULSES_PER_REVOLUTION = 9.0f;
        constexpr float CAP_MAX_VALID_RPM = 1200.0f;
        constexpr float CAP_PERIOD_FILTER_ALPHA = 0.30f;
        constexpr float CAP_REJECT_RPM_MIN = 325.0f;
        constexpr float CAP_REJECT_RPM_MAX = 340.0f;
        constexpr uint8_t CAP_REJECT_ZERO_COUNT = 10;
        constexpr uint8_t CAP_MIN_FILTER_SAMPLES = 3;
        constexpr uint32_t CAP_SIGNAL_TIMEOUT_MS = 500;
        constexpr size_t CAP_FEEDBACK_COUNT = 3;

        GPIO_TypeDef* const CAP_PORTS[] = { GPIOB, GPIOB, GPIOB };
        constexpr uint16_t CAP_PINS[] = {
            GPIO_PIN_13,
            GPIO_PIN_15,
            GPIO_PIN_14,
        };
        constexpr Chassis::Wheel CAP_WHEELS[] = {
            Chassis::Wheel::LF,
            Chassis::Wheel::RF,
            Chassis::Wheel::LB,
        };
    }

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
        config.max_wheel_rpm = 1000.0f;
        config.closed_loop_feedforward = 8.0f;
        config.speed_kp = 0.0005f;
        config.speed_ki = 0.00f;
        config.speed_kd = 0.000002f;
        config.speed_derivative_alpha = 0.03f;
        config.speed_derivative_limit = 0.10f;
        config.speed_effort_step_limit = 0.05f;
        config.speed_integral_limit = 3.0f;
        config.speed_deadband_rpm = 30.0f;
        config.command_timeout_ms = 200;
        config.control_type = ControlType::SpeedClosedLoop;
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

    bool Chassis::areFeedbackRpmsBelow(float threshold_rpm)
    {
        const auto& instance = getInstance();
        if (!instance.inited_)
        {
            return false;
        }

        const float threshold = abs(threshold_rpm);
        for (const Wheel wheel : CAP_WHEELS)
        {
            if (abs(instance.cap_feedback_[toIndex(wheel)].rpm) >= threshold)
            {
                return false;
            }
        }
        return true;
    }

    bool Chassis::isAnyFeedbackRpmAbove(float threshold_rpm)
    {
        const auto& instance = getInstance();
        if (!instance.inited_)
        {
            return false;
        }

        const float threshold = abs(threshold_rpm);
        for (const Wheel wheel : CAP_WHEELS)
        {
            if (abs(instance.cap_feedback_[toIndex(wheel)].rpm) > threshold)
            {
                return true;
            }
        }
        return false;
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
        max_wheel_rpm_ = config.max_wheel_rpm > 0.0f ?
            config.max_wheel_rpm : 1000.0f;
        closed_loop_feedforward_ = limit(config.closed_loop_feedforward,
                                         0.0f,
                                         max_speed_);
        speed_kp_ = config.speed_kp >= 0.0f ? config.speed_kp : 0.0f;
        speed_ki_ = config.speed_ki >= 0.0f ? config.speed_ki : 0.0f;
        speed_kd_ = config.speed_kd >= 0.0f ? config.speed_kd : 0.0f;
        speed_derivative_alpha_ = limit(config.speed_derivative_alpha,
                                        0.0f,
                                        1.0f);
        speed_derivative_limit_ = limit(config.speed_derivative_limit,
                                        0.0f,
                                        max_speed_);
        speed_effort_step_limit_ = limit(config.speed_effort_step_limit,
                                         0.0f,
                                         max_speed_);
        speed_integral_limit_ = limit(config.speed_integral_limit,
                                      0.0f,
                                      max_speed_);
        speed_deadband_rpm_ = config.speed_deadband_rpm >= 0.0f ?
            config.speed_deadband_rpm : 0.0f;
        command_timeout_ms_ = config.command_timeout_ms;
        control_type_ = config.control_type;
        mode_ = config.init_mode;
        command_ = {};
        last_command_tick_ = HAL_GetTick();
        resetSpeedControllers();
        initCapFeedback();
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

        updateCapFeedback();

        if (mode_ == Mode::Protect || mode_ == Mode::Relax || isCommandTimeout())
        {
            stopImpl();
            return;
        }

        if (control_type_ == ControlType::OpenLoopSpeed)
        {
            runOpenLoop();
        }
        else if (control_type_ == ControlType::SpeedClosedLoop)
        {
            runSpeedClosedLoop();
        }
    }

    void Chassis::stopImpl()
    {
        command_ = {};
        resetSpeedControllers();
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
        static uint8_t last_usb_sequence = 0;
        const uint32_t now = HAL_GetTick();
        if ((now - last_print_tick) < 100U)
        {
            return;
        }
        last_print_tick = now;

        const auto usb = MiniPC::getDebugData();
        if (usb.sequence != last_usb_sequence)
        {
            logger_printf("%.3f,%.3f,%.3f\r\n",
                          usb.vx,
                          usb.vy,
                          usb.wz);
            last_usb_sequence = usb.sequence;
        }

        // logger_printf("%.2f,%.2f,%.2f,%.2f\r\n",
        //     cap_feedback_[toIndex(Wheel::LF)].rpm,
        //     cap_feedback_[toIndex(Wheel::LB)].rpm,
        //     cap_feedback_[toIndex(Wheel::RF)].rpm,
        //     cap_feedback_[toIndex(Wheel::RB)].rpm);

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

    void Chassis::initCapFeedback()
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

        for (size_t i = 0; i < CAP_FEEDBACK_COUNT; ++i)
        {
            auto& feedback = cap_feedback_[toIndex(CAP_WHEELS[i])];
            feedback.gpio.reset();
            feedback.pulse_total = 0;
            feedback.last_pulse_cycle = 0;
            feedback.last_pulse_tick = 0;
            feedback.has_previous_pulse = false;
            for (auto& sample : feedback.period_samples)
            {
                sample = 0;
            }
            feedback.sample_sequence = 0;
            feedback.sample_count = 0;
            feedback.sample_write_index = 0;
            feedback.last_processed_sequence = 0;
            feedback.filtered_period_cycles = 0.0f;
            feedback.filter_valid = false;
            feedback.reject_count = 0;
            feedback.feedback_sequence = 0;
            feedback.feedback_update_tick = 0;
            feedback.period_us = 0.0f;
            feedback.rpm = 0.0f;

            feedback.gpio.emplace(GPIOInstance::Config{
                .port = CAP_PORTS[i],
                .pin = CAP_PINS[i],
                .exti_callback = [i]()
                {
                    Chassis::getInstance().onCapPulse(CAP_WHEELS[i]);
                },
                .default_state = GPIO_PIN_RESET,
                .exti_mode = GPIOInstance::EXTI_TRIG_MODE::RISING,
            });

            configureCapFeedbackPin(i);
        }

        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    }

    void Chassis::configureCapFeedbackPin(size_t index)
    {
        if (index >= CAP_FEEDBACK_COUNT)
        {
            return;
        }

        GPIO_InitTypeDef gpio_init{};
        gpio_init.Pin = CAP_PINS[index];
        gpio_init.Mode = GPIO_MODE_IT_RISING;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(CAP_PORTS[index], &gpio_init);
        __HAL_GPIO_EXTI_CLEAR_IT(CAP_PINS[index]);
    }

    bool Chassis::isCapFeedbackPinConfigured(size_t index) const
    {
        if (index >= CAP_FEEDBACK_COUNT)
        {
            return false;
        }

        uint32_t position = 0;
        while (position < 16U &&
               CAP_PINS[index] != static_cast<uint16_t>(1UL << position))
        {
            ++position;
        }
        if (position >= 16U)
        {
            return false;
        }

        const uint32_t mode_mask = 0x3UL << (position * 2U);
        const uint32_t pull_mask = 0x3UL << (position * 2U);
        const uint32_t exti_shift = (position & 0x3U) * 4U;
        const uint32_t exti_port =
            (SYSCFG->EXTICR[position >> 2U] >> exti_shift) & 0xFU;
        const uint32_t pin = CAP_PINS[index];

        return (CAP_PORTS[index]->MODER & mode_mask) == 0U &&
               (CAP_PORTS[index]->PUPDR & pull_mask) == 0U &&
               exti_port == GPIO_GET_INDEX(CAP_PORTS[index]) &&
               (EXTI->IMR & pin) != 0U &&
               (EXTI->RTSR & pin) != 0U &&
               (EXTI->FTSR & pin) == 0U;
    }

    void Chassis::updateCapFeedback()
    {
        const uint32_t now = HAL_GetTick();
        for (size_t index = 0; index < CAP_FEEDBACK_COUNT; ++index)
        {
            auto& feedback = cap_feedback_[toIndex(CAP_WHEELS[index])];
            if (!isCapFeedbackPinConfigured(index))
            {
                HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
                configureCapFeedbackPin(index);
                feedback.has_previous_pulse = false;
                feedback.sample_count = 0;
                feedback.sample_write_index = 0;
                HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
            }

            if (!feedback.has_previous_pulse ||
                (now - feedback.last_pulse_tick) > CAP_SIGNAL_TIMEOUT_MS)
            {
                feedback.filter_valid = false;
                feedback.filtered_period_cycles = 0.0f;
                feedback.reject_count = 0;
                feedback.period_us = 0.0f;
                feedback.rpm = 0.0f;
                continue;
            }

            uint32_t samples[CAP_PERIOD_WINDOW_SIZE]{};
            uint8_t sample_count = 0;
            uint32_t sample_sequence = 0;

            HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
            sample_count = feedback.sample_count;
            sample_sequence = feedback.sample_sequence;
            for (uint8_t i = 0; i < sample_count; ++i)
            {
                samples[i] = feedback.period_samples[i];
            }
            HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

            if (sample_count < CAP_MIN_FILTER_SAMPLES ||
                sample_sequence == feedback.last_processed_sequence ||
                SystemCoreClock == 0)
            {
                continue;
            }

            for (uint8_t i = 1; i < sample_count; ++i)
            {
                const uint32_t value = samples[i];
                uint8_t j = i;
                while (j > 0 && samples[j - 1] > value)
                {
                    samples[j] = samples[j - 1];
                    --j;
                }
                samples[j] = value;
            }

            float median_cycles = static_cast<float>(samples[sample_count / 2]);
            if ((sample_count & 1U) == 0U)
            {
                median_cycles = 0.5f *
                    static_cast<float>(samples[sample_count / 2 - 1] +
                                       samples[sample_count / 2]);
            }

            const float raw_rpm = static_cast<float>(SystemCoreClock) * 60.0f /
                (CAP_PULSES_PER_REVOLUTION * median_cycles);
            feedback.last_processed_sequence = sample_sequence;
            if (raw_rpm >= CAP_REJECT_RPM_MIN && raw_rpm <= CAP_REJECT_RPM_MAX)
            {
                if (feedback.reject_count < CAP_REJECT_ZERO_COUNT)
                {
                    ++feedback.reject_count;
                }
                if (feedback.reject_count >= CAP_REJECT_ZERO_COUNT)
                {
                    feedback.filtered_period_cycles = 0.0f;
                    feedback.filter_valid = false;
                    feedback.period_us = 0.0f;
                    feedback.rpm = 0.0f;
                }
                continue;
            }
            feedback.reject_count = 0;

            float candidate_period_cycles = median_cycles;
            if (feedback.filter_valid)
            {
                candidate_period_cycles = feedback.filtered_period_cycles +
                    CAP_PERIOD_FILTER_ALPHA *
                    (median_cycles - feedback.filtered_period_cycles);
            }

            feedback.filtered_period_cycles = candidate_period_cycles;
            feedback.filter_valid = true;
            feedback.period_us = candidate_period_cycles * 1000000.0f /
                                 static_cast<float>(SystemCoreClock);
            feedback.rpm = static_cast<float>(SystemCoreClock) * 60.0f /
                (CAP_PULSES_PER_REVOLUTION * candidate_period_cycles);
            feedback.feedback_update_tick = now;
            ++feedback.feedback_sequence;
        }
    }

    void Chassis::onCapPulse(Wheel wheel)
    {
        const size_t index = toIndex(wheel);
        if (index >= WHEEL_COUNT || wheel == Wheel::RB)
        {
            return;
        }

        auto& feedback = cap_feedback_[index];
        const uint32_t now_cycle = DWT->CYCCNT;
        const uint32_t now_tick = HAL_GetTick();

        if (!feedback.has_previous_pulse ||
            (now_tick - feedback.last_pulse_tick) > CAP_SIGNAL_TIMEOUT_MS)
        {
            feedback.last_pulse_cycle = now_cycle;
            feedback.last_pulse_tick = now_tick;
            feedback.has_previous_pulse = true;
            feedback.sample_count = 0;
            feedback.sample_write_index = 0;
            ++feedback.pulse_total;
            return;
        }

        const uint32_t period_cycles = now_cycle - feedback.last_pulse_cycle;
        const uint32_t min_period_cycles = static_cast<uint32_t>(
            static_cast<float>(SystemCoreClock) * 60.0f /
            (CAP_PULSES_PER_REVOLUTION * CAP_MAX_VALID_RPM));
        if (period_cycles < min_period_cycles)
        {
            return;
        }

        feedback.last_pulse_cycle = now_cycle;
        feedback.last_pulse_tick = now_tick;
        feedback.period_samples[feedback.sample_write_index] = period_cycles;
        feedback.sample_write_index = static_cast<uint8_t>(
            (feedback.sample_write_index + 1U) % CAP_PERIOD_WINDOW_SIZE);
        if (feedback.sample_count < CAP_PERIOD_WINDOW_SIZE)
        {
            ++feedback.sample_count;
        }
        ++feedback.sample_sequence;
        ++feedback.pulse_total;
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

    void Chassis::mixWheelCommands()
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
    }

    void Chassis::runOpenLoop()
    {
        mixWheelCommands();

        for (size_t i = 0; i < WHEEL_COUNT; ++i)
        {
            setWheelSpeed(static_cast<Wheel>(i), wheel_speed_[i] * max_speed_);
        }
    }

    void Chassis::runSpeedClosedLoop()
    {
        mixWheelCommands();

        for (size_t i = 0; i < WHEEL_COUNT; ++i)
        {
            const Wheel wheel = static_cast<Wheel>(i);
            auto& controller = speed_controllers_[i];
            const auto& feedback = cap_feedback_[i];

            if (!motors_[i].has_value())
            {
                resetSpeedController(wheel);
                continue;
            }

            controller.target_rpm = wheel_speed_[i] * max_wheel_rpm_;
            const float target_abs_rpm = abs(controller.target_rpm);
            if (target_abs_rpm < 5.0f)
            {
                resetSpeedController(wheel);
                setWheelSpeed(wheel, 0.0f);
                continue;
            }

            const int8_t direction = controller.target_rpm >= 0.0f ? 1 : -1;
            const float feedforward = limit(
                target_abs_rpm / max_wheel_rpm_ * closed_loop_feedforward_,
                0.0f,
                max_speed_);

            if (direction != controller.direction)
            {
                resetSpeedController(wheel);
                controller.target_rpm = wheel_speed_[i] * max_wheel_rpm_;
                controller.direction = direction;
                controller.effort = feedforward;
            }

            if (wheel == Wheel::RB)
            {
                controller.integral = 0.0f;
                controller.derivative_valid = false;
                controller.filtered_derivative_rpm_s = 0.0f;
                controller.effort = feedforward;
                setWheelSpeed(wheel,
                              static_cast<float>(direction) * controller.effort);
                continue;
            }

            if (!feedback.filter_valid)
            {
                controller.integral = 0.0f;
                controller.derivative_valid = false;
                controller.filtered_derivative_rpm_s = 0.0f;
                controller.effort = feedforward;
                controller.feedback_sequence = feedback.feedback_sequence;
                controller.feedback_tick = feedback.feedback_update_tick;
            }
            else if (feedback.feedback_sequence != controller.feedback_sequence)
            {
                float dt_s = 0.0f;
                if (controller.feedback_tick != 0)
                {
                    dt_s = static_cast<float>(
                        feedback.feedback_update_tick - controller.feedback_tick) * 0.001f;
                    dt_s = limit(dt_s, 0.001f, 0.2f);
                }

                const float error = target_abs_rpm - feedback.rpm;
                float derivative_output = 0.0f;
                if (controller.derivative_valid && dt_s > 0.0f)
                {
                    const float raw_derivative_rpm_s =
                        (feedback.rpm - controller.last_feedback_rpm) / dt_s;
                    controller.filtered_derivative_rpm_s +=
                        speed_derivative_alpha_ *
                        (raw_derivative_rpm_s -
                         controller.filtered_derivative_rpm_s);
                    derivative_output = limit(
                        -speed_kd_ * controller.filtered_derivative_rpm_s,
                        -speed_derivative_limit_,
                        speed_derivative_limit_);
                }
                else
                {
                    controller.filtered_derivative_rpm_s = 0.0f;
                }
                controller.last_feedback_rpm = feedback.rpm;
                controller.derivative_valid = true;

                if (abs(error) <= speed_deadband_rpm_)
                {
                    controller.feedback_sequence = feedback.feedback_sequence;
                    controller.feedback_tick = feedback.feedback_update_tick;
                    setWheelSpeed(wheel,
                                  static_cast<float>(direction) * controller.effort);
                    continue;
                }

                float candidate_integral = controller.integral +
                    speed_ki_ * error * dt_s;
                candidate_integral = limit(candidate_integral,
                                           -speed_integral_limit_,
                                           speed_integral_limit_);

                const float candidate_output = feedforward +
                    speed_kp_ * error + candidate_integral + derivative_output;
                const bool can_integrate =
                    (candidate_output >= 0.0f && candidate_output <= max_speed_) ||
                    (candidate_output > max_speed_ && error < 0.0f) ||
                    (candidate_output < 0.0f && error > 0.0f);
                if (can_integrate)
                {
                    controller.integral = candidate_integral;
                }

                const float desired_effort = limit(
                    feedforward + speed_kp_ * error +
                    controller.integral + derivative_output,
                    0.0f,
                    max_speed_);
                controller.effort = limit(
                    desired_effort,
                    controller.effort - speed_effort_step_limit_,
                    controller.effort + speed_effort_step_limit_);
                controller.feedback_sequence = feedback.feedback_sequence;
                controller.feedback_tick = feedback.feedback_update_tick;
            }

            setWheelSpeed(wheel, static_cast<float>(direction) * controller.effort);
        }
    }

    void Chassis::resetSpeedController(Wheel wheel)
    {
        const size_t index = toIndex(wheel);
        if (index >= WHEEL_COUNT)
        {
            return;
        }

        auto& controller = speed_controllers_[index];
        const auto& feedback = cap_feedback_[index];
        controller.target_rpm = 0.0f;
        controller.effort = 0.0f;
        controller.integral = 0.0f;
        controller.last_feedback_rpm = 0.0f;
        controller.filtered_derivative_rpm_s = 0.0f;
        controller.derivative_valid = false;
        controller.direction = 0;
        controller.feedback_sequence = feedback.feedback_sequence;
        controller.feedback_tick = feedback.feedback_update_tick;
    }

    void Chassis::resetSpeedControllers()
    {
        for (size_t i = 0; i < WHEEL_COUNT; ++i)
        {
            resetSpeedController(static_cast<Wheel>(i));
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
