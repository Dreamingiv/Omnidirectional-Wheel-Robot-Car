//
// Created by An on 2025/10/29.
//

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "XYTmotor/XYTMotor.h"
#include "driver_gpio.h"

namespace ega
{
    class Chassis
    {
    public:
        enum class Wheel : uint8_t
        {
            LF = 0,
            RF,
            LB,
            RB,
            Count,
        };

        enum class Mode : uint8_t
        {
            Relax = 0,
            OpenLoop,
            Protect,
        };

        enum class ControlType : uint8_t
        {
            OpenLoopSpeed = 0,
            SpeedClosedLoop,
        };

        struct Command
        {
            float vx = 0.0f;
            float vy = 0.0f;
            float wz = 0.0f;
        };

        struct Mixer
        {
            float vx = 0.0f;
            float vy = 0.0f;
            float wz = 0.0f;
        };

        static constexpr size_t WHEEL_COUNT = static_cast<size_t>(Wheel::Count);

        struct Config
        {
            std::array<XYTMotor::Config, WHEEL_COUNT> motors{};
            std::array<Mixer, WHEEL_COUNT> mixer{};

            float max_speed = 8.0f;
            float max_wheel_rpm = 1000.0f;
            float closed_loop_feedforward = 8.0f;
            float speed_kp = 0.0005f;
            float speed_ki = 0.0f;
            float speed_kd = 0.000002f;
            float speed_derivative_alpha = 0.03f;
            float speed_derivative_limit = 0.10f;
            float speed_effort_step_limit = 0.05f;
            float speed_integral_limit = 3.0f;
            float speed_deadband_rpm = 15.0f;
            uint32_t command_timeout_ms = 200;
            ControlType control_type = ControlType::OpenLoopSpeed;
            Mode init_mode = Mode::Relax;
        };

    public:
        static Chassis& getInstance();

        static Config makeDefaultConfig();
        static bool init(const Config& config);

        static void setMode(Mode mode);
        static Mode getMode();

        static void setCommand(const Command& command);
        static Command getCommand();
        static bool areFeedbackRpmsBelow(float threshold_rpm);
        static bool isAnyFeedbackRpmAbove(float threshold_rpm);

        static void update();
        static void stop();
        static void debug_printf();

        static bool isInited();

    private:
        Chassis() = default;
        ~Chassis() = default;

    public:
        Chassis(const Chassis&) = delete;
        Chassis& operator=(const Chassis&) = delete;

    private:
        static constexpr size_t toIndex(Wheel wheel)
        {
            return static_cast<size_t>(wheel);
        }

        static float limit(float value, float min, float max);
        static float abs(float value);

        bool initImpl(const Config& config);
        void setModeImpl(Mode mode);
        void setCommandImpl(const Command& command);
        void updateImpl();
        void stopImpl();
        void debugPrintfImpl() const;
        void initCapFeedback();
        void configureCapFeedbackPin(size_t index);
        bool isCapFeedbackPinConfigured(size_t index) const;
        void updateCapFeedback();
        void onCapPulse(Wheel wheel);

        bool isCommandTimeout() const;
        void mixWheelCommands();
        void runOpenLoop();
        void runSpeedClosedLoop();
        void resetSpeedController(Wheel wheel);
        void resetSpeedControllers();
        void setWheelSpeed(Wheel wheel, float speed);

    private:
        static constexpr size_t CAP_PERIOD_WINDOW_SIZE = 5;

        struct SpeedControllerState
        {
            float target_rpm = 0.0f;
            float effort = 0.0f;
            float integral = 0.0f;
            float last_feedback_rpm = 0.0f;
            float filtered_derivative_rpm_s = 0.0f;
            bool derivative_valid = false;
            int8_t direction = 0;
            uint32_t feedback_sequence = 0;
            uint32_t feedback_tick = 0;
        };

        struct CapFeedbackState
        {
            std::optional<GPIOInstance> gpio{};
            volatile uint32_t pulse_total = 0;
            volatile uint32_t last_pulse_cycle = 0;
            volatile uint32_t last_pulse_tick = 0;
            volatile bool has_previous_pulse = false;
            volatile uint32_t period_samples[CAP_PERIOD_WINDOW_SIZE]{};
            volatile uint32_t sample_sequence = 0;
            volatile uint8_t sample_count = 0;
            volatile uint8_t sample_write_index = 0;
            uint32_t last_processed_sequence = 0;
            float filtered_period_cycles = 0.0f;
            bool filter_valid = false;
            uint8_t reject_count = 0;
            uint32_t feedback_sequence = 0;
            uint32_t feedback_update_tick = 0;
            float period_us = 0.0f;
            float rpm = 0.0f;
        };

        bool inited_ = false;
        Mode mode_ = Mode::Relax;
        ControlType control_type_ = ControlType::OpenLoopSpeed;

        std::array<std::optional<XYTMotor>, WHEEL_COUNT> motors_{};
        std::array<Mixer, WHEEL_COUNT> mixer_{};
        std::array<float, WHEEL_COUNT> wheel_speed_{};

        Command command_{};
        float max_speed_ = 8.0f;
        float max_wheel_rpm_ = 1000.0f;
        float closed_loop_feedforward_ = 8.0f;
        float speed_kp_ = 0.0005f;
        float speed_ki_ = 0.0f;
        float speed_kd_ = 0.000002f;
        float speed_derivative_alpha_ = 0.03f;
        float speed_derivative_limit_ = 0.10f;
        float speed_effort_step_limit_ = 0.05f;
        float speed_integral_limit_ = 3.0f;
        float speed_deadband_rpm_ = 15.0f;
        uint32_t command_timeout_ms_ = 200;
        uint32_t last_command_tick_ = 0;

        std::array<SpeedControllerState, WHEEL_COUNT> speed_controllers_{};
        std::array<CapFeedbackState, WHEEL_COUNT> cap_feedback_{};

    };
} // namespace ega
