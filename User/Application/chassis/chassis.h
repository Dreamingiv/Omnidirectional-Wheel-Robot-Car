//
// Created by An on 2025/10/29.
//

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "XYTmotor/XYTMotor.h"

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

        bool isCommandTimeout() const;
        void runOpenLoop();
        void setWheelSpeed(Wheel wheel, float speed);

    private:
        bool inited_ = false;
        Mode mode_ = Mode::Relax;
        ControlType control_type_ = ControlType::OpenLoopSpeed;

        std::array<std::optional<XYTMotor>, WHEEL_COUNT> motors_{};
        std::array<Mixer, WHEEL_COUNT> mixer_{};
        std::array<float, WHEEL_COUNT> wheel_speed_{};

        Command command_{};
        float max_speed_ = 8.0f;
        uint32_t command_timeout_ms_ = 200;
        uint32_t last_command_tick_ = 0;

    };
} // namespace ega
