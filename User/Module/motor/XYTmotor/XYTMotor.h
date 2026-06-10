//
// Created by Codex on 2026/5/19.
//

#ifndef MODULE_XYTMOTOR_H_
#define MODULE_XYTMOTOR_H_

#include <array>
#include <optional>

#include "driver_pwm.h"
#include "gpio.h"
#include "motor.h"

namespace ega
{
    class XYTMotor : public Motor
    {
        /* ====================== Config ====================== */
    public:
        struct Config
        {
            Direction direction = Direction::NORMAL;

            PWMInstance::Config pwm_config = {};

            GPIO_TypeDef* dir_port = nullptr;
            uint16_t dir_pin = 0;
            GPIO_PinState forward_level = GPIO_PIN_SET;

            bool auto_start = true;
        };

        /* ====================== Constants ====================== */
    public:
        static constexpr size_t MAX_MOTORS = 8;
        static constexpr float OUTPUT_MAX = 1.0f;
        static constexpr float DUTY_DEADZONE = 0.12f;
        static constexpr float EFFORT_DEADBAND = 0.02f;

        /* ====================== Static API ====================== */
    public:
        static void controlAll();
        static void enableAll();
        static void disableAll();
        static void sendCommandAll();
        static void syncEnableStateAll();

        static bool hasDisabledMotor();
        static bool hasOfflineMotor();

        /* ====================== Constructor / Destructor ====================== */
    public:
        explicit XYTMotor(const Config& config);
        ~XYTMotor() override;

        XYTMotor(const XYTMotor&) = delete;
        XYTMotor& operator=(const XYTMotor&) = delete;

        /* ====================== Motor API ====================== */
    public:
        static constexpr float SPEED_MAX_ABS = EFFORT_MAX_ABS;

        void setSpeed(const float& speed) { setEffort(speed); }
        void unsetSpeed() { unsetEffort(); }
        [[nodiscard]] float getSpeedCommand() const { return getEffort(); }

        void sendCommand() override;
        void syncEnableState() override;

        void debug() override
        {
            logger_printf("XYTMotor SpeedCommand:%f\r\n", effort_);
        }

        /* ====================== Private API ====================== */
    private:
        void initDirectionGPIO() const;
        void setForward(bool is_forward) const;
        void setDuty(float duty);
        void applyOutput(float effort);
        void offlineCallback() override;

        /* ====================== Members ====================== */
    private:
        static inline std::array<XYTMotor*, MAX_MOTORS> motors_{};
        static inline size_t idx_ = 0;

        std::optional<PWMInstance> pwm_;

        GPIO_TypeDef* dir_port_ = nullptr;
        uint16_t dir_pin_ = 0;
        GPIO_PinState forward_level_ = GPIO_PIN_SET;
        GPIO_PinState reverse_level_ = GPIO_PIN_RESET;

        bool pwm_started_ = false;
    };
} // namespace ega

#endif // MODULE_XYTMOTOR_H_
