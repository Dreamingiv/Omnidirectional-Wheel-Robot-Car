#pragma once

#include <cstdint>
#include <optional>

#include "driver_can.h"

namespace ega
{
    class MiniPC
    {
    public:
        struct Command
        {
            constexpr Command()
                : vx(0.0f),
                  vy(0.0f),
                  wz(0.0f),
                  enable(false),
                  reset_yaw_target(false),
                  seq(0)
            {
            }

            constexpr Command(float vx_value,
                              float vy_value,
                              float wz_value,
                              bool enable_value,
                              bool reset_yaw_target_value,
                              uint8_t seq_value)
                : vx(vx_value),
                  vy(vy_value),
                  wz(wz_value),
                  enable(enable_value),
                  reset_yaw_target(reset_yaw_target_value),
                  seq(seq_value)
            {
            }

            float vx;
            float vy;
            float wz;
            bool enable;
            bool reset_yaw_target;
            uint8_t seq;
        };

        static void init();
        static bool isOnline();
        static Command getCommand();

    private:
        static constexpr uint32_t RX_ID = 0x301;
        static constexpr uint32_t TX_ID = 0x302;
        static constexpr uint8_t MAGIC = 0xA5;
        static constexpr uint8_t VERSION = 0x01;
        static constexpr uint32_t TIMEOUT_MS = 200;

        static uint8_t checksum(const uint8_t* data, uint8_t len);
        static void rxCallback(const uint8_t* data, uint8_t len);

    private:
        static inline bool inited_ = false;
        static inline std::optional<CANInstance> can_{};
        static inline Command command_{};
        static inline uint32_t last_rx_tick_ = 0;
    };
} // namespace ega
