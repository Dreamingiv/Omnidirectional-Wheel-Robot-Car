#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "driver_usb.h"

namespace ega
{
    class MiniPC
    {
    public:
        struct __attribute__((packed)) MiniPCToBoard
        {
            uint8_t nav_flag;
            float nav_vx;
            float nav_vy;
            float nav_vz;
        };

        static_assert(sizeof(MiniPCToBoard) == 13,
                      "MiniPCToBoard USB protocol must be 13 bytes");

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

        struct DebugData
        {
            uint8_t nav_flag = 0;
            float vx = 0.0f;
            float vy = 0.0f;
            float wz = 0.0f;
            uint8_t sequence = 0;
        };

        static void init();
        static bool isOnline();
        static Command getCommand();
        static DebugData getDebugData();

    private:
        static constexpr uint32_t TIMEOUT_MS = 200;
        static constexpr size_t RX_FRAME_SIZE = sizeof(MiniPCToBoard);

        static float limit(float value, float min, float max);
        static void usbRxCallback(uint8_t* data, uint16_t len);
        static void parseFrame(const uint8_t* data);

    private:
        static inline bool inited_ = false;
        static inline std::array<uint8_t, RX_FRAME_SIZE> rx_frame_{};
        static inline size_t rx_frame_size_ = 0;
        static inline Command command_{};
        static inline uint32_t last_rx_tick_ = 0;
        static inline uint8_t sequence_ = 0;
        static inline uint8_t received_nav_flag_ = 0;
        static inline float received_vx_ = 0.0f;
        static inline float received_vy_ = 0.0f;
        static inline float received_wz_ = 0.0f;
    };
} // namespace ega
