#include "minipc.h"

#include <cmath>
#include <cstring>

namespace ega
{
    void MiniPC::init()
    {
        if (inited_)
        {
            return;
        }

        rx_frame_size_ = 0;
        command_ = {};
        last_rx_tick_ = 0;
        sequence_ = 0;
        received_nav_flag_ = 0;
        received_vx_ = 0.0f;
        received_vy_ = 0.0f;
        received_wz_ = 0.0f;

        USB::init({
            .tx_callback = nullptr,
            .rx_callback = [](uint8_t* data, uint16_t len)
            {
                usbRxCallback(data, len);
            },
        });

        inited_ = true;
    }

    bool MiniPC::isOnline()
    {
        if (!inited_ || last_rx_tick_ == 0)
        {
            return false;
        }

        return (HAL_GetTick() - last_rx_tick_) <= TIMEOUT_MS;
    }

    MiniPC::Command MiniPC::getCommand()
    {
        if (!isOnline())
        {
            return {};
        }

        return command_;
    }

    MiniPC::DebugData MiniPC::getDebugData()
    {
        HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
        const DebugData data{
            .nav_flag = received_nav_flag_,
            .vx = received_vx_,
            .vy = received_vy_,
            .wz = received_wz_,
            .sequence = sequence_,
        };
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        return data;
    }

    float MiniPC::limit(float value, float min, float max)
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

    void MiniPC::usbRxCallback(uint8_t* data, uint16_t len)
    {
        if (data == nullptr || len == 0)
        {
            return;
        }

        size_t offset = 0;
        while (offset < len)
        {
            const size_t remaining = RX_FRAME_SIZE - rx_frame_size_;
            const size_t available = static_cast<size_t>(len) - offset;
            const size_t copy_size = available < remaining ? available : remaining;
            std::memcpy(rx_frame_.data() + rx_frame_size_,
                        data + offset,
                        copy_size);
            rx_frame_size_ += copy_size;
            offset += copy_size;

            if (rx_frame_size_ == RX_FRAME_SIZE)
            {
                parseFrame(rx_frame_.data());
                rx_frame_size_ = 0;
            }
        }
    }

    void MiniPC::parseFrame(const uint8_t* data)
    {
        float vx = 0.0f;
        float vy = 0.0f;
        float wz = 0.0f;
        std::memcpy(&vx, data + 1, sizeof(vx));
        std::memcpy(&vy, data + 5, sizeof(vy));
        std::memcpy(&wz, data + 9, sizeof(wz));

        if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz))
        {
            return;
        }

        const bool enable = data[0] != 0U;
        received_nav_flag_ = data[0];
        received_vx_ = vx;
        received_vy_ = vy;
        received_wz_ = wz;
        command_.enable = enable;
        command_.reset_yaw_target = false;
        command_.vx = enable ? limit(vx * 3.0f, -1.0f, 1.0f) : 0.0f;
        command_.vy = enable ? limit(vy * 3.0f, -1.0f, 1.0f) : 0.0f;
        command_.wz = enable ? limit(wz * 3.0f, -3.0f, 3.0f) : 0.0f;
        command_.seq = ++sequence_;
        last_rx_tick_ = HAL_GetTick();
    }
} // namespace ega
