#include "minipc.h"

#include "can.h"

namespace ega
{
    void MiniPC::init()
    {
        if (inited_)
        {
            return;
        }

        can_.emplace(CANInstance::Config{
            .handle = &hcan2,
            .tx_id = TX_ID,
            .rx_id = RX_ID,
            .rx_callback = [](const uint8_t* data, uint8_t len)
            {
                rxCallback(data, len);
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

    uint8_t MiniPC::checksum(const uint8_t* data, uint8_t len)
    {
        uint8_t result = 0;
        for (uint8_t i = 0; i < len; ++i)
        {
            result ^= data[i];
        }
        return result;
    }

    void MiniPC::rxCallback(const uint8_t* data, uint8_t len)
    {
        if (data == nullptr || len != 8)
        {
            return;
        }

        if (data[0] != MAGIC || data[1] != VERSION)
        {
            return;
        }

        if (checksum(data, 7) != data[7])
        {
            return;
        }

        const uint8_t flags = data[2];
        const bool enable = (flags & 0x01U) != 0U;

        command_.enable = enable;
        command_.reset_yaw_target = (flags & 0x02U) != 0U;
        command_.vx = enable ? static_cast<float>(static_cast<int8_t>(data[3])) / 100.0f : 0.0f;
        command_.vy = enable ? static_cast<float>(static_cast<int8_t>(data[4])) / 100.0f : 0.0f;
        command_.wz = enable ? static_cast<float>(static_cast<int8_t>(data[5])) / 100.0f : 0.0f;
        command_.seq = data[6];
        last_rx_tick_ = HAL_GetTick();
    }
} // namespace ega
