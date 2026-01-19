//
// Created by zhangzhiwen on 25-10-29.
//

#include "remote_FS_I6X.h"

#include "logger.h"
namespace ega
{
    static uint32_t rd_bits(const uint8_t* buf, uint16_t bit_off, uint8_t bit_len)
    {
        // 从 bit_off 开始读取 bit_len 位（最多 32 位），LSB-first 按自然位拼接
        uint32_t v = 0;
        for (uint8_t i = 0; i < bit_len; ++i)
        {
            uint16_t bit = bit_off + i;
            uint8_t byte_idx = bit >> 3;
            uint8_t bit_idx = bit & 0x7;
            uint8_t bit_val = (buf[byte_idx] >> bit_idx) & 0x1;
            v |= (static_cast<uint32_t>(bit_val) << i);
        }
        return v;
    }
    FS_I6X::FS_I6X(const UARTInstance::Config& config)
    {
        uart_instance_.emplace(config);
        uart_instance_->startReceive();

        daemon_.emplace(Daemon::Config{.reload_time_ms = 200, // 超时200ms认为遥控器丢失
                                       .init_time_ms = 100, // 可选：上电初始延时，按需调整
                                       .callback = [this]() { this->offlineCallback(); }});
    }
    FS_I6X::FS_I6X() :
        FS_I6X(UARTInstance::Config{
            .handle = &huart3,
            .tx_type = UARTInstance::DMA,
            .rx_type = UARTInstance::DMA_IDLE,
            .rx_callback = [this](uint8_t* data, uint16_t size) { this->SBUSCallback(data, size); },
            .rx_size = 32,
        })
    { /* 委托构造函数，不需要额外代码 */
    }

    void FS_I6X::stop()
    {
        this->is_online_ = false;
        this->is_frame_valid_ = false;
        this->uart_instance_->stopReceive();
    }

    void FS_I6X::start() { this->uart_instance_->startReceive(); }

    void FS_I6X::SBUSCallback(uint8_t* data, uint16_t size) { this->parseFrame(data, size); }

    FS_I6X::RCSwitchState FS_I6X::convertSwitchState(int16_t val)
    {
        switch (val)
        {
        case -784:
            return RCSwitchState::UP; // 1(这是一个枚举，对应的值就是1，下同)
        case 0:
            return RCSwitchState::MIDDLE; // 3
        case 783:
            return RCSwitchState::DOWN; // 2
        default:
            return RCSwitchState::UP; // 1
        }
    }

    void FS_I6X::parseFrame(const uint8_t* data, uint16_t size)
    {
        if (data[0] != 0x0F || data[24] != 0x00)
        {
            return;
        }

        // bool frame_lost = (data[23] >> 2) & 0x01; //帧丢失标志位，遥控器关闭立刻置1
        bool failsafe = (data[23] >> 3) & 0x01; // 帧丢失标志位(安全)，遥控器关闭持续一段时间置1

        // 如果遥控器安全帧丢失
        if (failsafe)
        {
            // 不在喂狗，所有数据归零，且不再读取通道值
            rc_last_ = rc_current_ = RCData{};
            is_frame_valid_ = false;
            return;
        }
        // 遥控器正常在线，喂狗
        daemon_->reload();
        is_online_ = true;
        rc_last_ = rc_current_;

        auto ch0 = static_cast<int16_t>(rd_bits(data, 8, 11));
        auto ch1 = static_cast<int16_t>(rd_bits(data, 19, 11));
        auto ch2 = static_cast<int16_t>(rd_bits(data, 30, 11));
        auto ch3 = static_cast<int16_t>(rd_bits(data, 41, 11));

        auto ch4 = static_cast<int16_t>(rd_bits(data, 52, 11));
        auto ch5 = static_cast<int16_t>(rd_bits(data, 63, 11));

        auto sa = static_cast<int16_t>(rd_bits(data, 74, 11));
        auto sb = static_cast<int16_t>(rd_bits(data, 85, 11));
        auto sc = static_cast<int16_t>(rd_bits(data, 96, 11));
        auto sd = static_cast<int16_t>(rd_bits(data, 107, 11));

        // logger_printf("%d, %d, %d, %d\n",sa, sb, sc, sd);

        rc_current_.rocker_r_h = ch0 - RC_CH_VALUE_OFFSET;
        rc_current_.rocker_r_v = ch1 - RC_CH_VALUE_OFFSET;
        rc_current_.rocker_l_v = ch2 - RC_CH_VALUE_OFFSET;
        rc_current_.rocker_l_h = ch3 - RC_CH_VALUE_OFFSET;

        rc_current_.knob_l = ch4 - RC_CH_VALUE_OFFSET;
        rc_current_.knob_r = ch5 - RC_CH_VALUE_OFFSET;

        rc_current_.switch_A = convertSwitchState(sa - RC_CH_VALUE_OFFSET);
        rc_current_.switch_B = convertSwitchState(sb - RC_CH_VALUE_OFFSET);
        rc_current_.switch_C = convertSwitchState(sc - RC_CH_VALUE_OFFSET);
        rc_current_.switch_D = convertSwitchState(sd - RC_CH_VALUE_OFFSET);

        // rc_current_.rocker_r_h = static_cast<int16_t>(((data[2] << 8 | data[1]) & 0x7FF) - RC_CH_VALUE_OFFSET);
        // rc_current_.rocker_r_v = static_cast<int16_t>(((data[3] << 5 | data[2] >> 3) & 0x7FF) - RC_CH_VALUE_OFFSET);
        // rc_current_.rocker_l_v =
        //     static_cast<int16_t>(((data[5] << 10 | data[4] << 2 | data[3] >> 6) & 0x7FF) - RC_CH_VALUE_OFFSET);
        // rc_current_.rocker_l_h = static_cast<int16_t>(((data[6] << 7 | data[5] >> 1) & 0x7FF) - RC_CH_VALUE_OFFSET);
        // rc_current_.knob_l = static_cast<int16_t>(((data[7] << 4 | data[6] >> 4) & 0x7FF) - RC_CH_VALUE_OFFSET);
        // rc_current_.knob_r =
        //     static_cast<int16_t>(((data[9] << 9 | data[8] << 1 | data[7] >> 7) & 0x7FF) - RC_CH_VALUE_OFFSET);
        // auto switch_data_A = static_cast<int16_t>(((data[10] << 6 | data[9] >> 2) & 0x7FF) - RC_CH_VALUE_OFFSET);
        // auto switch_data_B = static_cast<int16_t>(((data[11] << 3 | data[10] >> 5) & 0x7FF) - RC_CH_VALUE_OFFSET);
        // auto switch_data_C = static_cast<int16_t>(((data[13] << 8 | data[12]) & 0x7FF) - RC_CH_VALUE_OFFSET);
        // auto switch_data_D = static_cast<int16_t>(((data[14] << 5 | data[13] >> 3) & 0x7FF) - RC_CH_VALUE_OFFSET);
        //
        // rc_current_.switch_A = convertSwitchState(switch_data_A);
        // rc_current_.switch_B = convertSwitchState(switch_data_B);
        // rc_current_.switch_C = convertSwitchState(switch_data_C);
        // rc_current_.switch_D = convertSwitchState(switch_data_D);

        if (!checkRockerValid())
        {
            is_frame_valid_ = false;
            return;
        }
        is_frame_valid_ = true;
    }

    const FS_I6X::RCData& FS_I6X::getData() const { return rc_current_; }

    int16_t FS_I6X::getRockerValue(RCRockerIndex ch) const
    {
        switch (ch)
        {
        case RCRockerIndex::RIGHT_HORIZONTAL:
            return rc_current_.rocker_r_h;
        case RCRockerIndex::RIGHT_VERTICAL:
            return rc_current_.rocker_r_v;
        case RCRockerIndex::LEFT_HORIZONTAL:
            return rc_current_.rocker_l_h;
        case RCRockerIndex::LEFT_VERTICAL:
            return rc_current_.rocker_l_v;
        default:
            return 0;
        }
    }

    int16_t FS_I6X::getLeftKnobValue() const { return rc_current_.knob_l; }

    int16_t FS_I6X::getRightKnobValue() const { return rc_current_.knob_r; }

    FS_I6X::RCSwitchState FS_I6X::getSwitchA() const { return rc_current_.switch_A; }

    FS_I6X::RCSwitchState FS_I6X::getSwitchB() const { return rc_current_.switch_B; }

    FS_I6X::RCSwitchState FS_I6X::getSwitchC() const { return rc_current_.switch_C; }

    FS_I6X::RCSwitchState FS_I6X::getSwitchD() const { return rc_current_.switch_D; }

    bool FS_I6X::isFrameValid() const { return is_frame_valid_; }

    bool FS_I6X::isOnline() const { return is_online_; }

    void FS_I6X::offlineCallback()
    {
        is_online_ = false;
        rc_current_ = RCData{}; // 清空目前的所有遥控器数据
        if (uart_instance_)
            uart_instance_->startReceive(); // 遥控器尝试重新接收数据
        // 如果遥控器掉线后还需要做些别的就在这里加
    }

    void FS_I6X::debug()
    {
        auto r = getData();
        logger_printf("FS_I6X: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d,  %d\n",
            r.rocker_r_h, r.rocker_r_v, r.rocker_l_v, r.rocker_l_h,
            r.knob_l, r.knob_r,
            static_cast<uint8_t>(r.switch_A), r.switch_B, r.switch_C, r.switch_D,
            isOnline()
        );
    }

    bool FS_I6X::checkRockerValid() const
    {
        auto in_range = [](float val) { return val >= -FS_I6X_FORMAT && val <= FS_I6X_FORMAT; };

        return in_range(rc_current_.rocker_r_h) && in_range(rc_current_.rocker_l_h) &&
            in_range(rc_current_.rocker_r_v) && in_range(rc_current_.rocker_l_v);
    }
} // namespace ega
