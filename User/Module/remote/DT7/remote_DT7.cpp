#include "remote_DT7.h"
#include "logger.h"
namespace ega
{
    // ================ Bit 读取小工具 ================
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

    DT7::DT7(const UARTInstance::Config& config)
    {
        uart_instance_.emplace(config);
        uart_instance_->startReceive();

        daemon_.emplace(Daemon::Config{.reload_time_ms = 100, // 超时100ms认为遥控器丢失
                                       .init_time_ms = 100, // 可选：上电初始延时，按需调整
                                       .callback = [this]() { this->offlineCallback(); }});
    }

    DT7::DT7() :
        DT7(UARTInstance::Config{
            .handle = &huart3,
            .tx_type = UARTInstance::DMA,
            .rx_type = UARTInstance::DMA_IDLE,
            .rx_callback = [this](uint8_t* data, uint16_t size) { this->DBUSCallback(data, size); },
            .rx_size = 32,
        })
    { /* 委托构造函数，不需要额外代码 */
    }

    void DT7::stop()
    {
        this->is_online_ = false;
        this->is_frame_valid_ = false;
        this->uart_instance_->stopReceive();
    }

    void DT7::start() { this->uart_instance_->startReceive(); }

    void DT7::DBUSCallback(uint8_t* data, uint16_t size) { this->parseFrame(data, size); }

    DT7::RCSwitchState DT7::convertSwitchState(int16_t val)
    {
        switch (val)
        {
        case 1:
            return RCSwitchState::UP;
        case 2:
            return RCSwitchState::DOWN;
        case 3:
            return RCSwitchState::MIDDLE;
        default:
            return RCSwitchState::UP;
        }
    }

    void DT7::parseFrame(const uint8_t* data, uint16_t size)
    {
        if (size != DBUS_FRAME_SIZE)
        {
            is_frame_valid_ = false;
            return;
        }
        daemon_->reload(); // 如果接收到大小正确的数据就喂狗
        is_online_ = true;
        // 解包
        auto ch0 = static_cast<int16_t>(rd_bits(data, 0, 11));
        auto ch1 = static_cast<int16_t>(rd_bits(data, 11, 11));
        auto ch2 = static_cast<int16_t>(rd_bits(data, 22, 11));
        auto ch3 = static_cast<int16_t>(rd_bits(data, 33, 11));

        auto s1 = static_cast<uint8_t>(rd_bits(data, 44, 2));
        auto s2 = static_cast<uint8_t>(rd_bits(data, 46, 2));

        auto mx = static_cast<int16_t>(rd_bits(data, 48, 16));
        auto my = static_cast<int16_t>(rd_bits(data, 64, 16));
        auto mz = static_cast<int16_t>(rd_bits(data, 80, 16));

        auto mb_l = static_cast<uint8_t>(rd_bits(data, 96, 8));
        auto mb_r = static_cast<uint8_t>(rd_bits(data, 104, 8));

        auto raw_keys = static_cast<uint16_t>(rd_bits(data, 112, 16));

        auto dial = static_cast<int16_t>(rd_bits(data, 128, 16));
        // 摇杆
        rc_current_.rc.rocker_r_h = ch0 - RC_CH_VALUE_OFFSET;
        rc_current_.rc.rocker_r_v = ch1 - RC_CH_VALUE_OFFSET;
        rc_current_.rc.rocker_l_h = ch2 - RC_CH_VALUE_OFFSET;
        rc_current_.rc.rocker_l_v = ch3 - RC_CH_VALUE_OFFSET;

        rc_current_.rc.dial = dial - RC_CH_VALUE_OFFSET; // 返回的数据很奇怪？
        if (!checkRockerValid())
        {
            is_frame_valid_ = false;
            return;
        }
        is_frame_valid_ = true;

        // 开关
        rc_current_.rc.sw_right = convertSwitchState(s1);
        rc_current_.rc.sw_left = convertSwitchState(s2);

        // 鼠标
        rc_current_.mouse.x = mx;
        rc_current_.mouse.y = my;
        rc_current_.mouse.z = mz;
        rc_current_.mouse.press_l[static_cast<uint8_t>(MouseState::PRESSED)] = mb_l;
        rc_current_.mouse.press_r[static_cast<uint8_t>(MouseState::PRESSED)] = mb_r;
        // 检测点击（上升沿检测）
        rc_current_.mouse.press_l[static_cast<uint8_t>(MouseState::CLICKED)] =
            (rc_current_.mouse.press_l[static_cast<uint8_t>(MouseState::PRESSED)] == 1 &&
             rc_last_.mouse.press_l[static_cast<uint8_t>(MouseState::PRESSED)] == 0)
            ? 1
            : 0;
        rc_current_.mouse.press_r[static_cast<uint8_t>(MouseState::CLICKED)] =
            (rc_current_.mouse.press_r[static_cast<uint8_t>(MouseState::PRESSED)] == 1 &&
             rc_last_.mouse.press_r[static_cast<uint8_t>(MouseState::PRESSED)] == 0)
            ? 1
            : 0;

        // 键盘
        rc_current_.keys[static_cast<uint8_t>(KeyState::PRESSED)].raw = raw_keys;

        rc_current_.keys[static_cast<uint8_t>(KeyState::CLICKED)].raw =
            rc_current_.keys[static_cast<uint8_t>(KeyState::PRESSED)].raw &
            (~rc_last_.keys[static_cast<uint8_t>(KeyState::PRESSED)].raw);

        // 保存历史
        rc_last_ = rc_current_;
    }

    const DT7::RCData& DT7::getData() const { return rc_current_; }

    int16_t DT7::getRockerValue(RCRockerIndex ch) const
    {
        switch (ch)
        {
        case RCRockerIndex::RIGHT_HORIZONTAL:
            return rc_current_.rc.rocker_r_h;
        case RCRockerIndex::RIGHT_VERTICAL:
            return rc_current_.rc.rocker_r_v;
        case RCRockerIndex::LEFT_HORIZONTAL:
            return rc_current_.rc.rocker_l_h;
        case RCRockerIndex::LEFT_VERTICAL:
            return rc_current_.rc.rocker_l_v;
        default:
            return 0;
        }
    }

    DT7::RCSwitchState DT7::getLeftSwitch() const { return rc_current_.rc.sw_left; }
    DT7::RCSwitchState DT7::getRightSwitch() const { return rc_current_.rc.sw_right; }

    uint8_t DT7::getMouseValue(MouseIndex ch) const
    {
        switch (ch)
        {
        case MouseIndex::X:
            return static_cast<uint8_t>(rc_current_.mouse.x);
        case MouseIndex::Y:
            return static_cast<uint8_t>(rc_current_.mouse.y);
        case MouseIndex::Z:
            return static_cast<uint8_t>(rc_current_.mouse.z);
        default:
            return 0;
        }
    }

    bool DT7::isMouseLeftPressed() const
    {
        return rc_current_.mouse.press_l[static_cast<uint8_t>(MouseState::PRESSED)] == 1;
    }

    bool DT7::isMouseRightPressed() const
    {
        return rc_current_.mouse.press_r[static_cast<uint8_t>(MouseState::PRESSED)] == 1;
    }

    bool DT7::isMouseLeftClicked()
    {
        const bool clicked = rc_current_.mouse.press_l[static_cast<uint8_t>(MouseState::CLICKED)] == 1;
        rc_current_.mouse.press_l[static_cast<uint8_t>(MouseState::CLICKED)] = 0;
        return clicked;
    }

    bool DT7::isMouseRightClicked()
    {
        const bool clicked = rc_current_.mouse.press_r[static_cast<uint8_t>(MouseState::CLICKED)] == 1;
        rc_current_.mouse.press_r[static_cast<uint8_t>(MouseState::CLICKED)] = 0;
        return clicked;
    }

    bool DT7::isKeyPressed(KeyIndex key) const
    {
        return rc_current_.keys[static_cast<uint8_t>(KeyState::PRESSED)].get(key) == 1;
    }

    bool DT7::isKeyPressedWithCtrl(KeyIndex key) const
    {
        if (key == KeyIndex::CTRL)
        {
            return false;
        }
        if (isKeyPressed(KeyIndex::CTRL))
        {
            return rc_current_.keys[static_cast<uint8_t>(KeyState::PRESSED)].get(key) == 1;
        }
        return false;
    }

    bool DT7::isKeyPressedWithShift(KeyIndex key) const
    {
        if (key == KeyIndex::SHIFT)
        {
            return false;
        }
        if (isKeyPressed(KeyIndex::SHIFT))
        {
            return rc_current_.keys[static_cast<uint8_t>(KeyState::PRESSED)].get(key) == 1;
        }
        return false;
    }

    bool DT7::isKeyClicked(KeyIndex key)
    {
        const bool clicked = rc_current_.keys[static_cast<uint8_t>(KeyState::CLICKED)].get(key) == 1;
        rc_current_.keys[static_cast<uint8_t>(KeyState::CLICKED)].set(key, false);
        return clicked;
    }

    bool DT7::isKeyClickedWithCtrl(KeyIndex key)
    {
        if (key == KeyIndex::CTRL)
        {
            return false;
        }
        if (isKeyPressed(KeyIndex::CTRL))
        {
            const bool clicked = rc_current_.keys[static_cast<uint8_t>(KeyState::CLICKED)].get(key) == 1;
            rc_current_.keys[static_cast<uint8_t>(KeyState::CLICKED)].set(key, false);
            return clicked;
        }
        return false;
    }

    bool DT7::isKeyClickedWithShift(KeyIndex key)
    {
        if (key == KeyIndex::SHIFT)
        {
            return false;
        }
        if (isKeyPressed(KeyIndex::SHIFT))
        {
            const bool clicked = rc_current_.keys[static_cast<uint8_t>(KeyState::CLICKED)].get(key) == 1;
            rc_current_.keys[static_cast<uint8_t>(KeyState::CLICKED)].set(key, false);
            return clicked;
        }
        return false;
    }

    bool DT7::isFrameValid() const { return is_frame_valid_; }
    bool DT7::isOnline() const { return is_online_; }

    void DT7::offlineCallback()
    {
        //	logger_printf("DT7 offline!\r\n");

        is_online_ = false;
        is_frame_valid_ = false;
        rc_current_ = RCData{}; // 清空目前的所有遥控器数据
        if (uart_instance_)
            uart_instance_->startReceive(); // 遥控器尝试重新接收数据
        // 如果遥控器掉线后还需要做些别的就在这里加
    }
    bool DT7::checkRockerValid() const
    {
        auto in_range = [](int16_t val)
        { return val >= RC_CH_VALUE_MIN - RC_CH_VALUE_OFFSET && val <= RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET; };

        return in_range(rc_current_.rc.rocker_l_h) && in_range(rc_current_.rc.rocker_l_v) &&
            in_range(rc_current_.rc.rocker_r_h) && in_range(rc_current_.rc.rocker_r_v);
    }
    void DT7::debug(DT7::DebugMode mode)
    {
        auto debugRC = [this]
        {
            const auto& r = rc_current_.rc;
            logger_printf("DT7 RC: %d, %d, %d, %d, %d, %d, %d\r\n", r.rocker_r_h, r.rocker_r_v, r.rocker_l_h,
                          r.rocker_l_v, r.dial, r.sw_left, r.sw_right);
        };

        auto debugMouse = [this]
        {
            const auto& m = rc_current_.mouse;
            logger_printf("DT7 Mouse: %d, %d, %d, %d, %d, %d, %d\r\n", m.x, m.y, m.z, m.press_l[0], m.press_l[1],
                          m.press_r[0], m.press_r[1]);
        };

        auto debugKey = [this]
        {
            const auto& kp = rc_current_.keys[0];
            const auto& kc = rc_current_.keys[1];
            char buf1[16 + 3 + 1];
            char buf2[16 + 3 + 1];
            int p = 0;
            for (auto i = 0; i < 16; i++)
            {
                buf1[p++] = ((kp.raw >> i) & 1) ? '1' : '0';
                if ((i+1) % 4 == 0 && i != 15) {
                    buf1[p++] = '_';
                }
            }
            buf1[p] = '\0';
            p = 0;
            for (auto i = 0; i < 16; i++)
            {
                buf2[p++] = ((kc.raw >> i) & 1) ? '1' : '0';
                if ((i+1) % 4 == 0 && i != 15) {
                    buf2[p++] = '_';
                }
            }
            buf2[p] = '\0';
            logger_printf("DT7 Key: %s, %s\r\n", buf1, buf2);
        };

        switch (mode)
        {
        case DebugMode::RC:
            debugRC();
            break;
        case DebugMode::MOUSE:
            debugMouse();
            break;
        case DebugMode::KEY:
            debugKey();
            break;
        case DebugMode::ALL:
            debugRC();
            debugMouse();
            debugKey();
            break;
        default:
            break;
        }
    }
} // namespace ega
