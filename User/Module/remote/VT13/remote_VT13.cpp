#include "remote_VT13.h"
#include "logger.h"

namespace ega
{
    // 反射型 CCITT-16（等价表驱动），init=0xFFFF，xorout=0x0000
    static uint16_t Crc16Reflected(const uint8_t* p, uint16_t n)
    {
        uint16_t crc = 0xFFFF;
        while (n--)
        {
            crc ^= *p++;
            for (int i = 0; i < 8; ++i)
                crc = (crc & 0x0001) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
        }
        return crc; // 不做 ~crc
    }
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

    VT13::VT13(const UARTInstance::Config& config)
    {
        uart_instance_.emplace(config);
        daemon_.emplace(Daemon::Config{.reload_time_ms = 100, // 超时100ms认为遥控器丢失
                                       .init_time_ms = 10, // 可选：上电初始延时，按需调整
                                       .callback = [this]() { this->offlineCallback(); }});
        uart_instance_->startReceive();
    }

    VT13::VT13() :
        VT13(UARTInstance::Config{
            .handle = &huart6,
            .tx_type = UARTInstance::DMA,
            .rx_type = UARTInstance::DMA_IDLE,
            .rx_callback = [this](uint8_t* data, uint16_t size) { this->DBUSCallback(data, size); },
            .rx_size = 32,
        })
    { /* 委托构造函数，不需要额外代码 */
    }


    void VT13::start() { this->uart_instance_->startReceive(); }

    void VT13::stop()
    {
        this->is_online_ = false;
        this->is_frame_valid_ = false;
        this->uart_instance_->stopReceive();
    }

    // ================ RX 路由 ================
    void VT13::DBUSCallback(uint8_t* data, uint16_t size) { this->parseFrame(data, size); }

    VT13::RCSwitchState VT13::convertSwitchState(int16_t val)
    {
        switch (val)
        {
        case 0:
            return RCSwitchState::C;
        case 1:
            return RCSwitchState::N;
        case 2:
            return RCSwitchState::S;
        default:
            return RCSwitchState::N;
        }
    }

    // ================ 解析主逻辑 ================
    void VT13::parseFrame(const uint8_t* data, uint16_t size)
    {
        if (size != DBUS_FRAME_SIZE)
        {
            is_frame_valid_ = false;
            return;
        }

        // Header 校验（0..7=0xA9, 8..15=0x53）
        if (data[0] != 0xA9 || data[1] != 0x53)
        {
            is_frame_valid_ = false;
            return;
        }

        const auto rx_crc = static_cast<uint16_t>(rd_bits(data, 152, 16));
        const uint16_t calc = Crc16Reflected(data, 19);
        if (rx_crc != calc)
        {
            is_frame_valid_ = false;
            return;
        }

        // 帧通过 -> 喂狗、在线
        daemon_->reload();
        is_online_ = true;

        // ------- 根据手册的 bit 布局读取域（偏移单位：bit） -------
        // ch0..ch3 每个 11bit，范围 364/1024/1684
        // ch0: 16, ch1:27, ch2:38, ch3:49
        auto ch0 = static_cast<int16_t>(rd_bits(data, 16, 11));
        auto ch1 = static_cast<int16_t>(rd_bits(data, 27, 11));
        auto ch2 = static_cast<int16_t>(rd_bits(data, 38, 11));
        auto ch3 = static_cast<int16_t>(rd_bits(data, 49, 11));

        // Mode switch(2bit, 60), Pause(1bit,62), Left btn(1bit,63), Right btn(1bit,64)
        auto mode_sw = static_cast<uint8_t>(rd_bits(data, 60, 2));
        auto pause = static_cast<uint8_t>(rd_bits(data, 62, 1));
        auto btn_l = static_cast<uint8_t>(rd_bits(data, 63, 1));
        auto btn_r = static_cast<uint8_t>(rd_bits(data, 64, 1));

        // Dial 11bit @65
        auto dial = static_cast<int16_t>(rd_bits(data, 65, 11));

        // Trigger 1bit @76
        auto trigger = static_cast<uint8_t>(rd_bits(data, 76, 1));

        // Mouse X/Y/Z：有符号 16bit（80/96/112）
        auto s16 = [](uint32_t u16) -> int16_t { return static_cast<int16_t>(u16 & 0xFFFFu); };
        int16_t mx = s16(rd_bits(data, 80, 16));
        int16_t my = s16(rd_bits(data, 96, 16));
        int16_t mz = s16(rd_bits(data, 112, 16));

        // Mouse buttons: left/right/middle 各 2bit（128/130/132）
        uint8_t mb_l = static_cast<uint8_t>(rd_bits(data, 128, 2)) & 0x1;
        uint8_t mb_r = static_cast<uint8_t>(rd_bits(data, 130, 2)) & 0x1;
        uint8_t mb_m = static_cast<uint8_t>(rd_bits(data, 132, 2)) & 0x1;

        // Keyboard 16bit @136：bit0..15 -> W,S,A,D,Shift,Ctrl,Q,E,R,F,G,Z,X,C,V,B
        auto k_raw = static_cast<uint16_t>(rd_bits(data, 136, 16));

        // ---------- 填充 rc_current_ ----------
        // 通道：保留“中心偏移为 0”的语义，便于上层复用（与 DT7 风格一致）
        rc_current_.rc.rocker_r_h = ch0 - RC_CH_VALUE_OFFSET;
        rc_current_.rc.rocker_r_v = ch1 - RC_CH_VALUE_OFFSET;
        rc_current_.rc.rocker_l_v = ch2 - RC_CH_VALUE_OFFSET;
        rc_current_.rc.rocker_l_h = ch3 - RC_CH_VALUE_OFFSET;
        rc_current_.rc.dial = dial - RC_CH_VALUE_OFFSET;

        rc_current_.rc.mode_switch = convertSwitchState(mode_sw);
        rc_current_.rc.pause = pause;
        rc_current_.rc.btn_left = btn_l;
        rc_current_.rc.btn_right = btn_r;
        rc_current_.rc.trigger = trigger;

        rc_current_.mouse.x = mx;
        rc_current_.mouse.y = my;
        rc_current_.mouse.z = mz;

        // 鼠标键：PRESS
        rc_current_.mouse.press_l[static_cast<uint8_t>(MouseState::PRESSED)] = mb_l;
        rc_current_.mouse.press_r[static_cast<uint8_t>(MouseState::PRESSED)] = mb_r;
        rc_current_.mouse.press_m[static_cast<uint8_t>(MouseState::PRESSED)] = mb_m;
        // 鼠标键：CLICK（上升沿）
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
        rc_current_.mouse.press_m[static_cast<uint8_t>(MouseState::CLICKED)] =
            (rc_current_.mouse.press_m[static_cast<uint8_t>(MouseState::PRESSED)] == 1 &&
             rc_last_.mouse.press_m[static_cast<uint8_t>(MouseState::PRESSED)] == 0)
            ? 1
            : 0;

        // 键盘：PRESS
        rc_current_.keys[static_cast<uint8_t>(KeyState::PRESSED)].raw = k_raw;
        // 键盘：CLICK = 本帧按下 & 上一帧未按
        rc_current_.keys[static_cast<uint8_t>(KeyState::CLICKED)].raw =
            rc_current_.keys[static_cast<uint8_t>(KeyState::PRESSED)].raw &
            (~rc_last_.keys[static_cast<uint8_t>(KeyState::PRESSED)].raw);

        // 有效帧标记
        is_frame_valid_ = true;
        // 保存历史（供下一帧做上升沿）
        rc_last_ = rc_current_;
    }
    const VT13::RCData& VT13::getData() const { return rc_current_; }
    int16_t VT13::getRockerValue(RCRockerIndex ch) const
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
    int16_t VT13::getDialValue() const { return rc_current_.rc.dial; }

    VT13::RCSwitchState VT13::getModeSwitch() const { return rc_current_.rc.mode_switch; }
    bool VT13::isPausePressed() const { return rc_current_.rc.pause; }
    bool VT13::isTriggerPressed() const { return rc_current_.rc.trigger; }
    bool VT13::isLeftBtnPressed() const { return rc_current_.rc.btn_left; }
    bool VT13::isRightBtnPressed() const { return rc_current_.rc.btn_right; }
    uint8_t VT13::getMouseValue(MouseIndex ch) const
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
    bool VT13::isMouseLeftPressed() const
    {
        return rc_current_.mouse.press_l[static_cast<uint8_t>(MouseState::PRESSED)] == 1;
    }
    bool VT13::isMouseRightPressed() const
    {
        return rc_current_.mouse.press_r[static_cast<uint8_t>(MouseState::PRESSED)] == 1;
    }
    bool VT13::isMouseMiddlePressed() const
    {
        return rc_current_.mouse.press_m[static_cast<uint8_t>(MouseState::PRESSED)] == 1;
    }
    bool VT13::isMouseLeftClicked()
    {
        const bool clicked = rc_current_.mouse.press_l[static_cast<uint8_t>(MouseState::CLICKED)] == 1;
        rc_current_.mouse.press_l[static_cast<uint8_t>(MouseState::CLICKED)] = 0;
        return clicked;
    }

    bool VT13::isMouseRightClicked()
    {
        const bool clicked = rc_current_.mouse.press_r[static_cast<uint8_t>(MouseState::CLICKED)] == 1;
        rc_current_.mouse.press_r[static_cast<uint8_t>(MouseState::CLICKED)] = 0;
        return clicked;
    }
    bool VT13::isMouseMiddleClicked()
    {
        const bool clicked = rc_current_.mouse.press_m[static_cast<uint8_t>(MouseState::CLICKED)] == 1;
        rc_current_.mouse.press_m[static_cast<uint8_t>(MouseState::CLICKED)] = 0;
        return clicked;
    }
    bool VT13::isKeyPressed(KeyIndex key) const
    {
        return rc_current_.keys[static_cast<uint8_t>(KeyState::PRESSED)].get(key) == 1;
    }

    bool VT13::isKeyPressedWithCtrl(KeyIndex key) const
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

    bool VT13::isKeyPressedWithShift(KeyIndex key) const
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

    bool VT13::isKeyClicked(KeyIndex key)
    {
        const bool clicked = rc_current_.keys[static_cast<uint8_t>(KeyState::CLICKED)].get(key) == 1;
        rc_current_.keys[static_cast<uint8_t>(KeyState::CLICKED)].set(key, false);
        return clicked;
    }

    bool VT13::isKeyClickedWithCtrl(KeyIndex key)
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

    bool VT13::isKeyClickedWithShift(KeyIndex key)
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

    bool VT13::isFrameValid() const { return is_frame_valid_; }
    bool VT13::isOnline() const { return is_online_; }

    // ================ Offline ================
    void VT13::offlineCallback()
    {
        is_online_ = false;
        rc_current_ = RCData{}; // 清空当前数据
        if (uart_instance_)
            uart_instance_->startReceive(); // 试图恢复接收
    }
} // namespace ega
