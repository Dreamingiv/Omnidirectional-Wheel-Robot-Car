//
// Created by An on 2025/9/29.
//

#ifndef MODULE_REMOTE_VT13_H_
#define MODULE_REMOTE_VT13_H_

#include <cstdint>
#include <cstring>
#include <optional>
#include "daemon.h"
#include "driver_usart.h"

// 默认串口huart7
// 参考：VT13 手册 - 串口参数 921600/8N1/无校验/无流控；每 14ms 输出 21 字节帧；见"Remote Control Data" 章节
// 帧格式(位偏移)：Header0(0..7)=0xA9, Header1(8..15)=0x53, CRC 在 152..167；其余见 .cpp 解析表
namespace ega
{
    class VT13
    {
        /* ====================== 1. 编译期常量 ====================== */
    public:
        // 摇杆偏置
        static constexpr int16_t RC_CH_VALUE_MIN = 364;
        static constexpr int16_t RC_CH_VALUE_OFFSET = 1024;
        static constexpr int16_t RC_CH_VALUE_MAX = 1684;

    private:
        static constexpr uint16_t DBUS_FRAME_SIZE = 21;
        /* ====================== 2. 内部类型定义 ====================== */
    public:
        enum class RCSwitchState : uint8_t
        {
            C = 0,
            N = 1,
            S = 2,
        };
        enum class RCRockerIndex : uint8_t
        {
            RIGHT_HORIZONTAL = 0,
            RIGHT_VERTICAL = 1,
            LEFT_HORIZONTAL = 2,
            LEFT_VERTICAL = 3,
        };
        enum class MouseState : uint8_t
        {
            PRESSED = 0,
            CLICKED = 1,
        };
        enum class MouseIndex : uint8_t
        {
            X = 0,
            Y = 1,
            Z = 2,
        };
        enum class KeyState : uint8_t
        {
            PRESSED = 0,
            CLICKED = 1,
        };
        enum class KeyIndex : uint16_t
        {
            W = 0,
            S,
            A,
            D,
            SHIFT,
            CTRL,
            Q,
            E,
            R,
            F,
            G,
            Z,
            X,
            C,
            V,
            B,
        };
        struct KeyData
        {
            uint16_t raw;

            // 获取特定位的状态
            uint8_t get(KeyIndex key) const { return (raw & (1U << static_cast<uint16_t>(key))) ? 1 : 0; }

            // 设置特定位的状态
            void set(KeyIndex key, bool value)
            {
                auto mask = static_cast<uint16_t>(1U << static_cast<uint16_t>(key));
                if (value)
                {
                    raw |= mask; // 将对应位置为 1
                }
                else
                {
                    raw &= ~mask; // 将对应位置为 0
                }
            }
        };
        struct RCData
        {
            struct
            {
                int16_t rocker_r_h; // ch0 - 右水平
                int16_t rocker_r_v; // ch1 - 右竖直
                int16_t rocker_l_v; // ch2 - 左竖直
                int16_t rocker_l_h; // ch3 - 左水平
                int16_t dial; // 拨轮（同样 11 位，范围同上）
                RCSwitchState mode_switch; // 0/1/2 -> C/N/S
                uint8_t pause; // 0/1
                uint8_t btn_left; // 自定义左 0/1
                uint8_t btn_right; // 自定义右 0/1
                uint8_t trigger; // 扳机 0/1
            } rc;
            struct
            {
                int16_t x;
                int16_t y;
                int16_t z; // 滚轮速度（有正负）
                uint8_t press_l[2]; // [PRESS, CLICK]
                uint8_t press_r[2];
                uint8_t press_m[2];
            } mouse;
            KeyData keys[2]; // [PRESS, CLICK]
        };
        /* ====================== 3. 方法接口 ====================== */
    public:
        VT13(const VT13&) = delete;
        VT13& operator=(const VT13&) = delete;

        explicit VT13(const UARTInstance::Config& config);
        VT13();

        void start();
        void stop();

        void debug(); // TODO:
        /* ====================== 4. API ====================== */
    public:
        // 完整数据帧
        const RCData& getData() const;
        // 遥控器通道
        int16_t getRockerValue(RCRockerIndex ch) const;
        int16_t getDialValue() const;
        RCSwitchState getModeSwitch() const;
        bool isPausePressed() const;
        bool isTriggerPressed() const;
        bool isLeftBtnPressed() const;
        bool isRightBtnPressed() const;
        // 鼠标通道
        uint8_t getMouseValue(MouseIndex ch) const;
        bool isMouseLeftPressed() const;
        bool isMouseRightPressed() const;
        bool isMouseMiddlePressed() const;
        bool isMouseLeftClicked();
        bool isMouseRightClicked();
        bool isMouseMiddleClicked();
        // 键盘通道
        bool isKeyPressed(KeyIndex key) const;
        bool isKeyPressedWithCtrl(KeyIndex key) const;
        bool isKeyPressedWithShift(KeyIndex key) const;
        bool isKeyClicked(KeyIndex key);
        bool isKeyClickedWithCtrl(KeyIndex key);
        bool isKeyClickedWithShift(KeyIndex key);
        // 其他接口
        bool isFrameValid() const;
        bool isOnline() const;
        /* ====================== 5. 成员变量 ====================== */
    private:
        std::optional<UARTInstance> uart_instance_;
        std::optional<Daemon> daemon_;
        bool is_frame_valid_ = false;
        bool is_online_ = false;

        RCData rc_current_{};
        RCData rc_last_{};

        void DBUSCallback(uint8_t* data, uint16_t size);
        void parseFrame(const uint8_t* data, uint16_t size); // 解析遥控器协议
        static RCSwitchState convertSwitchState(int16_t val);
        void offlineCallback();
    };
} // namespace ega

#endif // MODULE_REMOTE_VT13_H_
