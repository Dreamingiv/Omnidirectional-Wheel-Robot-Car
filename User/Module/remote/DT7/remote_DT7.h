#ifndef MODULE_REMOTE_DT7_H_
#define MODULE_REMOTE_DT7_H_

#include <optional>
#include "daemon.h"
#include "driver_usart.h"


/**
 * CubeMX设置：
 * UART5，异步模式
 * Baud Rate:		100000Bit/s
 * Word Length:	9Bits(including Parity)
 * Parity:			Even
 * stop Bits:		1
 *
 * 本驱动为单例设计，不支持多实例（因为UART不支持对同一个外设多次初始化）
 * DT7遥控器发送频率为大约14ms一次
 */

// todo 后续取消全局单例设定，改为每个实例单独管理
namespace ega
{
    class DT7
    {
        /* ====================== 1. 编译期常量 ====================== */
    public:
        // 摇杆偏置
        static constexpr int16_t RC_CH_VALUE_MIN = 364;
        static constexpr int16_t RC_CH_VALUE_OFFSET = 1024;
        static constexpr int16_t RC_CH_VALUE_MAX = 1684;

    private:
        static constexpr uint16_t DBUS_FRAME_SIZE = 18;
        /* ====================== 2. 内部类型定义 ====================== */
    public:
        enum class DebugMode : uint8_t {
            RC = 0,
            MOUSE = 1,
            KEY = 2,
            ALL = 3
        };
        enum class RCSwitchState : uint8_t
        {
            UP = 1,
            DOWN = 2,
            MIDDLE = 3,
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
        // 键盘按键索引
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
        // 键盘数据
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
        // 总接收数据帧
        struct RCData
        {
            struct
            {
                int16_t rocker_r_h; // 右水平（horizontal）
                int16_t rocker_r_v; // 右竖直（vertical）
                int16_t rocker_l_h; // 左水平
                int16_t rocker_l_v; // 左竖直
                int16_t dial; // 拨轮
                RCSwitchState sw_left;
                RCSwitchState sw_right;
            } rc;
            struct
            {
                int16_t x;
                int16_t y;
                int16_t z; // 鼠标滚轮
                uint8_t press_l[2]; // 0: Pressed, 1: Clicked
                uint8_t press_r[2];
            } mouse;
            KeyData keys[2]; // 0: Pressed, 1: Clicked
        };
        /* ====================== 3. 方法接口 ====================== */
    public:
        DT7(const DT7&) = delete; // 禁止拷贝
        DT7& operator=(const DT7&) = delete;

        explicit DT7(const UARTInstance::Config& config);
        DT7();

        void start();
        void stop();
        void debug(DebugMode mode);
        // Debug 有四个 mode
        // RC：右摇杆水平、右摇杆竖直、左摇杆水平、左摇杆竖直、拨轮、左拨杆、右拨杆
        // MOUSE：鼠标x、鼠标y、鼠标滚轮、左键按下、左键点击、右键按下、右键点击
        // KEY：(W S A D _ Shift Ctrl Q E _ R F G Z _ X C V B) 按下、点击

        /* ====================== 4. API ====================== */
    public:
        // 完整数据帧
        const RCData& getData() const;
        // 遥控器通道
        int16_t getRockerValue(RCRockerIndex ch) const;
        RCSwitchState getLeftSwitch() const;
        RCSwitchState getRightSwitch() const;
        // 鼠标通道
        uint8_t getMouseValue(MouseIndex ch) const;
        bool isMouseLeftPressed() const;
        bool isMouseRightPressed() const;
        bool isMouseLeftClicked();
        bool isMouseRightClicked();
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

        RCData rc_current_ {};
        RCData rc_last_ {};

        void DBUSCallback(uint8_t* data, uint16_t size);
        void parseFrame(const uint8_t* data, uint16_t size); // 解析遥控器协议
        static RCSwitchState convertSwitchState(int16_t val);
        bool checkRockerValid() const;
        void offlineCallback();
    };
} // namespace ega
#endif // MODULE_REMOTE_DT7_H_
