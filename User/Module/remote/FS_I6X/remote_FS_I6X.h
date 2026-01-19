//
// Created by zhangzhiwen on 25-10-29.
//

#ifndef MODULE_REMOTE_FSI6X_H
#define MODULE_REMOTE_FSI6X_H
#include <optional>
#include <variant>

#include "daemon.h"
#include "driver_usart.h"

namespace ega
{
    class FS_I6X
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        static constexpr uint16_t SBUS_FRAME_SIZE = 25u;
        static constexpr uint16_t RC_CH_VALUE_OFFSET = 1024;
        static constexpr uint16_t FS_I6X_FORMAT = 784; // 遥控器归一化数值
        /* ====================== 2. 内部类型定义 ====================== */
    public:
        enum class RCSwitchState : uint8_t
        {
            UP = 0,
            MIDDLE = 1,
            DOWN = 2,
        };
        enum class RCRockerIndex : uint8_t
        {
            RIGHT_HORIZONTAL = 0,
            RIGHT_VERTICAL = 1,
            LEFT_HORIZONTAL = 2,
            LEFT_VERTICAL = 3,
        };

        struct RCData
        {
            int16_t rocker_l_h = 0;
            int16_t rocker_l_v = 0;
            int16_t rocker_r_h = 0;
            int16_t rocker_r_v = 0;
            RCSwitchState switch_A = RCSwitchState::UP;
            RCSwitchState switch_B = RCSwitchState::UP;
            RCSwitchState switch_C = RCSwitchState::UP;
            RCSwitchState switch_D = RCSwitchState::UP;
            int16_t knob_l = 0;
            int16_t knob_r = 0;
        };
        /* ====================== 3. 方法接口 ====================== */
    public:
        FS_I6X(const FS_I6X&) = delete; // 禁止拷贝
        FS_I6X& operator=(const FS_I6X&) = delete;

        explicit FS_I6X(const UARTInstance::Config& config);
        FS_I6X();

        void start();
        void stop();

        void debug();
        /* ====================== 4. API ====================== */
    public:
        // 完整数据帧
        const RCData& getData() const;
        // 遥控器通道
        int16_t getRockerValue(RCRockerIndex ch) const;
        int16_t getLeftKnobValue() const;
        int16_t getRightKnobValue() const;
        RCSwitchState getSwitchA() const;
        RCSwitchState getSwitchB() const;
        RCSwitchState getSwitchC() const;
        RCSwitchState getSwitchD() const;
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

        void SBUSCallback(uint8_t* data, uint16_t size);
        void parseFrame(const uint8_t* data, uint16_t size); // 解析遥控器协议
        static RCSwitchState convertSwitchState(int16_t val);
        bool checkRockerValid() const;
        void offlineCallback();
    };
} // namespace ega

#endif // MODULE_REMOTE_FSI6X_H
