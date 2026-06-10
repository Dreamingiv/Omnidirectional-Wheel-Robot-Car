//
// Created by An on 2025/11/21.
//
#include "robot_manager.h"

#include <type_traits>
#include "logger.h"

namespace ega
{
    namespace
    {
        constexpr int16_t DT7_ROCKER_DEADBAND = 20;

        float limitFloat(float value, float min, float max)
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

        float normalizeDT7Rocker(int16_t value)
        {
            if (value > -DT7_ROCKER_DEADBAND && value < DT7_ROCKER_DEADBAND)
            {
                return 0.0f;
            }

            constexpr float max_abs =
                static_cast<float>(DT7::RC_CH_VALUE_MAX - DT7::RC_CH_VALUE_OFFSET);
            return limitFloat(static_cast<float>(value) / max_abs, -1.0f, 1.0f);
        }
    }

    bool RobotManager::init(const Config& config)
    {
        auto& ins = getInstance();

        if (ins.inited_)
            return true;
        // 记录配置
        ins.board_type_ = config.board_type;
        ins.remote_type_ = config.remote_type;

        // VT13 / DT7 / FS_I6X 的构造与 start
        switch (config.remote_type)
        {
        case RemoteType::VT13:
            ins.remote_.emplace(std::in_place_type<VT13>);
            break;
        case RemoteType::DT7:
            ins.remote_.emplace(std::in_place_type<DT7>);
            break;
        case RemoteType::FS_I6X:
            ins.remote_.emplace(std::in_place_type<FS_I6X>);
            break;
        case RemoteType::None:
        default:
            // 遥控器类型为None或者其他类型时，什么都不做（建议上层保证只传入 VT13 / DT7 / FS_I6X）
            return false;
        }
        std::visit([](auto& r) { r.start(); }, ins.remote_.value());


        ins.inited_ = true;
        return true;
    }

    void RobotManager::update()
    {
        /* =================== 基本防护 =================*/
        auto& ins = getInstance();

        if (!ins.inited_ || !ins.remote_)
        {
            ins.command_ = {};
            ins.work_state_ = WorkState::Protect;
            return;
        }

        /* =================== 解析遥控器指令&保护模式判断 =================*/
        const bool remote_is_online = isRemoteOnline(ins.remote_.value());

        // todo 如果有其它保护模式判断条件，请在这里修改

        if (remote_is_online)
        {
            // 根据遥控器类型执行不同的 update 逻辑（例如映射通道到控制指令、边沿触发等）
            updateRemoteLogic(ins.remote_.value());
            ins.work_state_ = WorkState::Work;
        }
        else
        {
            ins.command_ = {};
            ins.work_state_ = WorkState::Protect;
        }

    }

    bool RobotManager::isRemoteOnline(const RemoteVariant& r)
    {
        // std::visit 会根据 r 的类型自动调用对应的 isOnline 方法，如果r没有isOnline方法，会编译错误
        return std::visit([](const auto& remote) { return remote.isOnline(); }, r);
    }

    void RobotManager::updateRemoteLogic(RemoteVariant& r)
    {
        // 默认：目前遥控器驱动在串口回调里完成解析与状态更新，这里更多用于“上层映射逻辑”
        // 你可以按遥控器类型分别写不同逻辑（例如：某个拨杆=急停、某个开关=模式切换等）
        std::visit([&](auto& remote)
        {
            using T = std::decay_t<decltype(remote)>;
            if constexpr (std::is_same_v<T, VT13>)
            {
                // TODO: VT13 特有的映射逻辑
                updateRemote_VT13(remote);
                (void)remote;
            }
            else if constexpr (std::is_same_v<T, DT7>)
            {
                // TODO: DT7 特有的映射逻辑
                updateRemote_DT7(remote);
                (void)remote;
            }
            else if constexpr (std::is_same_v<T, FS_I6X>)
            {
                // TODO: FS-I6X 特有的映射逻辑
                updateRemote_FS_I6X(remote);
                (void)remote;
            }
            else
            {
                // 如果r为其它类型，什么都不做（建议上层保证只传入 VT13 / DT7 / FS_I6X）
            }
        }, r);
    }

    void RobotManager::updateRemote_VT13(VT13& vt13)
    {
        // TODO: VT13 特有的映射逻辑
        (void)vt13;
    }
     void RobotManager::updateRemote_DT7(DT7& dt7)
    {
        auto& chassis_cmd = getInstance().command_.chassis;
        chassis_cmd = {};

        if (!dt7.isFrameValid())
        {
            return;
        }

        chassis_cmd.enable =
            dt7.getRightSwitch() == DT7::RCSwitchState::MIDDLE &&
            dt7.getLeftSwitch() == DT7::RCSwitchState::MIDDLE;
        if (!chassis_cmd.enable)
        {
            return;
        }

        chassis_cmd.vx = normalizeDT7Rocker(dt7.getRockerValue(DT7::RCRockerIndex::RIGHT_VERTICAL));
        chassis_cmd.vy = normalizeDT7Rocker(dt7.getRockerValue(DT7::RCRockerIndex::RIGHT_HORIZONTAL));
        chassis_cmd.wz = normalizeDT7Rocker(dt7.getRockerValue(DT7::RCRockerIndex::LEFT_HORIZONTAL));
    }
    void RobotManager::updateRemote_FS_I6X(FS_I6X& fs_i6x)
    {
        // TODO: FS-I6X 特有的映射逻辑
        (void)fs_i6x;
    }


}
