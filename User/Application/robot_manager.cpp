//
// Created by An on 2025/11/21.
//
#include "robot_manager.h"
#include "remote_FS_I6X.h"
#include "remote_DT7.h"
#include "remote_VT13.h"

namespace ega
{
    bool RobotManager::init(const Config& config)
    {

        auto& ins = getInstance();

        ins.remote_type_=config.remote_type;

        switch (ins.remote_type_)
        {
        case RemoteType::VT13:
            VT13::init();
            break;
        case RemoteType::DT7:
            DT7::init();
            break;
        case RemoteType::FS_I6X:
            FS_I6X::init();
            break;
        default:;
        }

        ins.inited_ = true;
        return true;
    }

    void RobotManager::update()
    {
        auto& ins = getInstance();

        bool remote_is_online = false;
        switch (ins.remote_type_)
        {
        case RemoteType::VT13:
            remote_is_online = VT13::getInstance().isOnline();
            break;
        case RemoteType::DT7:
            remote_is_online = DT7::getInstance().isOnline();
            break;
        case RemoteType::FS_I6X:
            remote_is_online = FS_I6X::getInstance().isOnline();
            break;
        default:;
        }

        // todo 如果有其它保护模式判断条件，请在这里修改

        if (remote_is_online)
        {
            ins.work_state_ = WorkState::Work;
        }
        else
        {
            ins.work_state_ = WorkState::Protect;
        }

    }
}
