#pragma once

#include "robot_manager.h"

/**
 * @brief 用户配置文件
 * * 将用户可修改的一些参数通过编译期常量写在这里
 */

namespace ega::configs
{
    //机器人管理器配置。在这里选择遥控器类型和板子类型
    constexpr RobotManager::Config robot_manager_config = {
        .board_type = RobotManager::BoardType::OneBoard,
        .remote_type = RobotManager::RemoteType::DT7,
    };

}