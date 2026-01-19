//
// Created by An on 2025/10/24.
//
#include "robot_task.h"
#include "cmsis_os.h"
#include "logger.h"
#include "robot_manager.h"
#include "user_configs.h"

[[noreturn]] void RobotTask(void* pv)
{
    using namespace ega;
    constexpr TickType_t period = pdMS_TO_TICKS(5);//周期200hz


    //初始化机器人管理器
    RobotManager::init(configs::robot_manager_config);

    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        RobotManager::update();

        vTaskDelayUntil(&last_wake_time, period);
    }
}
