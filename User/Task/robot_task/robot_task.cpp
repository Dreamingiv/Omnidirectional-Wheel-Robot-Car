//
// Created by An on 2025/10/24.
//
#include "robot_task.h"
#include "cmsis_os.h"
#include "logger.h"
#include "robot_manager.h"

[[noreturn]] void RobotTask(void* pv)
{
    using namespace ega;
    constexpr TickType_t period = pdMS_TO_TICKS(5);//周期200hz
    //初始化机器人管理器
    RobotManager::init({
        .remote_type = RobotManager::RemoteType::DT7
    });

    for (;;)
    {
        TickType_t current_time = xTaskGetTickCount();

        RobotManager::update();

        vTaskDelayUntil(&current_time, period / portTICK_RATE_MS);
    }
}
