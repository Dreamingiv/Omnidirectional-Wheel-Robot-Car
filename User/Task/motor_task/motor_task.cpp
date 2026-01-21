//
// Created by An on 2025/10/10.
//
#include "motor_task.h"
#include "cmsis_os.h"
#include "motor.h"

#include "robot_manager.h"

[[noreturn]] void MotorTask(void* pv)
{
    using namespace ega;
    constexpr TickType_t period = pdMS_TO_TICKS(2);//周期500hz。有需要可以再提

    const RobotManager& robot_manager = RobotManager::getInstance();

    for (;;)
    {
        TickType_t current_time = xTaskGetTickCount();

        switch (robot_manager.getWorkState())
        {
        case RobotManager::WorkState::Work: // 正常工作模式
            Motor::enableAll();
            break;

        case RobotManager::WorkState::Protect: // 保护模式
        default: // 其余非法情况等同于保护模式
            // Motor::disableAll();
            Motor::enableAll();
            break;
        }
        // 执行控制循环
        Motor::controlAll();

        vTaskDelayUntil(&current_time, period / portTICK_RATE_MS);
    }
}
