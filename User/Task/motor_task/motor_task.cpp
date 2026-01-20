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
    constexpr TickType_t period = pdMS_TO_TICKS(2); //周期500hz。有需要可以再提

    bool robot_is_protect_mode = true;

    const RobotManager& robot_manager = RobotManager::getInstance();

    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        //获取robotmanager的工作状态
        switch (robot_manager.getWorkState())
        {
        case RobotManager::WorkState::Work: // 如果为正常工作模式，将所有电机使能标志位设置为true，仅设置一次。
            //如果上一帧为保护模式，这一帧为正常工作模式，需要使能所有电机。
            if (robot_is_protect_mode == true)
            {
                Motor::enableAll();
            }
            robot_is_protect_mode = false;
            break;

        case RobotManager::WorkState::Protect: // 如果为保护模式，将所有电机使能标志位设置为false，反复执行。
        default: // 其余非法情况等同于保护模式
            Motor::disableAll();
            robot_is_protect_mode = true;
            break;
        }

        // 根据标志位同步所有电机的使能状态
        Motor::syncEnableStateAll();
        // 执行控制循环
        Motor::controlAll();

        vTaskDelayUntil(&last_wake_time, period);
    }
}
