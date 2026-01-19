//
// Created by An on 2025/10/3.
//
#include "daemon_task.h"
#include "cmsis_os.h"
#include "daemon.h"

[[noreturn]] void DaemonTask(void* pv)
{
    using namespace ega;
    constexpr TickType_t period = pdMS_TO_TICKS(Daemon::PERIOD_MS);


    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        Daemon::updateAll(); //所有已注册的守护进程tick一次

        vTaskDelayUntil(&last_wake_time, period);
    }
}
