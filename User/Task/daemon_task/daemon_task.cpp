//
// Created by An on 2025/10/3.
//
#include "daemon_task.h"
#include "cmsis_os.h"
#include "daemon.h"

[[noreturn]] void DaemonTask(void* pv)
{
    using namespace ega;
    constexpr TickType_t period = pdMS_TO_TICKS(10);

    for (;;)
    {
        TickType_t current_time = xTaskGetTickCount();

        Daemon::updateAll(); //所有已注册的守护进程tick一次

        vTaskDelayUntil(&current_time, period / portTICK_RATE_MS);
    }
}
