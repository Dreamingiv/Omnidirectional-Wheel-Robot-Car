//
// Created by An on 2025/10/14.
//
#include "imu_task.h"
#include "cmsis_os.h"

#include "bmi088.h"
#include "imu.h"

#include "logger.h"

[[noreturn]] void IMUTask(void* pv)
{
    using namespace ega;
    constexpr TickType_t period = pdMS_TO_TICKS(1);

    BMI088::Config config{
        .filter_type = IMU::FilterType::EKF,
        .sample_time = 0.001,
        .x_direction = ega::IMU::Xdirection::FRONT,
        .z_direction = ega::IMU::Zdirection::UP,
    };
    BMI088 bmi_088(config);


    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        IMU::updateAll();
        // auto q = bmi_088.getQuaternion();
        // logger_printf("%f, %f, %f, %f\n", q.w, q.x, q.y, q.z);
        // auto e = bmi_088.getEulerAngles();
        // logger_printf("%f,%f,%f\r\n",e.total_pitch_,e.total_roll_,e.total_yaw_);

        vTaskDelayUntil(&last_wake_time, period);
    }
}
