//
// Created by An on 2025/10/14.
//
#include "imu_task.h"
#include "cmsis_os.h"

#include "imu.h"
#include "bmi088.h"

#include "logger.h"

[[noreturn]] void IMUTask(void* pv)
{
    using namespace ega;
    constexpr TickType_t period = pdMS_TO_TICKS(1);
    IMU::Config imu_config{
        .type = IMU::Type::BMI088,
        .filter_type = IMU::FilterType::EKF,
        .sample_time = 0.001,
    };
    //std::unique_ptr<IMU> bmi088 = IMU::create(imu_config);
    BMI088 bmi_088(imu_config);

    for (;;)
    {
        TickType_t current_time = xTaskGetTickCount();

        IMU::updateAll();
        // auto q = bmi_088.getQuaternion();
        // logger_printf("%f, %f, %f, %f\n", q.w, q.x, q.y, q.z);
        // auto e = bmi_088.getEulerAngles();
        // logger_printf("%f,%f,%f\r\n",e.total_pitch_,e.total_roll_,e.total_yaw_);

        vTaskDelayUntil(&current_time, period / portTICK_RATE_MS);
    }
}
