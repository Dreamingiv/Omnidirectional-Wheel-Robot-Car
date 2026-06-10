//
// Created by zhangzhiwen on 25-11-7.
//
#include "imu.h"
#include "bmi088.h"

namespace ega
{
    // std::unique_ptr<IMU> IMU::create(const IMU::Config& config)
    // {
    //     std::unique_ptr<IMU> res = nullptr;
    //     switch (config.type)
    //     {
    //     case Type::BMI088:
    //         return std::make_unique<BMI088>(config);
    //
    //     default:
    //         return nullptr;
    //     }
    // }

    void inline IMU::update()
    {
        readData();
        calculate();
    }

    void IMU::updateAll()
    {
        for (uint8_t i = 0; i < index_; i++)
        {
            // instances_[i]->readData();
            // instances_[i]->calculate();
            instances_[i]->update();
        }
    }

    bool IMU::getPrimaryEulerAngle(EulerAngle& euler)
    {
        if (index_ <= 0 || instances_[0] == nullptr)
        {
            return false;
        }

        euler = instances_[0]->getEulerAngles();
        return true;
    }

    bool IMU::getPrimaryYaw(float& yaw)
    {
        EulerAngle euler{};
        if (!getPrimaryEulerAngle(euler))
        {
            return false;
        }

        yaw = euler.total_yaw_;
        return true;
    }

    bool IMU::getPrimaryGyroZ(float& gyro_z)
    {
        if (index_ <= 0 || instances_[0] == nullptr)
        {
            return false;
        }

        auto gyro = instances_[0]->getGyro();
        gyro_z = gyro(2, 0);
        return true;
    }

    bool IMU::getPrimaryAccel(Matrixf<3, 1>& accel)
    {
        if (index_ <= 0 || instances_[0] == nullptr)
        {
            return false;
        }

        accel = instances_[0]->getAccel();
        return true;
    }

    void IMU::clear()
    {
        quaternion_.w = 1;
        quaternion_.x = 0;
        quaternion_.y = 0;
        quaternion_.z = 0;
        memset(&euler_angle_, 0, sizeof(euler_angle_));
        gyro_data_.clear();
        accel_data_.clear();
        float quaternion[4] = {1, 0, 0, 0};
        // ekf_filter_->init(quaternion);
    }


}
