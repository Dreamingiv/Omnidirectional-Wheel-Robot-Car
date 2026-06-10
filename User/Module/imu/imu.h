//
// Created by zhangzhiwen on 25-10-12.
//

#ifndef MODULE_IMU_IMU_H_
#define MODULE_IMU_IMU_H_

#include <cstdint>
#include <optional>
#include "eulerAngle_and_quaternion.h"
#include "matrix.h"
#include "quaternionEKF.h"

#include "driver_can.h"
#include "driver_spi.h"
#include "driver_usart.h"
#include "madgwick_filter.h"
#include "quaternionEKF.h"

namespace ega
{
    // =============================================================
    // 通用 IMU 基类 —— 可派生出不同具体型号 (如 BMI088 / MPU6050)
    // =============================================================
    class IMU
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        static constexpr size_t MX_INS_NUM = 3;
        /* ====================== 2. 内部类型定义 ====================== */
    public:
        enum class CalibrationStatus
        {
            ONLINE, // 在线校准
            OFFLINE, // 使用固定值
        };

        enum class FilterType
        {
            NO_FILTER,
            EKF,
            MADGWICK,
        };

        enum class Xdirection
        {
            FRONT,
            LEFT,
            BACK,
            RIGHT,
        };

        enum class Zdirection
        {
            UP,
            DOWN,
        };


        /* ====================== 3. 静态接口 ====================== */
    public:
        // static std::unique_ptr<IMU> create(const Config& config); // 工厂函数，供上层调用
        static void
        updateAll(); // 静态循环，调用所有注册过的imu并更新数据，方便放在task里面循环跑（是否有实现的必要？）

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        static bool getPrimaryEulerAngle(EulerAngle& euler);
        static bool getPrimaryYaw(float& yaw);
        static bool getPrimaryGyroZ(float& gyro_z);
        static bool getPrimaryAccel(Matrixf<3, 1>& accel);

        IMU()
        {
            // 加入实例列表
            if (index_ < MX_INS_NUM)
            {
                instances_[index_++] = this;
            }
        }

        virtual ~IMU()
        {
            for (int i = 0; i < index_; ++i)
            {
                if (instances_[i] == this)
                {
                    // 用最后一个元素填补空洞
                    instances_[i] = instances_[index_ - 1];
                    instances_[index_ - 1] = nullptr;
                    --index_;
                    break;
                }
            }
        }

        /* ====================== 5. 公共接口 ====================== */
    public:
        virtual void readData() = 0; // 更新原始数据（加速度、角速度）
        virtual void calculate() = 0; // readData后调用，用于融合或解算姿态
        virtual void calibrate() = 0; // 可选：零偏/校准
        void clear();

        void inline update(); // 数据更新接口

        // 获取接口
        [[nodiscard]] const QuaternionF32& getQuaternion() const { return quaternion_; }
        [[nodiscard]] const EulerAngle& getEulerAngles() const { return euler_angle_; }
        [[nodiscard]] const Matrixf<3, 1>& getGyro() const { return gyro_data_; }
        [[nodiscard]] const Matrixf<3, 1>& getAccel() const { return accel_data_; }

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        static inline std::array<IMU*, MX_INS_NUM> instances_;
        static inline int16_t index_;

    protected:
        FilterType filter_type_ = FilterType::EKF;
        std::optional<QuaternionEKF> ekf_filter_;
        std::optional<MadgwickFilter> madgwick_filter_;

        float sample_time_ = 0.01; // 采样时间
        QuaternionF32 quaternion_; // 当前姿态（四元数）
        EulerAngle euler_angle_{}; // 当前姿态（欧拉角）
        Matrixf<3, 1> gyro_data_; // 角速度 (rad/s)
        Matrixf<3, 1> accel_data_; // 加速度 (m/s²)
        
        Matrixf<3, 3> rotation_; // 旋转矩阵
    };
} // namespace ega


#endif // MODULE_IMU_IMU_H_
