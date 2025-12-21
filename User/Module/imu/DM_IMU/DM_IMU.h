// //
// // Created by zhangzhiwen on 25-11-11.
// //
//
// #ifndef DM_IMU_H
// #define DM_IMU_H
//
// #include "main.h"
// #include "imu.h"
//
// //todo 暂时用不到，后面再规范style
//
// class DM_IMU : public IMU
// {
// public:
//     // 解包使用的数据结构
//     union data32
//     {
//         int32_t  L;
//         uint8_t  u8[4];
//         uint16_t u16[2];
//         uint32_t u32;
//         float    F;
//     };
//
//     explicit DM_IMU(const IMU::Config& config);
//     ~DM_IMU() override = default;
//
//     void init();
//     void readData() override;
//     void calibrate() override { ; }; // 校准在上位机完成
//     void calculate() override;
//
// private:
//     void decodeData(const uint8_t* data, uint8_t size);
//     void decodeAccel(const uint8_t* data);
//     void decodeGyro(const uint8_t* data);
//     void decodeQuaternion(const uint8_t* data);
//     void decodeEulerangle(const uint8_t* data);
//
//     std::optional<CANInstance> can_instance_;
//     const float                sample_time_ = 0.001;
//     uint16_t id_;
//     QuaternionF32              quaternion_raw_;     // 当前姿态（四元数）
//     EulerAngle                 euler_angle_raw_{}; // 当前姿态（欧拉角）
//
//     static float uint_to_float(int x_int, float x_min, float x_max, int bits)
//     {
//         /* converts unsigned int to float, given range and number of bits */
//         const float span   = x_max - x_min;
//         const float offset = x_min;
//         return static_cast<float>(x_int) * span
//             / static_cast<float>((1 << bits) - 1) + offset;
//     }
//
//     static constexpr float ACCEL_CAN_MAX  = (58.8f);
//     static constexpr float ACCEL_CAN_MIN  = (-58.8f);
//     static constexpr float GYRO_CAN_MAX   = (34.88f);
//     static constexpr float GYRO_CAN_MIN   = (-34.88f);
//     static constexpr float PITCH_CAN_MAX  = (90.0f);
//     static constexpr float PITCH_CAN_MIN  = (-90.0f);
//     static constexpr float ROLL_CAN_MAX   = (180.0f);
//     static constexpr float ROLL_CAN_MIN   = (-180.0f);
//     static constexpr float YAW_CAN_MAX    = (180.0f);
//     static constexpr float YAW_CAN_MIN    = (-180.0f);
//     static constexpr float TEMP_MIN       = (0.0f);
//     static constexpr float TEMP_MAX       = (60.0f);
//     static constexpr float Quaternion_MIN = (-1.0f);
//     static constexpr float Quaternion_MAX = (1.0f);
// };
//
// #endif //DM_IMU_H
