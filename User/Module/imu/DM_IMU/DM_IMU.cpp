// //
// // Created by zhangzhiwen on 25-11-11.
// //
//
// #include "DM_IMU.h"
//
// #include "driver_dwt.h"
// #include "FreeRTOS.h"
// #include "task.h"
// #include "logger.h"
//
// DM_IMU::DM_IMU(const Config& config)
//     : IMU(),id_(config.dm_imu_config.id)
// {
//     //初始化基类参数
//     initParams(config);
//
//     // 初始化can外设
//     CANInstance::Config cfg = {
//         .handle = config.dm_imu_config.can_config.handle,
//         .tx_id = 0x15,
//         .tx_callback = nullptr,
//         .rx_id = 0x07,
//         .rx_callback = [this](const uint8_t* data, uint8_t len) -> void
//         {
//             this->decodeData(data,len);
//         },
//     };
//     can_instance_.emplace(cfg);
//
//     //选择姿态融合滤波器类型
//     switch (filter_type_)
//     {
//     case FilterType::EKF:
//         {
//             ekf_filter_.emplace(QuaternionF32(), 10, 0.001, 1e+6, 1, 0, 0.005);
//         }
//     case FilterType::MADGWICK:
//         {
//             madgwick_filter_.emplace();
//         }
//     default: ;
//     }
//
//     init();
// }
//
// void DM_IMU::init()
// {
//     ;
// }
//
// void DM_IMU::readData()
// {
//     CANInstance::msg_t can_msg = {
//         .data = {0xCC,0x01,0x00,0xDD,0,0,0,0},
//         .length = 8,
//     };
//     can_instance_->send(can_msg);
//     DWTInstance::delay(0.00018);
//
//     can_msg.data[1] = 0x02;
//     can_instance_->send(can_msg);
//     DWTInstance::delay(0.00018);
//
//     can_msg.data[1] = 0x03;
//     can_instance_->send(can_msg);
//     DWTInstance::delay(0.00018);
//
//     can_msg.data[1] = 0x04;
//     can_instance_->send(can_msg);
// }
//
//
// void DM_IMU::calculate()
// {
//     switch (filter_type_)
//     {
//     case FilterType::EKF:
//         {
//         ekf_filter_->update(accel_data_, gyro_data_, sample_time_);
//         quaternion_ = ekf_filter_->getQuaternion();
//         euler_angle_ = ekf_filter_->getEulerAngle();
//             break;
//         }
//     case FilterType::MADGWICK:
//         {
//         madgwick_filter_->updateIMU(accel_data_, gyro_data_, quaternion_);
//         auto e = quaternion_.toEuler();
//         euler_angle_.roll_ = e.roll_;
//         euler_angle_.pitch_ = e.pitch_;
//         euler_angle_.yaw_ = e.yaw_;
//         euler_angle_.updateEulerAngle();
//             break;
//         }
//     case FilterType::NO_FILTER:
//         {
//             quaternion_        = quaternion_raw_;
//             euler_angle_.roll_  = euler_angle_raw_.roll_;
//             euler_angle_.pitch_ = euler_angle_raw_.pitch_;
//             euler_angle_.yaw_   = euler_angle_raw_.yaw_;
//             euler_angle_.updateEulerAngle();
//             break;
//         }
//     }
// }
//
//
// void DM_IMU::decodeData(const uint8_t* data, uint8_t size)
// {
//     switch (data[0])
//     {
//     case 1:
//         {
//             decodeAccel(data);
//             break;
//         }
//     case 2:
//         {
//             decodeGyro(data);
//             break;
//         }
//     case 3:
//         {
//             decodeEulerangle(data);
//             break;
//         }
//     case 4:
//         {
//             decodeQuaternion(data);
//             break;
//         }
//     default: ;
//     }
// }
//
// void DM_IMU::decodeAccel(const uint8_t* data)
// {
//     uint16_t tmp;
//     tmp = (data[3] << 8) | data[2];
//
//     accel_data_(0, 0) = uint_to_float(tmp, ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
//
//     tmp = (data[5] << 8) | data[4];
//
//     accel_data_(1, 0) = uint_to_float(tmp, ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
//
//     tmp = (data[7] << 8) | data[6];
//
//     accel_data_(2, 0) = uint_to_float(tmp, ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
// }
//
// void DM_IMU::decodeGyro(const uint8_t* data)
// {
//     uint16_t tmp;
//     tmp = (data[3] << 8) | data[2];
//
//     gyro_data_(0, 0) = uint_to_float(tmp, GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
//
//     tmp = (data[5] << 8) | data[4];
//
//     gyro_data_(1, 0) = uint_to_float(tmp, GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
//
//     tmp = (data[7] << 8) | data[6];
//
//     gyro_data_(2, 0) = uint_to_float(tmp, GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
// }
//
// void DM_IMU::decodeEulerangle(const uint8_t* data)
// {
//     uint16_t tmp;
//     tmp = (data[3] << 8) | data[2];
//
//     euler_angle_raw_.pitch_ = uint_to_float(tmp, PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
//
//     tmp = (data[5] << 8) | data[4];
//
//     euler_angle_raw_.yaw_ = uint_to_float(tmp, YAW_CAN_MIN, YAW_CAN_MAX, 16);
//
//     tmp = (data[7] << 8) | data[6];
//
//     euler_angle_raw_.roll_ = uint_to_float(tmp, ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
// }
//
// void DM_IMU::decodeQuaternion(const uint8_t* data)
// {
//     uint16_t tmp;
//     tmp = (data[1] << 6) | ((data[2] & 0xF8) >> 2);
//
//     quaternion_raw_.z = uint_to_float(tmp, Quaternion_MAX, Quaternion_MIN, 14);
//
//     tmp = ((data[2] & 0x03) << 12) | (data[3] << 4) | ((data[4] & 0xF0) >> 4);
//
//     quaternion_raw_.z = uint_to_float(tmp, Quaternion_MAX, Quaternion_MIN, 14);
//
//     tmp = ((data[4] & 0x0F) << 10) | (data[5] << 2) | ((data[6] & 0xC0) >> 6);
//
//     quaternion_raw_.z = uint_to_float(tmp, Quaternion_MAX, Quaternion_MIN, 14);
//
//     tmp = ((data[6] & 0x3F) << 8) | data[7];
//
//     quaternion_raw_.z = uint_to_float(tmp, Quaternion_MAX, Quaternion_MIN, 14);
// }
