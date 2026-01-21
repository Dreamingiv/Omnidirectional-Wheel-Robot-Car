#include "DJIMotor.h"
#include <algorithm>

#include "logger.h"
#include "utils.h"

namespace ega
{
    // ===== 静态成员定义（配合你新 DJIMotor.h 的声明）=====
    // DJIMotor* DJIMotor::motors_[CAN_DEV_NUM][MAX_MOTORS] = {};
    // uint8_t DJIMotor::idx_[CAN_DEV_NUM] = {};

    /**** 构造函数、析构函数、接口函数等 ****/
    DJIMotor::DJIMotor(const Config& config) :
        motor_id_(config.motor_id)
    {
        // 初始化基类参数
        // initParams(config.type, config.direction);
        type_ = config.type;
        direction_ = config.direction;

        /* ===== 计算参数 ===== */
        uint32_t rx_id;
        switch (type_)
        {
        case Type::GM6020:
            output_max_abs_ = OUTPUT_MAX_GM6020;
            dji_reduction_radio_ = 1.0f; // 云台电机 1:1
            dji_torque_constant_ = TORQUE_CONSTANT_GM6020 * dji_reduction_radio_ / DEFAULT_REDUCTION_RADIO_GM6020;
            current_bit_2_A_ = CURRENT_BIT_2_A_GM6020;
            rx_id = 0x204 + config.motor_id;
            break;

        case Type::GM6020_CURRENT:
            output_max_abs_ = OUTPUT_MAX_GM6020_CURRENT;
            dji_reduction_radio_ = 1.0f; // 云台电机 1:1
            dji_torque_constant_ = TORQUE_CONSTANT_GM6020 * dji_reduction_radio_ / DEFAULT_REDUCTION_RADIO_GM6020;
            current_bit_2_A_ = CURRENT_BIT_2_A_GM6020;
            rx_id = 0x204 + config.motor_id;
            break;

        case Type::M3508:
            output_max_abs_ = OUTPUT_MAX_M3508;
            dji_reduction_radio_ = (config.reduction_radio >= 1e-6f)
                ? config.reduction_radio
                : DEFAULT_REDUCTION_RADIO_M3508;
            dji_torque_constant_ = TORQUE_CONSTANT_M3508 * dji_reduction_radio_ / DEFAULT_REDUCTION_RADIO_M3508;
            current_bit_2_A_ = CURRENT_BIT_2_A_M3508;
            rx_id = 0x200 + config.motor_id;
            break;

        case Type::M2006:
            output_max_abs_ = OUTPUT_MAX_M2006;
            dji_reduction_radio_ = (config.reduction_radio >= 1e-6f)
                ? config.reduction_radio
                : DEFAULT_REDUCTION_RADIO_M2006;
            dji_torque_constant_ = TORQUE_CONSTANT_M2006 * dji_reduction_radio_ / DEFAULT_REDUCTION_RADIO_M2006;
            current_bit_2_A_ = CURRENT_BIT_2_A_M2006;
            rx_id = 0x200 + config.motor_id;
            break;

        default:
            // 理论上不该进到这里
            output_max_abs_ = 0.0f;
            dji_reduction_radio_ = (config.reduction_radio >= 1e-6f) ? config.reduction_radio : 1.0f;
            current_bit_2_A_ = 1.0f;
            rx_id = 0x00;
            break;
        }

        /* ===== 注册 CAN 回调 ===== */
        CANInstance::Config cfg = {
            .handle = config.can_handle,
            .rx_id = rx_id,
            .rx_callback = [this](const uint8_t* data, uint8_t len) { this->parseData(data, len); }};

        can_.emplace(cfg);

        // 实例化守护进程
        Daemon::Config daemon_cfg = {
            .reload_time_ms = 200, // 超时200ms认为电机掉线
            .init_time_ms = 100,
            .callback = [this]() { this->offlineCallback(); }};

        daemon_.emplace(daemon_cfg);

        /* ===== 注册静态数组 ===== */
        uint8_t can_dev = can_->getHandleIndex();

        if (idx_[can_dev] < MAX_MOTORS)
            motors_[can_dev][idx_[can_dev]++] = this;
    }

    DJIMotor::~DJIMotor()
    {
        uint8_t can_dev = can_->getHandleIndex();
        auto it = std::find(motors_[can_dev].begin(), motors_[can_dev].begin() + idx_[can_dev], this);
        if (it != motors_[can_dev].begin() + idx_[can_dev])
        {
            size_t pos = std::distance(motors_[can_dev].begin(), it);
            motors_[can_dev][pos] = motors_[can_dev][idx_[can_dev] - 1]; // 用最后一个覆盖
            motors_[can_dev][idx_[can_dev] - 1] = nullptr;
            idx_[can_dev]--;
        }
    }

    constexpr uint8_t getMotorFrameIndex(const DJIMotor::Type type, const uint8_t id)
    {
        switch (type)
        {
        case DJIMotor::Type::M2006:
        case DJIMotor::Type::M3508:
            return (id <= 4) ? 0 : 1;

        case DJIMotor::Type::GM6020:
            return (id <= 4) ? 1 : 2;

        case DJIMotor::Type::GM6020_CURRENT:
            return (id <= 4) ? 3 : 4;

        default:
            return 0;
        }
    }

    void DJIMotor::sendCommandAll()
    {
        // 每个 CAN 控制器维护 5 组帧 (0x200 / 0x1FF / 0x2FF / 0x1FE / 0x2FE)
        constexpr int FRAME_COUNT = 5;

        static constexpr uint32_t FRAME_TO_TXID[5] = {
            0x200, // frame 0
            0x1FF, // frame 1
            0x2FF, // frame 2
            0x1FE, // frame 3
            0x2FE // frame 4
        };

        // 遍历三个 CAN 设备
        for (int can_idx = 0; can_idx < CAN_DEV_NUM; can_idx++)
        {
            // CAN 报文缓冲区
            CANInstance::msg_t msg_buf[FRAME_COUNT]{}; // 默认构造，所有成员变量均默认初始化

            // 标记每个帧中是否有启用电机
            bool frame_has_motor[FRAME_COUNT] = {}; // bool 默认初始化为false

            // 遍历当前 CAN 下的电机
            for (int motor_idx = 0; motor_idx < (int)idx_[can_idx]; motor_idx++)
            {
                DJIMotor* motor = motors_[can_idx][motor_idx];
                // 跳过不存在的电机
                if (!motor)
                    continue;

                const auto motor_id = motor->motor_id_;
                const auto motor_type = motor->type_;

                if (motor_id < 1 || motor_id > 8)
                {
                    // WARNING: 电机ID必须在1-8之间
                    continue; // skip invalid motor
                }

                // 确定电机对应帧索引
                uint8_t frame_index = getMotorFrameIndex(motor_type, motor_id);
                frame_has_motor[frame_index] = true;

                // 每帧最多容纳4个电机：ID1-4->偏移0~6
                uint8_t offset = ((motor_id - 1) % 4) * 2;

                const int16_t output =
                    (motor->is_enabled_ && motor->is_effort_set_) ? static_cast<int16_t>(motor->mapEffortToOutput(motor->effort_)) : 0;

                msg_buf[frame_index].data[offset] = static_cast<uint8_t>(output >> 8);
                msg_buf[frame_index].data[offset + 1] = static_cast<uint8_t>(output & 0xFF);
            }

            // 发送帧：只要该帧中存在电机，就发送（即使数据全 0）
            for (int frame_idx = 0; frame_idx < FRAME_COUNT; frame_idx++)
            {
                if (!frame_has_motor[frame_idx])
                    continue;

                CANInstance& can_tx = getTransmitCANInstance(can_idx);
                can_tx.send(FRAME_TO_TXID[frame_idx], msg_buf[frame_idx]);
            }
        }
    }

    void DJIMotor::parseData(const uint8_t* data, uint8_t len)
    {
        if (len < 8 || data == nullptr)
        {
            return;
        }

        dji_measure_.last_encoder = dji_measure_.encoder;
        dji_measure_.encoder = static_cast<uint16_t>(data[0] << 8) | data[1];

        // 处理单圈角度
        float raw_angle_rotor = ECD_TO_RAD * static_cast<float>(dji_measure_.encoder);
        measure_.angle_rotor = static_cast<float>(direction_) * raw_angle_rotor;

        // 原始速度 / 电流
        int16_t raw_speed_rpm =
            static_cast<int16_t>(direction_) * static_cast<int16_t>((data[2] << 8) | data[3]);

        int16_t raw_current_bit =
            static_cast<int16_t>(direction_) * static_cast<int16_t>((data[4] << 8) | data[5]);

        // 直接使用原始反馈值（无平滑滤波）
        measure_.speed_rotor = RPM_TO_RADPS * static_cast<float>(raw_speed_rpm);
        measure_.speed = measure_.speed_rotor / dji_reduction_radio_;

        dji_measure_.current_bit = raw_current_bit;

        measure_.current = static_cast<float>(raw_current_bit) * current_bit_2_A_;
        measure_.torque = measure_.current * dji_torque_constant_;

        dji_measure_.temperature = data[6];

        // 如果电机是刚刚上线，跳过半圈检测
        if (is_online_)
        {
            int32_t delta =
                static_cast<int32_t>(dji_measure_.last_encoder) - static_cast<int32_t>(dji_measure_.encoder);
            delta *= static_cast<int>(direction_);

            if (delta > 4096)
            {
                measure_.round++;
            }
            else if (delta < -4096)
            {
                measure_.round--;
            }
        }

        // 计算转子和输出轴的总位置
        float raw_total_angle_rotor =
            2.0f * PI * static_cast<float>(measure_.round) * static_cast<float>(direction_) + raw_angle_rotor;

        measure_.total_angle_rotor = static_cast<float>(direction_) * raw_total_angle_rotor;
        measure_.total_angle = measure_.total_angle_rotor / dji_reduction_radio_;

        is_online_ = true; // 接收到回调，标记为在线
        daemon_->reload();
    }


    void DJIMotor::controlAll()
    {
        // 第二阶段：统一发送控制指令
        sendCommandAll();
    }

    void DJIMotor::enableAll()
    {
        for (int can = 0; can < 2; can++)
        {
            for (int i = 0; i < (int)idx_[can]; i++)
            {
                DJIMotor* motor = motors_[can][i];
                if (motor)
                {
                    motor->enable();
                }
            }
        }
    }

    void DJIMotor::disableAll()
    {
        for (int can = 0; can < 2; can++)
        {
            for (int i = 0; i < (int)idx_[can]; i++)
            {
                DJIMotor* motor = motors_[can][i];
                if (motor)
                {
                    motor->disable();
                }
            }
        }
    }

    void DJIMotor::offlineCallback()
    {
        // 电机掉线回调
        is_online_ = false;
        // 将电机标记为未设置参考值，防止重新上电后继续使用旧的参考值。有效性待验证
        unsetEffort();
        // 清除积分项等操作
        measure_ = {};
        dji_measure_ = {};
    }

    // void DJIMotor::sendCommand()
    // {
    //     // DJI 电机不支持逐个发送，留空或仅用于兼容接口
    // }
    //
    // //同步所有已注册的大疆电机的启用状态
    // void DJIMotor::syncEnableState()
    // {
    //     //大疆电机不需要手动启用，is_enabled_标志位即可控制其通断
    // }


    bool DJIMotor::hasDisabledMotor()
    {
        for (int can = 0; can < 3; can++)
        {
            for (int i = 0; i < (int)idx_[can]; i++)
            {
                DJIMotor* motor = motors_[can][i];
                if (motor && !motor->is_enabled_)
                    return true;
            }
        }
        return false;
    }

    bool DJIMotor::hasOfflineMotor()
    {
        for (int can = 0; can < 3; can++)
        {
            for (int i = 0; i < (int)idx_[can]; i++)
            {
                DJIMotor* motor = motors_[can][i];
                if (motor && !motor->is_online_)
                    return true;
            }
        }
        return false;
    }

} // namespace ega
