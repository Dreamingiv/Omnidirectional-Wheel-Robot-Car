#include "DJImotor.h"
#include <algorithm>
#include <cmath>

#include "PIDcontroller.h"
#include "controller.h"

#include "logger.h"
#include "utils.h"

namespace ega
{
    /**** 构造函数、析构函数、接口函数等 ****/
    DJIMotor::DJIMotor(const Config& config) : motor_id_(config.dji_motor_config.motor_id)
    {
        // 初始化基类参数
        initParams(config);

        /* ===== 计算参数 ===== */
        switch (type_)
        {
        case Type::GM6020:
            effort_max_abs_ = EFFORT_MAX_GM6020;
            dji_reduction_radio_ = 1.0f; // 云台电机 1:1
            dji_torque_constant_ = TORQUE_CONSTANT_GM6020 * dji_reduction_radio_ / DEFAULT_REDUCTION_RADIO_GM6020;
            current_bit_2_A = CURRENT_BIT_2_A_GM6020;
            break;
        case Type::GM6020_CURRENT:
            effort_max_abs_ = EFFORT_MAX_GM6020_CURRENT;
            dji_reduction_radio_ = 1.0f; // 云台电机 1:1
            dji_torque_constant_ = TORQUE_CONSTANT_GM6020 * dji_reduction_radio_ / DEFAULT_REDUCTION_RADIO_GM6020;
            current_bit_2_A = CURRENT_BIT_2_A_GM6020;
            break;
        case Type::M3508:
            effort_max_abs_ = EFFORT_MAX_M3508;
            dji_reduction_radio_ = (config.dji_motor_config.reduction_radio >= 1e-6f)
                ? config.dji_motor_config.reduction_radio
                : DEFAULT_REDUCTION_RADIO_M3508;
            dji_torque_constant_ = TORQUE_CONSTANT_M3508 * dji_reduction_radio_ / DEFAULT_REDUCTION_RADIO_M3508;
            current_bit_2_A = CURRENT_BIT_2_A_M3508;
            break;
        case Type::M2006:
            effort_max_abs_ = EFFORT_MAX_M2006;
            dji_reduction_radio_ = (config.dji_motor_config.reduction_radio >= 1e-6f)
                ? config.dji_motor_config.reduction_radio
                : DEFAULT_REDUCTION_RADIO_M2006;
            dji_torque_constant_ = TORQUE_CONSTANT_M2006 * dji_reduction_radio_ / DEFAULT_REDUCTION_RADIO_M2006;
            current_bit_2_A = CURRENT_BIT_2_A_M2006;
            break;
        default:
            //理论上不该进到这里
            effort_max_abs_ = 0.0f;
            dji_reduction_radio_ =
                (config.dji_motor_config.reduction_radio >= 1e-6f) ? config.dji_motor_config.reduction_radio : 1.0f;
            current_bit_2_A = 1.0f;
            break;
        }

        /* ===== 注册 CAN 回调 ===== */
        CANInstance::Config cfg = {
            .handle = config.can_config.handle,
            .rx_id =
                static_cast<uint16_t>((config.type == Type::GM6020 ? 0x204 : 0x200) + config.dji_motor_config.motor_id),
            .rx_callback = [this](const uint8_t* data, uint8_t len) { this->parseData(data, len); }};
        can_.emplace(cfg);

        // 实例化守护进程
        Daemon::Config daemon_cfg = {.reload_count = 20, // 超时200ms认为电机掉线
                                     .init_count = 10,
                                     .callback = [this]() { this->offlineCallback(); }};
        daemon_.emplace(daemon_cfg);

        /* ===== 注册静态数组 ===== */
        uint8_t can_dev = cfg.handle == &hcan1 ? 0 : cfg.handle == &hcan2 ? 1 : 2;

        if (idx_[can_dev] < MAX_MOTORS)
            motors_[can_dev][idx_[can_dev]++] = this;
    }

    DJIMotor::~DJIMotor()
    {
        uint8_t can_dev = can_->getHandle() == &hcan1 ? 0 : can_->getHandle() == &hcan2 ? 1 : 2;
        auto it = std::find(motors_[can_dev].begin(), motors_[can_dev].begin() + idx_[can_dev], this);
        if (it != motors_[can_dev].begin() + idx_[can_dev])
        {
            size_t pos = std::distance(motors_[can_dev].begin(), it);
            motors_[can_dev][pos] = motors_[can_dev][idx_[can_dev] - 1]; // 用最后一个覆盖
            motors_[can_dev][idx_[can_dev] - 1] = nullptr;
            idx_[can_dev]--;
        }
    }


    void DJIMotor::sendCommandAll()
    {
        // 每个 CAN 控制器维护 3 组帧 (0x200 / 0x1FF / 0x2FF)
        constexpr int FRAME_COUNT = 3;

        // 三帧 CAN 报文缓冲区
        CANInstance::msg_t msg_buf[FRAME_COUNT] = {
            {.data = {0}, .length = 8},
            {.data = {0}, .length = 8},
            {.data = {0}, .length = 8},
        };

        // 遍历三个 CAN 设备
        for (int can_idx = 0; can_idx < CAN_DEV_NUM; can_idx++)
        {
            // 清空缓冲
            for (auto& msg : msg_buf)
                std::fill(std::begin(msg.data), std::end(msg.data), 0);

            // 标记每个帧中是否有启用电机
            bool frame_has_motor[FRAME_COUNT] = {false, false, false};

            // 遍历当前 CAN 下的电机
            for (int motor_idx = 0; motor_idx < idx_[can_idx]; motor_idx++)
            {
                DJIMotor* motor = motors_[can_idx][motor_idx];
                // 跳过不存在的电机
                if (!motor)
                    continue;

                auto output = static_cast<int16_t>(motor->effort_);
                // 跳过没有被使能或者没有被人为设置目标值的电机，发送电流值为0。2006不发电流的话会维持上一帧！
                if (!(motor->isEnabled()) || !(motor->isEffortSet()))
                {
                    output = 0;
                }

                const auto id = motor->motor_id_;
                const auto type = motor->type_;

                // 确定电机对应帧索引
                uint8_t frame_index = 0;
                switch (type)
                {
                case Type::M2006:
                case Type::M3508:
                    frame_index = (id <= 4) ? 0 : 1; // 0x200 / 0x1FF
                    break;
                case Type::GM6020:
                    frame_index = (id <= 4) ? 1 : 2; // 0x1FF / 0x2FF
                    break;
                default:
                    frame_index = 0;
                    break;
                }

                frame_has_motor[frame_index] = true;

                // 每帧最多容纳4个电机：ID1-4->偏移0~6
                uint8_t offset = ((id - 1) % 4) * 2;

                msg_buf[frame_index].data[offset] = static_cast<uint8_t>(output >> 8);
                msg_buf[frame_index].data[offset + 1] = static_cast<uint8_t>(output & 0xFF);
            }

            // 发送三帧，只要该帧中存在启用电机，就发送（即使数据全 0）
            for (int frame_idx = 0; frame_idx < FRAME_COUNT; frame_idx++)
            {
                if (!frame_has_motor[frame_idx])
                    continue;

                // 映射至全局 CAN 实例索引
                int global_index = can_idx * FRAME_COUNT + frame_idx;
                CANInstance& can_tx = getTransmitCANInstance(global_index);
                can_tx.send(msg_buf[frame_idx]);
            }
        }
    }

    void DJIMotor::parseData(const uint8_t* data, uint8_t len)
    {
        if (len < 8 || data == nullptr)
        {
            return;
        }

        //todo 初始化时可能立刻触发一次半圈检测，待解决
        dji_measure_.last_encoder = dji_measure_.encoder;
        dji_measure_.encoder = static_cast<uint16_t>(data[0] << 8) | data[1];
        //处理单圈正反
        float raw_angle_rotor = ECD2DEGREE * static_cast<float>(dji_measure_.encoder);
        measure_.angle_rotor = float(direction_)*raw_angle_rotor;

        // ---- 修正版平滑滤波（防止FPU下溢） ----
        constexpr float EPS = 1e-6f;

        auto raw_speed_rpm = int16_t (direction_)*static_cast<int16_t>((data[2] << 8) | data[3]);
        auto raw_current_bit = int16_t (direction_)*static_cast<int16_t>((data[4] << 8) | data[5]);

        float filtered_speed_dps =
            (1.0f - SPEED_SMOOTH_COEF) * measure_.speed_rotor + SPEED_SMOOTH_COEF * RPM2DPS * (float)raw_speed_rpm;

        float filtered_current_bit = (1.0f - CURRENT_SMOOTH_COEF) * float(dji_measure_.current_bit) +
            CURRENT_SMOOTH_COEF * (float)raw_current_bit;

        // 下溢保护：若结果极小则直接归零
        if (fabsf(filtered_speed_dps) < EPS)
            filtered_speed_dps = 0.0f;
        if (fabsf(filtered_current_bit) < EPS)
            filtered_current_bit = 0.0f;

        // 计算转子速度和输出轴速度
        measure_.speed_rotor = filtered_speed_dps;
        measure_.speed = measure_.speed_rotor / dji_reduction_radio_;

        dji_measure_.current_bit = int16_t(filtered_current_bit);

        // 计算等效扭矩
        // todo 验证三种电机的扭矩是否正确
        measure_.current = filtered_current_bit * current_bit_2_A;
        measure_.torque = measure_.current * dji_torque_constant_;

        // ---- 其他原逻辑保持不变 ----
        dji_measure_.temperature = data[6];

        auto delta= dji_measure_.last_encoder - dji_measure_.encoder;
        delta *= int(direction_);
        if (delta > 4096)
        {
            measure_.round++;
        }
        else if (delta < -4096)
        {
            measure_.round--;
        }

        // 计算转子和输出轴的总位置
        float raw_total_angle_rotor = 360.0f * float(measure_.round)*float(direction_) + raw_angle_rotor;
        measure_.total_angle_rotor = float(direction_) * raw_total_angle_rotor;
//        measure_.total_angle_rotor = 360.0f * float(measure_.round) + measure_.angle_rotor;
        measure_.total_angle = measure_.total_angle_rotor / dji_reduction_radio_;

        is_online_ = true; // 接收到回调，标记为在线
        daemon_->reload();
    }

    void DJIMotor::controlAll()
    {
        // 第一阶段：计算所有电机控制输出
        // calculateAll();

        // 第二阶段：如果电机为使能状态，统一发送控制指令
        sendCommandAll();
    }

    void DJIMotor::enableAll()
    {
        for (int can = 0; can < 3; can++)
        {
            for (int i = 0; i < idx_[can]; i++)
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
        for (int can = 0; can < 3; can++)
        {
            for (int i = 0; i < idx_[can]; i++)
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
        // 清除积分项等操作 todo
    }
} // namespace ega
