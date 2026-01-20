//
// Created by lenovo on 2025/10/31.
//
#include "LKMotor.h"

namespace ega
{

    LKMotor::LKMotor(const Config& config) :
        motor_id_(config.motor_id),
        encoder_bits_(config.encoder_bits)
    {
        direction_=config.direction;

        // 对于翎控电机，最大effort值为转矩闭环命令最大值，范围是-2048到2048，但是具体到不同系列，实际电流值映射范围又有所不同。So fk u lk
        output_max_abs_ = OUTPUT_MAX;

        const uint32_t lk_tx_id = 0x140 + motor_id_;

        // 根据手册翎控的接收id应该是0x180+motor_id，但是实测下来发现是和发送id一样。So fk u lk
        const uint32_t lk_rx_id = 0x140 + motor_id_;

        // 实例化can
        CANInstance::Config can_cfg = {.handle = config.can_handle,
                                       .tx_id = lk_tx_id,
                                       .tx_callback = nullptr,
                                       .rx_id = lk_rx_id,
                                       .rx_callback = [this](const uint8_t* data, const uint8_t len)
                                       {
                                           this->parseData(data, len);
                                       }};
        can_.emplace(can_cfg);

        // 实例化守护进程
        Daemon::Config daemon_cfg = {.reload_time_ms = 200, // 超时200ms认为电机掉线
                                     .init_time_ms = 100,
                                     .callback = [this]() { this->offlineCallback(); }};
        daemon_.emplace(daemon_cfg);


        // 注册到列表中
        if (idx_ < MAX_MOTORS)
        {
            motors_[idx_++] = this;
        }
    }

    LKMotor::~LKMotor()
    {
        auto it = std::find(motors_.begin(), motors_.begin() + idx_, this);
        if (it != motors_.begin() + idx_)
        {
            size_t pos = std::distance(motors_.begin(), it);
            motors_[pos] = motors_[idx_ - 1]; // 用最后一个覆盖
            motors_[idx_ - 1] = nullptr;
            --idx_;
        }
    }

    void LKMotor::controlAll()
    {
        // calculateAll();
        sendCommandAll();
    }

    void LKMotor::enableAll()
    {
        // 遍历所有已注册电机，将其使能
        // 建议上电后等待几秒再调用
        for (size_t i = 0; i < idx_; ++i)
        {
            LKMotor* m = motors_[i];
            if (!m)
                continue; // 跳过未注册的电机
            m->enable();
        }
    }

    void LKMotor::disableAll()
    {
        // 遍历所有已注册电机，将其失能
        for (size_t i = 0; i < idx_; ++i)
        {
            LKMotor* m = motors_[i];
            if (!m)
                continue;
            m->disable();
        }
    }

    void LKMotor::sendCommandAll()
    {
        for (size_t i = 0; i < idx_; ++i)
        {
            LKMotor* m = motors_[i];
            if (!m)
                continue;
            m->sendCommand();
        }
    }


    void LKMotor::parseData(const uint8_t* data, const uint8_t len)
    {
        if (len < 8 || data == nullptr)
        {
            return;
        }
        // 只有力矩指令或者读取状态指令才认为反馈有效
        if (data[0] != TORQUE_CMD && data[0] != READ_CMD)
        {
            return;
        }

        //温度
        lk_measure_.temperature = static_cast<int8_t>(data[1]);
        if (lk_measure_.temperature == 0)
        {
            return; //如果温度为0，认为是异常数据，直接返回
            //翎控电机刚上电时第一帧数据全0
        }
        //电流
        lk_measure_.current_bit = static_cast<int8_t>(direction_)*static_cast<int16_t>(data[2] | (data[3] << 8));
        //速度 度每秒
        const int16_t temp_speed = static_cast<int16_t>(data[4] | (data[5] << 8)); // dps
        measure_.speed_rotor = static_cast<float>(direction_)*static_cast<float>(temp_speed); // dps
        measure_.speed = measure_.speed_rotor; // 这里假设翎控电机不带减速箱，所以速度和转速相同

        //角度
        //更新角度值
        lk_measure_.last_encoder = lk_measure_.encoder;

        lk_measure_.encoder = static_cast<uint16_t>(data[6] | (data[7] << 8)); // 编码器值

        measure_.angle_rotor = static_cast<float>(direction_)*static_cast<float>(lk_measure_.encoder) / (1 << encoder_bits_) * 2.0f * PI; // 转换为rad

        // 计算总圈数
        // 如果此时电机刚刚上线，跳过半圈检测
        if (is_online_)
        {
            int32_t diff = lk_measure_.encoder - lk_measure_.last_encoder; // 计算角度差
            diff *= static_cast<int8_t>(direction_);
            if (diff > ((1 << encoder_bits_) / 2))
            {
                measure_.round--;
            }
            else if (diff < -((1 << encoder_bits_) / 2))
            {
                measure_.round++;
            }
        }
        // // 计算累计位置
        measure_.total_angle_rotor = static_cast<float>(measure_.round) * 2.0f * PI + measure_.angle_rotor;
        measure_.total_angle=measure_.total_angle_rotor;

        is_online_ = true;
        daemon_->reload();
    }

    void LKMotor::sendCommand()
    {
        // if (!isEnabled())
        // {
        //     // 失能时，发送零力矩指令，防止电机继续转
        //     sendLKModeCommand(ModeCommand::TORQUE_ZERO_CMD);
        //     return;
        // }
        if (!isEffortSet() || !isEnabled())
        {
            sendLKModeCommand(ModeCommand::TORQUE_ZERO_CMD);
            return;
        }
        // 映射effort到输出值，范围从-10到10映射到该电机所允许的最大输出值
        int16_t tmp = static_cast<int16_t>(mapEffortToOutput(effort_));


        CANInstance::msg_t msg{};
        msg.length = 8;

        msg.data[0] = TORQUE_CMD;
        msg.data[1] = 0;
        msg.data[2] = 0;
        msg.data[3] = 0;
        msg.data[4] = static_cast<uint8_t>(tmp & 0xFF);
        msg.data[5] = static_cast<uint8_t>(tmp >> 8);
        msg.data[6] = 0;
        msg.data[7] = 0;

        // 如果电机使能，通过 CAN 实例发送
        if (isEnabled())
        {
            can_->send(msg);
        }
    }

    void LKMotor::enable()
    {
        // 与基类不同的是，主动发出一帧命令使能电机
        // 翎控上电后默认使能，这里仅启用软件使能，减少对电机硬件的管理
        // sendLKModeCommand(ModeCommand::MOTOR_MODE);

        // 赋值基类成员
        is_enabled_ = true;
    }

    void LKMotor::disable()
    {
        // 与基类不同的是，主动发出一帧命令失能电机
        // 翎控上电后默认使能，这里仅启用软件失能，减少对电机硬件的管理
        //sendLKModeCommand(ModeCommand::RESET_MODE);
        // 零力矩指令等价于停转指令，但是可以保证电机仍然有反馈
        // sendLKModeCommand(ModeCommand::TORQUE_ZERO_CMD);

        // 赋值基类成员
        is_enabled_ = false;
    }

    void LKMotor::syncEnableState()
    {
        if (isEnabled() and not isOnline())
        {
            //翎控上电后默认使能，这里仅启用软件失能，减少对电机硬件的管理,因此这里什么都不做
        }
        if (not isEnabled())
        {
            // 零力矩指令等价于停转指令，但是可以保证电机仍然有反馈
            sendLKModeCommand(ModeCommand::TORQUE_ZERO_CMD);
        }

    }

    void LKMotor::syncEnableStateAll()
    {
        // 遍历所有已注册电机，将其失能
        for (size_t i = 0; i < idx_; ++i)
        {
            LKMotor* m = motors_[i];
            if (!m)
                continue;
            m->syncEnableState();
        }

    }



    // void LKMotor::setZeroPosition()
    // {
    // }

    void LKMotor::offlineCallback()
    {
        // 电机掉线回调
        is_online_ = false;
        // 重置参考值标记位
        unsetEffort();
        // 清除积分项等操作
        measure_ = {};
        lk_measure_ = {};
    }

    void LKMotor::sendLKModeCommand(ModeCommand cmd)
    {
        // 使用can驱动中的msg类型
        CANInstance::msg_t msg;
        msg.length = 8;
        for (size_t i = 1; i < msg.length; i++)
        {
            msg.data[i] = 0xFF;
        }
        msg.data[0] = static_cast<uint8_t>(cmd);

        can_->send(msg);
    }

    bool LKMotor::hasDisabledMotor()
    {
        for (int i = 0; i < (int)idx_; i++)
        {
            if (motors_[i] && !motors_[i]->is_enabled_)
                return true;
        }
        return false;
    }

    bool LKMotor::hasOfflineMotor()
    {
        for (int i = 0; i < (int)idx_; i++)
        {
            if (motors_[i] && !motors_[i]->is_online_)
                return true;
        }
        return false;
    }
}
