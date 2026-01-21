#include "cmsis_os.h"

#include "DMMotor.h"

#include "driver_can.h"
#include "driver_dwt.h"

#include "logger.h"
#include "utils.h"

namespace ega
{
    /****构造函数、析构函数、接口函数等****/
    DMMotor::DMMotor(const Config& config) :
        use_mit_(config.use_mit), p_max_abs_(config.p_max_abs),
        v_max_abs_(config.v_max_abs), t_max_abs_(config.t_max_abs),
        dm_reduction_radio_(config.external_reduction_radio)
    {

        // 初始化基类参数
        // initParams(config);
        direction_=config.direction;

        // 对于达妙电机，最大effort值为最大扭矩值
        output_max_abs_ = t_max_abs_;

        // 实例化can
        CANInstance::Config can_cfg = {.handle = config.can_handle,
                                       .tx_id = config.can_tx_id,
                                       .tx_callback = nullptr,
                                       .rx_id = config.can_rx_id,
                                       .rx_callback = [this](const uint8_t* data, uint8_t len)
                                       { this->parseData(data, len); }};
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
    DMMotor::~DMMotor()
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

    void DMMotor::sendDMModeCommand(DMMotor::ModeCommand cmd)
    {
        // 使用can驱动中的msg类型
        CANInstance::msg_t msg;
        msg.length = 8;
        for (size_t i = 0; i < msg.length - 1; i++)
        {
            msg.data[i] = 0xFF;
        }
        msg.data[7] = static_cast<uint8_t>(cmd);

        can_->send(msg);
    }

    void DMMotor::setZeroPosition()
    {
        // 发送重设零点命令
        sendDMModeCommand(ModeCommand::ZERO_POSITION);
    }

    void DMMotor::offlineCallback()
    {
        // 电机掉线回调
        is_online_ = false;
        is_can_online_ = false;
        // 重置参考值标记位
        unsetEffort();
        // 清除积分项等操作
        measure_ = {};
        dm_measure_ = {};
    }

    void DMMotor::setMIT(float p_des, float v_des, float t_des, float kp, float kd)
    {
        if (!use_mit_)
        {
            // Waring
            return;
        }

        // 将计算好的参考值放进邮箱
        p_des_buf_ = float(direction_) * utils::limit(p_des, -p_max_abs_, p_max_abs_);
        v_des_buf_ = float(direction_) * utils::limit(v_des, -v_max_abs_, v_max_abs_);
        t_des_buf_ = float(direction_) * utils::limit(t_des, -t_max_abs_, t_max_abs_);
        kp_buf_ = utils::limit(kp, KP_MIN, KP_MAX);
        kd_buf_ = utils::limit(kd, KD_MIN, KD_MAX);

        is_effort_set_ = true;
    }

    void DMMotor::sendCommand()
    {
        if (!isEnabled())
        {
            //如果电机没有设置为使能，什么都不做
            return;
        }
        if (!isEffortSet())
        {
            //如果电机设置为了使能，但是effort未设置，发送一帧全零命令，维持电机can反馈，防止触发掉线
            mailbox_t zero_cmd{};
            zero_cmd.position_des = utils::float_to_u16(0, -p_max_abs_, p_max_abs_, 16);
            zero_cmd.velocity_des = utils::float_to_u16(0, -v_max_abs_, v_max_abs_, 12);
            zero_cmd.torque_des = utils::float_to_u16(0, -t_max_abs_, t_max_abs_, 12);
            zero_cmd.kp = utils::float_to_u16(0, KP_MIN, KP_MAX, 12);
            zero_cmd.kd = utils::float_to_u16(0, KD_MIN, KD_MAX, 12);

            CANInstance::msg_t msg{};
            msg.length = 8;
            msg.data[0] = static_cast<uint8_t>(zero_cmd.position_des >> 8);
            msg.data[1] = static_cast<uint8_t>(zero_cmd.position_des);
            msg.data[2] = static_cast<uint8_t>(zero_cmd.velocity_des >> 4);
            msg.data[3] = static_cast<uint8_t>(((zero_cmd.velocity_des & 0xF) << 4) | (zero_cmd.kp >> 8));
            msg.data[4] = static_cast<uint8_t>(zero_cmd.kp);
            msg.data[5] = static_cast<uint8_t>(zero_cmd.kd >> 4);
            msg.data[6] = static_cast<uint8_t>(((zero_cmd.kd & 0xF) << 4) | (zero_cmd.torque_des >> 8));
            msg.data[7] = static_cast<uint8_t>(zero_cmd.torque_des);

            can_->send(msg);
            return;
        }


        mailbox_t mailbox{};

        if (use_mit_)
        {
            // 如果使用mit模式，邮箱应该已经被setMIT函数填充
            mailbox.position_des = utils::float_to_u16(p_des_buf_, -p_max_abs_, p_max_abs_, 16);
            mailbox.velocity_des = utils::float_to_u16(v_des_buf_, -v_max_abs_, v_max_abs_, 12);
            mailbox.torque_des = utils::float_to_u16(t_des_buf_, -t_max_abs_, t_max_abs_, 12);
            mailbox.kp = utils::float_to_u16(kp_buf_, KP_MIN, KP_MAX, 12);
            mailbox.kd = utils::float_to_u16(kd_buf_, KD_MIN, KD_MAX, 12);
        }
        else
        {
            // 普通模式
            const float output = utils::limit(mapEffortToOutput(effort_), -t_max_abs_, t_max_abs_);
            mailbox.position_des = utils::float_to_u16(0, -p_max_abs_, p_max_abs_, 16);
            mailbox.velocity_des = utils::float_to_u16(0, -v_max_abs_, v_max_abs_, 12);
            mailbox.torque_des = utils::float_to_u16(output, -t_max_abs_, t_max_abs_, 12);
            mailbox.kp = utils::float_to_u16(0, KP_MIN, KP_MAX, 12);
            mailbox.kd = utils::float_to_u16(0, KD_MIN, KD_MAX, 12);
        }

        CANInstance::msg_t msg{};
        msg.length = 8;

        msg.data[0] = static_cast<uint8_t>(mailbox.position_des >> 8);
        msg.data[1] = static_cast<uint8_t>(mailbox.position_des);
        msg.data[2] = static_cast<uint8_t>(mailbox.velocity_des >> 4);
        msg.data[3] = static_cast<uint8_t>(((mailbox.velocity_des & 0xF) << 4) | (mailbox.kp >> 8));
        msg.data[4] = static_cast<uint8_t>(mailbox.kp);
        msg.data[5] = static_cast<uint8_t>(mailbox.kd >> 4);
        msg.data[6] = static_cast<uint8_t>(((mailbox.kd & 0xF) << 4) | (mailbox.torque_des >> 8));
        msg.data[7] = static_cast<uint8_t>(mailbox.torque_des);

        // 如果电机使能，通过 CAN 实例发送
        if (isEnabled())
        {
            can_->send(msg);
        }
    }

    void DMMotor::sendCommandAll()
    {
        for (size_t i = 0; i < idx_; ++i)
        {
            DMMotor* m = motors_[i];
            if (!m)
                continue;
            if (!m->isEnabled())
                continue;
            m->sendCommand();
        }
    }

    void DMMotor::controlAll()
    {
        sendCommandAll();
    }

    void DMMotor::enableAll()
    {
        // 遍历所有已注册电机，将其使能
        // 建议上电后等待几秒再调用
        for (size_t i = 0; i < idx_; ++i)
        {
            DMMotor* m = motors_[i];
            if (!m)
                continue; // 跳过未注册的电机
            m->enable();
        }
    }

    void DMMotor::disableAll()
    {
        // 遍历所有已注册电机，将其失能
        for (size_t i = 0; i < idx_; ++i)
        {
            DMMotor* m = motors_[i];
            if (!m)
                continue;
            m->disable();
        }
    }

    void DMMotor::enable()
    {

        // // 与基类不同的是，主动发出一帧命令使能电机
        // // 使用can驱动中的msg类型
        // sendDMModeCommand(ModeCommand::CLEAR_ERROR); // 设置了can timeout的需要先清除错误
        // // 等待200us，确保电机状态更新
        // // 因为理论上enable只在遥控器初次上电时触发一次，阻塞延迟200us暂时认为没影响。当然有更好的办法是最好的
        // DWTInstance::delay(0.0002);
        // sendDMModeCommand(ModeCommand::MOTOR_MODE);

        // 更新，为了实现更精细的电机使能控制，enable本身不直接尝试使能电机，
        // 而是将使能状态设置为true，然后通过syncEnableState()同步到电机
        // 这样可以保证在电机全局保护模式生效的前提下，用户可以手动控制电机使能状态

        // 赋值基类成员
        is_enabled_ = true;
    }

    void DMMotor::disable()
    {
        // // 与基类不同的是，主动发出一帧命令失能电机
        // sendDMModeCommand(ModeCommand::RESET_MODE);

        // 更新，为了实现更精细的电机使能控制，enable本身不直接尝试使能电机，
        // 而是将使能状态设置为true，然后通过syncEnableState()同步到电机
        // 这样可以保证在电机全局保护模式生效的前提下，用户可以手动控制电机使能状态

        // 赋值基类成员
        is_enabled_ = false;
    }

    void DMMotor::syncEnableState()
    {
        //同步使能状态
        //如果电机被设置了使能，且当前未在线，尝试发送使能命令
        if (isEnabled() and not isOnline())
        {
            sendDMModeCommand(ModeCommand::CLEAR_ERROR); // 设置了can timeout的需要先清除错误
            // 等待200us，确保电机状态更新
            // 因为理论上enable只在遥控器初次上电时触发一次，阻塞延迟200us暂时认为没影响。当然有更好的办法是最好的
            DWTInstance::delay(0.0002);
            sendDMModeCommand(ModeCommand::MOTOR_MODE);
        }
        //如果电机被设置为失能状态，无论当前是否在线，都发送失能命令
        if (not isEnabled())
        {
            sendDMModeCommand(ModeCommand::RESET_MODE);
        }
    }

    void DMMotor::syncEnableStateAll()
    {
        // 遍历所有已注册电机，根据is_enabled标志位同步使能状态
        for (size_t i = 0; i < idx_; ++i)
        {
            DMMotor* m = motors_[i];
            if (!m)
                continue;
            m->syncEnableState();
        }

    }



    bool DMMotor::hasDisabledMotor()
    {
        for (int i = 0; i < (int)idx_; i++)
        {
            if (motors_[i] && !motors_[i]->is_enabled_)
                return true;
        }
        return false;
    }

    bool DMMotor::hasOfflineMotor()
    {
        for (int i = 0; i < (int)idx_; i++)
        {
            if (motors_[i] && !motors_[i]->is_online_)
                return true;
        }
        return false;
    }

    void DMMotor::parseData(const uint8_t* data, uint8_t len)
    {
        if (len < 8 || data == nullptr)
        {
            return;
        }

        uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量

        // 保存上一次位置
        dm_measure_.last_angle = measure_.angle_rotor;

        // 解析基本数据
        dm_measure_.id = data[0] & 0x0f;
        dm_measure_.state = (data[0] >> 4) & 0x0f;

        tmp = static_cast<uint16_t>((data[1] << 8) | data[2]);
        measure_.angle_rotor = float(direction_) * utils::u16_to_float(tmp, -p_max_abs_, p_max_abs_, 16);
        tmp = static_cast<uint16_t>((data[3] << 4) | data[4] >> 4);
        measure_.speed_rotor = float(direction_) * utils::u16_to_float(tmp, -v_max_abs_, v_max_abs_, 12);
        tmp = static_cast<uint16_t>(((data[4] & 0x0f) << 8) | data[5]);
        measure_.torque = float(direction_) * utils::u16_to_float(tmp, -t_max_abs_, t_max_abs_, 12);

        // 如果提供了转矩常数，计算电流 todo

        dm_measure_.temperature_mos = (float)data[6];
        dm_measure_.temperature_rotor = (float)data[7];

        // 计算输出轴速度
        measure_.speed = measure_.speed_rotor / dm_reduction_radio_;

        // 计算总圈数（检测跨越 ±π 或 ±12.5 过零点）
        // 如果此时电机刚刚上线，跳过半圈检测
        if (is_can_online_)
        {
            float diff = measure_.angle_rotor - dm_measure_.last_angle;
            if (diff > (p_max_abs_ - (-p_max_abs_)) / 2)
            {
                measure_.round--;
            }
            else if (diff < - (p_max_abs_ - (-p_max_abs_)) / 2)
            {
                measure_.round++;
            }
        }

        // 计算累计位置
        measure_.total_angle_rotor =
            float(measure_.round) * (p_max_abs_ - (-p_max_abs_)) + measure_.angle_rotor;
        // 计算输出轴总位置
        measure_.total_angle = measure_.total_angle_rotor / dm_reduction_radio_;

        // 注意！在本框架的语境下，有回调但是state不使能同样视为掉线（can_timeout）。
        // 这要求达妙电机tx_id推荐设置范围为0x01-0x0F
        // 即txid 不能覆盖data[0]的高4位
        if (dm_measure_.state == 0x01)
        {
            is_online_ = true;
        }
        else
        {
            is_online_ = false;
        }
        //只要收到can消息，就认为电机的can是在线的（但是电机并不一定在线）
        is_can_online_ = true;

        daemon_->reload();
    }

} // namespace ega
