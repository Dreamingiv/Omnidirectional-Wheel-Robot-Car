#include "cmsis_os.h"

#include "DMmotor.h"

#include "driver_can.h"
#include "driver_dwt.h"

#include "logger.h"
#include "utils.h"

namespace ega
{
    /****构造函数、析构函数、接口函数等****/
    DMMotor::DMMotor(const Config& config) :
        use_mit_(config.dm_motor_config.use_mit), p_max_abs_(config.dm_motor_config.p_max_abs),
        v_max_abs_(config.dm_motor_config.v_max_abs), t_max_abs_(config.dm_motor_config.t_max_abs),
        dm_reduction_radio_(config.dm_motor_config.reduction_radio)
    {

        // 初始化基类参数
        initParams(config);

        //对于达妙电机，最大effort值为最大扭矩值
        effort_max_abs_=t_max_abs_;

        // 实例化can
        CANInstance::Config can_cfg = {.handle = config.can_config.handle,
                                       .tx_id = config.can_config.tx_id,
                                       .tx_callback = nullptr,
                                       .rx_id = config.can_config.rx_id,
                                       .rx_callback = [this](const uint8_t* data, uint8_t len)
                                       { this->parseData(data, len); }};
        can_.emplace(can_cfg);
        // 实例化守护进程
        Daemon::Config daemon_cfg = {.reload_count = 20, // 超时200ms认为电机掉线
                                     .init_count = 10,
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


    void DMMotor::sendCommandAll()
    {
        for (size_t i = 0; i < idx_; ++i)
        {
            DMMotor* m = motors_[i];
            if (!m)
                continue;
            if (!m->isEnabled())
                continue;
            //		if (!m->isOnline())
            //{//如果电机存在，且使能，但却掉线，尝试发送一帧命令唤醒它。更新：这一步在enableAll中应当已经完成，故注释掉
            //			m->sendModeCommand(ModeCommand::MOTOR_MODE);
            //			continue;
            //		}
            m->sendCommand();
        }
    }

    void DMMotor::controlAll()
    {
        // calculateAll();
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
            if (m->isEnabled() and m->isOnline())
                continue; // 跳过已经使能且确实被激活使能的电机，注意，这里的在线指的是电机有反馈帧，且state返回值为1
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
        // 与基类不同的是，主动发出一帧命令使能电机
        // 使用can驱动中的msg类型
        sendDMModeCommand(ModeCommand::CLEAR_ERROR); // 设置了can timeout的需要先清除错误
        // 等待200us，确保电机状态更新 todo 需要验证这里会不会造成阻塞等待？
        DWTInstance::delay(0.0002);
        sendDMModeCommand(ModeCommand::MOTOR_MODE);

        // 赋值基类成员
        is_enabled_ = true;
    }

    void DMMotor::disable()
    {
        // 与基类不同的是，主动发出一帧命令失能电机
        sendDMModeCommand(ModeCommand::RESET_MODE);

        // 赋值基类成员
        is_enabled_ = false;
    }

    void DMMotor::offlineCallback()
    {
        // 电机掉线回调
        is_online_ = false;
        // 重置参考值标记位
        unsetEffort();
        // 清除积分项等操作 todo
    }

    void DMMotor::parseData(const uint8_t* data, uint8_t len)
    {
        if (len < 8 || data == nullptr)
        {
            return;
        }

        uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量

        //保存上一次位置
        dm_measure_.last_angle = measure_.angle_rotor;

        // 解析基本数据
        dm_measure_.id = data[0] & 0x0f;
        dm_measure_.state = (data[0] >> 4) & 0x0f;

        tmp = static_cast<uint16_t>((data[1] << 8) | data[2]);
        dm_measure_.angle_rad = float(direction_)*utils::u16_to_float(tmp, -p_max_abs_, p_max_abs_, 16);
        tmp = static_cast<uint16_t>((data[3] << 4) | data[4] >> 4);
        dm_measure_.speed_rads = float(direction_)*utils::u16_to_float(tmp, -v_max_abs_, v_max_abs_, 12);
        tmp = static_cast<uint16_t>(((data[4] & 0x0f) << 8) | data[5]);
        measure_.torque = float(direction_)*utils::u16_to_float(tmp, -t_max_abs_, t_max_abs_, 12);

        //如果提供了转矩常数，计算电流

//        // ---- 修正版平滑滤波（防止FPU下溢） ----
//        constexpr float EPS = 1e-6f;
//        // 读取原始速度和力矩，单位保持rad
//        tmp = static_cast<uint16_t>((data[3] << 4) | data[4] >> 4);
//        auto raw_speed_rads = utils::u16_to_float(tmp, -v_max_abs_, v_max_abs_, 12);
//        tmp = static_cast<uint16_t>(((data[4] & 0x0f) << 8) | data[5]);
//        auto raw_torque = utils::u16_to_float(tmp, -t_max_abs_, t_max_abs_, 12);
//
//        float filtered_velocity =
//            (1.0f - SPEED_SMOOTH_COEF) * dm_measure_.speed_rads + SPEED_SMOOTH_COEF * (float)raw_speed_rads;
//        float filtered_torque =
//            (1.0f - TORQUE_SMOOTH_COEF) * float(measure_.torque) + TORQUE_SMOOTH_COEF * (float)raw_torque;
//        // 下溢保护：若结果极小则直接归零
//        if (fabsf(filtered_velocity) < EPS)
//            filtered_velocity = 0.0f;
//        if (fabsf(filtered_torque) < EPS)
//            filtered_torque = 0.0f;
//
//        dm_measure_.speed_rads = filtered_velocity;
//        measure_.torque = filtered_torque;

        dm_measure_.temperature_mos = (float)data[6];
        dm_measure_.temperature_rotor = (float)data[7];

        // 计算转子位置
        measure_.angle_rotor = RAD_2_DEGREE * dm_measure_.angle_rad;
        // 计算转子速度和输出轴速度
        measure_.speed_rotor = RAD_2_DEGREE * dm_measure_.speed_rads;
        measure_.speed = measure_.speed_rotor / dm_reduction_radio_;


        // 计算总圈数（检测跨越 ±π 或 ±12.5 过零点）
        // 电机刚刚上线时绕过半圈检测
        if (is_online_){
            float diff = measure_.angle_rotor - dm_measure_.last_angle;
            if (diff > RAD_2_DEGREE * (p_max_abs_ - (-p_max_abs_)) / 2)
            {
                measure_.round--;
            }
            else if (diff < -RAD_2_DEGREE * (p_max_abs_ - (-p_max_abs_)) / 2)
            {
                measure_.round++;
            }
        }

        // 计算累计位置
        measure_.total_angle_rotor =
            float(measure_.round) * RAD_2_DEGREE * (p_max_abs_ - (-p_max_abs_)) + measure_.angle_rotor;
        // 计算输出轴总位置
        measure_.total_angle = measure_.total_angle_rotor / dm_reduction_radio_;

        // 注意！在本框架的语境下，有回调但是state不使能同样视为掉线。
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

        daemon_->reload();
    }

    // 直接由外部给定三个参数，在MIT模式时可用,用户需要自己保证自己的参数对不对
    void DMMotor::setMIT(const float position, const float velocity, const float torque, const float kp, const float kd)
    {
        if (!use_mit_)
        {
            // Waring
            return;
        }

        // 将计算好的参考值放进邮箱
        p_des_buf_ = float(direction_)*utils::limit(position, -p_max_abs_, p_max_abs_);
        v_des_buf_ = float(direction_)*utils::limit(velocity, -v_max_abs_, v_max_abs_);
        t_des_buf_ = float(direction_)*utils::limit(torque, -t_max_abs_, t_max_abs_);
        kp_buf_ = utils::limit(kp, KP_MIN, KP_MAX);
        kd_buf_ = utils::limit(kd, KD_MIN, KD_MAX);

        is_effort_set = true;
    }


    void DMMotor::sendCommand()
    {
        if (!isEnabled())
            return;
        if (!isEffortSet())
            return;

        msg_t mailbox{};

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
            const float output = utils::limit(effort_, -t_max_abs_, t_max_abs_);
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
} // namespace ega
