//
// Created by An on 2025/9/29.
//

#ifndef MODULE_DMMOTOR_H_
#define MODULE_DMMOTOR_H_

#include "driver_can.h"
#include "motor.h"
#include "daemon.h"
#include <array>
#include <algorithm>  // for std::find
#include <optional>

namespace ega
{
    class DMMotor : public Motor
    {
        /* ====================== 0. Config（从基类拆出） ====================== */
    public:
        struct Config
        {
            // 公共字段（按你的要求在派生类重复）
            Direction direction = Direction::NORMAL;
            // CANInstance::Config can_config;
            CAN_HandleTypeDef* can_handle;
            uint32_t can_tx_id;            // 从0x00到0x0F
            uint32_t can_rx_id;             //建议不小于tx_id

            // 每款达妙电机的参数不一样，使用前请一定连接到上位机确认参数，否则会造成速度或者力矩输出错误！
            float p_max_abs;   // 强制用户输入
            float v_max_abs;
            float t_max_abs;
            float external_reduction_radio = 1.0f; // 外部减速比。达妙电机绝大多数能直接输出输出轴信息，默认减速比为1
            bool use_mit = false;         // 仅当use_mit为true时，才使用自带的MIT模式
        };

        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        static constexpr size_t MAX_MOTORS = 12; // 理论上同一个车的达妙电机数量没有上限，这里用于静态数组分配
        static constexpr float KP_MIN = 0.0f;
        static constexpr float KP_MAX = 500.0f;
        static constexpr float KD_MIN = 0.0f;
        static constexpr float KD_MAX = 5.0f;

        static constexpr float SPEED_SMOOTH_COEF = 0.85f;      // 最好大于0.85
        static constexpr float TORQUE_SMOOTH_COEF = 0.9f;      // 必须大于0.9

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        enum ModeCommand
        {
            MOTOR_MODE = 0xfc,    // 使能,会响应指令
            RESET_MODE = 0xfd,    // 停止
            ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
            CLEAR_ERROR = 0xfb    // 清除电机过热错误
        };

        struct DMMeasure
        {
            uint8_t id{};      // 电机ID低8位
            uint8_t state{};   // 错误状态码

            // 达妙原装数据
            // float angle_rad = 0.0f;    // rad,大概四圈
            // float speed_rads = 0.0f;   // rad/s
            float temperature_mos = 0.0f;      // 摄氏度
            float temperature_rotor = 0.0f;    // 摄氏度

            float last_angle = 0.0f; //rad
        };

        // CAN 帧结构（保持旧版）
        struct msg_t {
            uint16_t position_des;
            uint16_t velocity_des;
            uint16_t torque_des;
            uint16_t kp;
            uint16_t kd;
        };

        struct MITParam
        {
            float p_des;
            float v_des;
            float t_des;
            uint16_t kp;
            uint16_t kd;
        };

        /* ====================== 3. 静态接口 ====================== */
    public:
        static void controlAll(); // 控制循环
        static void enableAll();  // 使能所有电机，建议上电后等待几秒再启用
        static void disableAll(); // 禁用所有电机
        static void sendCommandAll();

        static bool hasDisabledMotor();
        static bool hasOfflineMotor();

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        explicit DMMotor(const Config& config);
        ~DMMotor() override;

        /* ====================== 5. 公共接口 ====================== */
    public:
        void sendCommand() override;

        void enable() override;
        void disable() override;
        //重设零点
        void setZeroPosition();

        // 电机模式设定（旧版应在 cpp 里实现）
        void setMIT(float p_des, float v_des, float t_des, float kp, float kd);

        // 解析反馈（旧版通常存在）
        void parseData(const uint8_t* data, uint8_t len);
        DMMeasure getDMMeasure() const { return dm_measure_; }

        //调试信息
        void debug() override{
            logger_printf("DMMotor Effort:%f\r\n", effort_); //用户可以自行修改想要查看的中间变量
        };

        /* ====================== 6. 成员变量（尽量保持旧版命名） ====================== */
    private:
        static inline std::array<DMMotor *, MAX_MOTORS> motors_{};
        static inline size_t idx_ = 0;

        DMMeasure dm_measure_{};

        /** 达妙电机需要区分 CAN在线 和 电机在线
         *  CAN在线，表示达妙电机能够正常反馈can消息，但此时可能电机仍未使能（灯为红色常亮）
         *  电机在线，表示在CAN在线的前提下，返回的state==0x01，此时灯为绿色常亮，这才是电机的真正在线状态
         */
        bool is_can_online_ = false;

        // MIT 参数缓存
        float p_des_buf_ = 0.0f;
        float v_des_buf_ = 0.0f;
        float t_des_buf_ = 0.0f;
        float kp_buf_ = 0.0f;
        float kd_buf_ = 0.0f;

        // 达妙参数
        float p_max_abs_ = 0.0f;
        float v_max_abs_ = 0.0f;
        float t_max_abs_ = 0.0f;

        bool use_mit_ = false;

        // 减速比：外部添加的减速比。达妙电机可以直接输出输出轴速度和位置
        float dm_reduction_radio_ = 1.0f;

    private:
        // 电机掉线回调
        void offlineCallback() override;

        // 发送使能失能或清除命令
        void sendDMModeCommand(ModeCommand cmd);
    };
} // namespace ega

#endif //MODULE_DMMOTOR_H_
