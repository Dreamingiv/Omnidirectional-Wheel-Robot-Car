//
// Created by An on 2025/9/29.
//

#ifndef MODULE_DJIMOTOR_H_
#define MODULE_DJIMOTOR_H_

#include "driver_can.h"
#include "led_on_board.h"
#include "motor.h"
namespace ega
{
    class DJIMotor : public Motor
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        static constexpr size_t MAX_MOTORS = 8; // 每条can线上挂载的最大电机数量，用于静态资源分配
        static constexpr size_t CAN_DEV_NUM = 3;

        static constexpr float RPM2DPS = 6.0f;
        static constexpr float ECD2DEGREE = 0.043945f;

        static constexpr float SPEED_SMOOTH_COEF = 0.85f; // 最好大于0.85
        static constexpr float CURRENT_SMOOTH_COEF = 0.9f; // 必须大于0.9

        static constexpr float TORQUE_CONSTANT_GM6020 = 0.741f; // N·m/A
        static constexpr float TORQUE_CONSTANT_M3508 = 0.3f;
        static constexpr float TORQUE_CONSTANT_M2006 = 0.18f;

        static constexpr float DEFAULT_REDUCTION_RADIO_GM6020 = 1.0f; // 减速比
        static constexpr float DEFAULT_REDUCTION_RADIO_M3508 = (3591.0f / 187.0f);
        static constexpr float DEFAULT_REDUCTION_RADIO_M2006 = 36.0f;

        static constexpr float CURRENT_BIT_2_A_GM6020 = 3.0f / 16384.0f; // 从电流bit转换为A,根据手册的输入bit到A映射的
        static constexpr float CURRENT_BIT_2_A_M3508 = 20.0f / 16384.0f; // 从电流bit转换为A
        static constexpr float CURRENT_BIT_2_A_M2006 = 10.0f / 10000.0f; // 从电流bit转换为A

        static constexpr float EFFORT_MAX_GM6020 = 25000.0f;
        static constexpr float EFFORT_MAX_GM6020_CURRENT = 16384.0f;
        static constexpr float EFFORT_MAX_M3508 = 16384.0f;
        static constexpr float EFFORT_MAX_M2006 = 10000.0f;

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        struct DJIMeasure
        {
            uint16_t encoder = 0; // 本次读取的编码器值 【0~8191】 3508上电时编码器数值很随机，不知为啥
            uint16_t last_encoder = 0; // 上次读取的编码器值
            int16_t current_bit = 0; // 实际转矩电流
            uint8_t temperature = 0; // 电机温度         # Celsius
        };

        /* ====================== 3. 静态接口 ====================== */
    public:
        static void controlAll(); // 所有大疆电机的控制循环

        static void enableAll(); // 使能所有电机，建议上电后等待几秒再启用
        static void disableAll(); // 禁用所有电机

        static void sendCommandAll();

    private:
        // 在 DJIMotor 内部：
        static CANInstance& getTransmitCANInstance(size_t index)
        {
            static CANInstance instances[6] = {
                {CANInstance::Config{.handle = &hcan1, .tx_id = 0x200}},
                {CANInstance::Config{.handle = &hcan1, .tx_id = 0x1FF}},
                {CANInstance::Config{.handle = &hcan1, .tx_id = 0x2FF}},
                {CANInstance::Config{.handle = &hcan2, .tx_id = 0x200}},
                {CANInstance::Config{.handle = &hcan2, .tx_id = 0x1FF}},
                {CANInstance::Config{.handle = &hcan2, .tx_id = 0x2FF}},
                // {CANInstance::Config{.handle = &hcan3, .tx_id = 0x200}},
                // {CANInstance::Config{.handle = &hcan3, .tx_id = 0x1FF}},
                // {CANInstance::Config{.handle = &hcan3, .tx_id = 0x2FF}},
            };
            return instances[index];
        }

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        explicit DJIMotor(const Motor::Config& config);
        ~DJIMotor() override;

        /* ====================== 5. 公共接口 ====================== */
    public:
        // void setEffort(const float& effort) override;
        void sendCommand() override {
            // Error 大疆电机不能单独调用sendcommand
        };

        void parseData(const uint8_t* data, uint8_t len) override;

        DJIMeasure& getDJIMeasure() { return dji_measure_; }
        float getDJIReductionRadio() const { return dji_reduction_radio_; };

        void debug() override
        {
            logger_printf("DJIMotor Effort:%f\r\n", effort_); // 用户可以自行修改想要查看的中间变量
        };

        /* ====================== 6. 受保护接口 ====================== */


        /* ====================== 7. 成员变量 ====================== */
    private:
        static inline std::array<DJIMotor*, MAX_MOTORS> motors_[CAN_DEV_NUM]{};
        static inline size_t idx_[CAN_DEV_NUM] = {0};

    private:
        const uint8_t motor_id_;

        DJIMeasure dji_measure_{};

        float dji_reduction_radio_; // 减速比，如果用户没有设置，默认是原装减速比。
        float dji_torque_constant_; // 转矩常数,考虑到电机只有三种，直接在构造函数中赋值
        float current_bit_2_A;    // 从电流bit转换为A的系数
    private:
        void offlineCallback() override;
    };
} // namespace ega
#endif // DJIMOTOR_H_
