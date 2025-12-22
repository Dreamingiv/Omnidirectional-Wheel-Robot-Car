//
// Created by 2b superman on 25-11-21.
//

#ifndef MODULE_SERVO_H_
#define MODULE_SERVO_H_

#include <cmath>
#include <optional>
#include "driver_pwm.h"
#include "gpio.h"

namespace ega
{
    /**
     * @brief 通用舵机设备驱动类
     * @details 基于 PWMInstance 实现，支持任意 PWM 频率和脉宽范围的舵机驱动
     * @note 需要在CubeMX中自行管理周期，舵机标准周期为50Hz，tim1和tim2主频240MHz，可设置预分频240-1，自动重载值20000-1
     * @attention 达妙开发板的PWM接口电源是可控电源，需要把PC15设置为HIGH才有电。
     */
    class Servo
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        /**
         * @brief 舵机初始化配置结构体
         * @note 请查阅舵机数据手册填写以下参数
         */
        struct Config
        {
            // 硬件关联
            //        TIM_HandleTypeDef* pwm_handle = nullptr;
            //        uint32_t           pwm_channel = 0;
            PWMInstance::Config pwm_config = {};

            // 舵机物理参数 (查阅 Datasheet)
            float max_angle = 180.0f; ///< 最大旋转角度 (常见 180.0 或 270.0)
            uint32_t min_pulse_us = 500; ///< 0度对应的脉宽 (微秒)，常见 500 或 1000
            uint32_t max_pulse_us = 2500; ///< 最大角度对应的脉宽 (微秒)，常见 2500 或 2000

            // 频率参数
            uint32_t period_us = 20000; ///< PWM 周期 (微秒)，标准模拟舵机为 20000 (50Hz)
            ///< 数字舵机可能为 3000 (333Hz)

            float init_angle = 0.0f; ///< 上电初始角度
        };

        /* ====================== 3. 静态接口 ====================== */

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        explicit Servo(const Config& config);
        ~Servo();

        // 禁止拷贝
        Servo(const Servo&) = delete;
        Servo& operator=(const Servo&) = delete;


        /* ====================== 5. 公共接口 ====================== */
    public:
        /**
         * @brief 启动舵机 (使能 PWM)
         */
        void start() const;

        /**
         * @brief 停止舵机 (关闭 PWM，舵机可能失力)
         */
        void stop() const;

        /**
         * @brief 设置舵机角度
         * @param angle 目标角度 (0.0 ~ max_angle)
         */
        void setAngle(float angle);

        /**
         * @brief [高级] 直接设置脉冲宽度 (微秒)
         * @param us 脉宽微秒数
         */
        void writeMicroseconds(uint32_t us);

        /**
         * @brief 获取当前设置的角度
         */
        float getAngle() const { return current_angle_; }

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        static inline int8_t servo_count_ = 0; ///< 活动舵机对象数量

    private:
        std::optional<PWMInstance> pwm_driver_; ///< 底层 PWM 驱动实例

        // 保存配置参数以供计算
        float max_angle_;
        uint32_t min_pulse_us_;
        uint32_t max_pulse_us_;
        uint32_t period_us_;

        float current_angle_; ///< 记录当前角度
    };
} // namespace ega
#endif // MODULE_SERVO_H_
