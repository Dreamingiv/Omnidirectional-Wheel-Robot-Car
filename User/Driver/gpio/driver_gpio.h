//
// Created by An on 2025/11/21.
//

#ifndef DRIVER_GPIO_H_
#define DRIVER_GPIO_H_

#include "gpio.h"          // STM32 HAL
#include <cstdint>
#include <functional>      // 可选：用于回调（若不支持，可用函数指针替代）

namespace ega
{
    class GPIOInstance
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        // HAL 回调函数声明为友元，以便访问私有变量
        friend void ::HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
        static constexpr size_t MX_INS_NUM = 16; ///< 最大GPIO实例数量
        using ExtiCallback = std::function<void()>; ///< GPIO 中断回调函数类型

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        enum class EXTI_TRIG_MODE : uint8_t
        {
            //枚举外部中断触发模式，方便软件筛选自定义回调
            RISING = 0,
            FALLING = 1,
            RISING_FALLING = 2,
            NONE = 3,
        };

        struct Config
        {
            GPIO_TypeDef* port; // GPIOA,GPIOB,GPIOC...
            uint16_t pin; // 引脚号,@note 这里的引脚号是GPIO_PIN_0,GPIO_PIN_1...
            ExtiCallback exti_callback = nullptr; // exti中断回调函数

            GPIO_PinState default_state = GPIO_PIN_RESET; // 默认引脚电平，不常用；主要用于判断触发回调时依赖的常态电平
            EXTI_TRIG_MODE exti_mode = EXTI_TRIG_MODE::NONE; // 用户自定义的exti中断模式,不常用；主要用于软件判断上升沿下降沿
        };

        /* ====================== 3. 静态接口 ====================== */

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        explicit GPIOInstance(const Config& config);
        ~GPIOInstance();

        GPIOInstance(const GPIOInstance&) = delete; //禁止拷贝
        GPIOInstance& operator=(const GPIOInstance&) = delete; //禁止赋值拷贝
        GPIOInstance(GPIOInstance&&) = delete; //禁止移动构造
        GPIOInstance& operator=(GPIOInstance&&) = delete; //禁止移动赋值

        /* ====================== 5. 公共接口 ====================== */
    public:
        /// 设置引脚为高电平
        void set() const { HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET); }
        /// 设置引脚为低电平
        void reset() const { HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET); }
        /// 翻转引脚电平
        void toggle() const { HAL_GPIO_TogglePin(port_, pin_); }

        /// 读取引脚电平
        [[nodiscard]] GPIO_PinState read() const { return HAL_GPIO_ReadPin(port_, pin_); }
        /// 获取默认电平
        [[nodiscard]] GPIO_PinState getDefaultState() const { return default_state_; }
        /// 获取触发模式
        [[nodiscard]] EXTI_TRIG_MODE getExtiMode() const { return exti_mode_; }

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        /// 所有类实例的指针列表（共享）
        static std::array<GPIOInstance*, MX_INS_NUM> instance_list_;
        static uint8_t instance_count_;

    private:
        GPIO_TypeDef* port_ = nullptr;
        uint16_t pin_ = 0;
        ExtiCallback exti_callback_ = nullptr;

        GPIO_PinState default_state_ = GPIO_PIN_RESET; // 默认引脚电平,不常用
        EXTI_TRIG_MODE exti_mode_ = EXTI_TRIG_MODE::NONE; // exti中断模式,不常用
    };
}
#endif //DRIVER_GPIO_H_
